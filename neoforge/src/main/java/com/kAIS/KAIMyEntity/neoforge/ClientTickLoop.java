package com.kAIS.KAIMyEntity.neoforge;

import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;
import com.kAIS.KAIMyEntity.urdf.control.JointControlBus;
import com.kAIS.KAIMyEntity.urdf.control.URDFArmRetargeter;
import com.kAIS.KAIMyEntity.urdf.control.URDFVmcMapper;
import com.kAIS.KAIMyEntity.urdf.control.VmcIk;
import com.kAIS.KAIMyEntity.urdf.control.VmcListenerManager;

import net.neoforged.neoforge.client.event.ClientTickEvent;   // ✅ NeoForge 클라 틱 이벤트
import net.neoforged.neoforge.common.NeoForge;               // ✅ 이벤트 버스

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * ClientTickLoop (NeoForge)
 *
 * VMC(UDP/OSC) → 본 스냅샷 폴링 → (리타게팅/IK) → BUS 합성 → renderer.setJointTarget(...)
 * 좌표/스케일 변환은 URDFVmcMapper 한 곳에서만 수행(중복 금지).
 *
 * 사용법:
 * - 클라이언트 초기화 시:
 *     ClientTickLoop.bindRenderer(rendererInstance);
 *     ClientTickLoop.register();
 * - 슬라이더 화면:
 *     new MotionEditorScreen(ClientTickLoop.renderer)  // 혹은 (renderer, ClientTickLoop.bus())
 */
public final class ClientTickLoop {

    private ClientTickLoop() {}

    // ───────── 외부에서 직접 쓰는 필드/메서드 ─────────

    /** 🔓 하위호환을 위해 공개: 다른 클래스가 직접 읽고/세팅할 수 있게 */
    public static volatile URDFModelOpenGLWithSTL renderer = null;

    /** BUS(우선순위 합성/EMA 스무딩) 접근자 — 슬라이더에서 주입용 */
    public static JointControlBus bus() { return BUS; }

    /** 렌더러 바인딩(+좌표계/스케일 설정) — 공개 setter 대체용 */
    public static void bindRenderer(URDFModelOpenGLWithSTL r) {
        renderer = r;
        URDFVmcMapper.setEnableCoordTransform(true);
        URDFVmcMapper.setGlobalScale(1.0f);
        log("[TickLoop] Renderer bound");
    }

    /** 틱 리스너 등록 — 클라 초기화에서 반드시 1회 호출 */
    public static void register() {
        // 주의: @EventBusSubscriber 대신 코드로 등록 (버전 차이/어노테이션 불일치 방지)
        NeoForge.EVENT_BUS.addListener(ClientTickLoop::onClientTick);
        log("[TickLoop] Listener registered");
    }

    // ───────── 내부 상태/파라미터 ─────────

    private static final JointControlBus   BUS        = new JointControlBus(0.30f); // EMA α
    private static final URDFArmRetargeter RETARGETER = new URDFArmRetargeter();

    private static final int  DEFAULT_VMC_PORT = 39539;
    private static boolean    vmcStarted       = false;

    private static long lastLogMs     = 0;
    private static int  lastBoneCount = 0;

    // ───────── 틱 핸들러 (NeoForge) ─────────

    /** NeoForge: ClientTickEvent.Post */
    private static void onClientTick(final ClientTickEvent.Post event) {
        if (renderer == null) return;

        try {
            // 1) VMC 내부 리스너 자동 시작(한 번만)
            ensureVmcStarted(resolveVmcPort());

            // 2) 본 스냅샷 폴링 (내부 리스너)
            Map<String, Object> bones = VmcListenerManager.getBones();

            // (선택) 외부 매니저(top.fifthlight...) 폴백
            if (bones.isEmpty()) bones = pollExternalVmcBones();

            // 3) 자동 소스 → BUS (RETARGET → IK 순, IK 우선순위 ↑)
            if (!bones.isEmpty()) {
                lastBoneCount = bones.size();

                Map<String, Float> rt = safeRetarget(bones);
                if (!rt.isEmpty()) BUS.push("retarget", JointControlBus.Priority.RETARGET, rt);

                Map<String, Float> ik = safeIk(bones);
                if (!ik.isEmpty()) BUS.push("ik", JointControlBus.Priority.IK, ik);
            } else {
                throttledLog("[TickLoop] VMC bones empty");
            }

            // 4) 최종 합성 & 적용(단일 출구)
            BUS.resolveAndApply(renderer);

            // 5) 기존 렌더러 주기 유지
            renderer.tickUpdate(1.0f / 20.0f);

        } catch (Throwable t) {
            throttledLog("[TickLoop] error: " + t.getClass().getSimpleName() + " - " + t.getMessage());
        }
    }

    // ───────── VMC 시작/폴링 ─────────

    private static int resolveVmcPort() {
        // 설정이 있으면 반영(없으면 기본 포트)
        try {
            Class<?> cfgHolder = Class.forName("top.fifthlight.armorstand.config.ConfigHolder");
            Object   cfg       = cfgHolder.getField("config").get(null);
            int p = (int) cfg.getClass().getField("vmcUdpPort").get(cfg);
            return (p > 0 && p <= 65535) ? p : DEFAULT_VMC_PORT;
        } catch (Throwable ignored) {
            return DEFAULT_VMC_PORT;
        }
    }

    private static void ensureVmcStarted(int port) {
        if (vmcStarted) return;
        try {
            VmcListenerManager.start(port);   // 내부 UDP/OSC 리스너
            vmcStarted = true;
            log("[TickLoop] Internal VMC listener started on port " + port);
        } catch (Throwable t) {
            throttledLog("[TickLoop] VMC start failed: " + t);
            vmcStarted = true; // 매 틱 재시도 방지 (원하면 재시도 로직 추가)
        }
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> pollExternalVmcBones() {
        try {
            Class<?> mgr = Class.forName("top.fifthlight.armorstand.vmc.VmcMarionetteManager");
            Object state = mgr.getMethod("getState").invoke(null);
            if (state == null) return Collections.emptyMap();

            Object mapObj = tryFieldOrGetter(state, "boneTransforms");
            if (mapObj == null) mapObj = tryFieldOrGetter(state, "bones");
            if (mapObj == null) return Collections.emptyMap();

            Map<String, Object> out = new HashMap<>();
            Map<Object, Object> m = (Map<Object, Object>) mapObj;
            for (Map.Entry<Object, Object> e : m.entrySet()) {
                Object k = e.getKey();
                String name = (k instanceof Enum<?> en) ? en.name() : String.valueOf(k);
                out.put(name, e.getValue());
            }
            return out;
        } catch (Throwable ignored) {
            return Collections.emptyMap();
        }
    }

    private static Object tryFieldOrGetter(Object obj, String name) {
        try {
            var f = obj.getClass().getField(name);
            f.setAccessible(true);
            Object v = f.get(obj);
            if (v != null) return v;
        } catch (Throwable ignored) {}
        try {
            var g = obj.getClass().getMethod("get" + cap(name));
            Object v = g.invoke(obj);
            if (v != null) return v;
        } catch (Throwable ignored) {}
        try {
            var g = obj.getClass().getMethod(name);
            Object v = g.invoke(obj);
            if (v != null) return v;
        } catch (Throwable ignored) {}
        return null;
    }

    private static String cap(String s) {
        if (s == null || s.isEmpty()) return s;
        return Character.toUpperCase(s.charAt(0)) + s.substring(1);
    }

    // ───────── 리타게팅/IK 안전 호출 ─────────

    private static Map<String, Float> safeRetarget(Map<String, Object> bones) {
        try {
            return RETARGETER.commands(bones);
        } catch (Throwable t) {
            throttledLog("[RETARGET] failed: " + t);
            return Collections.emptyMap();
        }
    }

    private static Map<String, Float> safeIk(Map<String, Object> bones) {
        try {
            return VmcIk.commandsFromBones(bones);
        } catch (Throwable t) {
            throttledLog("[IK] failed: " + t);
            return Collections.emptyMap();
        }
    }

    // ───────── 로깅 ─────────

    private static void throttledLog(String msg) {
        long now = System.currentTimeMillis();
        if (now - lastLogMs > 1000) {
            System.out.println(msg + " (bones=" + lastBoneCount + ")");
            lastLogMs = now;
        }
    }

    private static void log(String msg) {
        System.out.println(msg);
    }
}
