package com.kAIS.KAIMyEntity.urdf.control;

import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.HashMap;
import java.util.Map;

public final class VmcDrive {
    private VmcDrive() {}

    // 전역 버스/리타게터 (원하면 외부에서 생성해 넘겨도 됨)
    public static final JointControlBus BUS = new JointControlBus(0.30f);
    public static final URDFArmRetargeter RETARGETER = new URDFArmRetargeter();

    /** 틱마다 호출: VMC→리타게팅→버스→적용 */
    public static void tick(URDFModelOpenGLWithSTL renderer) {
        // 1) VMC 본 수집
        Object vmcState = reflectGetVmcState();
        if (vmcState != null) {
            Map<String, Object> bones = reflectCollectBoneMap(vmcState);

            // 2) 리타게팅 명령 산출
            Map<String, Float> cmds = RETARGETER.commands(bones);

            // 3) 버스에 자동 소스 푸시(RETARGET)
            if (!cmds.isEmpty()) {
                BUS.push("retarget", JointControlBus.Priority.RETARGET, cmds);
            }
        }

        // 4) (선택) IK가 있다면 BUS.push("ik", Priority.IK, ikCmds) 추가

        // 5) 최종 합성 & 적용
        BUS.resolveAndApply(renderer);
    }

    // ========= VMC Reflection helpers =========
    private static Object reflectGetVmcState() {
        try {
            Class<?> mgr = Class.forName("top.fifthlight.armorstand.vmc.VmcMarionetteManager");
            Method getState = mgr.getMethod("getState");
            return getState.invoke(null);
        } catch (Throwable ignored) { return null; }
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> reflectCollectBoneMap(Object vmcState) {
        Map<String, Object> out = new HashMap<>();
        if (vmcState == null) return out;
        Object mapObj = null;

        // boneTransforms / getBoneTransforms / bones / getBones 순으로 시도
        mapObj = tryFieldOrGetter(vmcState, "boneTransforms");
        if (mapObj == null) mapObj = tryFieldOrGetter(vmcState, "bones");
        if (mapObj == null) return out;

        try {
            Map<Object, Object> m = (Map<Object, Object>) mapObj;
            for (Map.Entry<Object, Object> e : m.entrySet()) {
                Object k = e.getKey();
                String name = (k instanceof Enum<?> en) ? en.name() : String.valueOf(k);
                out.put(name, e.getValue()); // Transform 그대로 저장 (URDFVmcMapper에서 파싱)
            }
        } catch (Throwable ignored) {}
        return out;
    }

    private static Object tryFieldOrGetter(Object obj, String name) {
        try {
            Field f = obj.getClass().getField(name);
            f.setAccessible(true);
            Object v = f.get(obj);
            if (v != null) return v;
        } catch (Throwable ignored) {}
        try {
            Method g = obj.getClass().getMethod("get" + cap(name));
            Object v = g.invoke(obj);
            if (v != null) return v;
        } catch (Throwable ignored) {}
        try {
            Method g = obj.getClass().getMethod(name);
            Object v = g.invoke(obj);
            if (v != null) return v;
        } catch (Throwable ignored) {}
        return null;
    }

    private static String cap(String s) {
        if (s == null || s.isEmpty()) return s;
        return Character.toUpperCase(s.charAt(0)) + s.substring(1);
    }
}
