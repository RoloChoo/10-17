package com.kAIS.KAIMyEntity.urdf.control;

import com.kAIS.KAIMyEntity.urdf.URDFJoint;
import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;

import java.util.*;

public class JointControlBus {

    public enum Priority { MANUAL(100), IK(50), RETARGET(10);
        final int v; Priority(int v){ this.v=v; } int value(){ return v; } }

    private final Map<String, Float> manual = new HashMap<>();
    private final List<SourceBuf> frameSources = new ArrayList<>();
    private final Map<String, Float> ema = new HashMap<>();
    private final float alpha; // EMA 계수(0~1)

    public JointControlBus(float smoothingAlpha) { this.alpha = smoothingAlpha; }

    /** 자동 소스(리타게터/IK)가 프레임 내에서 호출 */
    public void push(String name, Priority prio, Map<String, Float> cmds) {
        if (cmds == null || cmds.isEmpty()) return;
        frameSources.add(new SourceBuf(name, prio.value(), new HashMap<>(cmds)));
    }

    /** 수동 오버라이드(슬라이더). null이면 해제 */
    public void setManual(String joint, Float rad) {
        if (rad == null) manual.remove(joint); else manual.put(joint, rad);
    }
    public void clearManualAll() { manual.clear(); }

    /** 프레임 마지막에 딱 한 번 호출: 합성→스무딩→적용 */
    public void resolveAndApply(URDFModelOpenGLWithSTL renderer) {
        frameSources.sort((a, b) -> Integer.compare(b.prio, a.prio)); // 우선순위 내림차순
        List<URDFJoint> joints = renderer.getRobotModel().joints;

        for (URDFJoint j : joints) {
            String name = j.name;

            Float target = manual.get(name); // 수동 최우선
            if (target == null) {
                for (SourceBuf s : frameSources) { // 가장 높은 우선순위 소스에서 첫 매칭
                    Float v = s.cmds.get(name);
                    if (v != null) { target = v; break; }
                }
            }
            if (target == null) continue;

            // 관절 한계 클램프
            float lo = (j.limit != null && j.limit.hasLimits()) ? j.limit.lower : (float)Math.toRadians(-180);
            float hi = (j.limit != null && j.limit.hasLimits()) ? j.limit.upper : (float)Math.toRadians( 180);
            if (hi <= lo) { lo = (float)Math.toRadians(-180); hi = (float)Math.toRadians(180); }
            target = Math.max(lo, Math.min(hi, target));

            // EMA 스무딩
            float y = target;
            Float prev = ema.get(name);
            if (prev != null) y = prev + (target - prev) * alpha;
            ema.put(name, y);

            renderer.setJointTarget(name, y);
            renderer.setJointPreview(name, y); // 프리뷰도 동기화(원하면 주석 처리)
        }
        frameSources.clear(); // 프레임 버퍼 비움
    }

    private static final class SourceBuf {
        final String name; final int prio; final Map<String, Float> cmds;
        SourceBuf(String n, int p, Map<String, Float> c){ name=n; prio=p; cmds=c; }
    }
}
