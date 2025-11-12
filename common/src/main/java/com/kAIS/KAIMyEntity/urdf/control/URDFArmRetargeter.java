package com.kAIS.KAIMyEntity.urdf.control;

import org.joml.Matrix3f;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

public class URDFArmRetargeter {
    // 프로젝트에 맞게 조인트 이름만 바꿔도 됨
    public static final String L_YAW="left_shoulder_yaw_joint",  L_PITCH="left_shoulder_pitch_joint",  L_ELBOW="left_elbow_joint";
    public static final String R_YAW="right_shoulder_yaw_joint", R_PITCH="right_shoulder_pitch_joint", R_ELBOW="right_elbow_joint";

    private static final float ELBOW_SIGN_LEFT  = +1f;
    private static final float ELBOW_SIGN_RIGHT = +1f;

    /** 소비자 콜백 형태(바로 버스에 푸시할 때 편리) */
    public void applyBoth(Map<String, Object> bones, BiConsumer<String, Float> out) {
        Map<String, Float> m = commands(bones);
        m.forEach(out);
    }

    /** 명령 맵으로 반환(버스 push에 사용) */
    public Map<String, Float> commands(Map<String, Object> bones) {
        Map<String, Float> out = new HashMap<>();
        solveOne(bones, true,  out);
        solveOne(bones, false, out);
        return out;
    }

    private void solveOne(Map<String, Object> b, boolean left, Map<String, Float> out) {
        URDFVmcMapper.WristTarget chest = read(b.get("chest")); // 부모 회전 기준
        URDFVmcMapper.WristTarget upper = read(b.get(left ? "leftUpperArm"  : "rightUpperArm"));
        URDFVmcMapper.WristTarget lower = read(b.get(left ? "leftLowerArm"  : "rightLowerArm"));
        URDFVmcMapper.WristTarget hand  = read(b.get(left ? "leftHand"      : "rightHand"));
        if (upper == null || lower == null || hand == null) return;
        if (chest == null) chest = new URDFVmcMapper.WristTarget(new Vector3f(), new Quaternionf());

        Vector3f shoulder = new Vector3f(upper.position);
        Vector3f elbow    = new Vector3f(lower.position);
        Vector3f wrist    = new Vector3f(hand.position);

        Vector3f uDir = elbow.sub(shoulder, new Vector3f()).normalize();
        Vector3f lDir = wrist.sub(new Vector3f(elbow), new Vector3f()).normalize();

        Vector3f uLocal = toLocalDir(uDir, chest.rotation);
        float yaw   = (float) Math.atan2(uLocal.y, uLocal.x);                                // Z축
        float pitch = (float) Math.atan2(uLocal.z, Math.hypot(uLocal.x, uLocal.y));          // Y축

        float elbowAng = (float) Math.acos(clamp(uDir.dot(lDir), -1f, 1f));
        elbowAng *= left ? ELBOW_SIGN_LEFT : ELBOW_SIGN_RIGHT;

        if (left) {
            out.put(L_YAW, yaw); out.put(L_PITCH, pitch); out.put(L_ELBOW, elbowAng);
        } else {
            out.put(R_YAW, yaw); out.put(R_PITCH, pitch); out.put(R_ELBOW, elbowAng);
        }
    }

    private static Vector3f toLocalDir(Vector3f worldDir, Quaternionf parentWorldRot) {
        Matrix3f R = new Matrix3f().set(parentWorldRot); // world = R * local
        R.transpose();                                   // local = R^T * world
        Vector3f v = new Vector3f(worldDir);
        R.transform(v);
        return v.normalize();
    }

    private static float clamp(float v, float lo, float hi) { return v < lo ? lo : Math.min(hi, v); }
    private static URDFVmcMapper.WristTarget read(Object t) { return t == null ? null : URDFVmcMapper.vmcToWristTarget(t); }
}
