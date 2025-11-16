package com.kAIS.KAIMyEntity.urdf.control;

import org.joml.Matrix3f;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * VMC(=Unity/VRM) Transform -> URDF/ROS 포즈 변환 유틸.
 * - 좌표계 변환(Y-up, left-handed -> Z-up, right-handed)
 * - 스케일/오프셋 보정
 * - Reflection으로 position/rotation 추출
 *
 * 주의: 좌표계 변환은 이 클래스 한 곳에서만 수행(중복 변환 금지)
 */
public final class URDFVmcMapper {
    private URDFVmcMapper() {}

    /** IK/리타게팅 입력용 포즈 컨테이너 */
    public static final class WristTarget {
        public final Vector3f position;
        public final Quaternionf rotation;
        public WristTarget(Vector3f p, Quaternionf r) { this.position = p; this.rotation = r; }
        @Override public String toString() { return "WristTarget{pos="+position+", rot="+rotation+"}"; }
    }

    // ===== 설정값 =====
    private static volatile float       globalScale          = 1.0f;
    private static volatile boolean     enableCoordTransform = true;
    private static volatile Vector3f    positionOffset       = new Vector3f();
    private static volatile Quaternionf rotationOffset       = new Quaternionf(); // identity

    public static void setGlobalScale(float s){ globalScale = s; }
    public static void setEnableCoordTransform(boolean e){ enableCoordTransform = e; }
    public static void setPositionOffset(Vector3f o){ positionOffset = (o==null)?new Vector3f():new Vector3f(o); }
    public static void setRotationOffset(Quaternionf o){ rotationOffset = (o==null)?new Quaternionf():new Quaternionf(o).normalize(); }

    // ===== 좌표계 변환(Unity->ROS) : x_ros=z_u, y_ros=-x_u, z_ros=y_u =====
    private static final Matrix3f C_UNITY_TO_ROS = new Matrix3f(
            0f, 0f, 1f,
            -1f, 0f, 0f,
            0f, 1f, 0f
    );
    private static final Matrix3f C_T = new Matrix3f(C_UNITY_TO_ROS).transpose();

    private static Vector3f unityToRos(Vector3f v){ return new Vector3f(v.z, -v.x, v.y); }

    private static Quaternionf unityToRos(Quaternionf q){
        // JOML 버전 의존 제거: 행렬로 변환 후 C*R*C^T 적용 → 수식으로 다시 Quaternionf 생성
        Matrix3f rUnity = new Matrix3f();
        q.get(rUnity); // 안전: 대부분 JOML 버전에서 제공
        Matrix3f rRos   = new Matrix3f(C_UNITY_TO_ROS).mul(rUnity).mul(C_T);
        return quatFromMatrix(rRos); // JOML 버전 무관
    }

    /** Matrix3f -> Quaternionf (정규화) : 모든 JOML 버전 호환 */
    private static Quaternionf quatFromMatrix(Matrix3f m) {
        float m00 = m.m00, m01 = m.m01, m02 = m.m02;
        float m10 = m.m10, m11 = m.m11, m12 = m.m12;
        float m20 = m.m20, m21 = m.m21, m22 = m.m22;

        float t = m00 + m11 + m22;
        float x, y, z, w;
        if (t > 0f) {
            float S = (float) Math.sqrt(t + 1.0f) * 2f; // S=4*w
            w = 0.25f * S;
            x = (m21 - m12) / S;
            y = (m02 - m20) / S;
            z = (m10 - m01) / S;
        } else if (m00 > m11 && m00 > m22) {
            float S = (float) Math.sqrt(1.0f + m00 - m11 - m22) * 2f; // S=4*x
            w = (m21 - m12) / S;
            x = 0.25f * S;
            y = (m01 + m10) / S;
            z = (m02 + m20) / S;
        } else if (m11 > m22) {
            float S = (float) Math.sqrt(1.0f + m11 - m00 - m22) * 2f; // S=4*y
            w = (m02 - m20) / S;
            x = (m01 + m10) / S;
            y = 0.25f * S;
            z = (m12 + m21) / S;
        } else {
            float S = (float) Math.sqrt(1.0f + m22 - m00 - m11) * 2f; // S=4*z
            w = (m10 - m01) / S;
            x = (m02 + m20) / S;
            y = (m12 + m21) / S;
            z = 0.25f * S;
        }
        return new Quaternionf(x, y, z, w).normalize();
    }

    // ===== 공개 API =====
    public static WristTarget vmcToWristTarget(Object vmcTransform){
        return vmcToWristTarget(vmcTransform, globalScale, enableCoordTransform, positionOffset, rotationOffset);
    }

    public static WristTarget vmcToWristTarget(Object vmcTransform,
                                               float scale,
                                               boolean coordTransform,
                                               Vector3f posOffset,
                                               Quaternionf rotOffset){
        if (vmcTransform == null) return null;
        try{
            Object posObj = reflectGetFieldOrGetter(vmcTransform, "position");
            Object rotObj = reflectGetFieldOrGetter(vmcTransform, "rotation");

            Vector3f    pos = reflectVector3f(posObj);
            Quaternionf rot = reflectQuaternionf(rotObj);

            if (coordTransform){
                pos = unityToRos(pos);
                rot = unityToRos(rot);
            }
            if (scale != 1.0f) pos.mul(scale);

            if (rotOffset != null) rot = new Quaternionf(rotOffset).mul(rot).normalize();
            if (posOffset != null) pos.add(posOffset);

            return new WristTarget(pos, rot);
        }catch(Throwable t){
            return null;
        }
    }

    // ===== Reflection helpers =====
    private static Object reflectGetFieldOrGetter(Object obj, String name) throws Exception {
        Class<?> c = obj.getClass();
        try{ Field f = c.getField(name); f.setAccessible(true); Object v = f.get(obj); if(v!=null) return v; }catch(NoSuchFieldException ignored){}
        try{ Method m = c.getMethod("get"+cap(name)); Object v = m.invoke(obj); if(v!=null) return v; }catch(NoSuchMethodException ignored){}
        try{ Method m = c.getMethod(name); Object v = m.invoke(obj); if(v!=null) return v; }catch(NoSuchMethodException ignored){}
        throw new NoSuchFieldException("No field/getter '"+name+"' in "+c.getName());
    }

    private static Vector3f reflectVector3f(Object vecObj) throws Exception{
        if (vecObj == null) return new Vector3f();
        Float x = reflectFloatComponent(vecObj, "x");
        Float y = reflectFloatComponent(vecObj, "y");
        Float z = reflectFloatComponent(vecObj, "z");
        return new Vector3f(x!=null?x:0f, y!=null?y:0f, z!=null?z:0f);
    }

    private static Quaternionf reflectQuaternionf(Object qObj) throws Exception{
        if (qObj == null) return new Quaternionf();
        Float x = reflectFloatComponent(qObj, "x");
        Float y = reflectFloatComponent(qObj, "y");
        Float z = reflectFloatComponent(qObj, "z");
        Float w = reflectFloatComponent(qObj, "w");
        Quaternionf q = new Quaternionf(x!=null?x:0f, y!=null?y:0f, z!=null?z:0f, w!=null?w:1f);
        return q.normalize();
    }

    private static Float reflectFloatComponent(Object obj, String name) throws Exception{
        Class<?> c = obj.getClass();
        try{ Field f = c.getField(name); f.setAccessible(true); Object v = f.get(obj); return castFloat(v); }catch(NoSuchFieldException ignored){}
        try{ Method m = c.getMethod(name); Object v = m.invoke(obj); return castFloat(v); }catch(NoSuchMethodException ignored){}
        try{ Method m = c.getMethod("get"+cap(name)); Object v = m.invoke(obj); return castFloat(v); }catch(NoSuchMethodException ignored){}
        return null;
    }

    private static Float castFloat(Object v){
        if (v == null) return null;
        if (v instanceof Float) return (Float)v;
        if (v instanceof Double) return ((Double)v).floatValue();
        if (v instanceof Number) return ((Number)v).floatValue();
        return null;
    }

    private static String cap(String s){ return (s==null || s.isEmpty())? s : Character.toUpperCase(s.charAt(0)) + s.substring(1); }

    // (선택) 본->링크 매핑 예시
    public static final Map<String, String> BONE_TO_LINK;
    static{
        Map<String,String> m = new HashMap<>();
        m.put("hips","base_link"); m.put("chest","upper_torso"); m.put("neck","neck_link"); m.put("head","head_link");
        m.put("leftUpperArm","left_upper_arm_link");   m.put("leftLowerArm","left_forearm_link");   m.put("leftHand","left_hand_link");
        m.put("rightUpperArm","right_upper_arm_link"); m.put("rightLowerArm","right_forearm_link"); m.put("rightHand","right_hand_link");
        BONE_TO_LINK = Collections.unmodifiableMap(m);
    }
}
