package com.kAIS.KAIMyEntity.urdf.control;

import org.joml.Vector3f;

import java.util.HashMap;
import java.util.Map;

/**
 * VMC 본(ROS 프레임으로 변환된 값)에서 간단한 3DoF IK 각도를 생성.
 * - 어깨 원점: upperArm 본 위치를 사용(간편). 필요시 로봇 어깨 링크 위치로 대체/보정.
 * - 링크 길이: upper→lower, lower→hand 거리로 추정(초기 캘리브레이션 대체).
 *   로봇 링크 길이로 고정하고 싶으면 상수로 덮어쓰세요.
 */
public final class VmcIk {
    private VmcIk(){}

    // 워밍스타트용 내부 상태
    private static float[] qL = new float[]{0f,0f,0f};
    private static float[] qR = new float[]{0f,0f,0f};

    public static Map<String, Float> commandsFromBones(Map<String, Object> bones){
        Map<String, Float> out = new HashMap<>();
        // Left
        solveOne(bones, true,  out);
        // Right
        solveOne(bones, false, out);
        return out;
    }

    private static void solveOne(Map<String,Object> b, boolean left, Map<String,Float> out){
        URDFVmcMapper.WristTarget up = read(b.get(left?"leftUpperArm":"rightUpperArm"));
        URDFVmcMapper.WristTarget lo = read(b.get(left?"leftLowerArm":"rightLowerArm"));
        URDFVmcMapper.WristTarget hd = read(b.get(left?"leftHand":"rightHand"));
        if (up==null || lo==null || hd==null) return;

        Vector3f shoulder = new Vector3f(up.position);
        Vector3f elbow    = new Vector3f(lo.position);
        Vector3f wrist    = new Vector3f(hd.position);

        float L1 = new Vector3f(elbow).sub(shoulder).length();
        float L2 = new Vector3f(wrist).sub(elbow).length();
        if (L1 < 1e-4f) L1 = 0.25f;
        if (L2 < 1e-4f) L2 = 0.25f;

        float[] q0 = left ? qL : qR;
        ArmIK3DoF.Result q = ArmIK3DoF.solve(shoulder, wrist, q0, L1, L2, 10, 1e-3f);
        if (left){
            qL = new float[]{q.yaw, q.pitch, q.elbow};
            out.put(URDFArmRetargeter.L_YAW,   q.yaw);
            out.put(URDFArmRetargeter.L_PITCH, q.pitch);
            out.put(URDFArmRetargeter.L_ELBOW, q.elbow);
        }else{
            qR = new float[]{q.yaw, q.pitch, q.elbow};
            out.put(URDFArmRetargeter.R_YAW,   q.yaw);
            out.put(URDFArmRetargeter.R_PITCH, q.pitch);
            out.put(URDFArmRetargeter.R_ELBOW, q.elbow);
        }
    }

    private static URDFVmcMapper.WristTarget read(Object t){
        return t==null?null:URDFVmcMapper.vmcToWristTarget(t);
    }
}
