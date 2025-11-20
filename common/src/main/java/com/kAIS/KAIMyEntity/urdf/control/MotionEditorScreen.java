package com.kAIS.KAIMyEntity.urdf.control;

import com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL;
import net.minecraft.client.Minecraft;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.util.Map;

import static java.lang.Math.abs;

/**
 * 2025.11.20 VMC Direct Control (Fixed with Atomic Snapshot + Bone Mapping Fix)
 * - Atomic Snapshot 적용으로 Tearing 및 떨림/멈춤 방지
 * - World Coordinate 기반 역학 계산
 * - UpperChest 매핑 문제 해결 (VSeeFace/VMagicMirror 완벽 호환)
 */
public final class MotionEditorScreen {
    private MotionEditorScreen() {}

    static {
        VMCListenerController.VmcListener listener = VMCListenerController.VmcListener.getInstance();
        listener.setBoneNameNormalizer(original -> {
            if (original == null) return null;
            String lower = original.toLowerCase().trim();

            return switch (lower) {
                // 팔
                case "leftupperarm", "leftarm", "left_arm", "upperarm_left", "arm.l", "leftshoulder", "larm" -> "LeftUpperArm";
                case "leftlowerarm", "leftforearm", "lowerarm_left", "forearm.l", "leftelbow" -> "LeftLowerArm";
                case "lefthand", "hand.l", "hand_left", "left_wrist", "left_hand" -> "LeftHand";
                case "rightupperarm", "rightarm", "right_arm", "upperarm_right", "arm.r", "rightshoulder", "rarm" -> "RightUpperArm";
                case "rightlowerarm", "rightforearm", "lowerarm_right", "forearm.r", "rightelbow" -> "RightLowerArm";
                case "righthand", "hand.r", "hand_right", "right_wrist", "right_hand" -> "RightHand";

                // ★★★ Chest 매핑 확장 (VSeeFace/VMagicMirror UpperChest 대응) ★★★
                case "chest", "upperchest", "spine", "spine1", "spine2", "spine3", "torso", "upper_chest", "chest2" -> "Chest";

                default -> original;
            };
        });
    }

    public static void open(URDFModelOpenGLWithSTL renderer) {
        open(renderer, 39539);
    }

    public static void open(URDFModelOpenGLWithSTL renderer, int vmcPort) {
        VMCListenerController.VmcListener listener = VMCListenerController.VmcListener.getInstance();
        listener.start("0.0.0.0", vmcPort);
        Minecraft.getInstance().setScreen(new VMCListenerController(Minecraft.getInstance().screen, renderer));
    }

    public static void tick(URDFModelOpenGLWithSTL renderer) {
        VmcDrive.tick(renderer);
    }
}

/* ======================== VmcDrive (Atomic Snapshot 적용) ======================== */
final class VmcDrive {

    static void tick(URDFModelOpenGLWithSTL renderer) {
        var listener = VMCListenerController.VmcListener.getInstance();
        
        // [핵심] getBones() 대신 getSnapshot() 사용
        // 이 맵은 이 메서드가 실행되는 동안 절대 변하지 않음 (Atomic Reference)
        Map<String, VMCListenerController.VmcListener.Transform> bones = listener.getSnapshot();

        if (bones.isEmpty()) return;

        // ★ 디버깅용 로그 (Chest 매핑 확인) ★
        System.out.println("[VMC] tick - Bones: " + bones.size() + 
            " | Chest: " + bones.containsKey("Chest") +
            " | LeftUA: " + bones.containsKey("LeftUpperArm") +
            " | RightUA: " + bones.containsKey("RightUpperArm"));

        // 부모(Chest) 찾기
        VMCListenerController.VmcListener.Transform chest = bones.get("Chest");
        if (chest == null) chest = bones.get("Spine");
        if (chest == null) chest = bones.get("Hips");

        if (chest == null) {
            System.out.println("[VMC] ERROR: Chest bone not found! Available: " + bones.keySet());
            return;
        }

        processArmQuaternion(renderer, bones, chest, true);  // 왼팔
        processArmQuaternion(renderer, bones, chest, false); // 오른팔
    }

    private static void processArmQuaternion(URDFModelOpenGLWithSTL renderer,
                                             Map<String, VMCListenerController.VmcListener.Transform> bones,
                                             VMCListenerController.VmcListener.Transform parentBone,
                                             boolean isLeft) {
        String upperName = isLeft ? "LeftUpperArm" : "RightUpperArm";
        String lowerName = isLeft ? "LeftLowerArm" : "RightLowerArm";

        var upper = bones.get(upperName);
        var lower = bones.get(lowerName);

        if (upper == null) return;

        // [계산 로직] Atomic Snapshot 덕분에 parentBone과 upper는 같은 시간대의 데이터임이 보장됨.
        // 따라서 World -> Local 변환(Q_rel = Q_parent^-1 * Q_child)이 정확하게 수행됨.

        // === 1. 어깨 관절 (Shoulder) 계산 ===
        Quaternionf parentRot = new Quaternionf(parentBone.rotation);
        Quaternionf childRot  = new Quaternionf(upper.rotation);
        
        Quaternionf localShoulder = new Quaternionf(parentRot).conjugate().mul(childRot);

        Vector3f shoulderEuler = new Vector3f();
        localShoulder.getEulerAnglesXYZ(shoulderEuler);

        // === 2. 팔꿈치 관절 (Elbow) 계산 ===
        Vector3f elbowEuler = new Vector3f();
        if (lower != null) {
            Quaternionf upperRot = new Quaternionf(upper.rotation);
            Quaternionf lowerRot = new Quaternionf(lower.rotation);
            
            Quaternionf localElbow = new Quaternionf(upperRot).conjugate().mul(lowerRot);
            localElbow.getEulerAnglesXYZ(elbowEuler);
        }

        // === 3. URDF 적용 ===
        String pitchJoint = isLeft ? "l_sho_pitch" : "r_sho_pitch";
        String rollJoint  = isLeft ? "l_sho_roll"  : "r_sho_roll";
        String elbowJoint = isLeft ? "l_el"        : "r_el";

        renderer.setJointPreview(pitchJoint, shoulderEuler.x);
        renderer.setJointTarget(pitchJoint, shoulderEuler.x);

        renderer.setJointPreview(rollJoint, shoulderEuler.z);
        renderer.setJointTarget(rollJoint, shoulderEuler.z);

        float elbowAngle = abs(elbowEuler.z);
        if (isLeft) elbowAngle = -elbowAngle;

        renderer.setJointPreview(elbowJoint, elbowAngle);
        renderer.setJointTarget(elbowJoint, elbowAngle);
    }
}
