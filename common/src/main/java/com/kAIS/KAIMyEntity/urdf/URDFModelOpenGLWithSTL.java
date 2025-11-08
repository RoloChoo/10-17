package com.kAIS.KAIMyEntity.urdf;

import com.kAIS.KAIMyEntity.renderer.IMMDModel;
import com.kAIS.KAIMyEntity.urdf.control.URDFMotionEditor;
import com.kAIS.KAIMyEntity.urdf.control.URDFMotionPlayer;
import com.kAIS.KAIMyEntity.urdf.control.URDFSimpleController;
import com.kAIS.KAIMyEntity.vrm.VrmLoader;
import com.kAIS.KAIMyEntity.vrm.VrmLoader.VrmSkeleton;
import com.mojang.blaze3d.systems.RenderSystem;
import com.mojang.blaze3d.vertex.PoseStack;
import com.mojang.blaze3d.vertex.VertexConsumer;
import net.minecraft.client.Minecraft;
import net.minecraft.client.renderer.MultiBufferSource;
import net.minecraft.client.renderer.RenderType;
import net.minecraft.world.entity.Entity;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.joml.Matrix4f;         // ※ 프로젝트가 바닐라 Matrix4f를 요구하면 com.mojang.math.Matrix4f로 교체
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.io.File;
import java.util.*;

/**
 * URDF 모델 렌더링 (STL 메시 포함) + VRM 스켈레톤 오버레이 프리뷰
 *
 * - 루트에서만 업라이트 보정(ROS/STL 좌표 → Minecraft) 1회 적용
 * - setJointPreview(...) 즉시 반영 + tickUpdate(...) 컨트롤러 추종
 * - setPreviewSkeleton(...) 주입 시 VRM 스켈레톤 라인 오버레이
 * - VRM도 URDF와 동일한 루트 변환을 적용하여 화면 중앙에서 정확히 겹쳐 보이도록 구성
 */
public class URDFModelOpenGLWithSTL implements IMMDModel {
    private static final Logger logger = LogManager.getLogger();
    private static int renderCount = 0;

    private URDFRobotModel robotModel;
    private String modelDir;

    // 메시 캐시: link.name -> mesh
    private final Map<String, STLLoader.STLMesh> meshCache = new HashMap<>();

    // 전역 스케일
    private static final float GLOBAL_SCALE = 5.0f;

    // 법선 반전(필요 시)
    private static final boolean FLIP_NORMALS = true;

    // ------------ 업라이트 보정 설정 ------------
    /** URDF/STL 소스 좌표계 가정 */
    private static final Vector3f SRC_UP  = new Vector3f(0, 0, 1); // 보통 Z-up
    private static final Vector3f SRC_FWD = new Vector3f(1, 0, 0); // 보통 +X forward

    /** Minecraft 타깃 좌표계 (Up=+Y, Forward=±Z) */
    private static final boolean FORWARD_NEG_Z = true;             // 정면을 -Z(기본) / +Z
    private static final Vector3f DST_UP  = new Vector3f(0, 1, 0);
    private static final Vector3f DST_FWD = FORWARD_NEG_Z ? new Vector3f(0, 0, -1) : new Vector3f(0, 0,  1);

    /** 루트에서 1회만 적용하는 업라이트 보정 */
    private static final Quaternionf Q_ROS2MC = makeUprightQuat(SRC_UP, SRC_FWD, DST_UP, DST_FWD);

    // ------------ 모션/컨트롤 ------------
    private final URDFSimpleController ctrl;
    private final URDFMotionEditor motionEditor;
    private final URDFMotionPlayer motionPlayer = new URDFMotionPlayer();

    // ------------ VRM 스켈레톤 오버레이 ------------
    private VrmSkeleton vrmSkel;                        // 원본 스켈레톤(정적)
    private final Map<String, Pose> vrmLive = new HashMap<>(); // 실시간 포즈(VMC 적용)

    // 겹치기 보정 파라미터
    private float vrmViewScale = 1.0f;                  // 최종 미세 스케일 (GLOBAL_SCALE 적용 후 곱해짐)
    private String vrmRootBoneName = "hips";            // 중앙 정렬 기준 본
    private final Vector3f vrmRootCenter = new Vector3f(); // 루트 위치를 원점으로 이동시키기 위한 오프셋
    private boolean vrmCenterOnRoot = true;             // true면 루트 위치를 (0,0,0)에 정렬

    private static final class Pose {
        final Vector3f p = new Vector3f();
        final Quaternionf q = new Quaternionf();
    }

    public URDFModelOpenGLWithSTL(URDFRobotModel robotModel, String modelDir) {
        this.robotModel = robotModel;
        this.modelDir = modelDir;
        logger.info("=== URDF renderer Created ===");
        loadAllMeshes();

        // 컨트롤/모션 초기화
        this.ctrl = new URDFSimpleController(robotModel.joints);
        this.motionEditor = new URDFMotionEditor(null, this);

        // ✅ 모델 폴더에서 VRM/GLB 자동 탐색 & 로드 (있으면 스켈레톤 오버레이 활성화)
        try { loadVrmFromModelDir(); } catch (Throwable t) {
            logger.info("[URDF] VRM auto-load skipped: {}", t.toString());
        }
    }

    private void loadAllMeshes() {
        logger.info("=== Loading STL meshes ===");
        int loadedCount = 0;
        for (URDFLink link : robotModel.links) {
            if (link.visual != null && link.visual.geometry != null) {
                URDFLink.Geometry g = link.visual.geometry;
                if (g.type == URDFLink.Geometry.GeometryType.MESH && g.meshFilename != null) {
                    File f = new File(g.meshFilename);
                    if (f.exists()) {
                        STLLoader.STLMesh mesh = STLLoader.load(g.meshFilename);
                        if (mesh != null) {
                            if (g.scale != null && (g.scale.x != 1f || g.scale.y != 1f || g.scale.z != 1f)) {
                                STLLoader.scaleMesh(mesh, g.scale);
                            }
                            meshCache.put(link.name, mesh);
                            loadedCount++;
                            logger.info("  ✓ Loaded mesh for '{}': {} tris", link.name, mesh.getTriangleCount());
                        } else {
                            logger.error("  ✗ Failed to load mesh: {}", g.meshFilename);
                        }
                    } else {
                        logger.warn("  ✗ Mesh file not found: {}", g.meshFilename);
                    }
                }
            }
        }
        logger.info("=== STL Loading Complete: {}/{} meshes ===", loadedCount, robotModel.getLinkCount());
    }

    // ===== 틱 업데이트 (20Hz 권장) =====
    public void tickUpdate(float dt) {
        if (motionPlayer.isPlaying()) {
            motionPlayer.update(dt, this::setJointTarget);
        }
        ctrl.update(dt);
        // VRM 오버레이도 매 프레임 갱신
        Update(dt);
    }

    // ===== 외부 제어용 편의 API =====
    public void setJointTarget(String name, float value) { ctrl.setTarget(name, value); }
    public void setJointTargets(Map<String, Float> values) { ctrl.setTargets(values); }

    /** 즉시 반영(프리뷰): 현재 프레임에서 바로 보이게 currentPosition을 덮어씀 */
    @Override
    public void setJointPreview(String name, float value) {
        URDFJoint j = getJointByName(name);
        if (j != null) {
            j.currentPosition = value;   // 화면 즉시 반영
        }
    }

    public URDFMotionEditor getMotionEditor() { return motionEditor; }
    public URDFMotionPlayer getMotionPlayer() { return motionPlayer; }

    // (선택) VRM 보정 파라미터 세터
    public void setVrmViewScale(float s) { this.vrmViewScale = Math.max(1e-4f, s); }
    public void setVrmRootBoneName(String name) { if (name != null) this.vrmRootBoneName = name; }
    public void setVrmCenterOnRoot(boolean enabled) { this.vrmCenterOnRoot = enabled; }

    // (선택) 본 이름 목록 제공 — UI가 필요할 때 사용
    public List<String> getVrmBones() {
        if (!vrmLive.isEmpty()) return new ArrayList<>(vrmLive.keySet());
        return Collections.emptyList();
    }
    public List<String> getVrmSkeletonBones() {
        if (vrmSkel != null && vrmSkel.bones != null && !vrmSkel.bones.isEmpty()) {
            ArrayList<String> names = new ArrayList<>();
            for (var b : vrmSkel.bones) names.add(b.name);
            return names;
        }
        return Collections.emptyList();
    }

    // =======================================================
    // ✅ VRM 로딩/관리 유틸 (추가)
    // =======================================================

    /** 현재 VRM 스켈레톤이 준비돼 있는지 */
    public boolean hasVrm() { return vrmSkel != null; }

    /** VRM/GLB 파일을 지정해서 로딩 */
    public boolean loadVrm(File vrmOrGlb) {
        if (vrmOrGlb == null || !vrmOrGlb.exists()) {
            logger.warn("[URDF] VRM file not found: {}", (vrmOrGlb == null ? "null" : vrmOrGlb.getAbsolutePath()));
            return false;
        }
        VrmSkeleton s = VrmLoader.load(vrmOrGlb);
        if (s == null) {
            logger.warn("[URDF] Failed to load VRM: {}", vrmOrGlb.getAbsolutePath());
            return false;
        }
        setPreviewSkeleton(s); // IMMDModel 구현 메서드
        logger.info("[URDF] VRM loaded: {} (profile={}, bones={})", vrmOrGlb.getName(), s.profile, s.bones.size());
        return true;
    }

    /** 모델 디렉토리에서 .vrm/.glb 자동 탐색 후 로딩 */
    public boolean loadVrmFromModelDir() {
        if (modelDir == null) return false;
        File base = new File(modelDir);
        if (!base.isDirectory()) return false;

        // 1) 우선순위 파일명
        String[] prefer = { "avatar.vrm", "humanoid.vrm", "avatar.glb", "humanoid.glb" };
        for (String n : prefer) {
            File cand = new File(base, n);
            if (cand.exists() && cand.isFile()) {
                return loadVrm(cand);
            }
        }

        // 2) 같은 폴더에서 .vrm/.glb
        File[] list = base.listFiles((dir, name) -> {
            String s = name.toLowerCase(Locale.ROOT);
            return s.endsWith(".vrm") || s.endsWith(".glb");
        });
        if (list != null && list.length > 0) {
            Arrays.sort(list);
            return loadVrm(list[0]);
        }

        // 3) meshes/ 폴더에서 탐색
        File meshes = new File(base, "meshes");
        if (meshes.isDirectory()) {
            File[] m = meshes.listFiles((dir, name) -> {
                String s = name.toLowerCase(Locale.ROOT);
                return s.endsWith(".vrm") || s.endsWith(".glb");
            });
            if (m != null && m.length > 0) {
                Arrays.sort(m);
                return loadVrm(m[0]);
            }
        }

        logger.info("[URDF] No VRM/GLB avatar found under {}", base.getAbsolutePath());
        return false;
    }

    /** 현재 VRM 해제(스틱맨/오버레이 비활성화) */
    public void clearVrm() {
        this.vrmSkel = null;
        this.vrmLive.clear();
        this.vrmRootCenter.set(0,0,0);
        logger.info("[URDF] VRM cleared");
    }

    // ===== IMMDModel 구현 =====

    /** 레거시 경로(이전 렌더러 체인) */
    @Override
    public void Render(Entity entityIn, float entityYaw, float entityPitch,
                       Vector3f entityTrans, float tickDelta, PoseStack poseStack, int packedLight) {

        renderCount++;
        if (renderCount % 120 == 1) {
            logger.info("=== URDF RENDER #{} ===", renderCount);
        }

        // 전역 렌더 상태
        RenderSystem.enableBlend();
        RenderSystem.defaultBlendFunc();
        RenderSystem.enableDepthTest();
        RenderSystem.disableCull(); // 양면

        // 버퍼
        MultiBufferSource.BufferSource bufferSource = Minecraft.getInstance().renderBuffers().bufferSource();
        VertexConsumer vc = bufferSource.getBuffer(RenderType.solid());

        if (robotModel.rootLinkName != null) {
            poseStack.pushPose();

            // 전역 스케일
            poseStack.scale(GLOBAL_SCALE, GLOBAL_SCALE, GLOBAL_SCALE);

            // ★ 루트에만 업라이트 보정 적용 — "눕는 현상"은 여기서 해결
            poseStack.mulPose(new Quaternionf(Q_ROS2MC));

            // 루트부터 렌더
            renderLinkRecursive(robotModel.rootLinkName, poseStack, vc, packedLight);

            poseStack.popPose();
        }

        // VRM 스켈레톤 오버레이
        renderVrmOverlay(poseStack, vc, packedLight, 0);

        bufferSource.endBatch(RenderType.solid());
        RenderSystem.enableCull();
    }

    /** 새 파이프라인: VertexConsumer 기반 렌더 */
    @Override
    public void renderToBuffer(Entity entityIn,
                               float entityYaw, float entityPitch, Vector3f entityTrans, float tickDelta,
                               PoseStack pose,
                               VertexConsumer consumer,
                               int packedLight,
                               int overlay) {
        // 레거시 경로로 기본 URDF 렌더
        Render(entityIn, entityYaw, entityPitch, entityTrans, tickDelta, pose, packedLight);
        // VRM 오버레이 (중복 호출 방지 위해 Render에서 그리면 여기선 생략해도 되지만 안전하게 호출)
        renderVrmOverlay(pose, consumer, packedLight, overlay);
    }

    @Override public void ChangeAnim(long anim, long layer) { }
    @Override public void ResetPhysics() { logger.info("ResetPhysics called"); }
    @Override public long GetModelLong() { return 0; }
    @Override public String GetModelDir() { return modelDir; }

    @Override
    public Object getRobotModel() { return robotModel; } // 매핑 툴에서 참조

    @Override
    public void setPreviewSkeleton(Object skeleton) {
        if (!(skeleton instanceof VrmSkeleton s)) return;
        this.vrmSkel = s;
        vrmLive.clear();
        for (var b : s.bones) {
            Pose p = new Pose();
            p.p.set(b.translation);
            p.q.set(b.rotation);
            vrmLive.put(b.name, p);
        }
        logger.info("[URDF] VRM skeleton set (profile={}, bones={})", s.profile, s.bones.size());

        // hips(루트 본) 기준 중심 잡기
        if (vrmCenterOnRoot) {
            Pose root = vrmLive.get(vrmRootBoneName);
            if (root != null) {
                vrmRootCenter.set(root.p);
            } else {
                vrmRootCenter.set(0, 0, 0);
            }
        } else {
            vrmRootCenter.set(0, 0, 0);
        }

        // 보기 편하도록 기본 스케일 보정
        if (vrmViewScale <= 0.0f) vrmViewScale = 0.01f;
    }

    @Override
    public void Update(float deltaTime) {
        if (vrmSkel == null) return;
        Object vmc = reflectGetVmcState();
        if (vmc == null) return;
        Map<String, Object> map = reflectCollectBoneMap(vmc);
        for (var e : map.entrySet()) {
            Pose lp = vrmLive.get(e.getKey());
            if (lp == null) continue;
            Object tr = e.getValue();
            Vector3f pos = readPos(tr);
            Quaternionf rot = readQuat(tr);
            if (pos != null) lp.p.set(pos);
            if (rot != null) lp.q.set(rot);
        }
    }

    @Override public void onMappingUpdated(Object mapping) { /* 필요 시 구현 */ }
    @Override public void Dispose() { /* 선택: 리소스 해제 */ }

    public static URDFModelOpenGLWithSTL Create(String urdfPath, String modelDir) {
        File urdfFile = new File(urdfPath);
        if (!urdfFile.exists()) return null;
        URDFRobotModel robot = URDFParser.parse(urdfFile);
        if (robot == null || robot.rootLinkName == null) return null;
        return new URDFModelOpenGLWithSTL(robot, modelDir);
    }

    // ===== 내부 렌더링 =====

    private void renderLinkRecursive(String linkName, PoseStack poseStack, VertexConsumer vc, int packedLight) {
        URDFLink link = robotModel.getLink(linkName);
        if (link == null) return;

        poseStack.pushPose();

        if (link.visual != null) {
            renderVisual(link, poseStack, vc, packedLight);
        }

        for (URDFJoint childJoint : robotModel.getChildJoints(linkName)) {
            poseStack.pushPose();
            applyJointTransform(childJoint, poseStack);
            renderLinkRecursive(childJoint.childLinkName, poseStack, vc, packedLight);
            poseStack.popPose();
        }

        poseStack.popPose();
    }

    private void renderVisual(URDFLink link, PoseStack poseStack, VertexConsumer vc, int packedLight) {
        if (link.visual == null || link.visual.geometry == null) return;

        poseStack.pushPose();

        if (link.visual.origin != null) {
            applyLinkOriginTransform(link.visual.origin, poseStack);
        }

        STLLoader.STLMesh mesh = meshCache.get(link.name);
        if (mesh != null) {
            renderMesh(mesh, link, poseStack, vc, packedLight);
        }
        poseStack.popPose();
    }

    private void renderMesh(STLLoader.STLMesh mesh, URDFLink link, PoseStack poseStack,
                            VertexConsumer vc, int packedLight) {
        Matrix4f matrix = poseStack.last().pose();

        int r = 220, g = 220, b = 220, a = 255;
        if (link.visual.material != null && link.visual.material.color != null) {
            URDFLink.Material.Vector4f color = link.visual.material.color;
            r = (int)(color.x * 255);
            g = (int)(color.y * 255);
            b = (int)(color.z * 255);
            a = (int)(color.w * 255);
        }

        int blockLight = (packedLight & 0xFFFF);
        int skyLight   = (packedLight >> 16) & 0xFFFF;
        blockLight = Math.max(blockLight, 0xA0);
        skyLight  = Math.max(skyLight,  0xA0);

        for (STLLoader.Triangle tri : mesh.triangles) {
            for (int i = 2; i >= 0; i--) {
                Vector3f v = tri.vertices[i];
                Vector3f n = tri.normal;

                float nx = FLIP_NORMALS ? -n.x : n.x;
                float ny = FLIP_NORMALS ? -n.y : n.y;
                float nz = FLIP_NORMALS ? -n.z : n.z;

                vc.addVertex(matrix, v.x, v.y, v.z)
                  .setColor(r, g, b, a)
                  .setUv(0.5f, 0.5f)
                  .setUv2(blockLight, skyLight)
                  .setNormal(nx, ny, nz);
            }
        }
    }

    // ===== 변환 유틸 =====
    private void applyLinkOriginTransform(URDFLink.Origin origin, PoseStack poseStack) {
        poseStack.translate(origin.xyz.x, origin.xyz.y, origin.xyz.z);
        if (origin.rpy.x != 0f || origin.rpy.y != 0f || origin.rpy.z != 0f) {
            poseStack.mulPose(origin.getQuaternion());
        }
    }

    private void applyJointOriginTransform(URDFJoint.Origin origin, PoseStack poseStack) {
        poseStack.translate(origin.xyz.x, origin.xyz.y, origin.xyz.z);
        if (origin.rpy.x != 0f || origin.rpy.y != 0f || origin.rpy.z != 0f) {
            Quaternionf qx = new Quaternionf().rotateX(origin.rpy.x);
            Quaternionf qy = new Quaternionf().rotateY(origin.rpy.y);
            Quaternionf qz = new Quaternionf().rotateZ(origin.rpy.z);
            poseStack.mulPose(qz.mul(qy).mul(qx));
        }
    }

    private void applyJointTransform(URDFJoint joint, PoseStack poseStack) {
        if (joint.origin != null) {
            applyJointOriginTransform(joint.origin, poseStack);
        }
        if (joint.isMovable()) {
            applyJointMotion(joint, poseStack);
        }
    }

    /** 축 기본값/정규화 포함(비었거나 0-벡터면 X축) */
    private void applyJointMotion(URDFJoint joint, PoseStack poseStack) {
        if (joint == null) return;

        switch (joint.type) {
            case REVOLUTE:
            case CONTINUOUS: {
                Vector3f axis;
                if (joint.axis == null || joint.axis.xyz == null ||
                    joint.axis.xyz.lengthSquared() < 1e-12f) {
                    axis = new Vector3f(1, 0, 0); // 기본축 X
                } else {
                    axis = new Vector3f(joint.axis.xyz);
                    if (axis.lengthSquared() < 1e-12f) axis.set(1, 0, 0);
                    else axis.normalize();
                }
                Quaternionf quat = new Quaternionf().rotateAxis(joint.currentPosition, axis.x, axis.y, axis.z);
                poseStack.mulPose(quat);
                break;
            }
            case PRISMATIC: {
                Vector3f axis;
                if (joint.axis == null || joint.axis.xyz == null ||
                    joint.axis.xyz.lengthSquared() < 1e-12f) {
                    axis = new Vector3f(1, 0, 0); // 기본축 X
                } else {
                    axis = new Vector3f(joint.axis.xyz);
                    if (axis.lengthSquared() < 1e-12f) axis.set(1, 0, 0);
                    else axis.normalize();
                }
                Vector3f t = axis.mul(joint.currentPosition);
                poseStack.translate(t.x, t.y, t.z);
                break;
            }
            default:
                break;
        }
    }

    // ===== VRM 오버레이 렌더 =====

    private void renderVrmOverlay(PoseStack pose, VertexConsumer vc, int packedLight, int overlay) {
        if (vrmSkel == null) return;

        pose.pushPose();

        // ✅ URDF 루트와 동일한 루트 변환 적용: (1) 스케일 → (2) 업라이트 보정
        pose.scale(GLOBAL_SCALE, GLOBAL_SCALE, GLOBAL_SCALE);
        pose.mulPose(new Quaternionf(Q_ROS2MC));

        // ✅ hips(루트 본) 기준 중앙 정렬
        if (vrmCenterOnRoot) {
            pose.translate(-vrmRootCenter.x, -vrmRootCenter.y, -vrmRootCenter.z);
        }

        // ✅ 최종 미세 스케일
        pose.scale(vrmViewScale, vrmViewScale, vrmViewScale);

        Matrix4f m = pose.last().pose();

        int argb = 0xFFFFFFFF; // 흰색 라인
        int blockLight = (packedLight & 0xFFFF);
        int skyLight   = (packedLight >> 16) & 0xFFFF;

        for (var b : vrmSkel.bones) {
            if (b.parentNodeIndex == null) continue;
            String parent = findParentName(vrmSkel, b);
            if (parent == null) continue;

            Pose cp = vrmLive.get(b.name);
            Pose pp = vrmLive.get(parent);
            if (cp == null || pp == null) continue;

            addSegment(vc, m, pp.p, cp.p, argb, blockLight, skyLight, overlay);
        }

        pose.popPose();
    }

    private static void addSegment(VertexConsumer v, Matrix4f m,
                                   Vector3f a, Vector3f b, int argb,
                                   int blockLight, int skyLight, int overlay) {
        int r=(argb>>16)&255, g=(argb>>8)&255, bl=argb&255, a8=(argb>>>24)&255;

        // 간단 라인(두 점만) — 엔진 기본 라인 없으면 리본/박스 라인으로 대체 권장
        v.addVertex(m, a.x, a.y, a.z)
         .setColor(r, g, bl, a8)
         .setUv(0.5f, 0.5f)
         .setUv2(blockLight, skyLight)
         .setNormal(0, 1, 0);

        v.addVertex(m, b.x, b.y, b.z)
         .setColor(r, g, bl, a8)
         .setUv(0.5f, 0.5f)
         .setUv2(blockLight, skyLight)
         .setNormal(0, 1, 0);
    }

    private String findParentName(VrmSkeleton s, VrmLoader.Bone b) {
        Integer pi = b.parentNodeIndex;
        if (pi == null) return null;
        for (var x : s.bones) if (Objects.equals(x.nodeIndex, pi)) return x.name;
        if (pi >= 0 && pi < s.allNodes.size()) return s.allNodes.get(pi).name;
        return null;
    }

    // ===== 내부 유틸 =====
    private URDFJoint getJointByName(String name) {
        if (name == null) return null;
        for (URDFJoint j : robotModel.joints) {
            if (name.equals(j.name)) return j;
        }
        return null;
    }

    // ============================================
    // 업라이트 보정 유틸 (안전 버전)
    // ============================================

    /** Up을 먼저 맞추고 → Up에 수직인 평면에서 Forward만 정렬 (롤 꼬임 방지) */
    private static Quaternionf makeUprightQuat(Vector3f srcUp, Vector3f srcFwd,
                                               Vector3f dstUp, Vector3f dstFwd) {
        Vector3f su = new Vector3f(srcUp).normalize();
        Vector3f sf = new Vector3f(srcFwd).normalize();
        Vector3f du = new Vector3f(dstUp).normalize();
        Vector3f df = new Vector3f(dstFwd).normalize();

        // 1) Up 정렬
        Quaternionf qUp = fromToQuat(su, du);
        Vector3f sf1 = sf.rotate(new Quaternionf(qUp));      // Up 정렬 후 forward

        // 2) Up에 수직인 평면에서 Forward 정렬
        Vector3f sf1p = new Vector3f(sf1).sub(new Vector3f(du).mul(sf1.dot(du)));
        Vector3f dfp  = new Vector3f(df ).sub(new Vector3f(du).mul(df .dot(du)));
        if (sf1p.lengthSquared() < 1e-10f || dfp.lengthSquared() < 1e-10f) {
            return qUp.normalize(); // forward 정보가 퇴화 → Up만 맞춤
        }
        sf1p.normalize();
        dfp.normalize();

        float cos = clamp(sf1p.dot(dfp), -1f, 1f);
        float angle = (float)Math.acos(cos);
        Vector3f cross = sf1p.cross(dfp, new Vector3f());
        if (cross.dot(du) < 0) angle = -angle;

        Quaternionf qFwd = new Quaternionf().fromAxisAngleRad(du, angle);

        return qFwd.mul(qUp).normalize();
    }

    private static Quaternionf fromToQuat(Vector3f a, Vector3f b) {
        Vector3f v1 = new Vector3f(a).normalize();
        Vector3f v2 = new Vector3f(b).normalize();
        float dot = clamp(v1.dot(v2), -1f, 1f);

        if (dot > 1.0f - 1e-6f) {
            return new Quaternionf(); // 동일
        }
        if (dot < -1.0f + 1e-6f) {
            // 정반대: 임의의 수직축으로 180°
            Vector3f axis = pickAnyPerp(v1).normalize();
            return new Quaternionf().fromAxisAngleRad(axis, (float)Math.PI);
        }
        Vector3f axis = v1.cross(v2, new Vector3f()).normalize();
        float angle = (float)Math.acos(dot);
        return new Quaternionf().fromAxisAngleRad(axis, angle);
    }

    private static Vector3f pickAnyPerp(Vector3f v) {
        Vector3f x = new Vector3f(1,0,0), y = new Vector3f(0,1,0), z = new Vector3f(0,0,1);
        float dx = Math.abs(v.dot(x)), dy = Math.abs(v.dot(y)), dz = Math.abs(v.dot(z));
        return (dx < dy && dx < dz) ? x : ((dy < dz) ? y : z);
    }

    private static float clamp(float v, float lo, float hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    }

    // ===== VMC 리플렉션 =====

    private Object reflectGetVmcState() {
        try {
            Class<?> mgr = Class.forName("top.fifthlight.armorstand.vmc.VmcMarionetteManager");
            var getState = mgr.getMethod("getState");
            return getState.invoke(null);
        } catch (Throwable ignored) { return null; }
    }

    @SuppressWarnings("unchecked")
    private Map<String, Object> reflectCollectBoneMap(Object vmcState) {
        Map<String, Object> out = new HashMap<>();
        if (vmcState == null) return out;
        try {
            var f = vmcState.getClass().getField("boneTransforms");
            Object raw = f.get(vmcState);
            if (!(raw instanceof Map<?,?> m)) return out;
            for (var e : ((Map<Object,Object>)m).entrySet()) {
                Object tag = e.getKey();
                String name = tag.toString();
                try {
                    var nameM = tag.getClass().getMethod("name");
                    Object n = nameM.invoke(tag);
                    if (n != null) name = n.toString();
                } catch (Throwable ignored) {}
                out.put(name, e.getValue());
            }
        } catch (Throwable ignored) {}
        return out;
    }

    private Vector3f readPos(Object tr) {
        try {
            Object p = tr.getClass().getField("position").get(tr);
            float x = (float)p.getClass().getMethod("x").invoke(p);
            float y = (float)p.getClass().getMethod("y").invoke(p);
            float z = (float)p.getClass().getMethod("z").invoke(p);
            return new Vector3f(x,y,z);
        } catch (Throwable ignored) { return null; }
    }

    private Quaternionf readQuat(Object tr) {
        try {
            Object r = tr.getClass().getField("rotation").get(tr);
            float x = (float)r.getClass().getMethod("x").invoke(r);
            float y = (float)r.getClass().getMethod("y").invoke(r);
            float z = (float)r.getClass().getMethod("z").invoke(r);
            float w = (float)r.getClass().getMethod("w").invoke(r);
            return new Quaternionf(x,y,z,w);
        } catch (Throwable ignored) { return null; }
    }
}
