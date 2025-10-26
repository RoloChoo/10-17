package com.kAIS.KAIMyEntity.urdf;

import com.kAIS.KAIMyEntity.renderer.IMMDModel;
import com.mojang.blaze3d.systems.RenderSystem;
import com.mojang.blaze3d.vertex.*;
import net.minecraft.client.renderer.GameRenderer;
import net.minecraft.world.entity.Entity;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.joml.Matrix4f;
import org.joml.Vector3f;

import java.io.File;

/**
 * URDF 모델 렌더링 + STL 메시 지원
 */
public class URDFModelOpenGLWithSTL implements IMMDModel {
    private static final Logger logger = LogManager.getLogger();
    private static int renderCount = 0;

    private URDFRobotModel robotModel;
    private String modelDir;

    public URDFModelOpenGLWithSTL(URDFRobotModel robotModel, String modelDir) {
        this.robotModel = robotModel;
        this.modelDir = modelDir;
        logger.info("=== URDF renderer Created ===");
    }

    @Override
    public void Render(Entity entityIn, float entityYaw, float entityPitch,
                       Vector3f entityTrans, float tickDelta, PoseStack mat, int packedLight) {

        renderCount++;
        logger.info("=== URDF RENDER CALLED #" + renderCount + " ===");
        logger.info("Entity: " + entityIn.getName().getString());

        // 기본 렌더 상태
        RenderSystem.enableBlend();
        RenderSystem.defaultBlendFunc();
        RenderSystem.enableDepthTest();
        RenderSystem.disableCull(); // 오타 fix: disableDull -> disableCull

        // 정점 버퍼 셋업
        Tesselator tesselator = Tesselator.getInstance();
        BufferBuilder bufferBuilder = tesselator.getBuilder();
        bufferBuilder.begin(VertexFormat.Mode.QUADS, DefaultVertexFormat.POSITION_COLOR);

        Matrix4f matrix = mat.last().pose();
        float size = 1.0f;
        float r = 1.0f, g = 0.0f, b = 0.0f, a = 1.0f;

        // 6면 박스 (각 정점은 vertex(...).color(...).endVertex())
        // Front (+Z)
        bufferBuilder.vertex(matrix, -size, 0,  size).color(r, g, b, a).endVertex();
        bufferBuilder.vertex(matrix,  size, 0,  size).color(r, g, b, a).endVertex();
        bufferBuilder.vertex(matrix,  size, 2*size,  size).color(r, g, b, a).endVertex();
        bufferBuilder.vertex(matrix, -size, 2*size,  size).color(r, g, b, a).endVertex();

        // Back (-Z)
        bufferBuilder.vertex(matrix,  size, 0, -size).color(r, g, b, a).endVertex();
        bufferBuilder.vertex(matrix, -size, 0, -size).color(r, g, b, a).endVertex();
        bufferBuilder.vertex(matrix, -size, 2*size, -size).color(r, g, b, a).endVertex();
        bufferBuilder.vertex(matrix,  size, 2*size, -size).color(r, g, b, a).endVertex();

        // Top (+Y)
        bufferBuilder.vertex(matrix, -size, 2*size, -size).color(r, g, b, a).endVertex();
        bufferBuilder.vertex(matrix, -size, 2*size,  size).color(r, g, b, a).endVertex();
        bufferBuilder.vertex(matrix,  size, 2*size,  size).color(r, g, b, a).endVertex();
        bufferBuilder.vertex(matrix,  size, 2*size, -size).color(r, g, b, a).endVertex();

        // Bottom (0)
        bufferBuilder.vertex(matrix, -size, 0,  size).color(r, g, b, a).endVertex();
        bufferBuilder.vertex(matrix, -size, 0, -size).color(r, g, b, a).endVertex();
        bufferBuilder.vertex(matrix,  size, 0, -size).color(r, g, b, a).endVertex();
        bufferBuilder.vertex(matrix,  size, 0,  size).color(r, g, b, a).endVertex();

        // Right (+X)
        bufferBuilder.vertex(matrix,  size, 0,  size).color(r, g, b, a).endVertex();
        bufferBuilder.vertex(matrix,  size, 0, -size).color(r, g, b, a).endVertex();
        bufferBuilder.vertex(matrix,  size, 2*size, -size).color(r, g, b, a).endVertex();
        bufferBuilder.vertex(matrix,  size, 2*size,  size).color(r, g, b, a).endVertex();

        // Left (-X)
        bufferBuilder.vertex(matrix, -size, 0, -size).color(r, g, b, a).endVertex();
        bufferBuilder.vertex(matrix, -size, 0,  size).color(r, g, b, a).endVertex();
        bufferBuilder.vertex(matrix, -size, 2*size,  size).color(r, g, b, a).endVertex();
        bufferBuilder.vertex(matrix, -size, 2*size, -size).color(r, g, b, a).endVertex();

        // 셰이더 지정 후 드로우 (오타 fix: darwWithSharder -> drawWithShader)
        RenderSystem.setShader(GameRenderer::getPositionColorShader);
        BufferUploader.drawWithShader(bufferBuilder.end());

        RenderSystem.enableCull();

        logger.info("Red Box render done");
    }

    @Override
    public void ChangeAnim(long anim, long layer) {
        // 암것도 안함
    }

    @Override
    public void ResetPhysics() {
        logger.info("ResetPhysics called");
    }

    @Override
    public long GetModelLong() {
        return 0;
    }

    @Override
    public String GetModelDir() {
        return modelDir;
    }

    public static URDFModelOpenGLWithSTL Create(String urdfPath, String modelDir) {
        logger.info("=== Creating URDF Renderer ===");
        logger.info("URDF: " + urdfPath);

        File urdfFile = new File(urdfPath);
        if (!urdfFile.exists()) {
            logger.error("URDF not found!");
            return null;
        }

        URDFRobotModel robot = URDFParser.parse(urdfFile);
        if (robot == null || robot.rootLinkName == null) {
            logger.error("URDF parse failed!");
            return null;
        }

        logger.info("✓ URDF Renderer created");
        return new URDFModelOpenGLWithSTL(robot, modelDir);
    }
}
