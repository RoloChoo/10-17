package com.kAIS.KAIMyEntity.renderer;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;
import java.util.Properties;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import net.minecraft.client.Minecraft;

public class MMDModelManager {
    static final Logger logger = LogManager.getLogger();
    static final Minecraft MCinstance = Minecraft.getInstance();
    static Map<String, Model> models;
    static String gameDirectory = MCinstance.gameDirectory.getAbsolutePath();

    public static void Init() {
        models = new HashMap<>();
        logger.info("MMDModelManager.Init() finished");
    }

    /**
     * 모델 로딩 - URDF만 지원
     */
    public static IMMDModel LoadModel(String modelName) {
        File modelDir = new File(gameDirectory + "/KAIMyEntity/" + modelName);
        String modelDirStr = modelDir.getAbsolutePath();

        if (!modelDir.exists()) {
            logger.error("Model directory not found: " + modelDirStr);
            return null;
        }

        // URDF만 체크
        File urdfFile = new File(modelDir, "robot.urdf");
        if (urdfFile.isFile()) {
            logger.info("Loading URDF: " + modelName);
            IMMDModel urdfModel = com.kAIS.KAIMyEntity.urdf.URDFModelOpenGLWithSTL.Create(
                urdfFile.getAbsolutePath(),
                modelDirStr
            );
            if (urdfModel != null) {
                logger.info("✓ URDF loaded: " + modelName);
                return urdfModel;
            }
        }

        logger.error("No robot.urdf found in: " + modelDirStr);
        return null;
    }

    /**
     * 모델 가져오기 (캐시 포함)
     */
    public static Model GetModel(String modelName, String uuid) {
        if (models == null) {
            Init();
        }

        String fullName = modelName + uuid;
        Model model = models.get(fullName);

        if (model == null) {
            IMMDModel m = LoadModel(modelName);
            if (m == null) {
                return null;
            }

            // URDF 모델 등록
            URDFModelData urdfData = new URDFModelData();
            urdfData.entityName = fullName;
            urdfData.model = m;
            urdfData.modelName = modelName;

            m.ResetPhysics();

            models.put(fullName, urdfData);
            logger.info("✓ Model registered: " + fullName);

            model = urdfData;
        }
        return model;
    }

    public static Model GetModel(String modelName) {
        return GetModel(modelName, "");
    }

    public static void ReloadModel() {
        if (models != null) {
            models.clear();
        } else {
            Init();
        }
    }

    // ========== 모델 클래스 ==========

    public static abstract class Model {
        public IMMDModel model;
        public String entityName;
        public String modelName;
        public Properties properties = new Properties();
        boolean isPropertiesLoaded = false;

        /**
         * model.properties 로딩 (없으면 조용히 패스)
         * @param forceReload true면 다시 로드
         */
        public void loadModelProperties(boolean forceReload) {
            if (isPropertiesLoaded && !forceReload) return;

            String path2Properties = gameDirectory + "/KAIMyEntity/" + modelName + "/model.properties";
            try (InputStream istream = new FileInputStream(path2Properties)) {
                properties.load(istream);
            } catch (IOException ignored) {
                // properties 없어도 OK
            }

            // 공용(common) 코드에서 클라이언트 전용 클래스에 접근하지 않도록 변경
            // (기존: KAIMyEntityClient.reloadProperties = false;)
            isPropertiesLoaded = true;
        }

        public boolean isURDFModel() { return true; }
    }

    // 이 파일에서 참조되는 URDFModelData는 기존 프로젝트에 맞춰 별도 정의되어 있어야 합니다.
    // public static class URDFModelData extends Model { ... }
}
