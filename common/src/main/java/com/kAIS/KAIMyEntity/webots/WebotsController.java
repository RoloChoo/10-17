// common/src/main/java/com/kAIS/KAIMyEntity/webots/WebotsController.java
package com.kAIS.KAIMyEntity.webots;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.time.Duration;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.*;

/**
 * Webots DARwIn-OP 제어 클라이언트 (통합 버전)
 * - HTTP REST API로 관절 제어
 * - 레이트 리미팅 (50 req/sec)
 * - Delta Compression (변화량만 전송)
 * - 비동기 전송 (게임 프레임 블로킹 방지)
 * - IP/Port 동적 설정 지원
 */
public class WebotsController {
    private static final Logger LOGGER = LogManager.getLogger();
    private static WebotsController instance;
    
    private final HttpClient httpClient;
    private String webotsUrl;
    private String robotIp;
    private int robotPort;
    private final ExecutorService executor;
    
    // 레이트 리미팅
    private final ScheduledExecutorService scheduler;
    private final BlockingQueue<Command> commandQueue;
    private final Map<String, Float> lastSent;
    private static final float DELTA_THRESHOLD = 0.01f; // 0.01 rad ≈ 0.57도
    
    // 연결 상태
    private volatile boolean connected = false;
    private volatile int failureCount = 0;
    private static final int MAX_FAILURES = 10;
    
    // 통계
    private final Stats stats = new Stats();
    
    // Webots 관절 매핑
    private static final Map<String, JointMapping> JOINT_MAP = new HashMap<>();
    
    static {
        // 팔 (Arms)
        JOINT_MAP.put("l_sho_pitch", new JointMapping("ShoulderL", 1, -1.57f, 0.52f));
        JOINT_MAP.put("l_sho_roll",  new JointMapping("ArmUpperL", 3, -2.25f, 0.77f));
        JOINT_MAP.put("l_el",        new JointMapping("ArmLowerL", 5, -1.57f, -0.10f));
        
        JOINT_MAP.put("r_sho_pitch", new JointMapping("ShoulderR", 0, -1.57f, 0.52f));
        JOINT_MAP.put("r_sho_roll",  new JointMapping("ArmUpperR", 2, -0.68f, 2.30f));
        JOINT_MAP.put("r_el",        new JointMapping("ArmLowerR", 4, -1.57f, -0.10f));
        
        // 골반 (Pelvis)
        JOINT_MAP.put("pelv_y_r", new JointMapping("PelvYR", 6, -1.047f, 1.047f));
        JOINT_MAP.put("pelv_y_l", new JointMapping("PelvYL", 7, -0.69f, 2.50f));
        JOINT_MAP.put("pelv_r",   new JointMapping("PelvR", 8, -1.01f, 1.01f));
        JOINT_MAP.put("pelv_l",   new JointMapping("PelvL", 9, -0.35f, 0.35f));
        
        // 다리 (Legs)
        JOINT_MAP.put("l_hip_pitch", new JointMapping("LegUpperL", 11, -2.50f, 0.87f));
        JOINT_MAP.put("r_hip_pitch", new JointMapping("LegUpperR", 10, -2.50f, 0.87f));
        JOINT_MAP.put("l_hip_roll",  new JointMapping("LegLowerL", 13, -0.35f, 0.35f));
        JOINT_MAP.put("r_hip_roll",  new JointMapping("LegLowerR", 12, -0.35f, 0.35f));
        
        JOINT_MAP.put("l_knee",  new JointMapping("KneeL", 15, -0.1f, 2.09f));
        JOINT_MAP.put("r_knee",  new JointMapping("KneeR", 14, -0.1f, 2.09f));
        
        JOINT_MAP.put("l_ankle_pitch", new JointMapping("AnkleL", 15, -1.39f, 1.22f));
        JOINT_MAP.put("r_ankle_pitch", new JointMapping("AnkleR", 14, -0.87f, 0.87f));
        JOINT_MAP.put("l_ankle_roll",  new JointMapping("FootL", 17, -0.87f, 0.87f));
        JOINT_MAP.put("r_ankle_roll",  new JointMapping("FootR", 16, -0.87f, 0.87f));
        
        // 머리 (Head)
        JOINT_MAP.put("neck", new JointMapping("Neck", 18, -1.57f, 1.57f));
        JOINT_MAP.put("head", new JointMapping("Head", 19, -0.52f, 0.52f));
    }
    
    private WebotsController(String ip, int port) {
        this.robotIp = ip;
        this.robotPort = port;
        this.webotsUrl = String.format("http://%s:%d", ip, port);
        
        this.httpClient = HttpClient.newBuilder()
                .connectTimeout(Duration.ofMillis(500))
                .build();
        
        this.executor = Executors.newSingleThreadExecutor(r -> {
            Thread t = new Thread(r, "Webots-Sender");
            t.setDaemon(true);
            return t;
        });
        
        this.scheduler = Executors.newScheduledThreadPool(1, r -> {
            Thread t = new Thread(r, "Webots-Scheduler");
            t.setDaemon(true);
            return t;
        });
        
        this.commandQueue = new LinkedBlockingQueue<>();
        this.lastSent = new ConcurrentHashMap<>();
        
        // 20ms마다 큐 처리 (50 req/sec)
        scheduler.scheduleAtFixedRate(this::processQueue, 0, 20, TimeUnit.MILLISECONDS);
        
        // 초기 연결 테스트
        testConnection();
        
        LOGGER.info("✅ WebotsController initialized: {}", webotsUrl);
    }
    
    /**
     * 싱글톤 인스턴스 (기본: localhost:8080)
     */
    public static WebotsController getInstance() {
        if (instance == null) {
            instance = new WebotsController("localhost", 8080);
        }
        return instance;
    }
    
    /**
     * IP/Port 지정 인스턴스 생성
     */
    public static WebotsController getInstance(String ip, int port) {
        if (instance != null) {
            // 기존 인스턴스의 IP/Port와 다르면 재생성
            if (!instance.robotIp.equals(ip) || instance.robotPort != port) {
                LOGGER.info("🔄 Recreating WebotsController with new address: {}:{}", ip, port);
                instance.shutdown();
                instance = new WebotsController(ip, port);
            }
        } else {
            instance = new WebotsController(ip, port);
        }
        return instance;
    }
    
    /**
     * 연결 재설정
     */
    public void reconnect(String ip, int port) {
        LOGGER.info("🔄 Reconnecting to {}:{}", ip, port);
        this.robotIp = ip;
        this.robotPort = port;
        this.webotsUrl = String.format("http://%s:%d", ip, port);
        this.failureCount = 0;
        this.connected = false;
        
        // 큐 비우기
        commandQueue.clear();
        lastSent.clear();
        
        testConnection();
    }
    
    /**
     * 연결 테스트
     */
    private void testConnection() {
        executor.submit(() -> {
            try {
                String url = webotsUrl + "/?command=get_stats";
                HttpRequest request = HttpRequest.newBuilder()
                        .uri(URI.create(url))
                        .timeout(Duration.ofMillis(500))
                        .GET()
                        .build();
                
                HttpResponse<String> response = httpClient.send(request, 
                        HttpResponse.BodyHandlers.ofString());
                
                if (response.statusCode() == 200) {
                    connected = true;
                    failureCount = 0;
                    LOGGER.info("✅ Connected to Webots: {}", webotsUrl);
                } else {
                    LOGGER.warn("⚠️  Webots returned status {}", response.statusCode());
                }
                
            } catch (Exception e) {
                connected = false;
                LOGGER.error("❌ Failed to connect to Webots: {}", e.getMessage());
            }
        });
    }
    
    /**
     * 관절 제어 (비동기)
     * @param jointName URDF 조인트 이름 (예: "l_sho_pitch")
     * @param value 라디안 값
     */
    public void setJoint(String jointName, float value) {
        JointMapping mapping = JOINT_MAP.get(jointName);
        if (mapping == null) {
            if (stats.unknownJointWarnings.computeIfAbsent(jointName, k -> 0) < 3) {
                LOGGER.warn("Unknown joint: {} (warning {} of 3)", jointName, 
                           stats.unknownJointWarnings.merge(jointName, 1, Integer::sum));
            }
            return;
        }
        
        // Delta Compression: 변화량이 작으면 무시
        Float last = lastSent.get(jointName);
        if (last != null && Math.abs(value - last) < DELTA_THRESHOLD) {
            stats.deltaSkipped++;
            return;
        }
        
        // 범위 체크
        float clamped = clamp(value, mapping.min, mapping.max);
        if (Math.abs(clamped - value) > 0.001f) {
            stats.rangeClamped++;
        }
        
        // 큐에 추가
        if (commandQueue.offer(new Command(mapping.index, clamped))) {
            lastSent.put(jointName, clamped);
            stats.queued++;
        } else {
            stats.queueFull++;
        }
    }
    
    /**
     * 여러 관절 동시 제어
     */
    public void setJoints(Map<String, Float> joints) {
        joints.forEach(this::setJoint);
    }
    
    /**
     * 큐 처리 (스케줄러가 자동 호출)
     */
    private void processQueue() {
        Command cmd = commandQueue.poll();
        if (cmd == null) return;
        
        executor.submit(() -> sendToWebots(cmd.index, cmd.value));
    }
    
    /**
     * Webots HTTP API 호출
     */
    private void sendToWebots(int index, float value) {
        if (!connected && failureCount > MAX_FAILURES) {
            return; // 연결 끊김 상태면 전송 중단
        }
        
        try {
            String url = String.format("%s/?command=set_joint&index=%d&value=%.4f", 
                                      webotsUrl, index, value);
            
            HttpRequest request = HttpRequest.newBuilder()
                    .uri(URI.create(url))
                    .timeout(Duration.ofMillis(100))
                    .GET()
                    .build();
            
            HttpResponse<String> response = httpClient.send(request, 
                    HttpResponse.BodyHandlers.ofString());
            
            if (response.statusCode() == 200) {
                stats.sent++;
                failureCount = 0;
                if (!connected) {
                    connected = true;
                    LOGGER.info("✅ Reconnected to Webots");
                }
            } else {
                stats.failed++;
                LOGGER.warn("⚠️  Webots returned status {}", response.statusCode());
            }
            
        } catch (Exception e) {
            stats.failed++;
            failureCount++;
            
            if (failureCount == MAX_FAILURES) {
                connected = false;
                LOGGER.error("❌ Connection lost to Webots after {} failures", MAX_FAILURES);
            } else if (failureCount % 50 == 0) {
                LOGGER.warn("⚠️  Failed to send to Webots ({} failures): {}", 
                           failureCount, e.getMessage());
            }
        }
    }
    
    /**
     * 통계 조회 (JSON)
     */
    public String getStatsJson() {
        try {
            String url = webotsUrl + "/?command=get_stats";
            HttpRequest request = HttpRequest.newBuilder()
                    .uri(URI.create(url))
                    .timeout(Duration.ofMillis(200))
                    .GET()
                    .build();
            
            HttpResponse<String> response = httpClient.send(request, 
                    HttpResponse.BodyHandlers.ofString());
            
            return response.body();
            
        } catch (Exception e) {
            return String.format("{\"error\": \"%s\"}", e.getMessage());
        }
    }
    
    /**
     * 통계 출력
     */
    public void printStats() {
        LOGGER.info("=== Webots Controller Stats ===");
        LOGGER.info("  Target: {}:{} {}", robotIp, robotPort, connected ? "✅" : "❌");
        LOGGER.info("  Queued: {} | Sent: {} | Failed: {}", stats.queued, stats.sent, stats.failed);
        LOGGER.info("  Delta Skipped: {} | Range Clamped: {} | Queue Full: {}", 
                   stats.deltaSkipped, stats.rangeClamped, stats.queueFull);
        LOGGER.info("  Queue Size: {} | Failure Count: {}", commandQueue.size(), failureCount);
        
        // Webots 서버 통계
        String serverStats = getStatsJson();
        LOGGER.info("  Server Stats: {}", serverStats);
    }
    
    /**
     * 연결 상태 확인
     */
    public boolean isConnected() {
        return connected;
    }
    
    /**
     * 현재 설정된 IP/Port
     */
    public String getRobotAddress() {
        return String.format("%s:%d", robotIp, robotPort);
    }
    
    /**
     * 종료
     */
    public void shutdown() {
        LOGGER.info("🛑 Shutting down WebotsController...");
        scheduler.shutdown();
        executor.shutdown();
        try {
            if (!executor.awaitTermination(1, TimeUnit.SECONDS)) {
                executor.shutdownNow();
            }
        } catch (InterruptedException e) {
            executor.shutdownNow();
        }
        LOGGER.info("✅ WebotsController shutdown complete");
    }
    
    // ========== 내부 클래스 ==========
    
    private static class Command {
        final int index;
        final float value;
        final long timestamp;
        
        Command(int index, float value) {
            this.index = index;
            this.value = value;
            this.timestamp = System.currentTimeMillis();
        }
    }
    
    private static class JointMapping {
        final String webotsName;
        final int index;
        final float min;
        final float max;
        
        JointMapping(String webotsName, int index, float min, float max) {
            this.webotsName = webotsName;
            this.index = index;
            this.min = min;
            this.max = max;
        }
    }
    
    private static class Stats {
        long queued = 0;
        long sent = 0;
        long failed = 0;
        long deltaSkipped = 0;
        long rangeClamped = 0;
        long queueFull = 0;
        final Map<String, Integer> unknownJointWarnings = new ConcurrentHashMap<>();
    }
    
    private static float clamp(float value, float min, float max) {
        return Math.max(min, Math.min(max, value));
    }
    
    // ========== 유틸리티 메서드 ==========
    
    /**
     * 지원되는 모든 관절 이름 조회
     */
    public static String[] getSupportedJoints() {
        return JOINT_MAP.keySet().toArray(new String[0]);
    }
    
    /**
     * 관절 정보 조회
     */
    public static JointMapping getJointMapping(String jointName) {
        return JOINT_MAP.get(jointName);
    }
    
    /**
     * Webots 모터 인덱스 조회
     */
    public static Integer getMotorIndex(String jointName) {
        JointMapping mapping = JOINT_MAP.get(jointName);
        return mapping != null ? mapping.index : null;
    }
}