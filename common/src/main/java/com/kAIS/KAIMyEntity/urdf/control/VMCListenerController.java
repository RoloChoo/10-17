package com.kAIS.KAIMyEntity.urdf.control;

import net.minecraft.client.Minecraft;
import net.minecraft.client.gui.GuiGraphics;
import net.minecraft.client.gui.components.Button;
import net.minecraft.client.gui.components.EditBox;
import net.minecraft.client.gui.screens.Screen;
import net.minecraft.network.chat.Component;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.io.IOException;
import java.net.*;
import java.nio.charset.StandardCharsets;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

/**
 * VMCListenerController - 정석 VMC 최적화 + VSeeFace 실질적 차단 완성판
 * (2025.11.18 최종 진짜 끝판왕 버전)
 */
public class VMCListenerController extends Screen {
    private static final int BG_COLOR = 0xFF0E0E10;
    private static final int PANEL_COLOR = 0xFF1D1F24;
    private static final int TITLE_COLOR = 0xFFFFD770;
    private static final int TXT_MAIN = 0xFFFFFFFF;

    private final Screen parent;
    private final VmcListener listener = VmcListener.getInstance();

    private EditBox addressBox;
    private EditBox portBox;
    private Button startButton;
    private Button stopButton;
    private Button closeButton;
    private int autoRefreshTicker = 0;

    public VMCListenerController(Screen parent, Object rendererIgnored) {
        super(Component.literal("VMC Listener Controller"));
        this.parent = parent;
    }

    @Override
    protected void init() {
        super.init();
        int centerX = this.width / 2;
        int startY = 40;

        addressBox = new EditBox(this.font, centerX - 100, startY, 200, 20, Component.literal("Address"));
        addressBox.setValue("0.0.0.0");
        addressBox.setMaxLength(100);
        addRenderableWidget(addressBox);

        startY += 25;
        portBox = new EditBox(this.font, centerX - 100, startY, 200, 20, Component.literal("Port"));
        portBox.setValue("39539");
        portBox.setMaxLength(5);
        addRenderableWidget(portBox);

        startY += 30;
        startButton = Button.builder(Component.literal("Start VMC Listener"), b -> {
            String addr = addressBox.getValue();
            int port;
            try {
                port = Integer.parseInt(portBox.getValue());
                if (port < 1 || port > 65535) throw new NumberFormatException();
            } catch (NumberFormatException e) {
                minecraft.gui.getChat().addMessage(Component.literal("§c[VMC] Invalid port number"));
                return;
            }
            listener.start(addr, port);
            updateButtons();
        }).bounds(centerX - 100, startY, 200, 20).build();
        addRenderableWidget(startButton);

        startY += 25;
        stopButton = Button.builder(Component.literal("Stop VMC Listener"), b -> {
            listener.stop();
            updateButtons();
        }).bounds(centerX - 100, startY, 200, 20).build();
        addRenderableWidget(stopButton);

        closeButton = Button.builder(Component.literal("Close"), b -> this.onClose())
                .bounds(centerX - 50, this.height - 30, 100, 20).build();
        addRenderableWidget(closeButton);

        updateButtons();
    }

    private void updateButtons() {
        boolean running = listener.isRunning();
        startButton.active = !running;
        stopButton.active = running;
        addressBox.setEditable(!running);
        portBox.setEditable(!running);
    }

    @Override
    public void render(GuiGraphics graphics, int mouseX, int mouseY, float partialTicks) {
        graphics.fill(0, 0, this.width, this.height, BG_COLOR);
        int panelX = this.width / 2 - 220;
        int panelY = 140;
        int panelW = 440;
        int panelH = this.height - panelY - 50;
        graphics.fill(panelX, panelY, panelX + panelW, panelY + panelH, PANEL_COLOR);

        super.render(graphics, mouseX, mouseY, partialTicks);
        graphics.pose().pushPose();
        graphics.pose().translate(0, 0, 1000.0f);

        graphics.drawCenteredString(this.font, "VMC Listener Controller", this.width / 2, 10, TITLE_COLOR);
        graphics.drawCenteredString(this.font, "§c※ VSeeFace 등 비표준 프로그램은 실질적으로 동작하지 않습니다", this.width / 2, 24, 0xFFFF5555);

        VmcListener.Diagnostics diag = listener.getDiagnostics();
        List<String> statusLines = new ArrayList<>();

        if (diag.running()) {
            statusLines.add("§aStatus: RUNNING");
            long elapsed = System.currentTimeMillis() - diag.lastPacketTime();
            statusLines.add("Last packet: " + (diag.lastPacketTime() == 0 ? "Never" : elapsed + " ms ago"));
            statusLines.add("Total: " + diag.totalPackets() + " | Valid VMC: " + diag.vmcPackets());
            statusLines.add("Active Bones: " + listener.getBones().size() + " (표준 본만 카운트)");

            if (!diag.recentMessages().isEmpty()) {
                statusLines.add("");
                statusLines.add("§eRecent Messages:");
                for (int i = Math.max(0, diag.recentMessages().size() - 5); i < diag.recentMessages().size(); i++) {
                    statusLines.add(" " + diag.recentMessages().get(i));
                }
            }
        } else {
            statusLines.add("§cStatus: STOPPED");
            statusLines.add("");
            statusLines.add("정석 VMC 프로그램 추천: VMagicMirror, Luppet, Hitogata, VNyan");
        }

        int textY = panelY + 10;
        for (String line : statusLines) {
            graphics.drawString(this.font, line, panelX + 10, textY, TXT_MAIN, false);
            textY += 12;
        }

        graphics.pose().popPose();

        if (++autoRefreshTicker >= 10) {
            autoRefreshTicker = 0;
            updateButtons();
        }
    }

    @Override
    public void onClose() {
        this.minecraft.setScreen(this.parent);
    }

    @Override
    public boolean isPauseScreen() {
        return false;
    }

    /* ======================================================================== */
    public static final class VmcListener {
        private static final Logger logger = LogManager.getLogger();
        private static volatile VmcListener instance;

        private DatagramSocket socket;
        private Thread receiverThread;
        private final AtomicBoolean running = new AtomicBoolean(false);

        private final Map<String, BoneTransform> bones = new ConcurrentHashMap<>();
        private final Transform rootTransform = new Transform();
        private final Map<String, Float> blendShapes = new ConcurrentHashMap<>();

        private final AtomicLong totalPackets = new AtomicLong(0);
        private final AtomicLong vmcPackets = new AtomicLong(0);
        private final AtomicLong lastPacketTime = new AtomicLong(0);
        private final Deque<String> recentMessages = new LinkedList<>();
        private static final int MAX_RECENT = 10;

        // KAIMyEntity IK가 요구하는 정확한 표준 본 이름들 (이 이름이 아니면 저장 안 됨)
        private static final Set<String> STANDARD_BONE_NAMES = Set.of(
                "Hips", "Spine", "Chest", "UpperChest", "Neck", "Head",
                "LeftShoulder", "LeftUpperArm", "LeftLowerArm", "LeftHand",
                "RightShoulder", "RightUpperArm", "RightLowerArm", "RightHand",
                "LeftUpperLeg", "LeftLowerLeg", "LeftFoot", "LeftToes",
                "RightUpperLeg", "RightLowerLeg", "RightFoot", "RightToes",
                "LeftEye", "RightEye",
                "LeftThumbProximal", "LeftThumbIntermediate", "LeftThumbDistal",
                "LeftIndexProximal", "LeftIndexIntermediate", "LeftIndexDistal",
                "LeftMiddleProximal", "LeftMiddleIntermediate", "LeftMiddleDistal",
                "LeftRingProximal", "LeftRingIntermediate", "LeftRingDistal",
                "LeftLittleProximal", "LeftLittleIntermediate", "LeftLittleDistal",
                "RightThumbProximal", "RightThumbIntermediate", "RightThumbDistal",
                "RightIndexProximal", "RightIndexIntermediate", "RightIndexDistal",
                "RightMiddleProximal", "RightMiddleIntermediate", "RightMiddleDistal",
                "RightRingProximal", "RightRingIntermediate", "RightRingDistal",
                "RightLittleProximal", "RightLittleIntermediate", "RightLittleDistal"
        );

        // IK 시스템이 요구하는 이름 정규화 (이제 다시 허용!)
        private java.util.function.Function<String, String> boneNameNormalizer = name -> name;

        private VmcListener() {}

        public static VmcListener getInstance() {
            if (instance == null) {
                synchronized (VmcListener.class) {
                    if (instance == null) {
                        instance = new VmcListener();
                    }
                }
            }
            return instance;
        }

        // 반드시 있어야 IK 시스템이 컴파일됨
        public void setBoneNameNormalizer(java.util.function.Function<String, String> normalizer) {
            this.boneNameNormalizer = normalizer != null ? normalizer : name -> name;
        }

        public synchronized void start(String addr, int port) {
            if (running.get()) return;
            try {
                InetAddress bindAddr = "0.0.0.0".equals(addr) ? null : InetAddress.getByName(addr);
                socket = bindAddr == null ? new DatagramSocket(port) : new DatagramSocket(port, bindAddr);
                running.set(true);
                receiverThread = new Thread(this::receiveLoop, "VMC-Receiver");
                receiverThread.setDaemon(true);
                receiverThread.start();
                logger.info("VMC Listener started on {}:{}", addr, port);
                Minecraft.getInstance().execute(() ->
                        Minecraft.getInstance().gui.getChat().addMessage(
                                Component.literal("§a[VMC] Listening on " + addr + ":" + port)));
            } catch (Exception e) {
                logger.error("Failed to start VMC Listener", e);
                Minecraft.getInstance().execute(() ->
                        Minecraft.getInstance().gui.getChat().addMessage(
                                Component.literal("§c[VMC] Failed: " + e.getMessage())));
            }
        }

        public synchronized void stop() {
            if (!running.get()) return;
            running.set(false);
            if (socket != null) socket.close();
            bones.clear();
            blendShapes.clear();
            rootTransform.position.set(0, 0, 0);
            rootTransform.rotation.set(0, 0, 0, 1);
            logger.info("VMC Listener stopped");
            Minecraft.getInstance().execute(() ->
                    Minecraft.getInstance().gui.getChat().addMessage(Component.literal("§c[VMC] Stopped")));
        }

        private void receiveLoop() {
            byte[] buffer = new byte[65536];
            DatagramPacket packet = new DatagramPacket(buffer, buffer.length);
            boolean first = true;
            while (running.get()) {
                try {
                    socket.receive(packet);
                    lastPacketTime.set(System.currentTimeMillis());
                    totalPackets.incrementAndGet();
                    if (first) {
                        first = false;
                        Minecraft.getInstance().execute(() ->
                                Minecraft.getInstance().gui.getChat().addMessage(
                                        Component.literal("§b[VMC] Connected! Receiving data...")));
                    }
                    processOscPacket(buffer, packet.getLength());
                } catch (IOException ignored) {
                    if (!running.get()) break;
                }
            }
        }

        private void processOscPacket(byte[] data, int length) {
            if (length < 8) return;
            if (startsWith(data, 0, "#bundle")) {
                processBundleRecursive(data, 0, length);
            } else {
                processOscMessage(data, 0, length);
            }
        }

        private void processBundleRecursive(byte[] data, int offset, int length) {
            int pos = offset + padLen("#bundle") + 8;
            while (pos + 4 <= offset + length) {
                int size = readInt(data, pos);
                pos += 4;
                if (pos + size > offset + length) break;
                if (startsWith(data, pos, "#bundle")) {
                    processBundleRecursive(data, pos, size);
                } else {
                    processOscMessage(data, pos, size);
                }
                pos += size;
            }
        }

        private void processOscMessage(byte[] data, int offset, int length) {
            int pos = offset;
            int end = offset + length;
            String address = readString(data, pos, end);
            if (address == null) return;
            pos += padLen(address);

            synchronized (recentMessages) {
                if (recentMessages.size() >= MAX_RECENT) recentMessages.removeFirst();
                recentMessages.addLast(address);
            }

            String types = readString(data, pos, end);
            if (types == null || !types.startsWith(",")) return;
            pos += padLen(types);

            List<Object> args = new ArrayList<>();
            for (int i = 1; i < types.length(); i++) {
                char t = types.charAt(i);
                if (t == 'f' && pos + 4 <= end) {
                    args.add(Float.intBitsToFloat(readInt(data, pos)));
                    pos += 4;
                } else if (t == 'i' && pos + 4 <= end) {
                    args.add(readInt(data, pos));
                    pos += 4;
                } else if (t == 's') {
                    String s = readString(data, pos, end);
                    if (s == null) return;
                    args.add(s);
                    pos += padLen(s);
                } else return;
            }

            handleVmcMessage(address, args.toArray());
        }

        private void handleVmcMessage(String address, Object[] args) {
            if (!address.startsWith("/VMC/Ext/")) return;
            vmcPackets.incrementAndGet();

            switch (address) {
                case "/VMC/Ext/Root/Pos", "/VMC/Ext/Root/Pos/Local" -> {
                    if (args.length >= 8 && args[0] instanceof String) {
                        rootTransform.position.set(getFloat(args, 1), getFloat(args, 2), getFloat(args, 3));
                        rootTransform.rotation.set(getFloat(args, 4), getFloat(args, 5), getFloat(args, 6), getFloat(args, 7)).normalize();
                    }
                }
                case "/VMC/Ext/Bone/Pos", "/VMC/Ext/Bone/Pos/Local" -> {
                    if (args.length >= 8 && args[0] instanceof String rawName) {
                        String normalized = boneNameNormalizer.apply(rawName);

                        // 정규화된 이름이 표준 이름이 아니면 완전 무시 → VSeeFace 실질적 차단
                        if (normalized == null || !STANDARD_BONE_NAMES.contains(normalized)) {
                            return;
                        }

                        BoneTransform bone = bones.computeIfAbsent(normalized, k -> new BoneTransform());
                        bone.position.set(getFloat(args, 1), getFloat(args, 2), getFloat(args, 3));
                        bone.rotation.set(getFloat(args, 4), getFloat(args, 5), getFloat(args, 6), getFloat(args, 7)).normalize();
                    }
                }
                case "/VMC/Ext/Blend/Val" -> {
                    if (args.length >= 2 && args[0] instanceof String name) {
                        blendShapes.put(name, getFloat(args, 1));
                    }
                }
            }
        }

        // OSC 유틸 함수들
        private static boolean startsWith(byte[] d, int o, String p) {
            byte[] pb = p.getBytes(StandardCharsets.US_ASCII);
            if (o + pb.length > d.length) return false;
            for (int i = 0; i < pb.length; i++) if (d[o + i] != pb[i]) return false;
            return true;
        }

        private static int readInt(byte[] d, int o) {
            return ((d[o] & 0xFF) << 24) | ((d[o + 1] & 0xFF) << 16) | ((d[o + 2] & 0xFF) << 8) | (d[o + 3] & 0xFF);
        }

        private static String readString(byte[] d, int o, int e) {
            int l = 0;
            while (o + l < e && d[o + l] != 0) l++;
            return o + l >= e ? null : new String(d, o, l, StandardCharsets.US_ASCII);
        }

        private static int padLen(String s) {
            int l = s.getBytes(StandardCharsets.US_ASCII).length + 1;
            return l + ((4 - (l % 4)) & 3);
        }

        private static float getFloat(Object[] a, int i) {
            if (i >= a.length) return 0f;
            Object o = a[i];
            return o instanceof Number n ? n.floatValue() : 0f;
        }

        // Public API
        public boolean isRunning() { return running.get(); }
        public Map<String, BoneTransform> getBones() { return Collections.unmodifiableMap(bones); }
        public Transform getRootTransform() { return new Transform(rootTransform); }
        public Map<String, Float> getBlendShapes() { return Collections.unmodifiableMap(blendShapes); }

        public Diagnostics getDiagnostics() {
            synchronized (recentMessages) {
                return new Diagnostics(running.get(), lastPacketTime.get(), totalPackets.get(),
                        vmcPackets.get(), new ArrayList<>(recentMessages));
            }
        }

        public static class Transform {
            public final Vector3f position = new Vector3f();
            public final Quaternionf rotation = new Quaternionf();
            public Transform() {}
            public Transform(Transform o) { position.set(o.position); rotation.set(o.rotation); }
        }

        public static class BoneTransform extends Transform {}

        public record Diagnostics(boolean running, long lastPacketTime, long totalPackets,
                                  long vmcPackets, List<String> recentMessages) {}
    }
}