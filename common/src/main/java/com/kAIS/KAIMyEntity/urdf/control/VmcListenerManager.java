package com.kAIS.KAIMyEntity.urdf.control;

import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.io.Closeable;
import java.net.BindException;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetSocketAddress;
import java.net.SocketException;
import java.nio.charset.StandardCharsets;
import java.util.*;
import java.util.concurrent.atomic.AtomicReference;
import java.util.concurrent.locks.ReentrantLock;

/**
 * 최소 VMC 리스너:
 *  - UDP/OSC 수신(/VMC/Ext/Root/Pos, /VMC/Ext/Bone/Pos, /VMC/Ext/Blend/*, /Local 변형 허용)
 *  - 스레드 안전 더블버퍼 상태 유지
 *  - 틱 루프에서 getBones()로 읽기(락-프리 스냅샷)
 *
 * 좌표/스케일 변환은 URDFVmcMapper에서만 수행하세요(중복 금지).
 */
public final class VmcListenerManager {
    private VmcListenerManager() {}

    // ───────────── 퍼사드(외부 API) ─────────────
    private static final StateHolder STATE = new StateHolder();
    private static volatile UdpLoop loop;

    /** 실행 상태는 실제 스레드 라이프사이클로 판정(바인드 실패 직후 종료 반영) */
    public static boolean isRunning() {
        UdpLoop l = loop;
        return l != null && l.isAlive() && !l.isClosed();
    }

    public static synchronized void start(int port) {
        if (isRunning()) return;
        loop = new UdpLoop(port, STATE);
        loop.start();
        System.out.println("[VMC] listener starting on UDP " + port);
    }

    public static synchronized void stop() {
        if (loop != null) {
            loop.close();
            loop = null;
        }
        System.out.println("[VMC] listener stopped");
    }

    /** 스냅샷의 '본 맵'만 바로 꺼내 쓰기(URDFVmcMapper와 호환되는 Transform 객체 보유) */
    public static Map<String, Object> getBones() {
        Snapshot s = STATE.read();
        return (s == null) ? Collections.emptyMap() : s.boneTransforms;
    }

    /** 상태 전체 스냅샷(원하면 사용) */
    public static Snapshot getSnapshot() { return STATE.read(); }

    // ───────────── 진단(클라 채팅에서 사용) ─────────────
    private static volatile long lastPacketMs = 0;
    private static volatile int  totalPackets = 0;
    private static volatile int  vmcMsgCount  = 0;
    private static volatile int  nonVmcMsgCount = 0;
    private static final int MAX_ADDR_SAMPLES = 8;
    private static final Deque<String> recentAddrs = new ArrayDeque<>();

    public static final class Diagnostics {
        public final boolean running;
        public final long lastPacketMs;
        public final int totalPackets, vmcMsgCount, nonVmcMsgCount;
        public final List<String> recentAddresses;
        Diagnostics(boolean running, long lastPacketMs, int total, int vmc, int nonVmc, List<String> addrs){
            this.running = running; this.lastPacketMs = lastPacketMs;
            this.totalPackets = total; this.vmcMsgCount = vmc; this.nonVmcMsgCount = nonVmc;
            this.recentAddresses = addrs;
        }
    }
    public static Diagnostics getDiagnostics() {
        return new Diagnostics(
                isRunning(), lastPacketMs, totalPackets, vmcMsgCount, nonVmcMsgCount,
                new ArrayList<>(recentAddrs)
        );
    }

    // ───────────── 상태 표현 ─────────────
    /** 리플렉션을 위해 public 필드 사용: position(Vector3f), rotation(Quaternionf) */
    public static final class Transform {
        public final Vector3f position = new Vector3f();
        public final Quaternionf rotation = new Quaternionf();
    }

    /** 읽기용 스냅샷(불변) */
    public static final class Snapshot {
        public final Transform rootTransform;                    // nullable
        public final Map<String, Object> boneTransforms;         // Map<String, Transform>
        public final Map<String, Float>  blendShapes;
        public final long lastUpdateNanos;

        Snapshot(Transform root, Map<String, Object> bones, Map<String, Float> blends, long t) {
            this.rootTransform = root;
            this.boneTransforms = bones;
            this.blendShapes = blends;
            this.lastUpdateNanos = t;
        }
    }

    /** 더블버퍼 상태 보관 */
    private static final class StateHolder {
        private final ReentrantLock writeLock = new ReentrantLock();

        // write buffer (가변)
        private Transform rootW = null;
        private final Map<String, Transform> bonesW = new HashMap<>();
        private final Map<String, Float> blendsW = new HashMap<>();
        private final Map<String, Float> blendsPending = new HashMap<>();
        private long lastUpdateW = 0L;

        // read snapshot
        private final AtomicReference<Snapshot> readRef = new AtomicReference<>(null);

        void write(java.util.function.Consumer<StateHolder> block) {
            writeLock.lock();
            try {
                block.accept(this);
                // 커밋: 깊은 복사로 읽기 스냅샷 생성
                Transform rootR = null;
                if (rootW != null) {
                    rootR = new Transform();
                    rootR.position.set(rootW.position);
                    rootR.rotation.set(rootW.rotation);
                }
                Map<String, Object> bonesR = new HashMap<>(bonesW.size());
                for (Map.Entry<String, Transform> e : bonesW.entrySet()) {
                    Transform t = e.getValue();
                    Transform copy = new Transform();
                    copy.position.set(t.position);
                    copy.rotation.set(t.rotation);
                    bonesR.put(e.getKey(), copy);
                }
                Map<String, Float> blendsR = new HashMap<>(blendsW);
                readRef.set(new Snapshot(
                        rootR,
                        Collections.unmodifiableMap(bonesR),
                        Collections.unmodifiableMap(blendsR),
                        lastUpdateW
                ));
            } finally {
                writeLock.unlock();
            }
        }

        Snapshot read() { return readRef.get(); }

        // 수신 쓰기 도우미들 (write() 블록 안에서만 호출)
        void setRoot(float px, float py, float pz, float qx, float qy, float qz, float qw) {
            if (rootW == null) rootW = new Transform();
            rootW.position.set(px, py, pz);
            rootW.rotation.set(qx, qy, qz, qw).normalize();
            lastUpdateW = System.nanoTime();
        }

        void setBone(String name, float px, float py, float pz, float qx, float qy, float qz, float qw) {
            String key = normalizeBoneName(name);
            Transform t = bonesW.computeIfAbsent(key, k -> new Transform());
            t.position.set(px, py, pz);
            t.rotation.set(qx, qy, qz, qw).normalize();
            lastUpdateW = System.nanoTime();
        }

        void setBlendPending(String name, float v) {
            blendsPending.put(name, v);
        }

        void applyBlends() {
            if (!blendsPending.isEmpty()) {
                blendsW.putAll(blendsPending);
                blendsPending.clear();
                lastUpdateW = System.nanoTime();
            }
        }

        private static String normalizeBoneName(String src) {
            if (src == null || src.isEmpty()) return src;
            // "LeftUpperArm" -> "leftUpperArm", "Hips"->"hips" 등
            char c0 = Character.toLowerCase(src.charAt(0));
            return (src.length() == 1) ? String.valueOf(c0) : c0 + src.substring(1);
        }
    }

    // ───────────── UDP 수신 + OSC 파싱 ─────────────
    private static final class UdpLoop extends Thread implements Closeable {
        private final int port;
        private final StateHolder state;
        private volatile boolean closed = false;
        private DatagramSocket socket;

        UdpLoop(int port, StateHolder state) {
            super("VMC-UDP-Loop");
            setDaemon(true);
            this.port = port;
            this.state = state;
        }

        boolean isClosed() { return closed; }

        @Override public void run() {
            try {
                socket = new DatagramSocket(null);
                socket.setReuseAddress(true);
                socket.bind(new InetSocketAddress("0.0.0.0", port));
                socket.setReceiveBufferSize(1 << 20); // 1MB

                byte[] buf = new byte[65536];
                DatagramPacket pkt = new DatagramPacket(buf, buf.length);

                while (!closed) {
                    try {
                        socket.receive(pkt);
                    } catch (SocketException se) {
                        if (closed) break; // 정상 종료
                        System.out.println("[VMC] socket receive error: " + se);
                        continue;
                    }

                    final int len = pkt.getLength();
                    if (len <= 0) continue;

                    try {
                        processPacket(buf, len);
                    } catch (Throwable e) {
                        System.out.println("[VMC] decode error: " + e.getClass().getSimpleName());
                    }
                }
            } catch (BindException be) {
                System.out.println("[VMC] UDP bind failed on " + port + " (in use?): " + be);
                throw new RuntimeException(be);
            } catch (Exception e) {
                if (!closed) e.printStackTrace();
            } finally {
                if (socket != null) socket.close();
            }
        }

        private void processPacket(byte[] data, int len) {
            state.write(sh -> {
                Osc.decode(data, 0, len, (addr, args) -> {
                    // ── 진단 집계 ──
                    lastPacketMs = System.currentTimeMillis();
                    totalPackets++;
                    boolean isVmc = addr.startsWith("/VMC/Ext/");
                    if (isVmc) vmcMsgCount++; else nonVmcMsgCount++;
                    if (recentAddrs.size() >= MAX_ADDR_SAMPLES) recentAddrs.removeFirst();
                    recentAddrs.addLast(addr);

                    switch (addr) {
                        case "/VMC/Ext/Root/Pos":
                        case "/VMC/Ext/Root/Pos/Local": {
                            // ["root", px,py,pz, qx,qy,qz,qw]
                            if (args.length >= 8 && args[0] instanceof String) {
                                float px = f(args,1), py=f(args,2), pz=f(args,3);
                                float qx = f(args,4), qy=f(args,5), qz=f(args,6), qw=f(args,7);
                                sh.setRoot(px, py, pz, qx, qy, qz, qw);
                            }
                            break;
                        }
                        case "/VMC/Ext/Bone/Pos":
                        case "/VMC/Ext/Bone/Pos/Local": {
                            // [boneName, px,py,pz, qx,qy,qz,qw]
                            if (args.length >= 8 && args[0] instanceof String bone) {
                                float px = f(args,1), py=f(args,2), pz=f(args,3);
                                float qx = f(args,4), qy=f(args,5), qz=f(args,6), qw=f(args,7);
                                sh.setBone(bone, px, py, pz, qx, qy, qz, qw);
                            }
                            break;
                        }
                        case "/VMC/Ext/Blend/Val": {
                            if (args.length >= 2 && args[0] instanceof String name) {
                                sh.setBlendPending(name, f(args,1));
                            }
                            break;
                        }
                        case "/VMC/Ext/Blend/Apply": {
                            sh.applyBlends();
                            break;
                        }
                        default:
                            // 필요하면 여기서 VRChat OSC를 blends로 매핑할 수 있음:
                            // if (addr.startsWith("/avatar/parameters/")) { ... }
                            break;
                    }
                });
            });
        }

        private static float f(Object[] a, int i) {
            Object v = (i < a.length) ? a[i] : 0f;
            if (v instanceof Float) return (Float)v;
            if (v instanceof Double) return ((Double)v).floatValue();
            if (v instanceof Number) return ((Number)v).floatValue();
            return 0f;
        }

        @Override public void close() {
            closed = true;
            try { if (socket != null) socket.close(); } catch (Exception ignored) {}
        }
    }

    // ───────────── 아주 작은 OSC 파서(필요 타입만) ─────────────
    private static final class Osc {
        interface Handler { void onMessage(String address, Object[] args); }

        static void decode(byte[] buf, int off, int len, Handler h) {
            if (startsWith(buf, off, len, "#bundle")) {
                int p = off + padLen("#bundle");
                p += 8; // timetag skip
                while (p + 4 <= off + len) {
                    int elemSize = readInt(buf, p); p += 4;
                    if (elemSize <= 0 || p + elemSize > off + len) break;
                    decode(buf, p, elemSize, h);
                    p += elemSize;
                }
            } else {
                decodeMessage(buf, off, len, h);
            }
        }

        private static void decodeMessage(byte[] buf, int off, int len, Handler h) {
            int p = off;
            String address = readString(buf, p, off + len); if (address == null) return;
            p += padLen(address);

            String types = readString(buf, p, off + len);
            if (types == null || types.isEmpty() || types.charAt(0) != ',') return;
            p += padLen(types);

            List<Object> args = new ArrayList<>(Math.max(0, types.length() - 1));
            for (int i = 1; i < types.length(); i++) {
                char t = types.charAt(i);
                switch (t) {
                    case 's': {
                        String s = readString(buf, p, off + len); if (s == null) return;
                        args.add(s); p += padLen(s);
                        break;
                    }
                    case 'f': {
                        if (p + 4 > off + len) return;
                        float f = Float.intBitsToFloat(readInt(buf, p));
                        args.add(f); p += 4;
                        break;
                    }
                    case 'i': {
                        if (p + 4 > off + len) return;
                        int iv = readInt(buf, p);
                        args.add(iv); p += 4;
                        break;
                    }
                    default:
                        return;
                }
            }
            h.onMessage(address, args.toArray());
        }

        private static boolean startsWith(byte[] b, int off, int len, String s) {
            byte[] a = s.getBytes(StandardCharsets.US_ASCII);
            if (len < a.length) return false;
            for (int i = 0; i < a.length; i++) if (b[off + i] != a[i]) return false;
            return true;
        }
        private static int readInt(byte[] b, int p) {
            return ((b[p] & 0xFF) << 24)
                    | ((b[p+1] & 0xFF) << 16)
                    | ((b[p+2] & 0xFF) << 8)
                    |  (b[p+3] & 0xFF);
        }
        private static String readString(byte[] b, int p, int end) {
            int q = p;
            while (q < end && b[q] != 0) q++;
            if (q >= end) return null;
            return new String(b, p, q - p, StandardCharsets.US_ASCII);
        }
        private static int padLen(String s) {
            int n = s.getBytes(StandardCharsets.US_ASCII).length + 1;
            int pad = (4 - (n % 4)) & 3;
            return n + pad;
        }
    }
}
