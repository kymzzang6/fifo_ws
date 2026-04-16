import { SerialPort } from "serialport";
import { WebSocketServer, WebSocket } from "ws";

/* ===== 카드 UID 등록 ===== */
const ADMIN_CARDS = {
  "01D05005": { id: "A-001", name: "관리자", role: "관리자" },
};

const WORKER_CARDS = {
  "C790F665": { id: "W-001", name: "작업자1", role: "작업자" },
  "0B1AC201": { id: "W-002", name: "작업자2", role: "작업자" },
};

/* ===== 설정 ===== */
const SERIAL_PATH = process.env.RFID_PORT || "COM13";
const BAUD_RATE = Number(process.env.RFID_BAUD || 9600);
const COOLDOWN_MS = Number(process.env.RFID_COOLDOWN || 400);

const ENABLE_DTR_RESET =
  String(process.env.RFID_DTR_RESET || "false").toLowerCase() === "true";

const DEBUG =
  String(process.env.RFID_DEBUG || "true").toLowerCase() === "true";

/* ===== ROS2 rosbridge 설정 ===== */
const ROS_BRIDGE_URL = process.env.ROS_BRIDGE_URL || "ws://localhost:9090";
const ROS_TOPIC_ALL_DETECTED = "/all_detected";
const ROS_TOPIC_MODE = "/mode_command";
const ROS_TOPIC_TELEOP = "/teleop_cmd";
const ROS_TOPIC_TOWER = "/tower_command";

/* ===== WebSocket 서버 ===== */
const wss = new WebSocketServer({ port: 8765 });
const clients = new Set();

/* ===== rosbridge client ===== */
let rosWs = null;
let rosConnected = false;
let rosMsgId = 1;
const advertisedTopics = new Set();

function safeSend(ws, data) {
  try {
    if (ws.readyState === 1) {
      ws.send(JSON.stringify(data));
      return true;
    }
    return false;
  } catch (err) {
    console.log("❌ WS send error:", err?.message || err);
    return false;
  }
}

function broadcast(data) {
  const msg = JSON.stringify(data);

  if (DEBUG) {
    console.log("📡 broadcast:", msg);
    console.log("📡 broadcast 대상 clients:", clients.size);
  }

  for (const c of clients) {
    try {
      if (c.readyState === 1) c.send(msg);
    } catch (err) {
      console.log("❌ broadcast error:", err?.message || err);
    }
  }
}

function rosSend(obj) {
  if (!rosWs || rosWs.readyState !== WebSocket.OPEN) {
    console.log("❌ rosbridge 미연결");
    return false;
  }
  rosWs.send(JSON.stringify(obj));
  return true;
}

function advertiseTopic(topic, type) {
  if (advertisedTopics.has(topic)) return true;

  const ok = rosSend({
    op: "advertise",
    topic,
    type,
  });

  if (ok) {
    advertisedTopics.add(topic);
    console.log(`📢 advertise: ${topic} (${type})`);
  }
  return ok;
}

function publishRos(topic, type, msg) {
  if (!rosConnected) {
    console.log(`❌ publish 실패 - rosbridge 미연결: ${topic}`);
    return false;
  }

  advertiseTopic(topic, type);

  const ok = rosSend({
    op: "publish",
    topic,
    msg,
  });

  if (ok && DEBUG) {
    console.log(`🚀 ROS publish -> ${topic}`, msg);
  }
  return ok;
}

function connectRosbridge() {
  if (rosWs && (rosWs.readyState === WebSocket.OPEN || rosWs.readyState === WebSocket.CONNECTING)) {
    return;
  }

  console.log(`🔌 rosbridge 연결 시도: ${ROS_BRIDGE_URL}`);
  rosWs = new WebSocket(ROS_BRIDGE_URL);

  rosWs.on("open", () => {
    rosConnected = true;
    advertisedTopics.clear();
    console.log("✅ rosbridge 연결됨");

    broadcast({
      type: "rosbridge",
      status: "open",
      ts: new Date().toISOString(),
    });

    // /all_detected 구독
    rosSend({
      op: "subscribe",
      id: `sub-${rosMsgId++}`,
      topic: ROS_TOPIC_ALL_DETECTED,
      type: "std_msgs/msg/Bool",
    });
  });

  rosWs.on("message", (raw) => {
    try {
      const msg = JSON.parse(raw.toString());

      if (msg.op === "publish" && msg.topic === ROS_TOPIC_ALL_DETECTED) {
        broadcast({
          type: "ppe_all_detected",
          value: Boolean(msg?.msg?.data),
          ts: new Date().toISOString(),
        });
      }
    } catch (err) {
      console.log("❌ rosbridge message parse error:", err?.message || err);
    }
  });

  rosWs.on("close", () => {
    rosConnected = false;
    advertisedTopics.clear();
    console.log("❌ rosbridge 연결 종료");

    broadcast({
      type: "rosbridge",
      status: "closed",
      ts: new Date().toISOString(),
    });

    setTimeout(connectRosbridge, 1000);
  });

  rosWs.on("error", (err) => {
    rosConnected = false;
    console.log("❌ rosbridge error:", err?.message || err);
  });
}

/* ===== UI WebSocket ===== */
wss.on("connection", (ws, req) => {
  clients.add(ws);

  const ip =
    req?.socket?.remoteAddress ||
    req?.headers?.["x-forwarded-for"] ||
    "unknown";

  console.log(`🟢 UI 연결됨 (clients: ${clients.size}, ip: ${ip})`);

  safeSend(ws, {
    type: "server",
    status: "connected",
    ts: new Date().toISOString(),
    message: "WebSocket 연결 성공",
  });

  safeSend(ws, {
    type: "rosbridge",
    status: rosConnected ? "open" : "closed",
    ts: new Date().toISOString(),
  });

  ws.on("message", (raw) => {
    try {
      const msg = JSON.parse(raw.toString());

      if (DEBUG) {
        console.log("💬 WS <- UI:", msg);
      }

      // 1) 모드 전환
      if (msg?.type === "set_mode" && typeof msg?.mode === "string") {
        publishRos(
          ROS_TOPIC_MODE,
          "std_msgs/msg/String",
          { data: msg.mode }
        );

        broadcast({
          type: "robot_mode",
          mode: msg.mode,
          ts: new Date().toISOString(),
        });
        return;
      }

      // 2) 수동 주행
      if (msg?.type === "cmd_vel") {
        const linearX = Number(msg?.linear_x ?? 0);
        const angularZ = Number(msg?.angular_z ?? 0);

        publishRos(
          ROS_TOPIC_TELEOP,
          "geometry_msgs/msg/Twist",
          {
            linear: { x: linearX, y: 0.0, z: 0.0 },
            angular: { x: 0.0, y: 0.0, z: angularZ },
          }
        );
        return;
      }

      // 3) 타워램프
      if (msg?.type === "tower_command") {
        const payload = {
          color: String(msg?.color ?? ""),
          pattern: String(msg?.pattern ?? ""),
          duration: Number(msg?.duration ?? 0),
        };

        publishRos(
          ROS_TOPIC_TOWER,
          "std_msgs/msg/String",
          { data: JSON.stringify(payload) }
        );
        return;
      }
    } catch (err) {
      console.log("❌ UI message parse error:", err?.message || err);
    }
  });

  ws.on("close", (code, reason) => {
    clients.delete(ws);
    console.log(
      `⚪ UI 연결 해제 (clients: ${clients.size}, code: ${code}, reason: ${reason?.toString?.() || ""})`
    );
  });

  ws.on("error", (err) => {
    clients.delete(ws);
    console.log("❌ WS ERROR:", err?.message || err);
  });
});

/* ===== 시작 ===== */
connectRosbridge();

console.log(`✅ RFID 서버 실행됨 (ws://localhost:8765)`);
console.log(`🧪 DEBUG = ${DEBUG}`);
console.log(`🧪 SERIAL_PATH = ${SERIAL_PATH}`);
console.log(`🧪 BAUD_RATE = ${BAUD_RATE}`);
console.log(`🧪 COOLDOWN_MS = ${COOLDOWN_MS}`);
console.log(`🧪 ENABLE_DTR_RESET = ${ENABLE_DTR_RESET}`);
console.log(`🧪 ROS_BRIDGE_URL = ${ROS_BRIDGE_URL}`);
console.log(`🧪 ROS_TOPIC_ALL_DETECTED = ${ROS_TOPIC_ALL_DETECTED}`);
console.log(`🧪 ROS_TOPIC_MODE = ${ROS_TOPIC_MODE}`);
console.log(`🧪 ROS_TOPIC_TELEOP = ${ROS_TOPIC_TELEOP}`);
console.log(`🧪 ROS_TOPIC_TOWER = ${ROS_TOPIC_TOWER}`);