import { SerialPort } from "serialport";
import { ReadlineParser } from "@serialport/parser-readline";
import { WebSocketServer, WebSocket } from "ws";

console.log("FFFF NEW SERVER CODE IS RUNNING!!");

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

/* ===== WebSocket 서버 ===== */
const wss = new WebSocketServer({ port: 8765 });
const clients = new Set();

function broadcast(data) {
  const msg = JSON.stringify(data);

  console.log("📡 broadcast:", msg);

  for (const c of clients) {
    try {
      if (c.readyState === WebSocket.OPEN) {
        c.send(msg);
      }
    } catch (err) {
      console.log("❌ broadcast error:", err?.message || err);
    }
  }
}

/* ===== Arduino Serial 연결 ===== */
const port = new SerialPort({
  path: SERIAL_PATH,
  baudRate: BAUD_RATE,
  autoOpen: false,
});

const parser = port.pipe(new ReadlineParser({ delimiter: "\n" }));

port.open((err) => {
  if (err) {
    console.log("❌ Serial open error:", err.message);
    return;
  }

  console.log("✅ Arduino Serial 연결됨:", SERIAL_PATH);
});

parser.on("data", (line) => {
  const text = String(line).trim();

  if (!text) return;

  console.log("📥 SERIAL:", text);

  /* ===== 1. 소음 처리 ===== */
  if (text.includes("SOUND:")) {
    const valueText = text.split("SOUND:")[1].trim();
    const value = Number(valueText);

    if (!Number.isNaN(value)) {
      broadcast({
        type: "sound",
        value,
        ts: new Date().toISOString(),
      });

      return;
    }
  }

  /* ===== 2. RFID UID 처리 ===== */
  if (text.includes("UID:")) {
    const uid = text
      .split("UID:")[1]
      .trim()
      .replace(/\s/g, "")
      .toUpperCase();

    const admin = ADMIN_CARDS[uid];
    const worker = WORKER_CARDS[uid];

    if (admin) {
      broadcast({
        type: "admin",
        uid,
        ...admin,
        nonce: Date.now(),
      });

      return;
    }

    if (worker) {
      broadcast({
        type: "worker",
        uid,
        ...worker,
        nonce: Date.now(),
      });

      return;
    }

    broadcast({
      type: "unknown",
      uid,
      message: `미등록 카드 감지 (${uid})`,
      nonce: Date.now(),
    });

    return;
  }

  if (text.includes("UID SIZE:")) return;

  if (text.includes("SYSTEM READY")) {
    broadcast({
      type: "server",
      status: "arduino_ready",
      ts: new Date().toISOString(),
    });

    return;
  }

  console.log("🔇 SERIAL ignored:", text);
});

/* ===== UI WebSocket ===== */
wss.on("connection", (ws) => {
  clients.add(ws);

  console.log(`🟢 UI 연결됨 (${clients.size})`);

  ws.send(
    JSON.stringify({
      type: "server",
      status: "connected",
      ts: new Date().toISOString(),
    })
  );

  ws.on("close", () => {
    clients.delete(ws);
    console.log(`⚪ UI 연결 해제 (${clients.size})`);
  });

  ws.on("error", (err) => {
    clients.delete(ws);
    console.log("❌ WS ERROR:", err?.message || err);
  });
});

console.log("✅ RFID 서버 실행됨");
console.log("🌐 ws://localhost:8765");
console.log(`🧪 SERIAL_PATH = ${SERIAL_PATH}`);
console.log(`🧪 BAUD_RATE = ${BAUD_RATE}`);