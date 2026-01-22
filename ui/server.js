import { SerialPort } from "serialport";
import { WebSocketServer } from "ws";

/* ===== 카드 UID 등록 ===== */
const ADMIN_CARDS = {
  "45 45 07 AD": { id: "A-001", name: "관리자", role: "관리자" },
};

const WORKER_CARDS = {
  "C7 90 F6 65": { id: "W-001", name: "작업자1", role: "작업자" },
  "0B 1A C2 01": { id: "W-002", name: "작업자2", role: "작업자" },
};

/* ===== 설정 ===== */
const SERIAL_PATH = process.env.RFID_PORT || "COM9";
const BAUD_RATE = Number(process.env.RFID_BAUD || 9600);

// ✅ 같은 UID가 너무 짧은 시간에 여러 번 들어오는 걸 막는 쿨다운(밀리초)
// 200~700ms 사이로 보통 맞춤
const COOLDOWN_MS = Number(process.env.RFID_COOLDOWN || 400);

/* ===== WebSocket 서버 ===== */
const wss = new WebSocketServer({ port: 8765 });
const clients = new Set();

wss.on("connection", (ws) => {
  clients.add(ws);
  console.log("🟢 UI 연결됨 (clients:", clients.size, ")");

  ws.on("close", () => {
    clients.delete(ws);
    console.log("⚪ UI 연결 해제 (clients:", clients.size, ")");
  });

  ws.on("error", () => {
    clients.delete(ws);
  });
});

function broadcast(data) {
  const msg = JSON.stringify(data);
  for (const c of clients) {
    try {
      c.send(msg);
    } catch {}
  }
}

/* ===== (중요) 사용 가능한 포트 출력 ===== */
async function showPorts() {
  const ports = await SerialPort.list();
  console.log("\n📌 Available serial ports:");
  if (!ports.length) {
    console.log("  (none)");
    return;
  }
  for (const p of ports) {
    console.log(`- ${p.path} ${p.manufacturer ?? ""}`.trim());
  }
  console.log("");
}

/* ===== Arduino Serial 연결 + 수신 파싱 ===== */
let port = null;
let buffer = "";

// ✅ UID별 마지막 처리 시각 기록(중복 폭주 방지)
const lastSeenAt = new Map();

function normalizeUidLine(line) {
  const cleaned = line.replace(/\r/g, "").trim();
  if (/^[0-9A-Fa-f]{8}$/.test(cleaned)) {
    return cleaned.match(/.{1,2}/g).join(" ").toUpperCase();
  }
  return cleaned.toUpperCase();
}

// ✅ 이 이벤트는 고유하다는 것을 보장해주는 nonce (UI에서 sessionKey로 쓰기 좋음)
function makeNonce() {
  return `${Date.now()}-${Math.random().toString(16).slice(2)}`;
}

function handleUid(uid) {
  const now = Date.now();

  //  1) 너무 짧은 시간 중복은 서버에서 컷 (리더기 노이즈/중복 출력 방지)
  const last = lastSeenAt.get(uid) ?? 0;
  if (now - last < COOLDOWN_MS) {
    // 필요하면 디버깅용 로그
    // console.log("↩︎ (cooldown skip)", uid);
    return;
  }
  lastSeenAt.set(uid, now);

  //  2) UI가 매번 새 이벤트로 인식하도록 nonce 포함해서 보냄
  const nonce = makeNonce();
  console.log("📟 UID 수신:", uid, "(nonce:", nonce, ")");

  // 3) 분기 + 보안 멘트/코드 포함
  if (ADMIN_CARDS[uid]) {
    broadcast({
      type: "admin",
      uid,
      nonce,
      ...ADMIN_CARDS[uid],
      code: "ADMIN_CARD",
      severity: "info",
      displayMessage: "관리자 카드 인식",
    });
    return;
  }

  if (WORKER_CARDS[uid]) {
    const w = WORKER_CARDS[uid];
    broadcast({
      type: "worker",
      uid,
      nonce,
      ...w,
      code: "WORKER_CARD",
      severity: "info",
      displayMessage: `${w.name} 카드 인식`,
    });
    return;
  }

  // ✅ unknown: 강한 보안 로그 + UI에 문구 전달
  console.warn("🚨 [SECURITY] 미등록 카드 감지:", {
    uid,
    at: new Date().toISOString(),
    clients: clients.size,
  });

  broadcast({
    type: "unknown",
    uid,
    nonce,
    code: "UNREGISTERED_CARD",
    severity: "high",
    displayMessage: "🚨 미등록 카드입니다. 접근이 차단되었습니다. 관리자에게 문의하세요.",
  });
}

function connectSerial() {
  console.log(`🔌 Serial 연결 시도: ${SERIAL_PATH} @ ${BAUD_RATE}`);

  port = new SerialPort({
    path: SERIAL_PATH,
    baudRate: BAUD_RATE,
    autoOpen: true,
  });

port.on("open", () => {
  console.log("✅ Serial OPEN:", SERIAL_PATH);

  // 🔥 Arduino 강제 리셋 (IDE에서 업로드한 효과)
  port?.set({ dtr: false }, () => {
    setTimeout(() => {
      port?.set({ dtr: true });
    }, 120);
  });

  broadcast({ type: "serial", status: "open", path: SERIAL_PATH });
});


  port.on("error", (err) => {
    console.log("❌ Serial ERROR:", err?.message || err);
    broadcast({ type: "serial", status: "error", message: String(err?.message || err) });
  });

  port.on("close", () => {
    console.log("⚠️ Serial CLOSED. 1초 후 재연결 시도");
    broadcast({ type: "serial", status: "closed" });
    setTimeout(() => connectSerial(), 1000);
  });

  port.on("data", (data) => {
  //  0) 들어온 원본 바이트/문자열을 무조건 찍기
  const asUtf8 = data.toString("utf8");
  const asHex = Buffer.from(data).toString("hex").match(/.{1,2}/g)?.join(" ") ?? "";

  console.log("RAW(utf8):", JSON.stringify(asUtf8));
  console.log("RAW(hex) :", asHex);

  //  1) 버퍼에 누적
  buffer += asUtf8;

  //  2) 줄바꿈 기준 분리(Arduino가 \r\n 이면 \n으로 split 가능)
  const lines = buffer.split(/\n/);
  buffer = lines.pop() ?? "";

  //  3) 줄 단위로 찍고 normalize 후 handleUid
  for (const rawLine of lines) {
    const trimmed = rawLine.replace(/\r/g, "").trim();
    console.log("LINE:", JSON.stringify(trimmed));

    const uid = normalizeUidLine(trimmed);
    console.log("UID(normalized):", uid);

    if (!uid) continue;
    handleUid(uid);
  }
});

}

/* ===== 실행 (Top-level await 안전 버전) ===== */
(async () => {
  console.log("✅ RFID 서버 실행됨 (ws://localhost:8765)");
  await showPorts();
  connectSerial();
})();
