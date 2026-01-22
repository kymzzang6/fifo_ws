import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { Card, CardContent } from "./components/ui/card";
import { Button } from "./components/ui/button";
import { Progress } from "./components/ui/progress";
import { Bell } from "lucide-react";
import { AnimatePresence, motion } from "framer-motion";
import { handleRFID } from "./rfid/rfidHandler";

/* ───────────────────────── URL 설정 ───────────────────────── */

const YOLO_STREAM_URL =
  "http://localhost:8080/stream?topic=/debug/visualization&type=mjpeg";

const THERMO_STREAM_URL = "http://localhost:8090/thermo?type=mjpeg";

// ✅ 토큰은 .env로 분리 (보안)
const WS_URL =
  (import.meta as any).env?.VITE_HA_WS_URL ??
  "ws://homeassistant.local:8123/api/websocket";
const HA_TOKEN = (import.meta as any).env?.VITE_HA_TOKEN ?? "";

/* ───────────────────────── 타입 ───────────────────────── */

type HazardStatus = "open" | "resolved" | "false_alarm";

// pending: 자동 점검 중(대기) / active: 근무중 / blocked: 보류
type WorkerStatus = "pending" | "active" | "blocked";

type Worker = {
  id: string;
  uid?: string;
  name: string;
  role?: string;

  status: WorkerStatus;
  checkinStartedAt: number;
  sessionKey: string;
  failReasons?: string[];
  warnings?: string[];
};

type Hazard = {
  id: string;
  x: number; // 0~100 (%)
  y: number; // 0~100 (%)
  status: HazardStatus;
  note?: string;
};

/* ───────────────────────── 유틸 ───────────────────────── */

function formatTime(date: Date): string {
  const h = date.getHours().toString().padStart(2, "0");
  const m = date.getMinutes().toString().padStart(2, "0");
  return `${h}:${m}`;
}

function clamp(n: number, min: number, max: number) {
  return Math.max(min, Math.min(max, n));
}

// ✅ 고정키 생성 (중복 방지)
function makeWorkerKey(uid?: string, id?: string) {
  const u = (uid ?? "").trim();
  const i = (id ?? "").trim();
  return u || i; // uid가 있으면 uid 우선, 없으면 id
}

function upsertWorker(prev: Worker[], next: Worker) {
  const key = next.sessionKey;
  const idx = prev.findIndex((w) => w.sessionKey === key);
  if (idx >= 0) {
    const updated = [...prev];
    updated[idx] = { ...updated[idx], ...next };
    return updated;
  }
  return [next, ...prev];
}

/* ───────────────────────── 메인 ───────────────────────── */

export default function App() {
  /* ─── 모달 ON/OFF ───────────────────────────────────── */
  const [vitalsOpen, setVitalsOpen] = useState(false);
  const [workerModalOpen, setWorkerModalOpen] = useState(false);
  const [hazardModalOpen, setHazardModalOpen] = useState(false);
  const [robotModalOpen, setRobotModalOpen] = useState(false);
  

  /* ─── 현재 시간 ──────────────────────────────────────── */
  const [now, setNow] = useState<Date>(new Date());
  useEffect(() => {
    const timer = window.setInterval(() => setNow(new Date()), 1000 * 30);
    return () => window.clearInterval(timer);
  }, []);

  /* ─── 토스트 ─────────────────────────────────────────── */
  const [toast, setToast] = useState<string | null>(null);
  const toastTimerRef = useRef<number | null>(null);

  const showToast = useCallback((msg: string) => {
    setToast(msg);
    if (toastTimerRef.current) window.clearTimeout(toastTimerRef.current);
    toastTimerRef.current = window.setTimeout(() => setToast(null), 1600);
  }, []);

  useEffect(() => {
    return () => {
      if (toastTimerRef.current) window.clearTimeout(toastTimerRef.current);
    };
  }, []);

  /* ─── 관리자 권한 (10초) ─────────────────────────────── */
  const [adminUntil, setAdminUntil] = useState<number>(0);
  const isAdmin = adminUntil > Date.now();

  const grantAdminFor10s = useCallback(() => {
    setAdminUntil(Date.now() + 10_000);
  }, []);

  // ✅ 카운트다운 표시용 tick (리렌더 트리거)
  const [adminTick, setAdminTick] = useState(0);
  useEffect(() => {
    if (!isAdmin) return;
    const t = window.setInterval(() => setAdminTick((x) => x + 1), 250);
    return () => window.clearInterval(t);
  }, [isAdmin]);

  const adminRemainSec = useMemo(() => {
    void adminTick;
    return Math.max(0, Math.ceil((adminUntil - Date.now()) / 1000));
  }, [adminUntil, adminTick]);

  /* ─── mmWave / 생체값 ───────────────────────────────── */
  const [resp, setResp] = useState<number | null>(null);
  const [hr, setHr] = useState<number | null>(null);
  const [conf, setConf] = useState<number | null>(null);

  const [connected, setConnected] = useState(false);
  const [respHistory, setRespHistory] = useState<number[]>([]);
  const [hrHistory, setHrHistory] = useState<number[]>([]);

  const [mmwaveEnabled, setMmwaveEnabled] = useState<boolean | null>(null);
  const [hasTarget, setHasTarget] = useState<boolean | null>(null);

  /* ─── Thermoeye ─────────────────────────────────────── */
  const [bodyTemp, setBodyTemp] = useState<number | null>(null);

  // ✅ HA 연결과 스트림 연결 분리
  const [thermoDeviceConnected, setThermoDeviceConnected] =
    useState<boolean>(false);
  const [thermoStreamConnected, setThermoStreamConnected] =
    useState<boolean>(false);

  // ✅ YOLO도 스트림 연결로 통일
  const [yoloStreamConnected, setYoloStreamConnected] =
    useState<boolean>(false);

  // ✅ 파생 상태 (기존 thermoConnected / yoloConnected 대체)
  const thermoConnected = thermoDeviceConnected || thermoStreamConnected;
  const yoloConnected = yoloStreamConnected;

  /* ─── PPE 상태 ─────────────────── */
  const [ppeHelmet, setPpeHelmet] = useState<boolean>(true);
  const [ppeVest, setPpeVest] = useState<boolean>(true);
  const [ppeGloves, setPpeGloves] = useState<boolean>(false);

  /* ─── 임계값 ────────────────────────────────────────── */
  const [respMin, setRespMin] = useState(8);
  const [respMax, setRespMax] = useState(24);
  const [hrMin, setHrMin] = useState(50);
  const [hrMax, setHrMax] = useState(120);

  const [tempWarn, setTempWarn] = useState(37.5);
  const [tempDanger, setTempDanger] = useState(38.5);

  // 입력은 string으로
  const [respMinStr, setRespMinStr] = useState(String(respMin));
  const [respMaxStr, setRespMaxStr] = useState(String(respMax));
  const [hrMinStr, setHrMinStr] = useState(String(hrMin));
  const [hrMaxStr, setHrMaxStr] = useState(String(hrMax));
  const [tempWarnStr, setTempWarnStr] = useState(String(tempWarn));
  const [tempDangerStr, setTempDangerStr] = useState(String(tempDanger));

  useEffect(() => setRespMinStr(String(respMin)), [respMin]);
  useEffect(() => setRespMaxStr(String(respMax)), [respMax]);
  useEffect(() => setHrMinStr(String(hrMin)), [hrMin]);
  useEffect(() => setHrMaxStr(String(hrMax)), [hrMax]);
  useEffect(() => setTempWarnStr(String(tempWarn)), [tempWarn]);
  useEffect(() => setTempDangerStr(String(tempDanger)), [tempDanger]);

  /* ─── 데이터 ─────────────────────────────────────────── */
  const [workers, setWorkers] = useState<Worker[]>([
    {
      id: "W-001",
      name: "홍길동",
      role: "전기",
      status: "active",
      checkinStartedAt: Date.now(),
      sessionKey: makeWorkerKey(undefined, "W-001") || "W-001",
      failReasons: [],
      warnings: [],
    },
  ]);

  const [hazards, setHazards] = useState<Hazard[]>([
    { id: "H-1", x: 24, y: 50, status: "open" },
    { id: "H-2", x: 64, y: 30, status: "open" },
    { id: "H-3", x: 78, y: 66, status: "resolved" },
  ]);
  const [selectedHazard, setSelectedHazard] = useState<Hazard | null>(null);

  /* ─── 작업자 입력 폼 ──────────────────────────────────── */
  const [form, setForm] = useState<{ id: string; name: string; role?: string }>(
    { id: "", name: "", role: "" }
  );

  /* ───────────────────────── HA WebSocket ───────────────────────── */

  useEffect(() => {
    if (!HA_TOKEN) {
      console.warn("VITE_HA_TOKEN이 비어있음 (.env 확인)");
      return;
    }

    let ws: WebSocket | null = null;
    let msgId = 1;

    const processState = (entity: any) => {
      if (!entity) return;
      const entityId: string = entity.entity_id;
      const stateStr: string = entity.state;

      // mmWave 호흡
      if (
        entityId ===
        "sensor.seeedstudio_mr60bha2_kit_613f70_real_time_respiratory_rate"
      ) {
        const v = parseFloat(stateStr);
        if (!Number.isNaN(v)) {
          setResp(v);
          setRespHistory((p) => [...p.slice(-99), v]);
        }
      }

      // mmWave 심박
      if (
        entityId ===
        "sensor.seeedstudio_mr60bha2_kit_613f70_real_time_heart_rate"
      ) {
        const v = parseFloat(stateStr);
        if (!Number.isNaN(v)) {
          setHr(v);
          setHrHistory((p) => [...p.slice(-99), v]);
        }
      }

      // mmWave on/off
      if (entityId === "input_boolean.mmwave_enabled") {
        setMmwaveEnabled(stateStr === "on");
      }

      // 타겟 감지
      if (
        entityId ===
        "binary_sensor.seeedstudio_mr60bha2_kit_613f70_mr60bha2_has_target"
      ) {
        setHasTarget(stateStr === "on");
      }

      // (선택) conf 엔티티
      if (entityId === "sensor.mmwave_confidence") {
        const v = parseFloat(stateStr);
        if (!Number.isNaN(v)) setConf(v);
      }

      // Thermoeye 체온
      if (entityId === "sensor.thermoeye_body_temperature") {
        const v = parseFloat(stateStr);
        if (!Number.isNaN(v)) setBodyTemp(v);
      }

      // Thermoeye 장치 연결(HA)
      if (entityId === "binary_sensor.thermoeye_connected") {
        setThermoDeviceConnected(stateStr === "on");
      }

      // PPE (있으면 연동)
      if (entityId === "binary_sensor.ppe_helmet") setPpeHelmet(stateStr === "on");
      if (entityId === "binary_sensor.ppe_vest") setPpeVest(stateStr === "on");
      if (entityId === "binary_sensor.ppe_gloves") setPpeGloves(stateStr === "on");
    };

    try {
      ws = new WebSocket(WS_URL);

      ws.onclose = () => setConnected(false);
      ws.onerror = () => setConnected(false);

      ws.onmessage = (ev) => {
        try {
          const msg = JSON.parse(ev.data);

          if (msg.type === "auth_required") {
            ws?.send(JSON.stringify({ type: "auth", access_token: HA_TOKEN }));
            return;
          }

          if (msg.type === "auth_ok") {
            setConnected(true);

            ws?.send(
              JSON.stringify({
                id: msgId++,
                type: "subscribe_events",
                event_type: "state_changed",
              })
            );

            ws?.send(JSON.stringify({ id: msgId++, type: "get_states" }));
            return;
          }

          if (msg.type === "auth_invalid") {
            setConnected(false);
            ws?.close();
            return;
          }

          if (msg.type === "result" && msg.success && Array.isArray(msg.result)) {
            msg.result.forEach((st: any) => processState(st));
            return;
          }

          if (msg.type === "event" && msg.event?.event_type === "state_changed") {
            const newState = msg.event.data?.new_state;
            processState(newState);
            return;
          }
        } catch {
          // ignore
        }
      };
    } catch {
      setConnected(false);
    }

    return () => {
      if (ws) ws.close();
    };
  }, []);

  /* ───────────────────────── RFID WebSocket ───────────────────────── */
  useEffect(() => {
    let rws: WebSocket | null = null;
    let retryTimer: number | null = null;
    let stopped = false;

    const connect = () => {
      if (stopped) return;

      rws = new WebSocket("ws://127.0.0.1:8765");

      rws.onopen = () => {
        console.log("✅ RFID WS connected");
      };

rws.onmessage = (ev) => {
  try {
    const raw = JSON.parse(ev.data);

    // ✅ 공통 uid 추출 (unknown 포함)
    const uid =
      raw?.uid ?? raw?.tag ?? raw?.nuid ?? raw?.card_uid ?? "";

    // ✅ unknown 카드면 보안 멘트 + 종료
    if (raw?.type === "unknown") {
      // Node에서 displayMessage 내려주면 그걸 우선 사용
      const msg =
        raw?.displayMessage ??
        raw?.message ??
        `미등록 카드 감지 (${uid}) — 관리자에게 문의하세요.`;

      showToast(msg);
      return;
    }

    // ✅ 그 외(admin/worker/serial 등) 정규화
    const data = {
      type: raw?.type, // "admin" | "worker" 등
      uid,
      id: raw?.id,
      name: raw?.name,
      role: raw?.role,
      nonce: raw?.nonce,
    };

    console.log("RFID normalized:", data);

    // ✅ serial 상태 메시지는 handleRFID로 보내지 않는 게 안전 (원하면 토스트만)
    if (data.type === "serial") {
      // 예: showToast(`Serial: ${raw?.status ?? "unknown"}`);
      return;
    }

    handleRFID({
      data,
      showToast,
      setVitalsOpen,
      setForm,
      setWorkers,
      setPassStreak,
      onAdmin: grantAdminFor10s,
    });
  } catch (err) {
    console.warn("RFID message parse fail:", err, ev.data);
  }
};

      rws.onerror = (e) => {
        console.log("❌ RFID WS error", e);
      };

      rws.onclose = () => {
        console.log("❌ RFID WS disconnected");
        if (!stopped) {
          retryTimer = window.setTimeout(connect, 1000);
        }
      };
    };

    connect();

    return () => {
      stopped = true;
      if (retryTimer) window.clearTimeout(retryTimer);
      if (rws) rws.close();
    };
  }, [grantAdminFor10s, showToast]);

  /* ───────────────────────── 연결 감지(스트림 이미지 로드 기반) ───────────────────────── */
  const yoloImgRef = useRef<HTMLImageElement | null>(null);
  const thermoImgRef = useRef<HTMLImageElement | null>(null);

  /* ───────────────────────── 임계 기반 색상 ───────────────────────── */
  const respColor =
    resp == null
      ? "bg-gray-500"
      : resp < respMin || resp > respMax
      ? "bg-red-500"
      : "bg-emerald-500";

  const hrColor =
    hr == null
      ? "bg-gray-500"
      : hr < hrMin || hr > hrMax
      ? "bg-red-500"
      : "bg-emerald-500";

  const tempColor =
    bodyTemp == null
      ? "bg-gray-500"
      : bodyTemp >= tempDanger
      ? "bg-red-500"
      : bodyTemp >= tempWarn
      ? "bg-amber-500"
      : "bg-emerald-500";

  /* ───────────────────────── 자동 점검 ───────────────────────── */

  const CHECK_INTERVAL_MS = 1000;
  const CHECK_TIMEOUT_MS = 20000;
  const PASS_STREAK_REQUIRED = 3;

  // ✅ 실패하면 즉시 보류로 보낼지, 즉시 보류 금지: TIMEOUT 때만 보류
  const FAIL_IMMEDIATE_BLOCK = false;

  const [passStreak, setPassStreak] = useState<Record<string, number>>({});
  const passStreakRef = useRef<Record<string, number>>({});
  useEffect(() => {
    passStreakRef.current = passStreak;
  }, [passStreak]);

  const evaluateGate = useCallback(() => {
    const fails: string[] = [];
    const warns: string[] = [];

    // PPE
    if (!ppeHelmet) fails.push("Helmet 미착용");
    if (!ppeVest) fails.push("Vest 미착용");
    if (!ppeGloves) fails.push("Gloves 미착용");

    // Thermo (장치 OR 스트림)
    if (!(thermoDeviceConnected || thermoStreamConnected))
      fails.push("Thermoeye 미연결");

    if (bodyTemp == null) fails.push("체온 데이터 없음");
    else {
      if (bodyTemp >= tempDanger)
        fails.push(`체온 위험(${bodyTemp.toFixed(1)}°C)`);
      else if (bodyTemp >= tempWarn)
        warns.push(`체온 경고(${bodyTemp.toFixed(1)}°C)`);
    }

    // mmWave (엄격)
    if (mmwaveEnabled !== true) fails.push("mmWave OFF/미확인");
    if (hasTarget !== true) fails.push("mmWave 타겟 미감지/미확인");

    if (resp == null) fails.push("호흡 데이터 없음");
    else if (resp < respMin || resp > respMax)
      fails.push(`호흡 비정상(${resp.toFixed(1)})`);

    if (hr == null) fails.push("심박 데이터 없음");
    else if (hr < hrMin || hr > hrMax)
      fails.push(`심박 비정상(${hr.toFixed(0)})`);

    // (선택) YOLO 스트림도 게이트에 넣고 싶으면 아래 주석 해제
    // if (!yoloStreamConnected) fails.push("YOLO 미연결");

    return { pass: fails.length === 0, fails, warns };
  }, [
    ppeHelmet,
    ppeVest,
    ppeGloves,
    thermoDeviceConnected,
    thermoStreamConnected,
    bodyTemp,
    mmwaveEnabled,
    hasTarget,
    resp,
    hr,
    respMin,
    respMax,
    hrMin,
    hrMax,
    tempWarn,
    tempDanger,
    // yoloStreamConnected,
  ]);

  // ✅ interval은 1번만. setPassStreak는 tick당 1번만.
  useEffect(() => {
    const t = window.setInterval(() => {
      const { pass: gatePass, fails, warns } = evaluateGate();
      const nowTs = Date.now();

      const nextStreak: Record<string, number> = { ...passStreakRef.current };
      let toastMsg: string | null = null;

      setWorkers((prev) =>
        prev.map((w) => {
          if (w.status !== "pending") return w;

          const elapsed = nowTs - w.checkinStartedAt;

          // 타임아웃 -> blocked(보류)
          if (elapsed > CHECK_TIMEOUT_MS) {
            nextStreak[w.sessionKey] = 0;
            return {
              ...w,
              status: "blocked",
              failReasons: fails.length ? fails : ["시간 초과"],
              warnings: warns,
            };
          }

          // 실패
          if (!gatePass) {
            nextStreak[w.sessionKey] = 0;

            // 즉시 보류 안하면 pending 유지 + 사유만 갱신
            return {
              ...w,
              status: "pending",
              failReasons: fails,
              warnings: warns,
            };
          }

          // 통과 -> streak 증가
          const cur = nextStreak[w.sessionKey] ?? 0;
          const updated = cur + 1;
          nextStreak[w.sessionKey] = updated;

          if (updated >= PASS_STREAK_REQUIRED) {
            nextStreak[w.sessionKey] = 0;
            if (!toastMsg) toastMsg = `${w.name} 체크인 완료`;
            return { ...w, status: "active", failReasons: [], warnings: warns };
          }

          return { ...w, warnings: warns };
        })
      );

      setPassStreak(nextStreak);
      if (toastMsg) showToast(toastMsg);
    }, CHECK_INTERVAL_MS);

    return () => window.clearInterval(t);
  }, [evaluateGate, showToast]);

  /* ───────────────────────── 작업자 등록/삭제/재검사 ───────────────────────── */

  const deleteWorker = useCallback((sessionKey: string) => {
    setWorkers((prev) => prev.filter((w) => w.sessionKey !== sessionKey));
    setPassStreak((s) => {
      const next = { ...s };
      delete next[sessionKey];
      return next;
    });
  }, []);

  const retryWorker = useCallback(
    (sessionKey: string) => {
      const nowTs = Date.now();
      setWorkers((prev) =>
        prev.map((w) =>
          w.sessionKey === sessionKey
            ? {
                ...w,
                status: "pending",
                checkinStartedAt: nowTs,
                failReasons: [],
              }
            : w
        )
      );
      setPassStreak((s) => ({ ...s, [sessionKey]: 0 }));
      showToast("재검사 시작 — 자동 점검 중");
    },
    [showToast]
  );

  const submitWorker = useCallback(() => {
    const id = form.id.trim();
    const name = form.name.trim();
    const role = form.role?.trim() || "";

    if (!id || !name) {
      showToast("이름과 ID는 필수입니다.");
      return;
    }

    const nowTs = Date.now();
    const sessionKey = makeWorkerKey(undefined, id); // ✅ 고정키

    if (!sessionKey) {
      showToast("유효한 ID/UID가 없습니다.");
      return;
    }

    const w: Worker = {
      sessionKey,
      id,
      name,
      role,
      status: "pending",
      checkinStartedAt: nowTs,
      failReasons: [],
      warnings: [],
    };

    setWorkers((prev) => {
      const next = upsertWorker(prev, w);
      return next;
    });

    setPassStreak((s) => ({ ...s, [sessionKey]: 0 }));
    setForm({ id: "", name: "", role: "" });
    setWorkerModalOpen(false);
    showToast("등록/재검사 시작 (대기 중) — 자동 점검 중");
  }, [form.id, form.name, form.role, showToast]);

  /* ───────────────────────── 위험 업데이트 ───────────────────────── */

  const markHazard = useCallback(
    (type: "resolved" | "false_alarm") => {
      if (!selectedHazard) return;
      setHazards((prev) =>
        prev.map((h) =>
          h.id === selectedHazard.id ? { ...h, status: type } : h
        )
      );
      setSelectedHazard(null);
      showToast(
        type === "resolved"
          ? "문제 해결이 반영되었습니다."
          : "이상 없음으로 반영되었습니다."
      );
    },
    [selectedHazard, showToast]
  );

  /* ───────────────────────── UI 컴포넌트 ───────────────────────── */

  const SmallChip = ({
    label,
    value,
    unit,
    color,
  }: {
    label: string;
    value: number | null;
    unit: string;
    color: string;
  }) => (
    <div
      className={`${color} px-2.5 py-1 rounded-full text-[11px] font-semibold`}
    >
      {label}: {value == null ? "--" : value} {unit}
    </div>
  );

  const BigChip = ({
    icon,
    label,
    value,
    unit,
    color,
  }: {
    icon: string;
    label: string;
    value: number | null;
    unit: string;
    color: string;
  }) => (
    <div
      className={`${color} px-4 py-2 rounded-full text-base font-bold flex items-center gap-2`}
    >
      <span>{icon}</span>
      <span>
        {label}: {value == null ? "--" : value} {unit}
      </span>
    </div>
  );

  /* ───────────────────────── 화면용 계산 ───────────────────────── */

  const ppeRate = useMemo(() => {
    const ok = [ppeHelmet, ppeVest, ppeGloves].filter(Boolean).length;
    return Math.round((ok / 3) * 100);
  }, [ppeHelmet, ppeVest, ppeGloves]);

  const activeWorkers = useMemo(
    () => workers.filter((w) => w.status === "active").length,
    [workers]
  );

  // ✅ “밑에 보류 칸” 섹션 분리
  const listMain = useMemo(() => {
    return workers
      .filter((w) => w.status !== "blocked")
      .sort((a, b) => b.checkinStartedAt - a.checkinStartedAt);
  }, [workers]);

  const listBlocked = useMemo(() => {
    return workers
      .filter((w) => w.status === "blocked")
      .sort((a, b) => b.checkinStartedAt - a.checkinStartedAt);
  }, [workers]);

  /* ───────────────────────── 렌더 ───────────────────────── */

  return (
    <div className="min-h-screen bg-[#0E0E0E] text-white p-6 space-y-6 font-sans">
      {/* ✅ 백그라운드 스트림 프리로드 (모달 열기 전에도 연결상태 확보) */}
      <div className="hidden">
        <img
          src={YOLO_STREAM_URL}
          alt="yolo-preload"
          onLoad={() => setYoloStreamConnected(true)}
          onError={() => setYoloStreamConnected(false)}
        />
        <img
          src={THERMO_STREAM_URL}
          alt="thermo-preload"
          onLoad={() => setThermoStreamConnected(true)}
          onError={() => setThermoStreamConnected(false)}
        />
      </div>

      {/* Header */}
      <header className="flex justify-between items-center border-b border-gray-700 pb-4">
        <h1 className="text-3xl font-bold text-[#FFD400]">FIFO Safety Hub</h1>
        <div className="flex items-center space-x-4 text-gray-400">
          <p>{formatTime(now)} | Zone A</p>
          <Bell className="text-[#FFD400]" />
        </div>
      </header>

      {/* Main Grid */}
      <div className="grid grid-cols-4 gap-4">
        {/* Worker Summary */}
        <Card className="bg-[#1A1A1A] border border-[#2A2A2A] col-span-1 h-[460px]">
          <CardContent className="p-4 space-y-4">
            <h2 className="text-lg font-semibold text-[#FFD400]">현장 요약</h2>
            <div className="space-y-2 text-gray-200">
              <p>
                등록된 작업자:{" "}
                <span className="text-[#FFD400] font-semibold">
                  {workers.length}명
                </span>{" "}
                <span className="text-gray-400 text-sm">
                  (근무중 {activeWorkers}명)
                </span>
              </p>

              <p>PPE 착용률</p>
              <Progress value={ppeRate} className="h-2 bg-gray-700" />
              <p className="text-xs text-gray-400">
                현재 PPE 기준: {ppeRate}%
              </p>

              <p>위험 감지 로봇: 3대 (Online 2 / Offline 1)</p>

              <p>
                HA 연결 상태:{" "}
                <span className={connected ? "text-green-400" : "text-red-400"}>
                  {connected ? "연결됨" : "끊김"}
                </span>
              </p>

              <p>
                Thermoeye:{" "}
                <span
                  className={
                    thermoConnected ? "text-green-400" : "text-red-400"
                  }
                >
                  {thermoConnected ? "연결됨" : "끊김"}
                </span>
                <span className="ml-2 text-[11px] text-gray-500">
                  (장치:{thermoDeviceConnected ? "ON" : "OFF"} / 스트림:
                  {thermoStreamConnected ? "ON" : "OFF"})
                </span>
              </p>

              <p>
                YOLO:{" "}
                <span className={yoloConnected ? "text-green-400" : "text-red-400"}>
                  {yoloConnected ? "연결됨" : "끊김"}
                </span>
              </p>
            </div>
          </CardContent>
        </Card>

        {/* Map View */}
        <Card className="bg-[#1A1A1A] border border-[#2A2A2A] col-span-2 h-[460px]">
          <CardContent className="p-4">
            <h2 className="text-lg font-semibold text-[#FFD400] mb-2">
              실시간 현장 맵
            </h2>
            <div className="relative h-72 bg-[#121212] rounded-md overflow-hidden">
              <motion.div
                className="absolute left-[20%] top-[40%] bg-blue-500 h-5 w-5 rounded-full"
                animate={{ y: [0, -3, 0] }}
                transition={{ repeat: Infinity, duration: 2 }}
              />
              <motion.div
                className="absolute left-[55%] top-[70%] bg-red-500 h-5 w-5 rounded-full"
                animate={{ y: [0, -3, 0] }}
                transition={{ repeat: Infinity, duration: 2 }}
              />
              <motion.div
                className="absolute left-[80%] top-[25%] bg-yellow-400 h-5 w-5 rounded-full"
                animate={{ y: [0, -3, 0] }}
                transition={{ repeat: Infinity, duration: 2 }}
              />
              <p className="absolute bottom-3 right-4 text-xs text-gray-400">
                🔴 Zone B 위험 감지
              </p>
            </div>
          </CardContent>
        </Card>

        {/* PPE + Vitals Summary */}
        <Card className="bg-[#1A1A1A] border border-[#2A2A2A] col-span-1 h-[460px]">
          <CardContent className="p-4 space-y-3">
            <h2 className="text-lg font-semibold text-[#FFD400]">
              실시간 PPE 인식
            </h2>

            <div className="relative h-56 bg-[#111] rounded-md flex flex-col items-center justify-center space-y-2">
              <div
                className={ppeHelmet ? "text-gray-300 text-sm" : "text-red-400 text-sm"}
              >
                Helmet {ppeHelmet ? "✅" : "❌"}
              </div>
              <div
                className={ppeVest ? "text-gray-300 text-sm" : "text-red-400 text-sm"}
              >
                Vest {ppeVest ? "✅" : "❌"}
              </div>
              <div
                className={ppeGloves ? "text-gray-300 text-sm" : "text-red-400 text-sm"}
              >
                Gloves {ppeGloves ? "✅" : "❌"}
              </div>

              <div className="absolute top-2 right-2 flex flex-col gap-2">
                <SmallChip
                  label="🫁 호흡"
                  value={resp != null ? Number(resp.toFixed(1)) : null}
                  unit="bpm"
                  color={respColor}
                />
                <SmallChip
                  label="🫀 심박"
                  value={hr != null ? Number(hr.toFixed(0)) : null}
                  unit="bpm"
                  color={hrColor}
                />
                <SmallChip
                  label="🌡️ 체온"
                  value={bodyTemp != null ? Number(bodyTemp.toFixed(1)) : null}
                  unit="°C"
                  color={tempColor}
                />
              </div>
            </div>

            <p className="text-sm text-gray-400">
              {connected ? "HA 실시간 연동" : "HA 연결 대기"} · mmWave:{" "}
              {mmwaveEnabled == null ? "--" : mmwaveEnabled ? "ON" : "OFF"} · Target:{" "}
              {hasTarget == null ? "--" : hasTarget ? "감지됨" : "없음"}
            </p>

            <p className="text-xs text-gray-500">
              Confidence: {conf == null ? "--" : conf.toFixed(2)}
            </p>

            {/* 데모/테스트용: PPE 수동 토글 */}
            <div className="flex gap-2 pt-2">
              <button
                className={`text-xs px-2 py-1 rounded border ${
                  ppeHelmet
                    ? "border-emerald-600 text-emerald-300"
                    : "border-red-600 text-red-300"
                }`}
                onClick={() => setPpeHelmet((v) => !v)}
              >
                Helmet 토글
              </button>
              <button
                className={`text-xs px-2 py-1 rounded border ${
                  ppeVest
                    ? "border-emerald-600 text-emerald-300"
                    : "border-red-600 text-red-300"
                }`}
                onClick={() => setPpeVest((v) => !v)}
              >
                Vest 토글
              </button>
              <button
                className={`text-xs px-2 py-1 rounded border ${
                  ppeGloves
                    ? "border-emerald-600 text-emerald-300"
                    : "border-red-600 text-red-300"
                }`}
                onClick={() => setPpeGloves((v) => !v)}
              >
                Gloves 토글
              </button>
            </div>
          </CardContent>
        </Card>
      </div>

      {/* Bottom Buttons */}
      <div className="grid grid-cols-4 gap-4 mt-12">
        <Button
          className="bg-[#FFD400] hover:bg-[#E6C100] text-black font-semibold py-6 text-2xl rounded-lg"
          onClick={() => setWorkerModalOpen(true)}
        >
          작업자 등록
        </Button>

        <Button
          className="bg-[#FFD400] hover:bg-[#E6C100] text-black font-semibold py-6 text-2xl rounded-lg"
          onClick={() => setHazardModalOpen(true)}
        >
          위험 로그
        </Button>

        <Button
          className="bg-[#FFD400] hover:bg-[#E6C100] text-black font-semibold py-6 text-2xl rounded-lg"
          onClick={() => setRobotModalOpen(true)}
        >
          로봇 제어
        </Button>

        <Button
          className="bg-[#FFD400] hover:bg-[#E6C100] text-black font-semibold py-6 text-2xl rounded-lg"
          onClick={() => setVitalsOpen(true)}
        >
          생체 모니터링
        </Button>
      </div>

      {/* ───── 모달들 ─────────────────────────────────────────── */}

      {/* 생체 모니터링 모달 */}
      {vitalsOpen && (
        <Modal
          title="실시간 PPE + 열화상 + 호흡/심박 모니터링"
          onClose={() => setVitalsOpen(false)}
        >
          <div className="grid grid-cols-2 gap-4 w-full">
            {/* YOLO */}
            <div className="relative h-[360px] w-full bg-[#111] rounded-lg overflow-hidden border border-[#2A2A2A]">
              <img
                ref={(el) => (yoloImgRef.current = el)}
                src={YOLO_STREAM_URL}
                alt="YOLO stream"
                className="absolute inset-0 w-full h-full object-contain bg-black"
                onLoad={() => setYoloStreamConnected(true)}
                onError={() => setYoloStreamConnected(false)}
              />

              <div className="absolute top-4 left-4 text-xs text-gray-300 bg-black/40 px-3 py-1 rounded">
                {yoloConnected ? "YOLO 연결됨" : "YOLO 연결 대기"}
              </div>

              <div className="absolute top-4 right-4 flex flex-col gap-2">
                <BigChip
                  icon="🫁"
                  label="호흡"
                  value={resp != null ? Number(resp.toFixed(1)) : null}
                  unit="bpm"
                  color={respColor}
                />
                <BigChip
                  icon="🫀"
                  label="심박"
                  value={hr != null ? Number(hr.toFixed(0)) : null}
                  unit="bpm"
                  color={hrColor}
                />
              </div>

              <div className="absolute left-4 bottom-4 text-xs text-gray-300 bg-black/40 px-3 py-1 rounded">
                PPE / YOLO
              </div>
            </div>

            {/* Thermoeye */}
            <div className="relative h-[360px] w-full bg-[#111] rounded-lg overflow-hidden border border-[#2A2A2A]">
              <img
                ref={(el) => (thermoImgRef.current = el)}
                src={THERMO_STREAM_URL}
                alt="Thermoeye thermal stream"
                className="absolute inset-0 w-full h-full object-contain bg-black"
                onLoad={() => setThermoStreamConnected(true)}
                onError={() => setThermoStreamConnected(false)}
              />

              <div className="absolute top-4 right-4 flex flex-col gap-2">
                <BigChip
                  icon="🌡️"
                  label="체온"
                  value={bodyTemp != null ? Number(bodyTemp.toFixed(1)) : null}
                  unit="°C"
                  color={tempColor}
                />

                <div className="text-xs text-gray-300 bg-black/40 px-3 py-1 rounded">
                  {thermoConnected ? "Thermoeye 연결됨" : "Thermoeye 연결 대기"}
                </div>
              </div>

              <div className="absolute left-4 bottom-4 text-xs text-gray-300 bg-black/40 px-3 py-1 rounded">
                Thermoeye / Thermal
              </div>
            </div>
          </div>

          <div className="mt-3 text-sm text-gray-400">
            상태: {connected ? "HA 실시간 연결" : "HA 연결 대기"}{" "}
            {conf != null ? `• conf ${conf.toFixed(2)}` : ""}
            <span className="ml-3">
              · mmWave:{" "}
              {mmwaveEnabled == null ? "--" : mmwaveEnabled ? "ON" : "OFF"} · Target:{" "}
              {hasTarget == null ? "--" : hasTarget ? "감지됨" : "없음"}
            </span>
          </div>

          {/* 임계값 설정 */}
          <div className="mt-6 border-t border-[#2A2A2A] pt-4">
            <h4 className="text-sm font-semibold text-[#FFD400] mb-2">
              임계값 설정
              <span className="ml-2 text-xs text-gray-400">
                {isAdmin
                  ? `관리자 권한 ${adminRemainSec}초`
                  : "잠김(관리자 카드 필요)"}
              </span>
            </h4>

            <div className="grid grid-cols-3 gap-4">
              <LabeledNumberInput
                label="호흡 임계 (최소)"
                value={respMinStr}
                onChange={setRespMinStr}
                onCommit={(n) => setRespMin(clamp(n, 1, 200))}
                suffix="bpm"
                disabled={!isAdmin}
              />
              <LabeledNumberInput
                label="심박 임계 (최소)"
                value={hrMinStr}
                onChange={setHrMinStr}
                onCommit={(n) => setHrMin(clamp(n, 1, 250))}
                suffix="bpm"
                disabled={!isAdmin}
              />
              <LabeledNumberInput
                label="체온 임계 (경고)"
                value={tempWarnStr}
                onChange={setTempWarnStr}
                onCommit={(n) => setTempWarn(clamp(n, 30, 45))}
                suffix="°C"
                placeholder="예: 37.5"
                disabled={!isAdmin}
              />
              <LabeledNumberInput
                label="호흡 임계 (최대)"
                value={respMaxStr}
                onChange={setRespMaxStr}
                onCommit={(n) => setRespMax(clamp(n, 1, 200))}
                suffix="bpm"
                disabled={!isAdmin}
              />
              <LabeledNumberInput
                label="심박 임계 (최대)"
                value={hrMaxStr}
                onChange={setHrMaxStr}
                onCommit={(n) => setHrMax(clamp(n, 1, 250))}
                suffix="bpm"
                disabled={!isAdmin}
              />
              <LabeledNumberInput
                label="체온 임계 (위험)"
                value={tempDangerStr}
                onChange={setTempDangerStr}
                onCommit={(n) => setTempDanger(clamp(n, 30, 45))}
                suffix="°C"
                placeholder="예: 38.5"
                disabled={!isAdmin}
              />
            </div>
          </div>
        </Modal>
      )}

      {/* 작업자 등록 모달 */}
      {workerModalOpen && (
        <Modal title="작업자 등록" onClose={() => setWorkerModalOpen(false)}>
          <div className="h-[calc(86vh-56px)] grid grid-rows-[260px_auto_1fr] gap-4">
            {/* 1행: YOLO + 게이트 상태 */}
            <div className="grid grid-cols-2 gap-4">
              {/* YOLO preview */}
              <div className="relative h-[260px] w-full bg-[#111] rounded-lg overflow-hidden border border-[#2A2A2A]">
                <img
                  src={YOLO_STREAM_URL}
                  alt="YOLO preview"
                  className="absolute inset-0 w-full h-full object-contain bg-black"
                  onLoad={() => setYoloStreamConnected(true)}
                  onError={() => setYoloStreamConnected(false)}
                />
                <div className="absolute top-3 left-3 text-xs text-gray-300 bg-black/40 px-3 py-1 rounded">
                  {yoloConnected ? "YOLO 연결됨" : "YOLO 연결 대기"}
                </div>

                <div className="absolute left-3 bottom-3 text-xs text-gray-300 bg-black/40 px-3 py-1 rounded">
                  PPE 인식 프리뷰 (체크인 근거)
                </div>
              </div>

              {/* 게이트 상태 */}
              <div className="h-[260px] bg-[#111] rounded-lg border border-[#2A2A2A] p-4">
                <div className="text-sm font-semibold text-[#FFD400] mb-3">
                  등록 게이트 상태
                </div>

                <div className="space-y-2 text-sm">
                  <div className="flex items-center justify-between">
                    <span>PPE(Helmet/Vest/Gloves)</span>
                    <span
                      className={
                        ppeHelmet && ppeVest && ppeGloves
                          ? "text-emerald-300"
                          : "text-red-300"
                      }
                    >
                      {ppeHelmet && ppeVest && ppeGloves ? "✅ 통과" : "❌ 미통과"}
                    </span>
                  </div>

                  <div className="flex items-center justify-between">
                    <span>mmWave(ON + Target)</span>
                    <span
                      className={
                        mmwaveEnabled && hasTarget
                          ? "text-emerald-300"
                          : "text-red-300"
                      }
                    >
                      {mmwaveEnabled && hasTarget ? "✅ 통과" : "❌ 미통과"}
                    </span>
                  </div>

                  <div className="flex items-center justify-between">
                    <span>Thermoeye(장치/스트림)</span>
                    <span
                      className={
                        thermoConnected ? "text-emerald-300" : "text-red-300"
                      }
                    >
                      {thermoConnected ? "✅ 통과" : "❌ 미통과"}
                    </span>
                  </div>

                  <div className="mt-4 text-xs text-gray-400 leading-5">
                    현재 값: 호흡 {resp == null ? "--" : resp.toFixed(1)} / 심박{" "}
                    {hr == null ? "--" : hr.toFixed(0)} / 체온{" "}
                    {bodyTemp == null ? "--" : bodyTemp.toFixed(1)}
                  </div>

                  <div className="mt-2 text-[11px] text-gray-500">
                    ※ PPE 인식이 안 되면 YOLO 화면에서 자세/각도를 조정하세요.
                  </div>
                </div>
              </div>
            </div>

            {/* 2행: 입력 폼 */}
            <div className="grid grid-cols-3 gap-4 mt-1">
              <LabeledInput
                label="작업자 ID"
                value={form.id}
                onChange={(v) => setForm((f) => ({ ...f, id: v }))}
                placeholder="예: W-002"
                inputClassName="text-lg px-4 py-3 h-14"
              />
              <LabeledInput
                label="이름"
                value={form.name}
                onChange={(v) => setForm((f) => ({ ...f, name: v }))}
                placeholder="예: 김규민"
                inputClassName="text-lg px-4 py-3 h-14"
              />
              <LabeledInput
                label="직무(선택)"
                value={form.role || ""}
                onChange={(v) => setForm((f) => ({ ...f, role: v }))}
                placeholder="예: 배관"
                inputClassName="text-lg px-4 py-3 h-14"
              />
            </div>

            {/* 3행: 리스트 + 버튼 */}
            <div className="grid grid-rows-[auto_1fr]">
              {/* 버튼 영역 */}
              <div className="flex justify-end gap-2">
                <Button
                  className="bg-gray-700 hover:bg-gray-600"
                  onClick={() => setWorkerModalOpen(false)}
                >
                  취소
                </Button>
                <Button
                  className="bg-[#FFD400] hover:bg-[#E6C100] text-black font-semibold"
                  onClick={submitWorker}
                >
                  등록
                </Button>
              </div>

              {/* 리스트 영역 */}
              <div className="mt-3">
                <p className="text-sm text-gray-300 mb-2">최근 등록된 작업자</p>

                <div className="h-[calc(100%-28px)] overflow-auto space-y-2 text-sm text-gray-300 pr-1">
                  {/* ✅ 1) pending/active 먼저 */}
                  {listMain.map((w) => (
                    <WorkerRow
                      key={w.sessionKey}
                      w={w}
                      deleteWorker={deleteWorker}
                      retryWorker={retryWorker}
                      PASS_STREAK_REQUIRED={PASS_STREAK_REQUIRED}
                    />
                  ))}

                  {/* ✅ 2) 보류 칸(하단 섹션) */}
                  <div className="pt-3">
                    <div className="flex items-center justify-between mb-2">
                      <p className="text-sm text-red-300 font-semibold">
                        보류(미통과) 작업자
                      </p>
                      <p className="text-xs text-gray-500">
                        {FAIL_IMMEDIATE_BLOCK ? "미통과 즉시 보류" : "타임아웃 시 보류"}
                      </p>
                    </div>

                    {listBlocked.length === 0 ? (
                      <div className="text-xs text-gray-500 border border-[#2A2A2A] rounded-lg p-3 bg-[#101010]">
                        보류된 작업자가 없습니다.
                      </div>
                    ) : (
                      <div className="space-y-2">
                        {listBlocked.map((w) => (
                          <WorkerRow
                            key={w.sessionKey}
                            w={w}
                            deleteWorker={deleteWorker}
                            retryWorker={retryWorker}
                            PASS_STREAK_REQUIRED={PASS_STREAK_REQUIRED}
                          />
                        ))}
                      </div>
                    )}
                  </div>
                </div>
              </div>
            </div>
          </div>
        </Modal>
      )}

      {/* 위험 로그 모달 */}
      {hazardModalOpen && (
        <Modal title="위험 로그" onClose={() => setHazardModalOpen(false)}>
          <HazardMap
            hazards={hazards}
            selectedId={selectedHazard?.id ?? null}
            onSelect={(h) => setSelectedHazard(h)}
          />

          {selectedHazard && (
            <div className="mt-4 flex gap-3">
              <Button
                className="bg-emerald-600 hover:bg-emerald-700 text-white flex-1"
                onClick={() => markHazard("resolved")}
              >
                문제 해결
              </Button>
              <Button
                className="bg-gray-600 hover:bg-gray-700 text-white flex-1"
                onClick={() => markHazard("false_alarm")}
              >
                이상 없음
              </Button>
            </div>
          )}

          <p className="mt-3 text-sm text-gray-400">
            초록 점선은 로봇 순찰 경로, 빨강/초록/회색 점은 각각 미확인 위험 / 해결됨 / 오탐 입니다.
            점을 눌러 상태를 변경할 수 있습니다.
          </p>
        </Modal>
      )}

      {/* 로봇 제어 모달 */}
      {robotModalOpen && (
        <Modal title="로봇 제어" onClose={() => setRobotModalOpen(false)}>
          <div className="grid grid-cols-2 gap-4">
            <ControlCard title="서브 로봇 이동">
              <div className="flex gap-2">
                <input
                  className="flex-1 bg-[#0E0E0E] border border-[#2A2A2A] rounded px-3 py-2 text-sm"
                  placeholder="X (예: 12.5)"
                />
                <input
                  className="flex-1 bg-[#0E0E0E] border border-[#2A2A2A] rounded px-3 py-2 text-sm"
                  placeholder="Y (예: 34.0)"
                />
                <Button className="bg-[#FFD400] hover:bg-[#E6C100] text-black font-semibold">
                  이동
                </Button>
              </div>
            </ControlCard>

            <ControlCard title="순찰">
              <div className="flex gap-2">
                <Button className="bg-emerald-600 hover:bg-emerald-700 text-white flex-1">
                  시작
                </Button>
                <Button className="bg-red-600 hover:bg-red-700 text-white flex-1">
                  정지
                </Button>
              </div>
            </ControlCard>

            <ControlCard title="사이렌/스피커">
              <div className="flex gap-2">
                <Button className="bg-orange-500 hover:bg-orange-600 text-white flex-1">
                  경고 방송
                </Button>
                <Button className="bg-gray-600 hover:bg-gray-700 text-white flex-1">
                  정지
                </Button>
              </div>
            </ControlCard>

            <ControlCard title="도킹/충전">
              <div className="flex gap-2">
                <Button className="bg-sky-600 hover:bg-sky-700 text-white flex-1">
                  도킹 지시
                </Button>
                <Button className="bg-purple-600 hover:bg-purple-700 text-white flex-1">
                  펌웨어 업데이트
                </Button>
              </div>
            </ControlCard>
          </div>

          <p className="mt-3 text-sm text-gray-400">
            ※ 실제 로봇 제어는 나중에 백엔드 API / ROS 연동 필요. 지금은 UI만 구현해 둔 상태입니다.
          </p>
        </Modal>
      )}

      {/* Footer */}
      <footer className="pt-8 text-center text-gray-500 text-xs">
        FIFO Safety System © 2025 | Designed for Human-Centered Safety
      </footer>

      {/* 토스트 (상단 중앙 + 스르륵) */}
      <AnimatePresence>
        {toast && (
          <motion.div
            key="toast"
            className="fixed top-20 inset-x-0 z-[10000] flex justify-center"
            initial={{ opacity: 0, y: -12 }}
            animate={{ opacity: 1, y: 0 }}
            exit={{ opacity: 0, y: -12 }}
            transition={{ duration: 0.22 }}
          >
            <div className="bg-[#1A1A1A] border border-[#2A2A2A] text-sm text-gray-100 px-5 py-3 rounded-xl shadow-lg backdrop-blur">
              {toast}
            </div>
          </motion.div>
        )}
      </AnimatePresence>
    </div>
  );
}

/* ───────────────────────── WorkerRow ───────────────────────── */

function WorkerRow({
  w,
  deleteWorker,
  retryWorker,
  PASS_STREAK_REQUIRED,
}: {
  w: Worker;
  deleteWorker: (sessionKey: string) => void;
  retryWorker: (sessionKey: string) => void;
  PASS_STREAK_REQUIRED: number;
}) {
  return (
    <div className="border border-[#2A2A2A] rounded-lg p-3 bg-[#101010]">
      <div className="flex justify-between items-center">
        <div className="flex items-center gap-2">
          <span className="font-semibold">
            {w.id} · {w.name}
            {w.uid ? (
              <span className="ml-2 text-[11px] text-gray-500">
                ({w.uid.slice(0, 8)})
              </span>
            ) : null}
          </span>
          <StatusBadge status={w.status} />
          {w.warnings?.length ? (
            <span className="text-[11px] text-amber-300">(주의)</span>
          ) : null}
        </div>

        <div className="flex items-center gap-2">
          <span className="text-gray-400 text-xs">{w.role || "-"}</span>

          {w.status === "blocked" && (
            <Button
              className="bg-[#FFD400] hover:bg-[#E6C100] text-black font-semibold h-8 px-3 text-xs"
              onClick={() => retryWorker(w.sessionKey)}
            >
              재검사
            </Button>
          )}

          <button
            className="text-xs text-red-400 hover:text-red-300"
            onClick={() => deleteWorker(w.sessionKey)}
          >
            삭제
          </button>
        </div>
      </div>

      {w.status === "pending" ? (
        <div className="mt-2 text-xs text-gray-400">
          자동 점검 중… (연속 {PASS_STREAK_REQUIRED}회 통과 시 확정)
        </div>
      ) : null}

      {w.status === "blocked" && w.failReasons?.length ? (
        <div className="mt-2 text-xs text-red-300">
          보류 사유: {w.failReasons.join(" · ")}
        </div>
      ) : null}

      {w.status === "active" && w.warnings?.length ? (
        <div className="mt-2 text-xs text-amber-300">
          주의: {w.warnings.join(" · ")}
        </div>
      ) : null}
    </div>
  );
}

/* ───────────────────────── 공통 컴포넌트 ───────────────────────── */

function Modal({
  title,
  onClose,
  children,
}: {
  title: string;
  onClose: () => void;
  children: any;
}) {
  return (
    <div className="fixed inset-0 z-[9999] bg-black/80 overflow-hidden">
      <div className="h-full flex justify-center items-center px-6 py-6">
        <div
          className="
            w-[1120px] max-w-[95vw]
            h-[86vh] max-h-[86vh]
            bg-[#1A1A1A] rounded-xl relative
            border border-[#2A2A2A]
            overflow-hidden
          "
        >
          {/* Header */}
          <div className="absolute top-0 left-0 right-0 flex justify-between items-center px-4 py-2 bg-[#1A1A1A] border-b border-[#2A2A2A] z-50">
            <h3 className="text-xl font-semibold text-[#FFD400]">{title}</h3>
            <Button
              onClick={onClose}
              className="bg-red-600 hover:bg-red-700 text-white font-semibold px-4 py-2"
            >
              닫기 ✕
            </Button>
          </div>

          {/* Body */}
          <div className="pt-14 h-full">
            <div className="h-full px-4 pb-4">{children}</div>
          </div>
        </div>
      </div>
    </div>
  );
}

function StatusBadge({ status }: { status: WorkerStatus }) {
  const cls =
    status === "active"
      ? "border-emerald-700 text-emerald-300 bg-emerald-600/20"
      : status === "pending"
      ? "border-yellow-700 text-yellow-200 bg-yellow-600/20"
      : "border-red-700 text-red-300 bg-red-600/20";

  const label = status === "active" ? "근무중" : status === "pending" ? "대기" : "보류";

  return <span className={`text-[11px] px-2 py-0.5 rounded-full border ${cls}`}>{label}</span>;
}

function LabeledInput({
  label,
  value,
  onChange,
  placeholder,
  inputClassName,
}: {
  label: string;
  value: string;
  onChange: (v: string) => void;
  placeholder?: string;
  inputClassName?: string;
}) {
  const cls = `bg-[#0E0E0E] border border-[#2A2A2A] rounded focus:outline-none focus:border-[#FFD400] ${
    inputClassName ?? "px-3 py-2 text-sm"
  }`;

  return (
    <label className="flex flex-col gap-1 text-sm">
      <span className="text-gray-300">{label}</span>
      <input
        className={cls}
        value={value}
        onChange={(e) => onChange(e.target.value)}
        placeholder={placeholder}
      />
    </label>
  );
}

function LabeledNumberInput({
  label,
  value,
  onChange,
  onCommit,
  placeholder,
  suffix,
  disabled,
}: {
  label: string;
  value: string;
  onChange: (v: string) => void;
  onCommit: (num: number) => void;
  placeholder?: string;
  suffix?: string;
  disabled?: boolean;
}) {
  const commit = () => {
    if (disabled) return;
    if (value.trim() === "") return;
    const n = Number(value);
    if (!Number.isFinite(n)) return;
    onCommit(n);
  };

  return (
    <label className="flex flex-col gap-1 text-sm">
      <span className="text-gray-300">{label}</span>
      <div className="relative">
        <input
          disabled={!!disabled}
          className={`w-full bg-[#0E0E0E] border border-[#2A2A2A] rounded px-3 py-2 text-sm focus:outline-none focus:border-[#FFD400]
            ${disabled ? "opacity-50 cursor-not-allowed" : ""}
          `}
          value={value}
          onChange={(e) => {
            if (disabled) return;
            const next = e.target.value;
            if (/^[0-9]*\.?[0-9]*$/.test(next) || next === "") onChange(next);
          }}
          onBlur={commit}
          inputMode="decimal"
          placeholder={placeholder}
        />
        {suffix && (
          <span className="absolute right-3 top-1/2 -translate-y-1/2 text-xs text-gray-500">
            {suffix}
          </span>
        )}
      </div>
      {disabled ? (
        <span className="text-[11px] text-gray-500">
          관리자 카드 태그 후 10초 동안만 수정 가능
        </span>
      ) : null}
    </label>
  );
}

function ControlCard({ title, children }: { title: string; children: any }) {
  return (
    <div className="bg-[#111] border border-[#2A2A2A] rounded-lg p-4">
      <p className="text-sm text-[#FFD400] font-semibold mb-2">{title}</p>
      {children}
    </div>
  );
}

/* ───────────────────────── HazardMap ───────────────────────── */

function HazardMap({
  hazards,
  selectedId,
  onSelect,
}: {
  hazards: { id: string; x: number; y: number; status: HazardStatus }[];
  selectedId?: string | null;
  onSelect: (h: any) => void;
}) {
  const W = 1200,
    H = 520;
  const pct = (p: number, max: number) => (p / 100) * max;

  const pExit = { x: pct(8, W), y: pct(50, H) };
  const p1 = { x: pct(30, W), y: pct(50, H) };
  const p2 = { x: pct(30, W), y: pct(32, H) };
  const p3 = { x: pct(72, W), y: pct(32, H) };
  const pDockNear = { x: pct(90, W), y: pct(40, H) };

  const routePoints: [number, number][] = [
    [pExit.x, pExit.y],
    [p1.x, p1.y],
    [p2.x, p2.y],
    [p3.x, p3.y],
    [pDockNear.x, pDockNear.y],
  ];

  return (
    <svg
      viewBox={`0 0 ${W} ${H}`}
      className="w-full h-[520px] rounded-lg border border-[#2A2A2A] bg-[#050505]"
    >
      <defs>
        <pattern id="grid" width="40" height="40" patternUnits="userSpaceOnUse">
          <path d="M 40 0 L 0 0 0 40" fill="none" stroke="#161616" strokeWidth="1" />
        </pattern>
      </defs>

      <rect x="0" y="0" width={W} height={H} fill="url(#grid)" />

      <g fill="#101010" stroke="#262626">
        <rect x="70" y="60" width={1060} height={400} rx="16" />
        <rect x="90" y="80" width="360" height="140" rx="6" />
        <rect x="90" y="330" width="360" height="110" rx="6" />
        <rect x="660" y="80" width="450" height="360" rx="6" />
      </g>

      <g fill="#181818" stroke="#303030">
        {[0, 1, 2].map((i) => (
          <rect key={i} x={110 + i * 110} y={120} width="80" height="60" rx="4" />
        ))}
        {[0, 1, 2, 3].map((i) => (
          <rect key={i} x={110 + i * 80} y={360} width="60" height="60" rx="3" />
        ))}
        <rect x="690" y="130" width="140" height="80" rx="4" />
        <rect x="860" y="140" width="180" height="60" rx="4" />
        <rect x="700" y="310" width="120" height="80" rx="4" />
      </g>

      <g>
        <text x="100" y="105" fill="#e5e5e5" fontSize="14" fontWeight={500}>
          Zone A — Assembly
        </text>
        <text x="100" y="355" fill="#e5e5e5" fontSize="14" fontWeight={500}>
          Zone B — Storage
        </text>
        <text x="670" y="105" fill="#e5e5e5" fontSize="14" fontWeight={500}>
          Zone C — Maintenance
        </text>
      </g>

      <g>
        <rect x={pExit.x - 14} y={pExit.y - 22} width="24" height="44" fill="#0284c7" rx="4" />
        <text x={pExit.x - 18} y={pExit.y - 28} fill="#7dd3fc" fontSize="11" textAnchor="start">
          EXIT
        </text>

        <rect x={pct(93, W)} y={pct(45, H)} width="22" height="70" fill="#8b5cf6" rx="4" />
        <text x={pct(96, W)} y={pct(43, H)} fill="#ddd6fe" fontSize="11" textAnchor="start">
          DOCK
        </text>
      </g>

      <polyline
        points={routePoints.map(([x, y]) => `${x},${y}`).join(" ")}
        fill="none"
        stroke="#4ade80"
        strokeWidth={3}
        strokeDasharray="10 7"
        strokeLinecap="round"
        strokeLinejoin="round"
      />

      <motion.circle
        r={7}
        fill="#4ade80"
        stroke="black"
        strokeWidth={1.5}
        animate={{ cx: routePoints.map(([x]) => x), cy: routePoints.map(([, y]) => y) }}
        transition={{ duration: 6, repeat: Infinity, ease: "linear" }}
      />

      {hazards.map((h) => {
        const x = pct(h.x, W);
        const y = pct(h.y, H);
        const color =
          h.status === "open" ? "#ef4444" : h.status === "resolved" ? "#22c55e" : "#9ca3af";
        const ring = selectedId === h.id ? "#facc15" : "#020617";

        return (
          <g key={h.id} onClick={() => onSelect(h)} style={{ cursor: "pointer" }}>
            <circle cx={x} cy={y} r={9} fill={color} stroke={ring} strokeWidth={2} />
            <text x={x} y={y - 14} fill="#e5e5e5" fontSize="12" textAnchor="middle">
              {h.id}
            </text>
          </g>
        );
      })}

      <g>
        <rect x={W - 220} y={H - 125} width="200" height="110" rx="8" fill="#050505" stroke="#27272a" />
        <text x={W - 210} y={H - 106} fill="#facc15" fontSize="12">
          Legend
        </text>

        <circle cx={W - 200} cy={H - 86} r={5} fill="#4ade80" />
        <text x={W - 188} y={H - 82} fill="#a3a3a3" fontSize="12">
          Robot route
        </text>

        <circle cx={W - 200} cy={H - 66} r={5} fill="#ef4444" />
        <text x={W - 188} y={H - 62} fill="#a3a3a3" fontSize="12">
          Open hazard
        </text>

        <circle cx={W - 200} cy={H - 46} r={5} fill="#22c55e" />
        <text x={W - 188} y={H - 42} fill="#a3a3a3" fontSize="12">
          Resolved
        </text>

        <circle cx={W - 200} cy={H - 26} r={5} fill="#9ca3af" />
        <text x={W - 188} y={H - 22} fill="#a3a3a3" fontSize="12">
          False alarm
        </text>
      </g>
    </svg>
  );
}
