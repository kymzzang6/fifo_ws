import {
  useCallback,
  useEffect,
  useMemo,
  useRef,
  useState,
  type ReactNode,
} from "react";
import { Card, CardContent } from "./components/ui/card";
import { Button } from "./components/ui/button";
import { Progress } from "./components/ui/progress";
import { Bell } from "lucide-react";
import { AnimatePresence, motion } from "framer-motion";
import { handleRFID } from "./rfid/rfidHandler";

/* ───────────────────────── URL 설정 ───────────────────────── */

const YOLO_STREAM_URL =
  "http://localhost:8080/stream?topic=/ppe_debug&type=mjpeg";

const THERMO_STREAM_URL = "http://localhost:8080/stream?topic=/thermal/video&type=mjpeg";

const SLAM_STREAM_URL =
  (import.meta as any).env?.VITE_SLAM_STREAM_URL ??
  "http://192.168.1.92:5000/stream";

const WS_URL =
  (import.meta as any).env?.VITE_HA_WS_URL ??
  "http://localhost:8123/api/websocket";
const HA_TOKEN = (import.meta as any).env?.VITE_HA_TOKEN ?? "";

const RFID_WS_URL =
  (import.meta as any).env?.VITE_RFID_WS_URL ?? "ws://127.0.0.1:8765";

/* ───────────────────────── 타입 ───────────────────────── */

type HazardStatus = "open" | "resolved" | "false_alarm";
type WorkerStatus = "pending" | "active" | "blocked";
type RobotMode = "manual" | "following" | "patrol";

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
  x: number;
  y: number;
  status: HazardStatus;
  note?: string;
};

type NotificationLevel = "info" | "warning" | "danger" | "success";

type NotificationItem = {
  id: string;
  title: string;
  message: string;
  level: NotificationLevel;
  createdAt: number;
};

/* ───────────────────────── 유틸 ───────────────────────── */

function formatTime(date: Date): string {
  const h = date.getHours().toString().padStart(2, "0");
  const m = date.getMinutes().toString().padStart(2, "0");
  return `${h}:${m}`;
}

function formatDateTime(ts: number): string {
  const d = new Date(ts);
  const hh = d.getHours().toString().padStart(2, "0");
  const mm = d.getMinutes().toString().padStart(2, "0");
  const ss = d.getSeconds().toString().padStart(2, "0");
  return `${hh}:${mm}:${ss}`;
}

function clamp(n: number, min: number, max: number) {
  return Math.max(min, Math.min(max, n));
}

function makeWorkerKey(uid?: string, id?: string) {
  const u = (uid ?? "").trim();
  const i = (id ?? "").trim();
  return u || i;
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
  const [notifOpen, setNotifOpen] = useState(false);
  const [conditionAlertOpen, setConditionAlertOpen] = useState(false);
  const conditionAlertTimerRef = useRef<number | null>(null);
  const conditionAlertIntervalRef = useRef<number | null>(null);
  const conditionAlertCloseRef = useRef<number | null>(null);
  const [conditionAlertCountdown, setConditionAlertCountdown] = useState(30);
  const [conditionAlertAcknowledged, setConditionAlertAcknowledged] = useState(false);

  /* ─── 현재 시간 ──────────────────────────────────────── */
  const [now, setNow] = useState<Date>(new Date());
  useEffect(() => {
    const timer = window.setInterval(() => setNow(new Date()), 1000 * 30);
    return () => window.clearInterval(timer);
  }, []);

  /* ─── 토스트 / 알림 ─────────────────────────────────── */
  const [toast, setToast] = useState<string | null>(null);
  const [notifications, setNotifications] = useState<NotificationItem[]>([]);
  const toastTimerRef = useRef<number | null>(null);
  const lastPpeDetectedRef = useRef<boolean | null>(null);

  const showToast = useCallback(
    (
      msg: string,
      opts?: {
        title?: string;
        level?: NotificationLevel;
        save?: boolean;
      }
    ) => {
      setToast(msg);

      if (toastTimerRef.current) window.clearTimeout(toastTimerRef.current);
      toastTimerRef.current = window.setTimeout(() => setToast(null), 1600);

      const shouldSave = opts?.save ?? true;
      if (shouldSave) {
        const item: NotificationItem = {
          id: `${Date.now()}-${Math.random().toString(16).slice(2)}`,
          title: opts?.title ?? "시스템 알림",
          message: msg,
          level: opts?.level ?? "info",
          createdAt: Date.now(),
        };
        setNotifications((prev) => [item, ...prev].slice(0, 50));
      }
    },
    []
  );

  useEffect(() => {
    return () => {
      if (toastTimerRef.current) window.clearTimeout(toastTimerRef.current);
      if (conditionAlertTimerRef.current) {
        window.clearTimeout(conditionAlertTimerRef.current);
      }
      if (conditionAlertIntervalRef.current) {
        window.clearInterval(conditionAlertIntervalRef.current);
      }
      if (conditionAlertCloseRef.current) {
        window.clearTimeout(conditionAlertCloseRef.current);
      }
    };
  }, []);

  useEffect(() => {
    if (workerModalOpen || hazardModalOpen || robotModalOpen || vitalsOpen) {
      setNotifOpen(false);
    }
  }, [workerModalOpen, hazardModalOpen, robotModalOpen, vitalsOpen]);

  /* ─── 관리자 권한 (10초) ─────────────────────────────── */
  const [adminUntil, setAdminUntil] = useState<number>(0);
  const isAdmin = adminUntil > Date.now();

  const grantAdminFor10s = useCallback(() => {
    setAdminUntil(Date.now() + 10_000);
  }, []);

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

  /* ─── backend / robot control ───────────────────────── */
  const rwsRef = useRef<WebSocket | null>(null);
  const [backendRosConnected, setBackendRosConnected] = useState(false);
  const [backendWsConnected, setBackendWsConnected] = useState(false);
  const [robotMode, setRobotMode] = useState<RobotMode>("manual");
  const [keepGoingDirection, setKeepGoingDirection] = useState<"forward" | "backward" | null>(null);

  const sendBackendMessage = useCallback(
    (payload: any) => {
      const ws = rwsRef.current;
      if (!ws || ws.readyState !== WebSocket.OPEN) {
          showToast("백엔드 제어 서버 연결이 끊겨 있습니다.", {
            title: "제어 오류",
            level: "danger",
            save: false,
          });
        return false;
      }

      ws.send(JSON.stringify(payload));
      return true;
    },
    [showToast]
  );

  const setRobotModeAndPublish = useCallback(
    (mode: RobotMode) => {
      setRobotMode(mode);

      const ok = sendBackendMessage({
        type: "set_mode",
        mode,
      });

      if (ok) {
        showToast(`로봇 모드 변경: ${mode}`, {
          title: "로봇 제어",
          level: "info",
        });
      }
    },
    [sendBackendMessage, showToast]
  );

  const publishCmdVel = useCallback(
    (linearX: number, angularZ: number) => {
      if (robotMode !== "manual") {
        showToast("수동제어는 manual 모드에서만 가능합니다.", {
          title: "모드 제한",
          level: "warning",
          save: false,
        });
        return;
      }

      sendBackendMessage({
        type: "cmd_vel",
        linear_x: linearX,
        angular_z: angularZ,
      });
    },
    [robotMode, sendBackendMessage, showToast]
  );

  const stopRobot = useCallback(() => {
    setKeepGoingDirection(null);
    sendBackendMessage({
      type: "cmd_vel",
      linear_x: 0.0,
      angular_z: 0.0,
    });
  }, [sendBackendMessage]);

  const sendTowerCommand = useCallback(
    (color: string, pattern: string, duration = 0) => {
      sendBackendMessage({
        type: "tower_command",
        color,
        pattern,
        duration,
      });
    },
    [sendBackendMessage]
  );

  const sendSirenPreset = useCallback(
  (
    preset:
      | "danger"
      | "warning"
      | "unregistered"
      | "registered"
      | "admin"
      | "moving"
      | "sound"
      | "off"
  ) => {

    // 🔴 위험
    if (preset === "danger") {
      sendTowerCommand("red", "steady", 5);
      return;
    }

    // 🟡 컨디션 저조
    if (preset === "warning") {
      sendTowerCommand("yellow", "steady", 5);
      return;
    }

    // 🔴 미등록 카드
    if (preset === "unregistered") {
      sendTowerCommand("red", "blink", 5);
      return;
    }

    // 🟢 등록 완료
    if (preset === "registered") {
      sendTowerCommand("green", "steady", 5);
      return;
    }

    // 🔵 관리자
    if (preset === "admin") {
      sendTowerCommand("blue", "steady", 10);
      return;
    }

    // 🔵 이동 표시
    if (preset === "moving") {
      sendTowerCommand("blue", "steady", 0);
      return;
    }

    // 🔊🔴 소음 위험
    if (preset === "sound") {
      sendTowerCommand("red", "steady", 5);
      return;
    }

    // OFF
    sendTowerCommand("off", "off", 0);

  },
  [sendTowerCommand]
);

  const triggerConditionAlert = useCallback(() => {
    setConditionAlertOpen(true);
    setConditionAlertCountdown(30);
    setConditionAlertAcknowledged(false);
    sendSirenPreset("warning");

    if (conditionAlertTimerRef.current) {
      window.clearTimeout(conditionAlertTimerRef.current);
    }

    if (conditionAlertIntervalRef.current) {
      window.clearInterval(conditionAlertIntervalRef.current);
    }

    if (conditionAlertCloseRef.current) {
      window.clearTimeout(conditionAlertCloseRef.current);
    }

    conditionAlertIntervalRef.current = window.setInterval(() => {
      setConditionAlertCountdown((prev) => (prev > 0 ? prev - 1 : 0));
    }, 1000);

    conditionAlertTimerRef.current = window.setTimeout(() => {
      if (conditionAlertIntervalRef.current) {
        window.clearInterval(conditionAlertIntervalRef.current);
      }
      setConditionAlertAcknowledged(false);
      setConditionAlertOpen(false);
      showToast("30초가 지나 작업자 상태가 상부로 자동 보고되었습니다.", {
        title: "상부 보고",
        level: "danger",
      });
    }, 30000);
  }, [sendSirenPreset, showToast]);

  const acknowledgeConditionAlert = useCallback(() => {
    if (conditionAlertTimerRef.current) {
      window.clearTimeout(conditionAlertTimerRef.current);
    }

    if (conditionAlertIntervalRef.current) {
      window.clearInterval(conditionAlertIntervalRef.current);
    }

    if (conditionAlertCloseRef.current) {
      window.clearTimeout(conditionAlertCloseRef.current);
    }

    setConditionAlertAcknowledged(true);
    showToast("컨디션 저조 경고 확인 완료", {
      title: "경고 확인",
      level: "success",
    });

    conditionAlertCloseRef.current = window.setTimeout(() => {
      setConditionAlertOpen(false);
      setConditionAlertAcknowledged(false);
    }, 1500);
  }, [showToast]);

  useEffect(() => {
    if (!keepGoingDirection || robotMode !== "manual") return;

    const timer = window.setInterval(() => {
      publishCmdVel(keepGoingDirection === "forward" ? -0.2 : 0.3, 0.0);
      sendSirenPreset("moving");
    }, 180);

    return () => window.clearInterval(timer);
  }, [keepGoingDirection, publishCmdVel, robotMode, sendSirenPreset]);

  /* ─── mmWave / 생체값 / 그리고 소리.. ───────────────────────────────── */
  const [resp, setResp] = useState<number | null>(null);
  const [hr, setHr] = useState<number | null>(null);
  const [conf, setConf] = useState<number | null>(null);

  const [connected, setConnected] = useState(false);
  const [, setRespHistory] = useState<number[]>([]);
  const [, setHrHistory] = useState<number[]>([]);

  const [mmwaveEnabled, setMmwaveEnabled] = useState<boolean | null>(null);
  const [hasTarget, setHasTarget] = useState<boolean | null>(null);

  const [soundValue, setSoundValue] = useState<number | null>(null);
  const [soundUpdatedAt, setSoundUpdatedAt] = useState<number | null>(null);
  const soundDangerTimer = useRef<number | null>(null);

  useEffect(() => {
  if (soundValue == null) return;

  if (soundValue >= 100) {
    if (soundDangerTimer.current) return;

    soundDangerTimer.current = window.setTimeout(() => {
      sendSirenPreset("sound");

      showToast("소음 위험 감지", {
        title: "소음 경고",
        level: "warning",
      });

      soundDangerTimer.current = null;
    }, 2000);
  } else {
    if (soundDangerTimer.current) {
      clearTimeout(soundDangerTimer.current);
      soundDangerTimer.current = null;
    }
  }
}, [soundValue, sendSirenPreset, showToast]);

  /* ─── Thermoeye ─────────────────────────────────────── */
  const [bodyTemp, setBodyTemp] = useState<number | null>(null);
  const [thermoDeviceConnected, setThermoDeviceConnected] =
    useState<boolean>(false);
  const [thermoStreamConnected, setThermoStreamConnected] =
    useState<boolean>(false);
  const [yoloStreamConnected, setYoloStreamConnected] =
    useState<boolean>(false);

  const thermoConnected = thermoDeviceConnected || thermoStreamConnected;
  const yoloConnected = yoloStreamConnected;

  /* ─── PPE 상태 ─────────────────── */
  const [ppeHelmet, setPpeHelmet] = useState<boolean>(true);
  const [ppeVest, setPpeVest] = useState<boolean>(true);
  const [ppeGloves, setPpeGloves] = useState<boolean>(false);
  const [ppeAllDetected, setPpeAllDetected] = useState<boolean | null>(null);

  /* ─── 임계값 ────────────────────────────────────────── */
  const [respMin, setRespMin] = useState(8);
  const [respMax, setRespMax] = useState(24);
  const [hrMin, setHrMin] = useState(50);
  const [hrMax, setHrMax] = useState(120);

  const [tempWarn, setTempWarn] = useState(37.5);
  const [tempDanger, setTempDanger] = useState(38.5);

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

  useEffect(() => {
  console.log("🔥 thermal/PPE useEffect start");

  const ros = new WebSocket("ws://localhost:9090");

  ros.onopen = () => {
    console.log("✅ ROS bridge connected for thermal + PPE");

    ros.send(
      JSON.stringify({
        op: "subscribe",
        topic: "/thermal/face_temp",
      })
    );

    ros.send(
      JSON.stringify({
        op: "subscribe",
        topic: "/all_detected",
      })
    );
  };

  ros.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);

      if (msg.op === "publish" && msg.topic === "/thermal/face_temp") {
        const value = Number(msg.msg?.data);

        console.log("🌡️ BODY TEMP:", value);

        if (!Number.isNaN(value)) {
          setBodyTemp(value);
          setThermoDeviceConnected(true);
        }

        return;
      }

      if (msg.op === "publish" && msg.topic === "/all_detected") {
        const detected = Boolean(msg.msg?.data);

        console.log("🦺 PPE ALL DETECTED:", detected);

        setPpeAllDetected(detected);

        return;
      }
    } catch (err) {
      console.warn("thermal/PPE ros parse error:", err);
    }
  };

  ros.onerror = () => {
    console.log("❌ ROS bridge thermal/PPE connection error");
    setThermoDeviceConnected(false);
  };

  ros.onclose = () => {
    console.log("❌ ROS bridge thermal/PPE connection closed");
    setThermoDeviceConnected(false);
  };

  return () => {
    ros.close();
  };
}, []);

  /* ─── 데이터 ─────────────────────────────────────────── */
  const [workers, setWorkers] = useState<Worker[]>([]);

  const [hazards, setHazards] = useState<Hazard[]>([
    { id: "H-1", x: 24, y: 50, status: "open" },
    { id: "H-2", x: 63, y: 32, status: "open" },
    { id: "H-3", x: 78, y: 67, status: "resolved" },
  ]);
  const [selectedHazard, setSelectedHazard] = useState<Hazard | null>(null);

  const [form, setForm] = useState<{ id: string; name: string; role?: string }>(
    { id: "", name: "", role: "" }
  );

  /* ───────────────────────── WASD 수동 제어 ───────────────────────── */

  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (!robotModalOpen || robotMode !== "manual") return;

      const key = e.key.toLowerCase();

      if (key === "w") {
        publishCmdVel(-0.2, 0.0);
        sendSirenPreset("moving");
      } else if (key === "s") {
        publishCmdVel(0.3, 0.0);
        sendSirenPreset("moving");
      } else if (key === "a") {
        publishCmdVel(0.0, 0.6);
        sendSirenPreset("moving");
      } else if (key === "d") {
        publishCmdVel(0.0, -0.6);
        sendSirenPreset("moving");
      } else if (key === " ") {
        e.preventDefault();
        stopRobot();
      }
    };

    const handleKeyUp = (e: KeyboardEvent) => {
      if (!robotModalOpen || robotMode !== "manual") return;

      const key = e.key.toLowerCase();
      if (["w", "a", "s", "d"].includes(key)) {
        stopRobot();
      }
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
    };
  }, [robotModalOpen, robotMode, publishCmdVel, stopRobot, sendSirenPreset]);

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

  // 🫁 호흡
  if (entityId === "sensor.mr60bha2_breath_rate") {
    const v = parseFloat(stateStr);
    if (!Number.isNaN(v)) {
      setResp(v);
      setRespHistory((p) => [...p.slice(-99), v]);

      if (v > 0) setHasTarget(true);
    }
  }

  // ❤️ 심박
  if (entityId === "sensor.mr60bha2_heart_rate") {
    const v = parseFloat(stateStr);
    if (!Number.isNaN(v)) {
      setHr(v);
      setHrHistory((p) => [...p.slice(-99), v]);

      if (v > 0) setHasTarget(true);
    }
  }

  // 📏 거리값
if (entityId === "sensor.mr60bha2_distance") {
  const v = parseFloat(stateStr);

  if (!Number.isNaN(v) && v > 0 && v < 300) {
    setHasTarget(true);
  }
}

// 👤 사람 감지 센서
if (entityId === "binary_sensor.mr60bha2_person_detected") {
  setHasTarget(stateStr === "on");
}

// 🎯 타겟 수
if (entityId === "sensor.mr60bha2_target_count") {
  const v = parseFloat(stateStr);

  if (!Number.isNaN(v)) {
    setHasTarget(v > 0);
  }
}

  // 기존 로직 유지
  if (entityId === "input_boolean.mmwave_enabled") {
    setMmwaveEnabled(stateStr === "on");
  }

  if (entityId === "sensor.mmwave_confidence") {
    const v = parseFloat(stateStr);
    if (!Number.isNaN(v)) setConf(v);
  }

  if (entityId === "sensor.thermoeye_body_temperature") {
    const v = parseFloat(stateStr);
    if (!Number.isNaN(v)) setBodyTemp(v);
  }

  if (entityId === "binary_sensor.thermoeye_connected") {
    setThermoDeviceConnected(stateStr === "on");
  }

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
          console.log("HA WS MSG:", msg.event?.data?.new_state); 

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

  /* ───────────────────────── RFID / backend WebSocket ───────────────────────── */
  useEffect(() => {
    let rws: WebSocket | null = null;
    let retryTimer: number | null = null;
    let stopped = false;

    const connect = () => {
      if (stopped) return;

      rws = new WebSocket(RFID_WS_URL);

      rws.onopen = () => {
        console.log("✅ RFID WS connected");
        rwsRef.current = rws;
        setBackendWsConnected(true);
      };

      rws.onmessage = (ev) => {
        try {
          const raw = JSON.parse(ev.data);
           if (raw?.type === "sound") {
            const value = Number(raw?.value);

              if (!Number.isNaN(value)) {
                 setSoundValue(value);
                setSoundUpdatedAt(Date.now());
            }

            return;
          }

           if (raw?.type === "server") {
  return;
           }

          if (raw?.type === "rosbridge") {
            setBackendRosConnected(raw?.status === "open");
            return;
          }

          if (raw?.type === "robot_mode") {
            if (
              raw?.mode === "manual" ||
              raw?.mode === "following" ||
              raw?.mode === "patrol"
            ) {
              setRobotMode(raw.mode);
            }
            return;
          }

          if (raw?.type === "ppe_all_detected") {
            const detected = Boolean(raw?.value);
            setPpeAllDetected(detected);

            if (lastPpeDetectedRef.current !== detected) {
              showToast(detected ? "PPE 전체 감지 완료" : "PPE 전체 감지 실패", {
                title: "PPE 판정",
                level: detected ? "success" : "warning",
                save: false,
              });
              lastPpeDetectedRef.current = detected;
            }
            return;
          }

          const uid =
            raw?.uid ?? raw?.tag ?? raw?.nuid ?? raw?.card_uid ?? "";

          if (raw?.type === "unknown") {
            const msg =
              raw?.displayMessage ??
              raw?.message ??
              `미등록 카드 감지 (${uid}) — 관리자에게 문의하세요.`;

            showToast(msg, {
              title: "보안 경고",
              level: "danger",
            });

            sendSirenPreset("unregistered");
            return;
          }

          const data = {
            type: raw?.type,
            uid,
            id: raw?.id,
            name: raw?.name,
            role: raw?.role,
            nonce: raw?.nonce,
          };

          console.log("RFID normalized:", data);

          if (data.type === "admin") {
            sendSirenPreset("admin");
          }

          if (data.type === "worker") {
            sendSirenPreset("registered");
          }

          if (
            data.type === "serial" ||
            data.type === "rosbridge" ||
            data.type === "server"
          ) {
            return;
          }

          handleRFID({
            data,
            showToast,
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
        rwsRef.current = null;
        setBackendWsConnected(false);
        setBackendRosConnected(false);

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
      rwsRef.current = null;
    };
  }, [grantAdminFor10s, showToast, sendSirenPreset]);

  /* ───────────────────────── 연결 감지 ───────────────────────── */
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
  const CHECK_TIMEOUT_MS = 15000;
  const PASS_STREAK_REQUIRED = 2;
  const FAIL_IMMEDIATE_BLOCK = false;

  const [passStreak, setPassStreak] = useState<Record<string, number>>({});
  const passStreakRef = useRef<Record<string, number>>({});
  useEffect(() => {
    passStreakRef.current = passStreak;
  }, [passStreak]);

  const evaluateGate = useCallback(() => {
    const fails: string[] = [];
    const warns: string[] = [];

console.log("🧪 GATE DEBUG", {
  ppeAllDetected,
  ppeHelmet,
  ppeVest,
  ppeGloves,
  hasTarget,
  resp,
  hr,
  bodyTemp,
  thermoConnected:
    thermoDeviceConnected || thermoStreamConnected,
});

    if (ppeAllDetected === null) {
      if (!ppeHelmet) fails.push("Helmet 미착용");
      if (!ppeVest) fails.push("Vest 미착용");
      if (!ppeGloves) fails.push("Gloves 미착용");
    } else if (ppeAllDetected !== true) {
      fails.push("PPE 전체 미통과");
    }

    if (!(thermoDeviceConnected || thermoStreamConnected))
      fails.push("Thermoeye 미연결");

    if (bodyTemp == null) fails.push("체온 데이터 없음");
    else {
      if (bodyTemp >= tempDanger)
        fails.push(`체온 위험(${bodyTemp.toFixed(1)}°C)`);
      else if (bodyTemp >= tempWarn)
        warns.push(`체온 경고(${bodyTemp.toFixed(1)}°C)`);
    }

    if (hasTarget !== true) fails.push("mmWave 타겟 미감지/미확인");

    if (resp == null) fails.push("호흡 데이터 없음");
    else if (resp < respMin || resp > respMax)
      fails.push(`호흡 비정상(${resp.toFixed(1)})`);

    if (hr == null) fails.push("심박 데이터 없음");
    else if (hr < hrMin || hr > hrMax)
      fails.push(`심박 비정상(${hr.toFixed(0)})`);

console.log("🚨 FAIL REASONS", fails);
console.log("⚠️ WARNINGS", warns);
console.log("✅ FINAL PASS", fails.length === 0);

    return { pass: fails.length === 0, fails, warns };
  }, [
    ppeAllDetected,
    ppeHelmet,
    ppeVest,
    ppeGloves,
    thermoDeviceConnected,
    thermoStreamConnected,
    bodyTemp,
    hasTarget,
    resp,
    hr,
    respMin,
    respMax,
    hrMin,
    hrMax,
    tempWarn,
    tempDanger,
  ]);

  // 💡 추가된 부분: evaluateGate 함수의 최신 상태를 담아둘 ref 선언
  const evaluateGateRef = useRef(evaluateGate);

  // 💡 추가된 부분: evaluateGate가 갱신될 때마다 ref 값을 업데이트
  useEffect(() => {
    evaluateGateRef.current = evaluateGate;
  }, [evaluateGate]);

  useEffect(() => {
  const t = window.setInterval(() => {
    console.log("🔄 AUTO CHECK RUNNING");
    
    // 💡 수정된 부분: evaluateGate() 대신 evaluateGateRef.current() 사용
    const { pass: gatePass, fails, warns } = evaluateGateRef.current();
    const nowTs = Date.now();

    const nextStreak: Record<string, number> = {
      ...passStreakRef.current,
    };

    let toastMsg: string | null = null;

    setWorkers((prev) =>
      prev.map((w) => {
        if (w.status !== "pending") return w;

        const elapsed = nowTs - w.checkinStartedAt;

        if (elapsed > CHECK_TIMEOUT_MS) {
          delete nextStreak[w.sessionKey];

          return {
            ...w,
            status: "blocked",
            failReasons: fails.length ? fails : ["시간 초과"],
            warnings: warns,
          };
        }

        if (!gatePass) {
  console.log("❌ GATE FAIL", fails);

  return {
    ...w,
    status: "pending",
    failReasons: fails,
    warnings: warns,
  };
}

const cur = passStreakRef.current[w.sessionKey] ?? 0;
const updated = cur + 1;

nextStreak[w.sessionKey] = updated;

console.log("✅ PASS COUNT", {
  worker: w.name,
  updated,
  required: PASS_STREAK_REQUIRED,
});

        console.log("✅ PASS COUNT", {
         worker: w.name,
         updated,
        required: PASS_STREAK_REQUIRED,
      });

        if (updated >= PASS_STREAK_REQUIRED) {
          delete nextStreak[w.sessionKey];
          toastMsg = `${w.name} 체크인 완료`;

          return {
            ...w,
            status: "active",
            failReasons: [],
            warnings: warns,
          };
        }

        return {
          ...w,
          status: "pending",
          failReasons: [],
          warnings: warns,
        };
      })
    );

    setPassStreak(nextStreak);

    if (toastMsg) {
      showToast(toastMsg, {
        title: "출입 체크인",
        level: "success",
      });
    }
  }, CHECK_INTERVAL_MS);

  return () => window.clearInterval(t);
}, [showToast]); // 💡 수정된 부분: evaluateGate 의존성을 제거하여 무한 리셋 방지

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
      showToast("재검사 시작 — 자동 점검 중", {
        title: "재검사",
        level: "warning",
      });
    },
    [showToast]
  );

  const submitWorker = useCallback(() => {
    const id = form.id.trim();
    const name = form.name.trim();
    const role = form.role?.trim() || "";

    if (!id || !name) {
      showToast("이름과 ID는 필수입니다.", {
        title: "입력 오류",
        level: "warning",
      });
      return;
    }

    const nowTs = Date.now();
    const sessionKey = makeWorkerKey(undefined, id);

    if (!sessionKey) {
      showToast("유효한 ID/UID가 없습니다.", {
        title: "등록 실패",
        level: "danger",
      });
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

    setWorkers((prev) => upsertWorker(prev, w));
    setPassStreak((s) => ({ ...s, [sessionKey]: 0 }));
    setForm({ id: "", name: "", role: "" });
    setWorkerModalOpen(false);
    showToast("등록/재검사 시작 (대기 중) — 자동 점검 중", {
      title: "작업자 등록",
      level: "info",
    });
    sendSirenPreset("registered");
  }, [form.id, form.name, form.role, showToast, sendSirenPreset]);

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
          : "이상 없음으로 반영되었습니다.",
        {
          title: "위험 로그",
          level: "success",
        }
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
    if (ppeAllDetected !== null) return ppeAllDetected ? 100 : 0;
    const ok = [ppeHelmet, ppeVest, ppeGloves].filter(Boolean).length;
    return Math.round((ok / 3) * 100);
  }, [ppeAllDetected, ppeHelmet, ppeVest, ppeGloves]);

  const activeWorkers = useMemo(
    () => workers.filter((w) => w.status === "active").length,
    [workers]
  );

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

      <header className="flex justify-between items-center border-b border-gray-700 pb-4">
        <h1 className="text-3xl font-bold text-[#FFD400]">FIFO Safety Hub</h1>
        <div className="flex items-center space-x-4 text-gray-400">
          <p>{formatTime(now)} | Zone A</p>

          <button
            type="button"
            onClick={() => setNotifOpen(true)}
            className="relative rounded-full p-2 hover:bg-white/10 transition"
          >
            <Bell className="text-[#FFD400]" />
            {notifications.length > 0 && (
              <span className="absolute -top-1 -right-1 min-w-[18px] h-[18px] px-1 rounded-full bg-red-500 text-[10px] text-white flex items-center justify-center font-bold">
                {notifications.length > 99 ? "99+" : notifications.length}
              </span>
            )}
          </button>
        </div>
      </header>

      <div className="grid grid-cols-4 gap-4">
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
              <p className="text-xs text-gray-400">현재 PPE 기준: {ppeRate}%</p>

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

              <p>
                Backend WS:{" "}
                <span
                  className={backendWsConnected ? "text-green-400" : "text-red-400"}
                >
                  {backendWsConnected ? "연결됨" : "끊김"}
                </span>
              </p>

              <p>
                ROS bridge:{" "}
                <span
                  className={
                    backendRosConnected ? "text-green-400" : "text-red-400"
                  }
                >
                  {backendRosConnected ? "연결됨" : "끊김"}
                </span>
              </p>

              <p>
                로봇 모드:{" "}
                <span className="text-[#FFD400] font-semibold">{robotMode}</span>
              </p>
                <p>
                 소음:{" "}
                 <span
                 className={
                 soundValue == null
                  ? "text-gray-400"
                  : soundValue >= 85
                  ? "text-red-400 font-semibold"
                  : soundValue >= 70
                 ? "text-yellow-400 font-semibold"
                 : "text-green-400 font-semibold"
                  }
                 >
                 {soundValue == null ? "--" : `${soundValue.toFixed(1)} dB`}
                 </span>
               </p>
            </div>
          </CardContent>
        </Card>

        <Card className="bg-[#1A1A1A] border border-[#2A2A2A] col-span-2 h-[460px]">
          <CardContent className="p-4">
            <h2 className="text-lg font-semibold text-[#FFD400] mb-2">
              실시간 현장 맵
            </h2>

            <div className="relative h-72 bg-[#121212] rounded-md overflow-hidden border border-[#2A2A2A]">
              <img
                src={SLAM_STREAM_URL}
                alt="SLAM map"
                className="absolute inset-0 h-full w-full object-contain bg-black"
              />

              <div className="absolute top-3 left-3 text-xs text-gray-300 bg-black/50 px-3 py-1 rounded">
                SLAM 실시간 맵
              </div>

              <div className="absolute bottom-3 right-3 text-xs text-gray-300 bg-black/50 px-3 py-1 rounded">
                /map 기반 실시간 뷰
              </div>
            </div>

            <div className="mt-3 text-xs text-gray-400">
              로봇 위치, 순찰 경로, 작업자 위치 연동용 영역
            </div>
          </CardContent>
        </Card>

        <Card className="bg-[#1A1A1A] border border-[#2A2A2A] col-span-1 h-[460px]">
          <CardContent className="p-4 space-y-3">
            <h2 className="text-lg font-semibold text-[#FFD400]">
              실시간 PPE 인식
            </h2>

            <div className="relative h-56 bg-[#111] rounded-md overflow-hidden border border-[#2A2A2A]">
  <img
    src={YOLO_STREAM_URL}
    alt="PPE YOLO stream"
    className="absolute inset-0 w-full h-full object-contain bg-black"
    onLoad={() => setYoloStreamConnected(true)}
    onError={() => setYoloStreamConnected(false)}
  />

  <div className="absolute top-2 left-2 text-xs text-gray-300 bg-black/50 px-2 py-1 rounded">
    {yoloConnected ? "YOLO 연결됨" : "YOLO 연결 대기"}
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

  <div className="absolute left-2 bottom-2 text-xs bg-black/60 px-2 py-1 rounded flex flex-wrap gap-2">
    <span className={ppeHelmet ? "text-emerald-300" : "text-red-300"}>
      Helmet {ppeHelmet ? "✅" : "❌"}
    </span>
    <span className={ppeVest ? "text-emerald-300" : "text-red-300"}>
      Vest {ppeVest ? "✅" : "❌"}
    </span>
    <span className={ppeGloves ? "text-emerald-300" : "text-red-300"}>
      Gloves {ppeGloves ? "✅" : "❌"}
    </span>
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

            <p className="text-xs text-gray-400">
              전체 PPE 판정:{" "}
              {ppeAllDetected == null
                ? "--"
                : ppeAllDetected
                ? "모두 감지됨"
                : "미감지"}
            </p>

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

      {vitalsOpen && (
        <Modal
          title="실시간 PPE + 열화상 + 호흡/심박 모니터링"
          onClose={() => setVitalsOpen(false)}
        >
          <div className="grid grid-cols-2 gap-4 w-full">
            <div className="relative h-[360px] w-full bg-[#111] rounded-lg overflow-hidden border border-[#2A2A2A]">
              <img
                ref={(el) => {
                  yoloImgRef.current = el;
                }}
                src={YOLO_STREAM_URL}
                alt="YOLO stream"
                className="absolute inset-0 w-full h-full object-contain bg-black pointer-events-none"
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

            <div className="relative h-[360px] w-full bg-[#111] rounded-lg overflow-hidden border border-[#2A2A2A]">
              <img
                ref={(el) => {
                  thermoImgRef.current = el;
                }}
                src={THERMO_STREAM_URL}
                alt="Thermoeye thermal stream"
                className="absolute inset-0 w-full h-full object-contain bg-black pointer-events-none"
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
            <span className="ml-3">
              · PPE 전체:{" "}
              {ppeAllDetected == null
                ? "--"
                : ppeAllDetected
                ? "감지됨"
                : "미감지"}
            </span>
          </div>

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

      {workerModalOpen && (
        <Modal title="작업자 등록" onClose={() => setWorkerModalOpen(false)}>
          <div className="h-[calc(86vh-56px)] grid grid-rows-[300px_auto_1fr] gap-4">
            <div className="grid items-start grid-cols-[minmax(0,2fr)_minmax(0,1fr)] gap-4">
              <div className="relative h-[300px] w-full bg-[#111] rounded-lg overflow-hidden border border-[#2A2A2A]">
                <img
                  src={YOLO_STREAM_URL}
                  alt="YOLO preview"
                  className="absolute inset-0 w-full h-full object-contain bg-black pointer-events-none"
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

              <div className="h-[300px] bg-[#111] rounded-lg border border-[#2A2A2A] p-4">
                <div className="text-sm font-semibold text-[#FFD400] mb-3">
                  등록 게이트 상태
                </div>

                <div className="space-y-2 text-sm">
                  <div className="flex items-center justify-between">
                    <span>PPE 최종 판정</span>
                    <span
                      className={
                        ppeAllDetected === true
                          ? "text-emerald-300"
                          : "text-red-300"
                      }
                    >
                      {ppeAllDetected === true ? "✅ 통과" : "❌ 미통과"}
                    </span>
                  </div>

                  <div className="flex items-center justify-between">
                   <span>mmWave 생체값</span>

                   <span
                    className={
                     hasTarget &&
                     resp != null &&
                     hr != null &&
                     resp >= respMin &&
                     resp <= respMax &&
                     hr >= hrMin &&
                     hr <= hrMax
                     ? "text-emerald-300"
                     : "text-red-300"
                }
              >
                 {hasTarget &&
                 resp != null &&
                 hr != null &&
                 resp >= respMin &&
                 resp <= respMax &&
                 hr >= hrMin &&
                 hr <= hrMax
                  ? "✅ 통과"
                  : "❌ 미통과"}
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
                    ※ PPE 전체 판정은 ROS2 /all_detected 값을 사용합니다.
                  </div>
                </div>
              </div>
            </div>

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

            <div className="grid grid-rows-[auto_1fr]">
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

              <div className="mt-3">
                <p className="text-sm text-gray-300 mb-2">최근 등록된 작업자</p>

                <div className="h-[calc(100%-28px)] overflow-auto space-y-2 text-sm text-gray-300 pr-1">
                  {listMain.map((w) => (
                    <WorkerRow
                      key={w.sessionKey}
                      w={w}
                      deleteWorker={deleteWorker}
                      retryWorker={retryWorker}
                      PASS_STREAK_REQUIRED={PASS_STREAK_REQUIRED}
                    />
                  ))}

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

      {hazardModalOpen && (
  <Modal title="위험 로그" onClose={() => setHazardModalOpen(false)}>
    <div className="space-y-5">
      <div className="rounded-[24px] border border-white/10 bg-[#141414] p-4 shadow-[0_18px_40px_rgba(0,0,0,0.28)]">
        <div className="flex items-center justify-between mb-3">
          <div>
            <p className="text-[22px] font-semibold tracking-[-0.03em] text-white">
              Hazard Monitoring
            </p>
            <p className="mt-1 text-sm text-gray-500">
              위험 지점을 선택하고 상태를 즉시 갱신합니다.
            </p>
          </div>

          <div className="rounded-full border border-white/10 bg-[#1A1A1A] px-3 py-1.5 text-xs tracking-[0.08em] text-gray-400">
            TOTAL · {hazards.length}
          </div>
        </div>

        <HazardMap
          hazards={hazards}
          selectedId={selectedHazard?.id ?? null}
          onSelect={(h) => setSelectedHazard(h)}
        />
      </div>

      <div className="grid grid-cols-[1.15fr_0.85fr] gap-4">
        <div className="rounded-[22px] border border-white/10 bg-[#141414] p-5 shadow-[0_14px_34px_rgba(0,0,0,0.24)]">
          <p className="text-lg font-semibold text-white">위험 상세</p>
          <p className="mt-1 text-sm text-gray-500">
            맵의 위험 지점을 선택하면 상세 정보가 표시됩니다.
          </p>

          <div className="mt-4 overflow-hidden rounded-[20px] border border-red-500/20 bg-black">
           <div className="border-b border-white/10 px-4 py-2">
            <p className="text-sm font-semibold text-red-300">
            장애물 탐지 실시간 영상
            </p>
       </div>

       <img
       src="http://localhost:8080/stream?topic=/image_debug&type=mjpeg"
       alt="Obstacle Detection"
       className="h-[260px] w-full object-cover"
       />
    </div>

          {selectedHazard ? (
            <div className="mt-4 space-y-4">
              <div className="grid grid-cols-2 gap-3">
                <HazardInfoCard
                  label="HAZARD ID"
                  value={selectedHazard.id}
                  accent="yellow"
                />
                <HazardInfoCard
                  label="STATUS"
                  value={
                    selectedHazard.status === "open"
                      ? "OPEN"
                      : selectedHazard.status === "resolved"
                      ? "RESOLVED"
                      : "FALSE ALARM"
                  }
                  accent={
                    selectedHazard.status === "open"
                      ? "red"
                      : selectedHazard.status === "resolved"
                      ? "green"
                      : "gray"
                  }
                />
                <HazardInfoCard
                  label="X POSITION"
                  value={`${selectedHazard.x}%`}
                />
                <HazardInfoCard
                  label="Y POSITION"
                  value={`${selectedHazard.y}%`}
                />
              </div>

              <div className="rounded-[18px] border border-white/10 bg-[#101010] px-4 py-4">
                <p className="text-[11px] uppercase tracking-[0.18em] text-gray-500">
                  ACTION
                </p>
                <p className="mt-2 text-sm text-gray-300 leading-6">
                  선택된 위험 지점의 현재 상태를 반영합니다. 실제 현장 처리 완료 시
                  <span className="text-emerald-300"> 문제 해결</span>, 오탐이면
                  <span className="text-gray-300"> 이상 없음</span>으로 처리하세요.
                </p>

                <div className="mt-4 flex gap-3">
                  <Button
                    className="flex-1 h-12 rounded-xl border border-emerald-500/25 bg-[#10241C] text-emerald-200 hover:border-emerald-400 hover:bg-[#123126]"
                    onClick={() => markHazard("resolved")}
                  >
                    문제 해결
                  </Button>
                  <Button
                    className="flex-1 h-12 rounded-xl border border-white/10 bg-[#1A1A1A] text-gray-200 hover:border-[#FFD400] hover:bg-[#202020]"
                    onClick={() => markHazard("false_alarm")}
                  >
                    이상 없음
                  </Button>
                </div>
              </div>
            </div>
          ) : (
            <div className="mt-4 rounded-[18px] border border-dashed border-white/10 bg-[#101010] px-5 py-10 text-center">
              <p className="text-sm text-gray-400">
                아직 선택된 위험 지점이 없습니다.
              </p>
              <p className="mt-2 text-xs text-gray-500">
                맵에서 빨간/초록/회색 점을 클릭해 주세요.
              </p>
            </div>
          )}
        </div>

        <div className="rounded-[22px] border border-white/10 bg-[#141414] p-5 shadow-[0_14px_34px_rgba(0,0,0,0.24)]">
          <p className="text-lg font-semibold text-white">상태 기준</p>
          <div className="mt-4 space-y-3">
            <LegendRow color="bg-red-500" label="OPEN" desc="미확인 위험" />
            <LegendRow color="bg-emerald-500" label="RESOLVED" desc="문제 해결 완료" />
            <LegendRow color="bg-gray-400" label="FALSE ALARM" desc="이상 없음 / 오탐" />
          </div>

          <div className="mt-6 rounded-[18px] border border-white/10 bg-[#101010] p-4">
            <p className="text-[11px] uppercase tracking-[0.18em] text-gray-500">
              GUIDE
            </p>
            <p className="mt-2 text-sm leading-6 text-gray-400">
              초록 점선은 로봇 순찰 경로입니다. 점을 클릭하면 선택 상태가 강조되고,
              우측 카드에서 즉시 처리할 수 있습니다.
            </p>
          </div>
        </div>
      </div>
    </div>
  </Modal>
)}

      {robotModalOpen && (
        <Modal title="로봇 제어" onClose={() => setRobotModalOpen(false)}>
          <div className="space-y-6">
            <motion.section
              initial={{ opacity: 0, y: 12 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.28, ease: "easeOut" }}
              className="rounded-[30px] border border-white/10 bg-[#141414] p-7 shadow-[0_24px_70px_rgba(0,0,0,0.42)]"
            >
              <div className="flex items-start justify-between gap-4">
                <div>
                  <p className="text-[34px] font-bold tracking-[-0.04em] text-white">
                    ROBOT CONTROL
                  </p>
                  <p className="mt-3 max-w-2xl text-sm leading-6 text-gray-400">
                    기능은 단단하게, 화면은 제품처럼. 모드 전환, 경광등 제어, 수동 조작을
                    하나의 운영 패널로 정리했습니다.
                  </p>
                </div>

                <div className="flex items-center gap-3">
                  <ProductStatusCard
                    label="BACKEND"
                    value={backendWsConnected ? "ONLINE" : "OFFLINE"}
                    tone={backendWsConnected ? "ok" : "danger"}
                  />
                  <ProductStatusCard
                    label="ROS"
                    value={backendRosConnected ? "ONLINE" : "OFFLINE"}
                    tone={backendRosConnected ? "ok" : "danger"}
                  />
                </div>
              </div>
            </motion.section>

            <div className="grid grid-cols-[minmax(0,2fr)_minmax(0,1fr)] gap-4">
              <motion.section
                initial={{ opacity: 0, y: 18 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.3, delay: 0.03, ease: "easeOut" }}
                className="rounded-[26px] border border-white/10 bg-[#141414] px-6 pt-6 pb-4 shadow-[0_18px_50px_rgba(0,0,0,0.35)]"
              >
                <div className="mb-5 flex items-center justify-between">
                  <div>
                    <p className="text-[22px] font-semibold tracking-[-0.03em] text-white">
                      Drive Mode
                    </p>
                    <p className="mt-1 text-sm text-gray-500">
                      현재 운용 모드를 선택합니다
                    </p>
                  </div>
                  <div className="rounded-full border border-white/10 bg-[#1A1A1A] px-3 py-1.5 text-xs font-medium tracking-[0.08em] text-gray-400">
                    CURRENT · {robotMode.toUpperCase()}
                  </div>
                </div>

                <div className="grid grid-cols-3 gap-3">
                  <ProductModeCard
                    title="MANUAL"
                    subtitle="직접 조작"
                    active={robotMode === "manual"}
                    onClick={() => setRobotModeAndPublish("manual")}
                  />
                  <ProductModeCard
                    title="FOLLOWING"
                    subtitle="작업자 추종"
                    active={robotMode === "following"}
                    onClick={() => setRobotModeAndPublish("following")}
                  />
                  <ProductModeCard
                    title="PATROL"
                    subtitle="자동 순찰"
                    active={robotMode === "patrol"}
                    onClick={() => setRobotModeAndPublish("patrol")}
                  />
                </div>

                {robotMode === "manual" ? (
                  <div className="mt-2 h-[396px] overflow-hidden">
                    <div className="grid h-full w-full grid-cols-[minmax(0,1fr)_236px] items-stretch gap-4">
                      <div className="flex h-full items-center justify-center pl-10">
                        <div className="grid grid-cols-[132px_132px_132px] place-items-center justify-center gap-x-2 gap-y-2">
                          <div />
                          <DirectionPadButton
                            label="Forward"
                            direction="up"
                            onMouseDown={() => {
                              setKeepGoingDirection(null);
                              publishCmdVel(-0.2, 0.0);
                              sendSirenPreset("moving");
                            }}
                            onMouseUp={stopRobot}
                            onMouseLeave={stopRobot}
                            onTouchStart={() => {
                              setKeepGoingDirection(null);
                              publishCmdVel(-0.2, 0.0);
                              sendSirenPreset("moving");
                            }}
                            onTouchEnd={stopRobot}
                          />
                          <div />

                          <DirectionPadButton
                            label="Left"
                            direction="left"
                            onMouseDown={() => {
                              publishCmdVel(0.0, 0.6);
                              sendSirenPreset("moving");
                            }}
                            onMouseUp={stopRobot}
                            onMouseLeave={stopRobot}
                            onTouchStart={() => {
                              publishCmdVel(0.0, 0.6);
                              sendSirenPreset("moving");
                            }}
                            onTouchEnd={stopRobot}
                          />

                          <motion.div whileHover={{ y: -2 }} whileTap={{ scale: 0.985 }}>
                            <Button
                              className="h-[132px] w-[132px] rounded-full border border-transparent bg-transparent text-[18px] font-bold tracking-[0.22em] text-[#FFD400] shadow-none hover:border-white/10 hover:bg-[linear-gradient(180deg,#1E1E1E_0%,#161616_100%)] hover:text-white hover:shadow-[inset_0_1px_0_rgba(255,255,255,0.04),0_18px_32px_rgba(0,0,0,0.24)]"
                              onClick={() => {
                                stopRobot();
                                sendSirenPreset("off");
                              }}
                            >
                              STOP
                            </Button>
                          </motion.div>

                          <DirectionPadButton
                            label="Right"
                            direction="right"
                            onMouseDown={() => {
                              publishCmdVel(0.0, -0.6);
                              sendSirenPreset("moving");
                            }}
                            onMouseUp={stopRobot}
                            onMouseLeave={stopRobot}
                            onTouchStart={() => {
                              publishCmdVel(0.0, -0.6);
                              sendSirenPreset("moving");
                            }}
                            onTouchEnd={stopRobot}
                          />

                          <div />
                          <DirectionPadButton
                            label="Backward"
                            direction="down"
                            onMouseDown={() => {
                              setKeepGoingDirection(null);
                              publishCmdVel(0.3, 0.0);
                              sendSirenPreset("moving");
                            }}
                            onMouseUp={stopRobot}
                            onMouseLeave={stopRobot}
                            onTouchStart={() => {
                              setKeepGoingDirection(null);
                              publishCmdVel(0.3, 0.0);
                              sendSirenPreset("moving");
                            }}
                            onTouchEnd={stopRobot}
                          />
                          <div />
                        </div>
                      </div>

                      <div className="flex min-w-[236px] flex-col justify-center items-end gap-3">
                        <div className="w-[164px]">
                          <MiniInfoCard
                            title="State"
                            value={backendWsConnected ? "Ready" : "Disconnected"}
                            highlight={backendWsConnected ? "ok" : "danger"}
                          />
                        </div>
                        <button
                          type="button"
                          onClick={() =>
                            setKeepGoingDirection((prev) =>
                              prev === "forward" ? null : "forward"
                            )
                          }
                          className={`min-h-[78px] w-[164px] rounded-[18px] border px-4 py-3 text-left transition ${
                            keepGoingDirection === "forward"
                              ? "border-[#FFD400]/55 bg-[linear-gradient(180deg,#2A230D_0%,#18140A_100%)] text-[#FFD400] shadow-[0_0_24px_rgba(255,212,0,0.08)]"
                              : "border-white/12 bg-[linear-gradient(180deg,#191919_0%,#141414_100%)] text-white hover:border-[#FFD400]/35 hover:text-[#FFD400]"
                          }`}
                        >
                          <div
                            className={`text-[10px] font-medium uppercase tracking-[0.18em] ${
                              keepGoingDirection === "forward" ? "text-[#A99857]" : "text-gray-500"
                            }`}
                          >
                            Assisted Move
                          </div>
                          <div className="mt-2 text-[13px] font-semibold tracking-[0.08em]">
                            Keep Forward
                          </div>
                        </button>

                        <button
                          type="button"
                          onClick={() =>
                            setKeepGoingDirection((prev) =>
                              prev === "backward" ? null : "backward"
                            )
                          }
                          className={`min-h-[78px] w-[164px] rounded-[18px] border px-4 py-3 text-left transition ${
                            keepGoingDirection === "backward"
                              ? "border-[#FFD400]/55 bg-[linear-gradient(180deg,#2A230D_0%,#18140A_100%)] text-[#FFD400] shadow-[0_0_24px_rgba(255,212,0,0.08)]"
                              : "border-white/12 bg-[linear-gradient(180deg,#191919_0%,#141414_100%)] text-white hover:border-[#FFD400]/35 hover:text-[#FFD400]"
                          }`}
                        >
                          <div
                            className={`text-[10px] font-medium uppercase tracking-[0.18em] ${
                              keepGoingDirection === "backward" ? "text-[#A99857]" : "text-gray-500"
                            }`}
                          >
                            Assisted Move
                          </div>
                          <div className="mt-2 text-[13px] font-semibold tracking-[0.08em]">
                            Keep Backward
                          </div>
                        </button>
                      </div>
                    </div>
                  </div>
                ) : (
                  <div className="mt-2 flex h-[396px] flex-col justify-between overflow-hidden">
                    <div className="relative h-[300px] w-full overflow-hidden rounded-xl border border-[#2A2A2A] bg-[#101010]">
                      <img
                        src={YOLO_STREAM_URL}
                        alt="Robot camera"
                        className="absolute inset-0 h-full w-full object-cover bg-black"
                        onLoad={() => setYoloStreamConnected(true)}
                        onError={() => setYoloStreamConnected(false)}
                      />
                      <div className="absolute left-4 top-4 rounded bg-black/45 px-3 py-1 text-xs text-gray-200">
                        LIVE CAMERA
                      </div>
                      <div className="absolute right-4 top-4 rounded bg-black/45 px-3 py-1 text-xs text-gray-200">
                        {robotMode.toUpperCase()}
                      </div>
                    </div>

                    <div className="grid grid-cols-4 gap-2">
                      <MiniInfoCard
                        title="Mode"
                        value={robotMode === "following" ? "Worker Follow" : "Route Patrol"}
                      />
                      <MiniInfoCard
                        title="Stream"
                        value={yoloConnected ? "Live" : "Waiting"}
                        highlight={yoloConnected ? "ok" : "danger"}
                      />
                      <MiniInfoCard
                        title="State"
                        value={backendWsConnected ? "Ready" : "Disconnected"}
                        highlight={backendWsConnected ? "ok" : "danger"}
                      />
                      <MiniInfoCard
                        title="Target"
                        value={robotMode === "following" ? "Worker" : "Route"}
                      />
                    </div>
                  </div>
                )}
              </motion.section>

              <motion.section
                initial={{ opacity: 0, y: 18 }}
                animate={{ opacity: 1, y: 0 }}
                transition={{ duration: 0.3, delay: 0.07, ease: "easeOut" }}
                className="rounded-[26px] border border-white/10 bg-[#141414] px-6 pt-6 pb-4 shadow-[0_18px_50px_rgba(0,0,0,0.35)]"
              >
                <div className="mb-5">
                  <p className="text-[22px] font-semibold tracking-[-0.03em] text-white">
                    Alarm / Tower Panel
                  </p>
                  <p className="mt-1 text-sm text-gray-500">
                    상황별 경광등 및 사이렌 제어
                  </p>
                </div>

                <div className="grid grid-cols-2 gap-3">
                  <ProductActionButton
                    className="border-red-500/20 bg-[#2A1313] text-red-200 hover:border-red-500/40 hover:bg-[#361717] hover:text-red-100"
                    onClick={() => sendSirenPreset("danger")}
                  >
                    위험
                  </ProductActionButton>
                  <ProductActionButton
                    className="border-yellow-500/20 bg-[#2B2410] text-yellow-200 hover:border-yellow-500/40 hover:bg-[#3A3114] hover:text-yellow-100"
                    onClick={() => sendSirenPreset("warning")}
                  >
                    컨디션 저조
                  </ProductActionButton>
                  <ProductActionButton
                    className="border-amber-500/20 bg-[#2A1E10] text-amber-200 hover:border-amber-500/40 hover:bg-[#352614] hover:text-amber-100"
                    onClick={() => sendSirenPreset("unregistered")}
                  >
                    미등록 카드
                  </ProductActionButton>
                  <ProductActionButton
                    className="border-emerald-500/20 bg-[#10241C] text-emerald-200 hover:border-emerald-500/40 hover:bg-[#153126] hover:text-emerald-100"
                    onClick={() => sendSirenPreset("registered")}
                  >
                    등록 완료
                  </ProductActionButton>
                  <ProductActionButton
                    className="border-sky-500/20 bg-[#10202B] text-sky-200 hover:border-sky-500/40 hover:bg-[#163040] hover:text-sky-100"
                    onClick={() => sendSirenPreset("moving")}
                  >
                    이동 표시
                  </ProductActionButton>
                  <ProductActionButton
                    className="border-orange-500/20 bg-[#2A1B10] text-orange-200 hover:border-orange-500/40 hover:bg-[#382313] hover:text-orange-100"                    onClick={() => sendSirenPreset("off")}
                  >
                    소음 제어
                  </ProductActionButton>
                </div>

                <motion.div whileHover={{ y: -1 }} whileTap={{ scale: 0.99 }}>
                  <Button
                    className="mt-4 h-12 w-full rounded-2xl border border-[#FFD400]/35 bg-[#FFD400] text-black font-semibold shadow-[0_12px_30px_rgba(255,212,0,0.18)] hover:bg-[#EBC400]"
                    onClick={triggerConditionAlert}
                  >
                    컨디션 저조 테스트 팝업
</Button>
</motion.div>

{/* 🔊 SOUND 카드 추가 */}
<div className="mt-3 rounded-[18px] border border-white/10 bg-[#101010] px-4 py-3">
  <div className="flex items-center justify-between">
    <div>
      <p className="text-[11px] uppercase tracking-[0.18em] text-gray-500">
        SOUND LEVEL
      </p>
      <p className="mt-1 text-xs text-gray-400">실시간 소음 센서</p>
    </div>

    <div
      className={
        soundValue == null
          ? "text-gray-400"
          : soundValue >= 85
          ? "text-red-300"
          : soundValue >= 75
          ? "text-yellow-300"
          : "text-emerald-300"
      }
    >
      <span className="text-2xl font-bold">
        {soundValue == null ? "--" : soundValue}
      </span>
    </div>
  </div>

  <div className="mt-2 h-2 overflow-hidden rounded-full bg-white/10">
    <div
      className={
        "h-full rounded-full transition-all " +
        (soundValue == null
          ? "bg-gray-500"
          : soundValue >= 85
          ? "bg-red-500"
          : soundValue >= 75
          ? "bg-yellow-400"
          : "bg-emerald-500")
      }
      style={{
        width: `${Math.min(100, Math.max(0, ((soundValue ?? 0) / 200) * 100))}%`,
      }}
    />
  </div>

  <p className="mt-2 text-[11px] text-gray-500">
    {soundUpdatedAt == null
      ? "대기 중"
      : `업데이트 ${formatDateTime(soundUpdatedAt)}`}
  </p>
</div>
              </motion.section>
            </div>

            <motion.section
              initial={{ opacity: 0, y: 20 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.32, delay: 0.11, ease: "easeOut" }}
              className="hidden rounded-[26px] border border-white/10 bg-[#141414] p-6 shadow-[0_18px_56px_rgba(0,0,0,0.38)]"
            >
              <div className="mb-5 flex items-center justify-between">
                <div>
                  <p className="text-[22px] font-semibold tracking-[-0.03em] text-white">
                    Movement Control
                  </p>
                  <p className="mt-1 text-sm text-gray-500">
                    manual 모드 전용 · WASD / Space 지원
                  </p>
                </div>
                <div className="rounded-2xl border border-white/10 bg-[#1A1A1A] px-4 py-2 text-right">
                  <p className="text-[11px] uppercase tracking-[0.18em] text-gray-500">
                    Target
                  </p>
                  <p className="mt-1 text-sm font-semibold text-[#FFD400]">
                    Sub Robot
                  </p>
                </div>
              </div>

              <div className="grid grid-cols-[1fr_340px_1fr] items-center gap-8">
                <div className="space-y-3">
                  <MiniInfoCard title="Input" value="W / A / S / D" />
                  <MiniInfoCard title="Stop" value="Space / Center" />
                  <MiniInfoCard title="Control" value="Press & Hold" />
                </div>

                <div className="mx-auto grid w-[340px] grid-cols-3 gap-4">
                  <div />
                  <DirectionPadButton
                    label="Forward"
                    direction="up"
                    onMouseDown={() => {
                      publishCmdVel(0.3, 0.0);
                      sendSirenPreset("moving");
                    }}
                    onMouseUp={stopRobot}
                    onMouseLeave={stopRobot}
                    onTouchStart={() => {
                      publishCmdVel(0.3, 0.0);
                      sendSirenPreset("moving");
                    }}
                    onTouchEnd={stopRobot}
                  />
                  <div />

                  <DirectionPadButton
                    label="Left"
                    direction="left"
                    onMouseDown={() => {
                      publishCmdVel(0.0, 0.6);
                      sendSirenPreset("moving");
                    }}
                    onMouseUp={stopRobot}
                    onMouseLeave={stopRobot}
                    onTouchStart={() => {
                      publishCmdVel(0.0, 0.6);
                      sendSirenPreset("moving");
                    }}
                    onTouchEnd={stopRobot}
                  />

                  <motion.div whileHover={{ y: -2 }} whileTap={{ scale: 0.985 }}>
                    <Button
                      className="h-24 w-24 rounded-[22px] border border-[#FFD400]/20 bg-[#FFD400] text-[11px] font-bold tracking-[0.12em] text-black shadow-[0_14px_30px_rgba(255,212,0,0.16)] hover:bg-[#EBC400]"
                      onClick={() => {
                        stopRobot();
                        sendSirenPreset("off");
                      }}
                    >
                      STOP
                    </Button>
                  </motion.div>

                  <DirectionPadButton
                    label="Right"
                    direction="right"
                    onMouseDown={() => {
                      publishCmdVel(0.0, -0.6);
                      sendSirenPreset("moving");
                    }}
                    onMouseUp={stopRobot}
                    onMouseLeave={stopRobot}
                    onTouchStart={() => {
                      publishCmdVel(0.0, -0.6);
                      sendSirenPreset("moving");
                    }}
                    onTouchEnd={stopRobot}
                  />

                  <div />
                  <DirectionPadButton
                    label="Backward"
                    direction="down"
                    onMouseDown={() => {
                      publishCmdVel(-0.2, 0.0);
                      sendSirenPreset("moving");
                    }}
                    onMouseUp={stopRobot}
                    onMouseLeave={stopRobot}
                    onTouchStart={() => {
                      publishCmdVel(-0.2, 0.0);
                      sendSirenPreset("moving");
                    }}
                    onTouchEnd={stopRobot}
                  />
                  <div />
                </div>

                <div className="space-y-3">
                  <MiniInfoCard title="Linear" value="0.3 / -0.2" />
                  <MiniInfoCard title="Angular" value="± 0.6" />
                  <MiniInfoCard
                    title="State"
                    value={backendWsConnected ? "Ready" : "Disconnected"}
                    highlight={backendWsConnected ? "ok" : "danger"}
                  />
                </div>
              </div>
            </motion.section>
          </div>
        </Modal>
      )}

      <footer className="pt-8 text-center text-gray-500 text-xs">
        FIFO Safety System © 2025 | Designed for Human-Centered Safety
      </footer>

      <AnimatePresence>
        {notifOpen && (
          <>
            <motion.div
              className="fixed inset-0 z-[9997] bg-black/40"
              initial={{ opacity: 0 }}
              animate={{ opacity: 1 }}
              exit={{ opacity: 0 }}
              onClick={() => setNotifOpen(false)}
            />

            <motion.aside
              className="fixed top-0 right-0 z-[9998] h-screen w-[380px] max-w-[92vw] bg-[#111111] border-l border-[#2A2A2A] shadow-2xl flex flex-col"
              initial={{ x: "100%" }}
              animate={{ x: 0 }}
              exit={{ x: "100%" }}
              transition={{ type: "tween", duration: 0.22 }}
            >
              <div className="flex items-center justify-between px-4 py-4 border-b border-[#2A2A2A]">
                <div>
                  <h3 className="text-lg font-semibold text-[#FFD400]">알림</h3>
                  <p className="text-xs text-gray-500">
                    최근 시스템 로그 및 이벤트
                  </p>
                </div>

                <div className="flex items-center gap-2">
                  <button
                    type="button"
                    className="text-xs px-3 py-1.5 rounded-md border border-[#2A2A2A] text-gray-300 hover:bg-white/5"
                    onClick={() => setNotifications([])}
                  >
                    전체 삭제
                  </button>
                  <button
                    type="button"
                    className="text-sm px-3 py-1.5 rounded-md bg-red-600 hover:bg-red-700 text-white"
                    onClick={() => setNotifOpen(false)}
                  >
                    닫기
                  </button>
                </div>
              </div>

              <div className="flex-1 overflow-y-auto px-3 py-3 space-y-3">
                {notifications.length === 0 ? (
                  <div className="h-full flex items-center justify-center text-sm text-gray-500">
                    저장된 알림이 없습니다.
                  </div>
                ) : (
                  notifications.map((n) => (
                    <SwipeDismissNotification
                      key={n.id}
                      item={n}
                      onRemove={(id) =>
                        setNotifications((prev) =>
                          prev.filter((x) => x.id !== id)
                        )
                      }
                    />
                  ))
                )}
              </div>
            </motion.aside>
          </>
        )}
      </AnimatePresence>

      <AnimatePresence>
        {toast && (
          <motion.div
            key="toast"
            className="fixed top-6 inset-x-0 z-[10050] flex justify-center"
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

      <AnimatePresence>
        {conditionAlertOpen && (
          <motion.div
            className={`fixed inset-0 z-[10040] flex items-center justify-center p-6 ${
              conditionAlertAcknowledged ? "bg-black/75" : "bg-red-950/82"
            }`}
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            exit={{ opacity: 0 }}
            transition={{ duration: 0.2 }}
          >
            <motion.div
              className={`w-full max-w-md rounded-2xl p-6 ${
                conditionAlertAcknowledged
                  ? "border border-emerald-400 bg-[#1A1010]"
                  : "border border-red-500 bg-[#1A1010]"
              }`}
              animate={
                conditionAlertAcknowledged
                  ? {
                      scale: 1,
                      boxShadow: "0 0 0 rgba(16,185,129,0)",
                    }
                  : {
                      scale: [1, 1.02, 1],
                      boxShadow: [
                        "0 0 0 rgba(239,68,68,0.0)",
                        "0 0 46px rgba(220,38,38,0.55)",
                        "0 0 0 rgba(239,68,68,0.0)",
                      ],
                    }
              }
              transition={
                conditionAlertAcknowledged
                  ? { duration: 0.2 }
                  : { duration: 0.55, repeat: Infinity, ease: "easeInOut" }
              }
            >
              <h3 className="text-xl font-bold text-amber-300 mb-3">주의</h3>
              <p className="hidden text-sm text-gray-200 leading-6 whitespace-pre-line">
                {"작업자 컨디션 저조가 감지되었습니다. 확인 버튼을 누르지\n않으면 상부 보고가 자동 진행됩니다."}
              </p>
              <p className="text-sm text-gray-200 leading-6 whitespace-pre-line">
                {"작업자 컨디션 저조가 감지되었습니다.\n확인 버튼을 누르지 않으면 상부 보고가 자동 진행됩니다."}
              </p>
              <p className="hidden text-sm text-gray-200 leading-6 whitespace-pre-line">
                작업자 컨디션 저조가 감지되었습니다.
                확인 버튼을 누르지 않으면 상부 보고가 자동 진행됩니다.
              </p>

              <div className="mt-4 rounded-xl border border-amber-500/30 bg-amber-500/10 px-4 py-3 text-center">
                <p className="text-[11px] font-semibold tracking-[0.18em] text-amber-200">
                  COUNTDOWN
                </p>
                <p className="mt-1 text-3xl font-bold text-amber-300">
                  {conditionAlertCountdown}초
                </p>
                <p className="mt-1 text-xs text-amber-100/80">
                  30초 후 작업자 상태가 상부로 자동 보고됩니다.
                </p>
              </div>

              <div className="mt-5 flex justify-end">
                <Button
                  className="bg-amber-500 hover:bg-amber-600 text-black font-semibold"
                  onClick={acknowledgeConditionAlert}
                >
                  확인
                </Button>
              </div>
            </motion.div>
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

/* ───────────────────────── 알림 카드 ───────────────────────── */

function SwipeDismissNotification({
  item,
  onRemove,
}: {
  item: NotificationItem;
  onRemove: (id: string) => void;
}) {
  const borderClass =
    item.level === "danger"
      ? "border-red-700/60"
      : item.level === "warning"
      ? "border-amber-700/60"
      : item.level === "success"
      ? "border-emerald-700/60"
      : "border-[#2A2A2A]";

  const badgeClass =
    item.level === "danger"
      ? "bg-red-500/15 text-red-300"
      : item.level === "warning"
      ? "bg-amber-500/15 text-amber-300"
      : item.level === "success"
      ? "bg-emerald-500/15 text-emerald-300"
      : "bg-sky-500/15 text-sky-300";

  return (
    <motion.div
      layout
      drag="x"
      dragConstraints={{ left: 0, right: 0 }}
      dragElastic={0.12}
      onDragEnd={(_, info) => {
        if (info.offset.x > 120) {
          onRemove(item.id);
        }
      }}
      whileDrag={{ scale: 0.985 }}
      initial={{ opacity: 0, x: 40 }}
      animate={{ opacity: 1, x: 0 }}
      exit={{ opacity: 0, x: 120 }}
      transition={{ duration: 0.18 }}
      className={`relative overflow-hidden rounded-xl border ${borderClass} bg-[#181818]`}
    >
      <div className="absolute inset-y-0 left-0 w-24 bg-red-600 flex items-center justify-center text-xs font-semibold text-white">
        밀어서 삭제
      </div>

      <motion.div className="relative z-10 bg-[#181818] px-4 py-3">
        <div className="flex items-start justify-between gap-3">
          <div className="min-w-0">
            <div className="flex items-center gap-2 mb-1">
              <span className={`text-[11px] px-2 py-0.5 rounded-full ${badgeClass}`}>
                {item.level}
              </span>
              <span className="text-[11px] text-gray-500">
                {formatDateTime(item.createdAt)}
              </span>
            </div>

            <p className="text-sm font-semibold text-white">{item.title}</p>
            <p className="text-sm text-gray-300 mt-1 break-words">
              {item.message}
            </p>
          </div>

          <button
            type="button"
            onClick={() => onRemove(item.id)}
            className="text-xs text-gray-400 hover:text-white"
          >
            삭제
          </button>
        </div>
      </motion.div>
    </motion.div>
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
  children: ReactNode;
}) {
  return (
    <div className="fixed inset-0 z-[10020] bg-black/80">
      <div className="h-full flex items-center justify-center px-6 py-4">
        <div
          className="
            pointer-events-auto
            relative z-[10021]
            w-[1120px] max-w-[95vw]
            h-[92vh] max-h-[92vh]
            bg-[#1A1A1A] rounded-xl
            border border-[#2A2A2A]
            flex flex-col overflow-hidden
          "
        >
          <div className="shrink-0 flex justify-between items-center px-4 py-3 border-b border-[#2A2A2A]">
            <h3 className="text-xl font-semibold text-[#FFD400]">{title}</h3>
            <button
              type="button"
              onClick={onClose}
              className="pointer-events-auto relative z-[10022] bg-red-600 hover:bg-red-700 text-white font-semibold px-4 py-2 rounded-md"
            >
              닫기 ✕
            </button>
          </div>

          <div className="flex-1 min-h-0 overflow-auto px-4 pb-4 pt-4">
            {children}
          </div>
        </div>
      </div>
    </div>
  );
}

function StatusBadge({ status }: { status: WorkerStatus }) {
  const cls =
    status === "active"
      ? "bg-emerald-500/15 text-emerald-300 border-emerald-700/50"
      : status === "pending"
      ? "bg-amber-500/15 text-amber-300 border-amber-700/50"
      : "bg-red-500/15 text-red-300 border-red-700/50";

  const label =
    status === "active" ? "근무중" : status === "pending" ? "점검중" : "보류";

  return <span className={`text-[11px] px-2 py-0.5 rounded-full border ${cls}`}>{label}</span>;
}

function ProductStatusCard({
  label,
  value,
  tone,
}: {
  label: string;
  value: string;
  tone: "ok" | "danger";
}) {
  return (
    <div className="rounded-[18px] border border-white/10 bg-[#1A1A1A] px-4 py-3 text-right shadow-[0_10px_24px_rgba(0,0,0,0.18)]">
      <p className="text-[11px] uppercase tracking-[0.18em] text-gray-500">{label}</p>
      <p
        className={`mt-1 text-sm font-semibold tracking-[0.06em] ${
          tone === "ok" ? "text-emerald-400" : "text-red-400"
        }`}
      >
        {value}
      </p>
    </div>
  );
}

function ProductModeCard({
  title,
  subtitle,
  active,
  disabled,
  onClick,
}: {
  title: string;
  subtitle: string;
  active: boolean;
  disabled?: boolean;
  onClick: () => void;
}) {
  return (
    <motion.button
      type="button"
      disabled={disabled}
      onClick={onClick}
      whileHover={disabled ? undefined : { y: -2 }}
      whileTap={disabled ? undefined : { scale: 0.988 }}
      className={`h-20 rounded-[22px] border transition disabled:opacity-45 ${
        active
          ? "border-[#FFD400]/55 bg-[#FFD400] text-black shadow-none"
          : "border-white/10 bg-[#1A1A1A] text-white hover:border-[#FFD400]/40"
      }`}
    >
      <div className="flex flex-col items-center leading-tight">
        <span className="text-base font-bold tracking-[0.08em]">{title}</span>
        <span className="mt-2 text-xs opacity-80">{subtitle}</span>
      </div>
    </motion.button>
  );
}

function ProductActionButton({
  children,
  className,
  disabled,
  onClick,
}: {
  children: ReactNode;
  className: string;
  disabled?: boolean;
  onClick: () => void;
}) {
  return (
    <motion.div whileHover={disabled ? undefined : { y: -1 }} whileTap={disabled ? undefined : { scale: 0.988 }}>
      <Button
        className={`h-[72px] w-full rounded-[18px] border px-4 text-[18px] font-semibold leading-none shadow-[0_10px_22px_rgba(0,0,0,0.16)] disabled:opacity-45 ${className}`}
        disabled={disabled}
        onClick={onClick}
      >
        {children}
      </Button>
    </motion.div>
  );
}

function TriangleArrow({ direction }: { direction: "up" | "down" | "left" | "right" }) {
  const points =
    direction === "up"
      ? "24,10 10,30 38,30"
      : direction === "down"
      ? "10,14 38,14 24,34"
      : direction === "left"
      ? "10,24 30,10 30,38"
      : "18,10 38,24 18,38";
  return (
    <svg width="96" height="96" viewBox="0 0 48 48" aria-hidden="true">
      <polygon points={points} fill="#FFD400" />
    </svg>
  );
}

function DirectionPadButton({
  label,
  direction,
  disabled,
  onMouseDown,
  onMouseUp,
  onMouseLeave,
  onTouchStart,
  onTouchEnd,
}: {
  label: string;
  direction: "up" | "down" | "left" | "right";
  disabled?: boolean;
  onMouseDown: () => void;
  onMouseUp: () => void;
  onMouseLeave: () => void;
  onTouchStart: () => void;
  onTouchEnd: () => void;
}) {
  return (
    <motion.div whileHover={disabled ? undefined : { y: -2 }} whileTap={disabled ? undefined : { scale: 0.985 }}>
      <button
        type="button"
        disabled={disabled}
        onMouseDown={onMouseDown}
        onMouseUp={onMouseUp}
        onMouseLeave={onMouseLeave}
        onTouchStart={onTouchStart}
        onTouchEnd={onTouchEnd}
        className="relative flex h-[118px] w-[126px] items-center justify-center rounded-[24px] border border-transparent bg-transparent text-white shadow-none transition hover:border-white/10 hover:bg-[linear-gradient(180deg,#1D1D1D_0%,#161616_100%)] hover:shadow-[inset_0_1px_0_rgba(255,255,255,0.06),0_18px_32px_rgba(0,0,0,0.28)] disabled:opacity-45"
      >
        <div className="absolute inset-0 flex items-center justify-center">
          <TriangleArrow direction={direction} />
        </div>
        <span
          className={`absolute inset-x-0 text-center text-[13px] font-medium tracking-[0.12em] text-gray-300 ${
            direction === "left" || direction === "right" ? "bottom-1" : "bottom-3"
          }`}
        >
          {label}
        </span>
      </button>
    </motion.div>
  );
}

function MiniInfoCard({
  title,
  value,
  highlight,
  compact,
}: {
  title: string;
  value: string;
  highlight?: "ok" | "danger";
  compact?: boolean;
}) {
  return (
    <div
      className={`rounded-[18px] border border-white/10 bg-[#1A1A1A] ${
        compact ? "px-3 py-3" : "px-4 py-4"
      }`}
    >
      <p className={`${compact ? "text-[10px]" : "text-[11px]"} uppercase tracking-[0.18em] text-gray-500`}>
        {title}
      </p>
      <p
        className={`${compact ? "mt-1.5 text-sm" : "mt-2 text-base"} font-semibold tracking-[0.02em] ${
          highlight === "ok"
            ? "text-emerald-300"
            : highlight === "danger"
            ? "text-red-300"
            : "text-white"
        }`}
      >
        {value}
      </p>
    </div>
  );
}

function LabeledInput({
  label,
  value,
  onChange,
  placeholder,
  inputClassName = "",
}: {
  label: string;
  value: string;
  onChange: (v: string) => void;
  placeholder?: string;
  inputClassName?: string;
}) {
  return (
    <label className="block space-y-2">
      <span className="text-sm text-gray-300">{label}</span>
      <input
        value={value}
        onChange={(e) => onChange(e.target.value)}
        placeholder={placeholder}
        className={`w-full rounded-lg border border-[#2A2A2A] bg-[#111] text-white px-3 py-2 outline-none focus:border-[#FFD400] ${inputClassName}`}
      />
    </label>
  );
}

function LabeledNumberInput({
  label,
  value,
  onChange,
  onCommit,
  suffix,
  placeholder,
  disabled,
}: {
  label: string;
  value: string;
  onChange: (v: string) => void;
  onCommit: (n: number) => void;
  suffix?: string;
  placeholder?: string;
  disabled?: boolean;
}) {
  return (
    <label className="block space-y-2">
      <span className="text-sm text-gray-300">{label}</span>
      <div className="flex items-center gap-2 rounded-lg border border-[#2A2A2A] bg-[#111] px-3 py-2 focus-within:border-[#FFD400]">
        <input
          value={value}
          disabled={disabled}
          onChange={(e) => onChange(e.target.value)}
          onBlur={() => {
            const n = parseFloat(value);
            if (!Number.isNaN(n)) onCommit(n);
          }}
          onKeyDown={(e) => {
            if (e.key === "Enter") {
              const n = parseFloat(value);
              if (!Number.isNaN(n)) onCommit(n);
            }
          }}
          placeholder={placeholder}
          className="w-full bg-transparent text-white outline-none disabled:text-gray-500"
        />
        {suffix ? <span className="text-xs text-gray-500">{suffix}</span> : null}
      </div>
    </label>
  );
}

function HazardMap({
  hazards,
  selectedId,
  onSelect,
}: {
  hazards: { id: string; x: number; y: number; status: HazardStatus }[];
  selectedId?: string | null;
  onSelect: (h: any) => void;
}) {
  const W = 1200;
  const H = 520;
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
          <path
            d="M 40 0 L 0 0 0 40"
            fill="none"
            stroke="#161616"
            strokeWidth="1"
          />
        </pattern>
      </defs>

      <rect x="0" y="0" width={W} height={H} fill="url(#grid)" />

      {/* 메인 구역 박스 */}
      <g fill="#0B0D11" stroke="#24262B" strokeWidth="2">
        <rect x="70" y="60" width="1060" height="400" rx="16" />

        {/* Zone A */}
        <rect x="90" y="80" width="360" height="140" rx="8" />
        {/* Zone B */}
        <rect x="90" y="310" width="360" height="110" rx="8" />
        {/* Zone C */}
        <rect x="660" y="80" width="430" height="290" rx="8" />
      </g>

      {/* 내부 설비 블록 */}
      <g fill="#14161A" stroke="#2B2E34" strokeWidth="1.2">
        {/* Zone A */}
        <rect x="115" y="125" width="80" height="58" rx="4" />
        <rect x="220" y="125" width="80" height="58" rx="4" />
        <rect x="325" y="125" width="80" height="58" rx="4" />

        {/* Zone B */}
        <rect x="115" y="340" width="60" height="60" rx="4" />
        <rect x="190" y="340" width="60" height="60" rx="4" />
        <rect x="265" y="340" width="60" height="60" rx="4" />
        <rect x="340" y="340" width="60" height="60" rx="4" />

        {/* Zone C */}
        <rect x="690" y="125" width="150" height="70" rx="4" />
        <rect x="870" y="125" width="180" height="58" rx="4" />
        <rect x="750" y="290" width="120" height="72" rx="4" />
      </g>

      {/* 구역 라벨 */}
      <g fill="#CFCFCF" fontSize="14" fontWeight={600}>
        <text x="100" y="112">Zone A — Assembly</text>
        <text x="100" y="332">Zone B — Storage</text>
        <text x="670" y="112">Zone C — Maintenance</text>
      </g>

      {/* EXIT / DOCK */}
      <g>
        <rect
          x={pExit.x - 12}
          y={pExit.y - 22}
          width="24"
          height="44"
          rx="4"
          fill="#0EA5E9"
        />
        <text
          x={pExit.x - 18}
          y={pExit.y - 28}
          fill="#7DD3FC"
          fontSize="11"
          fontWeight={700}
        >
          EXIT
        </text>

        <rect
          x={pct(92.5, W)}
          y={pct(45, H)}
          width="24"
          height="70"
          rx="4"
          fill="#8B5CF6"
        />
        <text
          x={pct(95.5, W)}
          y={pct(44, H)}
          fill="#DDD6FE"
          fontSize="11"
          fontWeight={700}
        >
          DOCK
        </text>
      </g>

      {/* 로봇 경로 */}
      <polyline
        points={routePoints.map(([x, y]) => `${x},${y}`).join(" ")}
        fill="none"
        stroke="#34D399"
        strokeWidth="4"
        strokeDasharray="10 8"
        strokeLinecap="round"
        strokeLinejoin="round"
      />

      {/* 경로 위 이동 점 */}
      <motion.circle
        r={7}
        fill="#34D399"
        stroke="#0B0B0B"
        strokeWidth={2}
        animate={{
          cx: routePoints.map(([x]) => x),
          cy: routePoints.map(([, y]) => y),
        }}
        transition={{
          duration: 6,
          repeat: Infinity,
          ease: "linear",
        }}
      />

      {/* 위험 포인트 */}
      {hazards.map((h) => {
        const x = pct(h.x, W);
        const y = pct(h.y, H);

        const color =
          h.status === "open"
            ? "#EF4444"
            : h.status === "resolved"
            ? "#22C55E"
            : "#9CA3AF";

        const ring = selectedId === h.id ? "#FACC15" : "#0B0B0B";

        return (
          <g
            key={h.id}
            onClick={() => onSelect(h)}
            style={{ cursor: "pointer" }}
          >
            <circle
              cx={x}
              cy={y}
              r={9}
              fill={color}
              stroke={ring}
              strokeWidth={3}
            />
            <text
              x={x + 12}
              y={y - 10}
              fill="#E5E7EB"
              fontSize="13"
              fontWeight={600}
            >
              {h.id}
            </text>
          </g>
        );
      })}

      {/* Legend */}
      <g>
        <rect
          x={W - 210}
          y={H - 120}
          width="180"
          height="95"
          rx="10"
          fill="#050505"
          stroke="#2A2A2A"
        />
        <text x={W - 190} y={H - 95} fill="#FACC15" fontSize="12" fontWeight={700}>
          Legend
        </text>

        <circle cx={W - 185} cy={H - 75} r={5} fill="#34D399" />
        <text x={W - 172} y={H - 71} fill="#A3A3A3" fontSize="12">
          Robot route
        </text>

        <circle cx={W - 185} cy={H - 55} r={5} fill="#EF4444" />
        <text x={W - 172} y={H - 51} fill="#A3A3A3" fontSize="12">
          Open hazard
        </text>

        <circle cx={W - 185} cy={H - 35} r={5} fill="#22C55E" />
        <text x={W - 172} y={H - 31} fill="#A3A3A3" fontSize="12">
          Resolved
        </text>
      </g>
    </svg>
  );
}

function HazardInfoCard({
  label,
  value,
  accent,
}: {
  label: string;
  value: string;
  accent?: "yellow" | "red" | "green" | "gray";
}) {
  const accentClass =
    accent === "yellow"
      ? "text-[#FFD400]"
      : accent === "red"
      ? "text-red-300"
      : accent === "green"
      ? "text-emerald-300"
      : accent === "gray"
      ? "text-gray-300"
      : "text-white";

  return (
    <div className="rounded-[18px] border border-white/10 bg-[#101010] px-4 py-4">
      <p className="text-[11px] uppercase tracking-[0.18em] text-gray-500">
        {label}
      </p>
      <p className={`mt-2 text-base font-semibold tracking-[0.02em] ${accentClass}`}>
        {value}
      </p>
    </div>
  );
}

function LegendRow({
  color,
  label,
  desc,
}: {
  color: string;
  label: string;
  desc: string;
}) {
  return (
    <div className="flex items-center gap-3 rounded-[16px] border border-white/10 bg-[#101010] px-4 py-3">
      <div className={`h-3 w-3 rounded-full ${color}`} />
      <div className="min-w-0">
        <p className="text-sm font-semibold text-white">{label}</p>
        <p className="text-xs text-gray-500">{desc}</p>
      </div>
    </div>
  );
}