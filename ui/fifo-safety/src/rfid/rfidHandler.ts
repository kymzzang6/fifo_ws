// ./rfid/rfidHandler.ts

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

type RFIDPayload = {
  type?: string; // "admin" | "worker" (리더에서 내려주는 값, 없어도 됨)
  uid?: string;
  id?: string;
  name?: string;
  role?: string;
  nonce?: string | number; // 있으면 이벤트 단위 중복 방지(짧게)
};

type HandleRFIDArgs = {
  data: RFIDPayload;

  showToast: (msg: string) => void;

  // App.tsx에서 넘기고 있으니 타입만 유지 (handler에서 직접 안 써도 됨)
  setVitalsOpen: (v: boolean) => void;
  setForm: (f: any) => void;

  setWorkers: (updater: (prev: Worker[]) => Worker[]) => void;
  setPassStreak: (updater: any) => void;

  onAdmin: () => void; // 관리자 권한 10초 부여
};

/**
 * 목표 동작
 * - 관리자 카드: 매번 권한 부여(스팸만 짧게 차단), worker 흐름 절대 안 탐
 * - 작업자 카드: "찍을 때마다" 무조건 재검사(pending)로 리셋
 *   (단, 같은 패킷이 1~2번 중복 수신되는 것만 매우 짧게 차단)
 */

const PACKET_GUARD_MS = 120; // "동일 패킷"만 막는 매우 짧은 가드

// uid/id 기준 마지막 처리 시각
const lastSeenByKey: Record<string, number> = {};
// (선택) nonce 기준 마지막 처리 시각
const lastSeenByNonce: Record<string, number> = {};

function norm(s?: string) {
  return (s ?? "").trim();
}

function canonKey(s?: string) {
  return norm(s).toLowerCase();
}

/**
 * ✅ 카드 키
 * - uid 우선
 * - 없으면 id
 * - 없으면 tmp:... (업서트 불안정 → 리더에서 uid 또는 id 보내는 걸 권장)
 */
function getCardKey(data: RFIDPayload, now: number) {
  const uid = canonKey(data.uid);
  if (uid) return uid;

  const id = canonKey(data.id);
  if (id) return id;

  return `tmp:${now}`;
}

/**
 * workerId는 화면용 ID
 * - payload id 있으면 그대로
 * - 없으면 cardKey
 */
function getWorkerId(data: RFIDPayload, cardKey: string) {
  const id = norm(data.id);
  if (id) return id;

  if (cardKey.startsWith("tmp:")) return "UNKNOWN";
  return cardKey || "UNKNOWN";
}

/**
 * ✅ 관리자 카드 판정: 오판 방지 위해 "명시적" 조건만
 * - type === admin
 * - id가 A-로 시작
 */
function isAdminByCard(data: RFIDPayload) {
  if (canonKey(data.type) === "admin") return true;

  const id = norm(data.id).toUpperCase();
  if (id.startsWith("A-")) return true;

  return false;
}

/**
 * ✅ "중복 수신 패킷"만 방지하는 가드
 * - nonce가 있으면 (cardKey + nonce) 기준으로도 아주 짧게 막음
 * - nonce가 없으면 cardKey 기준으로 아주 짧게 막음
 *
 * ⚠️ 여기서 시간을 크게 잡으면 "연속 태깅"이 죽는다.
 */
function shouldDropAsDuplicate(cardKey: string, nonce: RFIDPayload["nonce"], now: number) {
  // 1) nonce 기준 가드(있는 경우)
  if (nonce != null) {
    const nk = `${cardKey}::${String(nonce)}`;
    const lastN = lastSeenByNonce[nk] ?? 0;
    if (now - lastN < PACKET_GUARD_MS) return true;
    lastSeenByNonce[nk] = now;
    // nonce가 있으면 여기서만 막아도 충분한 경우가 많음.
    // 그래도 cardKey 폭주가 있으면 아래 cardKey 가드도 같이 적용.
  }

  // 2) cardKey 기준 가드
  const lastK = lastSeenByKey[cardKey] ?? 0;
  if (now - lastK < PACKET_GUARD_MS) return true;
  lastSeenByKey[cardKey] = now;

  return false;
}

export function handleRFID({
  data,
  showToast,
  setVitalsOpen,
  setForm,
  setWorkers,
  setPassStreak,
  onAdmin,
}: HandleRFIDArgs) {
  const now = Date.now();

  const cardKey = getCardKey(data, now);
  if (!cardKey) return;

  // 1) 관리자 판정 먼저
  const admin = isAdminByCard(data);

  // 2) 중복 패킷만 짧게 차단 (관리자/작업자 모두 적용)
  if (shouldDropAsDuplicate(cardKey, data.nonce, now)) return;

  // 3) 관리자: 권한만 부여하고 종료 (worker 흐름 절대 안 탐)
  if (admin) {
    onAdmin();
    showToast("관리자 권한 활성화 (10초)");
    return;
  }

  // 4) 작업자: 찍을 때마다 무조건 재검사(pending)로 리셋
  const sessionKey = `card:${cardKey}`;
  const workerId = getWorkerId(data, cardKey);

  const name = norm(data.name) || "미등록";
  const role = norm(data.role) || "";
  const uidRaw = norm(data.uid) || undefined;

  setWorkers((prev) => {
    // 동일 카드/사람 매칭 우선순위:
    // 1) uid 동일
    // 2) id 동일
    // 3) sessionKey 동일
    const idx = prev.findIndex((w) => {
      const wUid = canonKey(w.uid);
      const dUid = canonKey(uidRaw);
      if (dUid && wUid && wUid === dUid) return true;

      const wId = canonKey(w.id);
      const dId = canonKey(workerId);
      if (dId && wId && wId === dId) return true;

      return canonKey(w.sessionKey) === canonKey(sessionKey);
    });

    const moveToTop = (arr: Worker[], index: number) => {
      if (index <= 0) return arr;
      const copy = [...arr];
      const [it] = copy.splice(index, 1);
      copy.unshift(it);
      return copy;
    };

    // (A) 처음 보는 카드/사람 -> 추가 + pending 시작
    if (idx < 0) {
      setPassStreak((s: any) => ({ ...s, [sessionKey]: 0 }));
      showToast(`${name} 체크인 시작 (대기)`);

      const newWorker: Worker = {
        sessionKey,
        id: workerId,
        uid: uidRaw,
        name,
        role,
        status: "pending",
        checkinStartedAt: now,
        failReasons: [],
        warnings: [],
      };

      return [newWorker, ...prev];
    }

    // (B) 이미 있는 카드/사람 -> 재검사 리셋 (항상 pending)
    const cur = prev[idx];

    const next: Worker = {
      ...cur,
      id: workerId || cur.id,
      uid: uidRaw ?? cur.uid,
      name: name || cur.name,
      role: role || cur.role,

      status: "pending",
      checkinStartedAt: now,
      failReasons: [], // 재검사 시작이므로 초기화
      // warnings는 유지(원하면 []로 바꿔도 됨)
    };

    setPassStreak((s: any) => ({ ...s, [sessionKey]: 0 }));
    showToast(`${next.name} 재검사 시작`);

    const updated = [...prev];
    updated[idx] = next;
    return moveToTop(updated, idx);
  });

  // (선택) 태그 시 폼/모달 자동 처리하고 싶으면 아래를 켜도 됨
  // setForm((f: any) => ({ ...f, id: workerId, name, role }));
  // setVitalsOpen(true);
}
