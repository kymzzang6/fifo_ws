// ./rfid/rfidHandler.ts

export function handleRFID({
  data,
  showToast,
  setWorkers,
  setPassStreak,
  onAdmin,
}: any) {
  const now = Date.now();
  
  // 1. UID 추출 및 정규화
  const rawUid = (data.uid || "").trim().toUpperCase();
  if (!rawUid) return;

  const sessionKey = `card:${rawUid}`;

  // 2. 관리자 카드 여부 확인
  if (data.type === "admin") {
    onAdmin();
    showToast("관리자 권한 활성화 (10초)");
    return;
  }

  // 3. 작업자 또는 미등록 카드 처리
  const name = data.name || (data.type === "unknown" ? "미등록 카드" : "알 수 없음");
  const role = data.role || "";

  setWorkers((prev: any[]) => {
    // 동일인 여부 판단 (UID 기준)
    const idx = prev.findIndex(w => w.uid === rawUid);

    if (idx < 0) {
      // 신규 등록 또는 처음 찍은 카드
      showToast(`${name} 체크인 시작 (대기)`);
      const newWorker = {
        sessionKey,
        id: data.id || "W-UNKNOWN",
        uid: rawUid,
        name,
        role,
        status: "pending",
        checkinStartedAt: now,
        failReasons: [],
        warnings: [],
      };
      return [newWorker, ...prev];
    }

    // 이미 목록에 있는 경우 -> 상태를 pending으로 리셋하여 재검사 유도
    const updated = [...prev];
    updated[idx] = {
      ...updated[idx],
      status: "pending",
      checkinStartedAt: now,
      failReasons: [],
    };
    
    // 화면 최상단으로 이동 (선택 사항)
    const [target] = updated.splice(idx, 1);
    updated.unshift(target);

    setPassStreak((s: any) => ({ ...s, [sessionKey]: 0 }));
    showToast(`${name} 재검사 시작`);
    return updated;
  });
}