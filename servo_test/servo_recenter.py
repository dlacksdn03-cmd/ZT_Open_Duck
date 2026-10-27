"""STS3215를 중앙(2048)으로 천천히 이동시키고 토크 OFF.

허용 범위(±300) 밖에서 시작할 때 손으로 돌리기 어려운 경우 한 번 쓰는 용도.
과전류, 걸림(진행 없음), 에러 비트, 과열이 감지되면 즉시 토크 OFF.

    python servo_recenter.py                  # 포트 자동 검색
    python servo_recenter.py --port COM13     # Linux: /dev/ttyUSB0, macOS: /dev/cu.usbserial-xxx
"""
import argparse, time, sys
from scservo_sdk import COMM_SUCCESS, SCS_TOHOST
from servo_common import find_port_or_exit, open_bus

ap = argparse.ArgumentParser()
ap.add_argument('--port', help='예: COM13, /dev/ttyUSB0 (생략하면 자동 검색)')
ap.add_argument('--baud', type=int, default=1000000)
ap.add_argument('--id', type=int, default=1)
ap.add_argument('--speed', type=int, default=300, help='스텝/s (2000스텝 ≈ 7초)')
ap.add_argument('--acc', type=int, default=20)
ap.add_argument('--max-ma', type=float, default=1200, help='이 전류가 0.3초 넘게 이어지면 중단')
args = ap.parse_args()
args.port = args.port or find_port_or_exit()

ID, CENTER = args.id, 2048
try:
    port, s = open_bus(args.port, args.baud)
except RuntimeError as e:
    sys.exit(str(e))


def state():
    pos, _, r1, _ = s.ReadPosSpeed(ID)
    moving, r2, _ = s.read1ByteTxRx(ID, 66)
    cur, r3, _ = s.read2ByteTxRx(ID, 69)
    status, r4, _ = s.read1ByteTxRx(ID, 65)
    temp, r5, _ = s.read1ByteTxRx(ID, 63)
    if any(r != COMM_SUCCESS for r in (r1, r2, r3, r4, r5)):
        return None
    return pos, moving, abs(SCS_TOHOST(cur, 15)) * 6.5, status, temp


try:
    st = state()
    assert st is not None, "서보 응답 없음"
    start = st[0]
    print(f"시작 위치 {start} → 목표 {CENTER} ({CENTER - start:+d} 스텝, {(CENTER - start) * 360 / 4096:+.1f}°)")
    s.WritePosEx(ID, start, args.speed, args.acc)      # 목표 = 현재 위치 → 토크 ON 시 그대로 유지
    s.write1ByteTxRx(ID, 40, 1)
    time.sleep(0.2)
    s.WritePosEx(ID, CENTER, args.speed, args.acc)

    t0 = time.perf_counter()
    peak_ma, peak_temp, over_since, reason = 0.0, st[4], None, None
    last_prog_t, last_prog_pos, settled_since, pos = t0, start, None, start
    while True:
        now = time.perf_counter()
        st = state()
        if st is None:
            reason = "통신 실패"; break
        pos, moving, ma, status, temp = st
        peak_ma, peak_temp = max(peak_ma, ma), max(peak_temp, temp)
        if ma > args.max_ma:
            over_since = over_since or now
            if now - over_since > 0.3:
                reason = f"과전류 {ma:.0f} mA"; break
        else:
            over_since = None
        if status:
            reason = f"에러 비트 {status:08b}"; break
        if temp >= 55:
            reason = f"온도 {temp}℃"; break
        if abs(pos - last_prog_pos) >= 3:
            last_prog_t, last_prog_pos = now, pos
        elif abs(pos - CENTER) > 10 and now - last_prog_t > 1.0 and now - t0 > 0.5:
            reason = f"걸림 의심 - 1초간 진행 없음 (위치 {pos})"; break
        if not moving and abs(pos - CENTER) <= 10:
            settled_since = settled_since or now
            if now - settled_since > 0.3:
                break
        else:
            settled_since = None
        if now - t0 > 20:
            reason = "타임아웃 20초"; break
        if int((now - t0) * 2) != int((now - t0 - 0.02) * 2):    # 0.5초마다 출력
            print(f"  t={now - t0:5.1f}s  위치 {pos:4d}  전류 {ma:6.0f} mA  온도 {temp}℃")
        time.sleep(0.02)

    print(("중단: " + reason) if reason else "중앙 도달")
    print(f"최종 위치 {pos}, 오차 {pos - CENTER:+d} 스텝, 소요 {time.perf_counter() - t0:.1f}s, "
          f"최대 전류 {peak_ma:.0f} mA, 최고 온도 {peak_temp}℃")
finally:
    for _ in range(3):
        s.write1ByteTxRx(ID, 40, 0)
        v, r, _ = s.read1ByteTxRx(ID, 40)
        if r == COMM_SUCCESS and v == 0:
            print("토크 OFF 확인됨"); break
    port.closePort()
