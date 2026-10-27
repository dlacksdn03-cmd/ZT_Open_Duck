"""STS3215 정밀 제어 테스트

사용법:
    pip install -r requirements.txt
    python servo_precision_test.py                 # 포트 자동 검색
    python servo_precision_test.py --port COM5     # Linux: /dev/ttyUSB0, macOS: /dev/cu.usbserial-xxx
    그래프 창 없이 저장만: MPLBACKEND=Agg (PowerShell: $env:MPLBACKEND='Agg')
"""
import argparse, time, random, statistics, csv, sys
from scservo_sdk import COMM_SUCCESS
from servo_common import find_port_or_exit, open_bus, setup_korean_font

ap = argparse.ArgumentParser()
ap.add_argument('--port', help='예: COM5, /dev/ttyUSB0 (생략하면 자동 검색)')
ap.add_argument('--baud', type=int, default=1000000)
ap.add_argument('--id', type=int, default=1)
ap.add_argument('--range', type=int, default=300, help='중앙 기준 이동 범위(스텝)')
args = ap.parse_args()
args.port = args.port or find_port_or_exit()

ID, CENTER, RANGE = args.id, 2048, args.range

try:
    port, s = open_bus(args.port, args.baud)
except RuntimeError as e:
    sys.exit(str(e))
if s.ping(ID)[1] != COMM_SUCCESS:
    port.closePort()
    sys.exit(f"서보 ID {ID} 응답 없음 (외부 전원, 어댑터 점퍼, 3핀 케이블, --id/--baud 확인)")

volt, _, _ = s.read1ByteTxRx(ID, 62)
temp, _, _ = s.read1ByteTxRx(ID, 63)
print(f"연결 OK - 전압 {volt/10:.1f}V, 온도 {temp}℃")

def read_pos():
    pos, _, res, _ = s.ReadPosSpeed(ID)
    return pos if res == COMM_SUCCESS else None

def move_and_settle(goal, speed=1500, acc=50, timeout=3.0):
    s.WritePosEx(ID, goal, speed, acc)
    time.sleep(0.05)
    t0 = time.time()
    while time.time() - t0 < timeout:
        moving, res, _ = s.read1ByteTxRx(ID, 66)   # 66: 이동 중 플래그
        if res == COMM_SUCCESS and moving == 0:
            break
        time.sleep(0.01)
    time.sleep(0.3)
    return read_pos()

try:
    # [1] 미세 스텝 (분해능)
    print("\n[1] 미세 스텝 테스트")
    for step in [1, 2, 5, 10]:
        for sign in [+1, -1]:
            goal = CENTER + sign * step
            start = move_and_settle(CENTER)
            end = move_and_settle(goal)
            print(f"  명령 {sign*step:+3d} 스텝 → 실제 이동 {end-start:+3d} (오차 {end-goal:+d})")

    # [2] 반복 정밀도 + 방향 히스테리시스
    print("\n[2] 반복 정밀도 테스트 (방향별 15회)")
    errs = {'+방향 접근': [], '-방향 접근': []}
    for _ in range(15):
        for label, sign in [('+방향 접근', -1), ('-방향 접근', +1)]:
            move_and_settle(CENTER + sign * random.randint(100, RANGE))
            errs[label].append(move_and_settle(CENTER) - CENTER)
    for label, e in errs.items():
        print(f"  {label}: 평균 {statistics.mean(e):+.2f}, 표준편차 {statistics.pstdev(e):.2f}, "
              f"최대 |오차| {max(map(abs, e))} 스텝")
    hyst = statistics.mean(errs['+방향 접근']) - statistics.mean(errs['-방향 접근'])
    print(f"  방향 히스테리시스: {hyst:+.2f} 스텝 ({hyst*0.088:+.3f}°)")

    # [3] 스텝 응답 로깅
    print("\n[3] 스텝 응답 테스트")
    move_and_settle(CENTER)
    goal = CENTER + RANGE
    log = []
    t0 = time.perf_counter()
    s.WritePosEx(ID, goal, 0, 0)          # 속도 0 = 최대 속도, 가속 0 = 최대 가속
    while time.perf_counter() - t0 < 1.0:
        p = read_pos()
        if p is not None:
            log.append((time.perf_counter() - t0, p))
    with open('step_response.csv', 'w', newline='') as f:
        csv.writer(f).writerows([('t_s', 'pos')] + log)

    final = log[-1][1]
    peak = max(p for _, p in log)
    t90 = next((t for t, p in log if p >= CENTER + 0.9 * RANGE), None)
    t90_txt = f"{t90*1000:.0f} ms" if t90 is not None else "도달 못함"
    print(f"  샘플 {len(log)}개, 90% 도달 {t90_txt}, "
          f"오버슈트 {peak-goal:+d} 스텝, 최종 오차 {final-goal:+d} 스텝")
    print("  → step_response.csv 저장됨")
finally:
    s.write1ByteTxRx(ID, 40, 0)           # 토크 OFF
    port.closePort()

import matplotlib.pyplot as plt
setup_korean_font(plt)
t, p = zip(*log)
plt.plot(t, p, '.-'); plt.axhline(goal, ls='--', c='r', label='goal')
plt.xlabel('time [s]'); plt.ylabel('position [step]'); plt.legend(); plt.grid()
plt.title('STS3215 step response')
plt.savefig('step_response.png', dpi=120)
print("  → step_response.png 저장됨")
plt.show()
