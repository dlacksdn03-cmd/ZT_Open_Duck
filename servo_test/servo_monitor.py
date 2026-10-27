"""STS3215 실시간 모니터 + 위치 제어

사용법:
    파일 더블클릭                                     # 포트 자동 검색, 모니터 + 제어
    python servo_monitor.py                           # 포트 자동 검색
    python servo_monitor.py --port COM13              # Windows 예 (Linux: /dev/ttyUSB0, macOS: /dev/cu.usbserial-xxx)
    python servo_monitor.py --readonly                # 모니터만 (아무것도 쓰지 않음)
    python servo_monitor.py --plot                    # 실시간 그래프 (읽기 전용)
    python servo_monitor.py --csv log.csv --duration 30

제어 명령 (입력 후 Enter):
    on / off      토크 ON / OFF          c         중앙(2048)으로
    2100          절대 위치 [스텝]       +50, -20  현재 목표 기준 상대 이동 [스텝]
    180d          절대 각도 [°]          +10d      상대 각도 [°]
    v 500         속도 [스텝/s], 0=최대  a 30      가속도 [100 스텝/s²], 0=최대
    2100 v1000 a10   이번 이동에만 속도/가속도 지정 (이후에는 기본값으로 돌아감)
    h             도움말                 q         종료 (토크 OFF)
    ← → : ±1 스텝,  ↑ ↓ : ±10 스텝  (토크 ON일 때)

위치는 0~4095 전체 사용. 0/4095 경계를 넘는 이동은 반대 방향으로 돌아가니 주의.
종료 시 항상 토크 OFF. EEPROM(ID, 보드레이트, 각도 제한, PID 등)은 쓰지 않는다.
"""
import argparse, time, csv, os, sys, collections
from scservo_sdk import COMM_SUCCESS
from servo_common import (CENTER, POS_MIN as LO, POS_MAX as HI, SPEED_MAX, ACC_MAX, STEP_DEG,
                          pause_and_exit, find_port_or_exit, open_bus, read_state as read_servo,
                          status_names, KeyReader, setup_korean_font)

ap = argparse.ArgumentParser()
ap.add_argument('--port', help='예: COM13, /dev/ttyUSB0 (생략하면 자동 검색)')
ap.add_argument('--baud', type=int, default=1000000)
ap.add_argument('--id', type=int, default=1)
ap.add_argument('--hz', type=float, default=20, help='갱신 주기')
ap.add_argument('--duration', type=float, default=0, help='초, 0 = 종료 명령까지')
ap.add_argument('--csv', help='CSV로 기록할 파일 이름')
ap.add_argument('--plot', action='store_true', help='실시간 그래프 창 (읽기 전용)')
ap.add_argument('--readonly', action='store_true', help='제어 없이 모니터만')
ap.add_argument('--speed', type=int, default=500, help='기본 이동 속도 [스텝/s], 0 = 최대')
ap.add_argument('--acc', type=int, default=30, help='기본 가속도 [100 스텝/s²], 0 = 최대')
args = ap.parse_args()
args.port = args.port or find_port_or_exit()

ID = args.id
FIELDS = ['t_s', 'torque', 'goal', 'pos', 'speed', 'load_pct', 'volt', 'temp', 'current_mA', 'moving', 'status']

try:
    port, s = open_bus(args.port, args.baud)
except RuntimeError as e:
    pause_and_exit(str(e), 1)

# 서보에 설정된 전압 범위(EEPROM 14, 15)를 읽어서 경고 기준으로 사용 (7.4V형 8.0V, 12V형은 더 높음)
_v, _r, _ = s.read1ByteTxRx(ID, 14)
MAX_VOLT = _v / 10 if _r == COMM_SUCCESS and _v else 8.4
_v, _r, _ = s.read1ByteTxRx(ID, 15)
MIN_VOLT = _v / 10 if _r == COMM_SUCCESS and _v else 4.5


def read_state():
    return read_servo(s, ID)


def status_text(v):
    names = status_names(v)
    return '정상' if not names else '경고: ' + ', '.join(names)


def render(d, t, fails):
    lines = [
        f"STS3215 ID {ID} @ {args.port}   t={t:7.1f}s   통신 실패 {fails}회",
        "-" * 64,
    ]
    if d is None:
        lines += ["  응답 없음 - 전원/케이블 확인", "", "", "", "", "", ""]
    else:
        volt_warn = (f"※ 서보 최대 {MAX_VOLT:.1f}V 초과!" if d['volt'] > MAX_VOLT
                     else (f"※ 서보 최소 {MIN_VOLT:.1f}V 미만" if d['volt'] < MIN_VOLT else ""))
        lines += [
            f"  [핀 VCC]  전압      {d['volt']:5.1f} V   {volt_warn}",
            f"  [핀 DATA] 통신      정상 ({args.baud} bps)",
            f"  토크      {'ON ' if d['torque'] else 'OFF'}         목표 위치 {d['goal']:5d}",
            f"  위치      {d['pos']:5d} 스텝  ({d['pos'] * STEP_DEG:6.1f}°)   속도 {d['speed']:+6d} 스텝/s",
            f"  전류      {d['current_mA']:7.1f} mA      부하 {d['load_pct']:+6.1f} %",
            f"  온도      {d['temp']:5d} ℃         이동 중 {'예' if d['moving'] else '아니오'}",
            f"  상태      {d['status']:08b}  {status_text(d['status'])}",
        ]
    return lines


class Controller:
    """명령 해석, 위치/속도/가속도 검사, 이동 결과(도달 위치/오차/시간/최대 전류) 기록."""
    HELP = [
        "명령: on/off  c(중앙)  2100(절대)  +50/-20(상대)  180d/+10d(각도)  q(종료)",
        "      v 500(속도 스텝/s)  a 30(가속 100스텝/s²)  0=최대   2100 v1000 a10 = 이번 이동만",
        f"      ←→ ±1 스텝  ↑↓ ±10 스텝   위치 범위 {LO}~{HI}",
    ]

    def __init__(self):
        self.speed = max(0, min(args.speed, SPEED_MAX))
        self.acc = max(0, min(args.acc, ACC_MAX))
        self.msg = "토크 OFF 상태입니다. 'on'으로 토크를 켜면 현재 위치를 유지합니다."
        self.history = collections.deque(maxlen=6)
        self.move = None
        self.quit = False

    @staticmethod
    def fmt_speed(v):
        return "최대" if v == 0 else f"{v} 스텝/s ({v * STEP_DEG:.0f}°/s)"

    @staticmethod
    def fmt_acc(a):
        return "최대" if a == 0 else f"{a} ({a * 100 * STEP_DEG:.0f}°/s²)"

    def torque_on(self, d):
        s.WritePosEx(ID, d['pos'], self.speed, self.acc)   # 목표 = 현재 위치 → 켜져도 움직이지 않음
        s.write1ByteTxRx(ID, 40, 1)
        self.msg = f"토크 ON - 위치 {d['pos']} 유지"

    def torque_off(self, why="토크 OFF"):
        s.write1ByteTxRx(ID, 40, 0)
        self.move = None
        self.msg = why

    def move_to(self, goal, d, speed=None, acc=None):
        speed = self.speed if speed is None else speed
        acc = self.acc if acc is None else acc
        if not d['torque']:
            self.msg = "거부: 토크가 OFF입니다. 먼저 'on'을 입력하세요."
        elif not LO <= goal <= HI:
            self.msg = f"거부: 목표 {goal}가 위치 범위 {LO}~{HI} 밖입니다."
        else:
            s.WritePosEx(ID, goal, speed, acc)
            self.move = {'t0': time.perf_counter(), 'start': d['pos'], 'goal': goal,
                         'speed': speed, 'acc': acc,
                         'seen_moving': False, 'settle': None, 'peak_mA': 0.0}
            self.msg = (f"이동: {d['pos']} → {goal} ({goal - d['pos']:+d} 스텝, {(goal - d['pos']) * STEP_DEG:+.1f}°)"
                        f"  속도 {self.fmt_speed(speed)}, 가속 {self.fmt_acc(acc)}")

    def parse_int(self, text, lo, hi, name):
        try:
            n = int(text)
        except ValueError:
            self.msg = f"{name} 값이 숫자가 아닙니다: {text}"; return None
        if not lo <= n <= hi:
            self.msg = f"{name}는 {lo}~{hi} 사이여야 합니다 (입력 {n})"; return None
        return n

    def handle(self, text, d):
        text = text.strip().lower()
        if not text:
            return
        if text in ('q', 'quit', 'exit'):
            self.quit = True; return
        if text in ('h', 'help', '?'):
            self.msg = "명령은 위 세 줄 참고. 숫자=절대 스텝, +/-숫자=상대 스텝, 끝에 d=각도"; return
        if text == 'off':
            self.torque_off(); return
        parts = text.split()
        if len(parts) == 2 and parts[0] in ('v', 'a'):      # 기본 속도/가속도 변경
            if parts[0] == 'v':
                n = self.parse_int(parts[1], 0, SPEED_MAX, '속도')
                if n is not None:
                    self.speed = n; self.msg = f"기본 속도 = {self.fmt_speed(n)}"
            else:
                n = self.parse_int(parts[1], 0, ACC_MAX, '가속도')
                if n is not None:
                    self.acc = n; self.msg = f"기본 가속도 = {self.fmt_acc(n)}"
            return
        if d is None:
            self.msg = "통신 실패 - 명령을 보내지 않았습니다."; return
        if text == 'on':
            self.torque_on(d); return

        # 이동 명령: 첫 토큰 = 위치, 이후 v<속도> a<가속도> 는 이번 이동에만 적용
        target, speed, acc = parts[0], None, None
        for opt in parts[1:]:
            if opt[0] == 'v' and len(opt) > 1:
                speed = self.parse_int(opt[1:], 0, SPEED_MAX, '속도')
                if speed is None: return
            elif opt[0] == 'a' and len(opt) > 1:
                acc = self.parse_int(opt[1:], 0, ACC_MAX, '가속도')
                if acc is None: return
            else:
                self.msg = f"알 수 없는 옵션: {opt}  (예: 2100 v1000 a10)"; return
        if target in ('c', 'center'):
            self.move_to(CENTER, d, speed, acc); return
        deg = target.endswith('d') or target.endswith('°')
        num = target.rstrip('d°')
        try:
            value = float(num)
        except ValueError:
            self.msg = f"알 수 없는 명령: {text}  (h = 도움말)"; return
        steps = round(value / STEP_DEG) if deg else round(value)
        if num[0] in '+-':
            base = d['goal'] if d['torque'] else d['pos']
            self.move_to(base + steps, d, speed, acc)
        else:
            self.move_to(steps, d, speed, acc)

    def jog(self, delta, d):
        if d is not None:
            self.move_to((d['goal'] if d['torque'] else d['pos']) + delta, d)

    def update(self, d):
        """매 프레임 호출: 이동 완료 판정."""
        if d is None:
            return
        m = self.move
        if m is None:
            return
        now = time.perf_counter()
        elapsed = now - m['t0']
        m['peak_mA'] = max(m['peak_mA'], abs(d['current_mA']))
        if d['moving']:
            m['seen_moving'], m['settle'] = True, None
        elif m['settle'] is None and (m['seen_moving'] or elapsed > 0.3):
            m['settle'] = now + 0.2       # 멈춘 뒤 0.2초 안정화 후 기록
        timeout = elapsed > 5.0
        if (m['settle'] is not None and now >= m['settle']) or timeout:
            err = d['pos'] - m['goal']
            t_ms = (elapsed - (0.2 if not timeout else 0)) * 1000
            self.history.appendleft(
                f"  {m['start']:4d} → 목표 {m['goal']:4d} : 도달 {d['pos']:4d}  오차 {err:+3d} 스텝 ({err * STEP_DEG:+.2f}°)"
                f"  {t_ms:5.0f} ms  최대 {m['peak_mA']:5.0f} mA  (v{m['speed']} a{m['acc']})"
                + ("  [타임아웃]" if timeout else ""))
            self.msg = "이동 완료" if not timeout else "이동 타임아웃 - 소리나 걸림이 있는지 확인하세요."
            self.move = None


JOG = {'LEFT': -1, 'RIGHT': +1, 'UP': +10, 'DOWN': -10}


def read_keys(keys, buf, ctl, d):
    """키 입력을 처리하고 갱신된 입력 버퍼를 반환."""
    for k in keys.read():
        if k in JOG:
            ctl.jog(JOG[k], d)
        elif k == 'ENTER':
            ctl.handle(buf, d); buf = ''
        elif k == 'BACKSPACE':
            buf = buf[:-1]
        elif k == 'ESC':
            buf = ''
        elif len(k) == 1:
            buf += k
    return buf


def run_terminal(control):
    if sys.platform.startswith('win'):
        os.system('')                 # Windows 콘솔 ANSI 활성화
    writer = None
    if args.csv:
        f = open(args.csv, 'w', newline='')
        writer = csv.DictWriter(f, FIELDS); writer.writeheader()
    ctl = Controller() if control else None
    t0, fails, n_prev, buf = time.perf_counter(), 0, 0, ''
    with KeyReader() as keys:
        _terminal_loop(ctl, writer, t0, fails, n_prev, buf, keys)
    if writer:
        f.close(); print(f"→ {args.csv} 저장됨")


def _terminal_loop(ctl, writer, t0, fails, n_prev, buf, keys):
    try:
        while not args.duration or time.perf_counter() - t0 < args.duration:
            t = time.perf_counter() - t0
            d = read_state()
            if d is None:
                fails += 1
            elif writer:
                writer.writerow({'t_s': round(t, 3), **d})
            lines = render(d, t, fails)
            if ctl:
                buf = read_keys(keys, buf, ctl, d)
                ctl.update(d)
                if ctl.quit:
                    break
                hist = list(ctl.history) + [''] * (ctl.history.maxlen - len(ctl.history))
                lines += [''] + ctl.HELP + [f"기본 속도 {ctl.fmt_speed(ctl.speed)}, 기본 가속 {ctl.fmt_acc(ctl.acc)}",
                                       "최근 이동 결과 (최신이 위)"] + hist
                lines += [f"  ▶ {ctl.msg}", f"> {buf}"]
            else:
                lines += ["(읽기 전용 모드 - Ctrl+C 종료)"]
            if n_prev:
                sys.stdout.write(f"\x1b[{n_prev - 1}F" if n_prev > 1 else "\r")
            sys.stdout.write('\x1b[K\n'.join(lines) + '\x1b[K')
            sys.stdout.flush()
            n_prev = len(lines)
            time.sleep(max(0, 1 / args.hz - (time.perf_counter() - t0 - t)))
    except KeyboardInterrupt:
        pass
    finally:
        print()


def run_plot():
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    N = int(args.hz * 20)             # 최근 20초
    buf = {k: collections.deque(maxlen=N) for k in ['t', 'pos', 'current_mA', 'load_pct', 'temp', 'volt']}
    setup_korean_font(plt)
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(8, 8))
    specs = [('pos', '위치 [step]'), ('current_mA', '전류 [mA]'), ('load_pct', '부하 [%]'), ('temp', '온도/전압')]
    lines = {}
    for ax, (k, label) in zip(axes, specs):
        lines[k], = ax.plot([], [], '-')
        ax.set_ylabel(label); ax.grid(True)
    lines['volt'], = axes[3].plot([], [], '-', label='전압 x10 [V]')
    lines['temp'].set_label('온도 [℃]'); axes[3].legend(loc='upper left')
    axes[-1].set_xlabel('time [s]')
    title = fig.suptitle('')
    t0 = time.perf_counter()

    def update(_):
        d = read_state()
        if d is None:
            title.set_text('응답 없음'); return
        buf['t'].append(time.perf_counter() - t0)
        for k in ['pos', 'current_mA', 'load_pct', 'temp']:
            buf[k].append(d[k])
        buf['volt'].append(d['volt'] * 10)
        for k, ln in lines.items():
            ln.set_data(buf['t'], buf[k])
        for ax in axes:
            ax.relim(); ax.autoscale_view()
        title.set_text(f"토크 {'ON' if d['torque'] else 'OFF'} | {d['volt']:.1f} V | {d['temp']} ℃ | {status_text(d['status'])}")

    anim = FuncAnimation(fig, update, interval=1000 / args.hz, cache_frame_data=False)
    plt.show()


def ensure_torque_off():
    for _ in range(3):
        s.write1ByteTxRx(ID, 40, 0)
        v, res, _ = s.read1ByteTxRx(ID, 40)
        if res == COMM_SUCCESS and v == 0:
            print("토크 OFF 확인됨")
            return
        time.sleep(0.05)
    print("※ 토크 OFF 확인 실패 - 전원을 분리하세요.")


control = not (args.readonly or args.plot)
try:
    run_plot() if args.plot else run_terminal(control)
except Exception as e:
    print(f"\n오류: {type(e).__name__}: {e}")
finally:
    if control:
        ensure_torque_off()
    port.closePort()
pause_and_exit()
