"""여러 스크립트가 함께 쓰는 공통 도구 (Windows / macOS / Linux).

- SDK 호환 래퍼(sms_sts) - 항상 sts_compat 것을 써서 SDK 버전 차이를 없앤다
- USB 시리얼 어댑터 자동 검색과 OS별 문제 해결 안내
- STS 상태 레지스터 일괄 읽기
- 논블로킹 키 입력 (Windows: msvcrt, macOS/Linux: termios)
- matplotlib 한글 폰트 선택
"""
import os, sys, time
from scservo_sdk import PortHandler, COMM_SUCCESS, SCS_MAKEWORD, SCS_TOHOST, \
    ERRBIT_VOLTAGE, ERRBIT_ANGLE, ERRBIT_OVERHEAT, ERRBIT_OVERELE, ERRBIT_OVERLOAD
from sts_compat import sms_sts

ADAPTER_CHIPS = {0x1A86: 'CH340/CH343', 0x10C4: 'CP210x', 0x0403: 'FTDI'}
STEP_DEG = 360 / 4096
CENTER = 2048
POS_MIN, POS_MAX = 0, 4095
SPEED_MAX, ACC_MAX = 4000, 254            # 목표 속도(46) / 가속도(41) 레지스터 허용값
STATUS_BITS = [(ERRBIT_VOLTAGE, '전압'), (ERRBIT_ANGLE, '엔코더'), (ERRBIT_OVERHEAT, '과열'),
               (ERRBIT_OVERELE, '과전류'), (ERRBIT_OVERLOAD, '과부하')]
DOUBLE_CLICKED = len(sys.argv) == 1       # 인자 없이 실행 = 파일 탐색기에서 더블클릭

for _stream in (sys.stdout, sys.stderr):  # 한글을 못 쓰는 콘솔 인코딩에서도 죽지 않게
    try:
        _stream.reconfigure(errors='replace')
    except (AttributeError, ValueError):
        pass


def pause_and_exit(msg=None, code=0):
    if msg:
        print(msg)
    if DOUBLE_CLICKED and sys.stdin and sys.stdin.isatty():
        input("\nEnter를 누르면 창이 닫힙니다...")
    sys.exit(code)


# ---------- 포트 ----------
def list_serial_ports():
    """시리얼 포트 목록. 서보 어댑터로 보이는 것이 앞에 온다."""
    from serial.tools import list_ports
    ports = list(list_ports.comports())
    devices = {p.device for p in ports}
    out = []
    for p in ports:
        # macOS는 같은 장치를 /dev/tty.X 와 /dev/cu.X 로 두 번 보여 준다 → cu 만 사용
        if p.device.startswith('/dev/tty.') and p.device.replace('/dev/tty.', '/dev/cu.', 1) in devices:
            continue
        out.append({'device': p.device, 'description': p.description or '',
                    'adapter': p.vid in ADAPTER_CHIPS, 'chip': ADAPTER_CHIPS.get(p.vid)})
    out.sort(key=lambda p: (not p['adapter'], p['device']))
    return out


def find_port():
    """서보 어댑터 포트 이름, 없으면 None."""
    ports = [p for p in list_serial_ports() if p['adapter']]
    if len(ports) > 1:
        print("어댑터가 여러 개입니다:", ', '.join(p['device'] for p in ports), "→", ports[0]['device'], "사용")
    return ports[0]['device'] if ports else None


def port_hint():
    if sys.platform.startswith('win'):
        return ("확인할 것: 장치 관리자 > 포트(COM & LPT)에 장치가 보이는지, "
                "CH340/CH343 드라이버(WCH) 설치, 충전 전용이 아닌 데이터 USB 케이블")
    if sys.platform == 'darwin':
        return ("확인할 것: 터미널에서 ls /dev/cu.* 로 장치가 보이는지, "
                "필요하면 WCH CH34x 드라이버 설치, 데이터 USB 케이블")
    return ("확인할 것: ls /dev/ttyUSB* /dev/ttyACM* 로 장치가 보이는지, "
            "권한 오류면 sudo usermod -aG dialout $USER 후 다시 로그인, 데이터 USB 케이블")


def find_port_or_exit():
    port = find_port()
    if not port:
        pause_and_exit("서보 어댑터(USB 시리얼)를 찾지 못했습니다.\n" + port_hint(), 1)
    return port


def open_bus(port_name, baud=1000000):
    """(PortHandler, sms_sts) 반환. 실패하면 원인 안내와 함께 RuntimeError."""
    port = PortHandler(port_name)
    try:
        ok = port.openPort() and port.setBaudRate(int(baud))
    except Exception as e:
        reason = str(e)
        if 'PermissionError' in reason or 'Access is denied' in reason or '액세스' in reason or 'Permission denied' in reason:
            reason = "다른 프로그램이 포트를 쓰고 있거나 접근 권한이 없습니다"
        raise RuntimeError(f"{port_name} 열기 실패: {reason}\n{port_hint()}") from None
    if not ok:
        raise RuntimeError(f"{port_name} 열기 실패 (지원하지 않는 보드레이트 {baud}?)")
    return port, sms_sts(port)


# ---------- 상태 읽기 ----------
def _word(d, i):
    return SCS_MAKEWORD(d[i], d[i + 1])


def read_state(s, sid):
    """40~47(제어)과 56~70(상태) 블록을 읽어 dict로 반환. 실패 시 None."""
    ctrl, r1, _ = s.readTxRx(sid, 40, 8)
    st, r2, _ = s.readTxRx(sid, 56, 15)
    if r1 != COMM_SUCCESS or r2 != COMM_SUCCESS:
        return None
    return {
        'torque': ctrl[0],                                   # 40
        'acc': ctrl[1],                                      # 41
        'goal': _word(ctrl, 2),                              # 42
        'goal_speed': _word(ctrl, 6),                        # 46
        'pos': _word(st, 0),                                 # 56
        'speed': SCS_TOHOST(_word(st, 2), 15),               # 58, 스텝/s
        'load_pct': SCS_TOHOST(_word(st, 4), 10) / 10,       # 60, 0.1% 단위, bit10 = 방향
        'volt': st[6] / 10,                                  # 62
        'temp': st[7],                                       # 63
        'status': st[9],                                     # 65
        'moving': st[10],                                    # 66
        'current_mA': SCS_TOHOST(_word(st, 13), 15) * 6.5,   # 69, 1단위 = 6.5 mA
    }


def status_names(v):
    return [n for b, n in STATUS_BITS if v & b]


# ---------- 키 입력 ----------
class KeyReader:
    """논블로킹 키 입력. read()는 문자 또는 'ENTER', 'BACKSPACE', 'ESC', 'LEFT', 'RIGHT', 'UP', 'DOWN' 목록을 돌려준다."""

    def __enter__(self):
        self.win = sys.platform.startswith('win')
        if not self.win and sys.stdin.isatty():
            import termios, tty
            self._fd = sys.stdin.fileno()
            self._old = termios.tcgetattr(self._fd)
            tty.setcbreak(self._fd)                 # 한 글자씩 읽기, Ctrl+C는 그대로 동작
        return self

    def __exit__(self, *exc):
        if not self.win and getattr(self, '_old', None) is not None:
            import termios
            termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def read(self):
        return self._read_win() if self.win else self._read_posix()

    def _read_win(self):
        import msvcrt
        keys = []
        while msvcrt.kbhit():
            ch = msvcrt.getwch()
            if ch in ('\x00', '\xe0'):
                keys.append({'K': 'LEFT', 'M': 'RIGHT', 'H': 'UP', 'P': 'DOWN'}.get(msvcrt.getwch()))
            elif ch == '\x03':
                raise KeyboardInterrupt
            else:
                keys.append(self._map(ch))
        return [k for k in keys if k]

    def _read_posix(self):
        import select
        if not sys.stdin.isatty():
            return []
        keys = []
        while select.select([sys.stdin], [], [], 0)[0]:
            ch = os.read(sys.stdin.fileno(), 1).decode(errors='ignore')
            if ch == '\x1b':                            # 화살표: ESC [ A/B/C/D
                seq = ''
                while len(seq) < 2 and select.select([sys.stdin], [], [], 0.01)[0]:
                    seq += os.read(sys.stdin.fileno(), 1).decode(errors='ignore')
                keys.append({'[A': 'UP', '[B': 'DOWN', '[C': 'RIGHT', '[D': 'LEFT'}.get(seq, 'ESC' if not seq else None))
            else:
                keys.append(self._map(ch))
        return [k for k in keys if k]

    @staticmethod
    def _map(ch):
        if ch in ('\r', '\n'):
            return 'ENTER'
        if ch in ('\x08', '\x7f'):
            return 'BACKSPACE'
        if ch == '\x1b':
            return 'ESC'
        return ch if ch.isprintable() else None


# ---------- 그래프 ----------
def setup_korean_font(plt):
    """설치된 한글 폰트 중 하나를 matplotlib에 지정."""
    from matplotlib import font_manager
    have = {f.name for f in font_manager.fontManager.ttflist}
    for name in ['Malgun Gothic', 'AppleGothic', 'Apple SD Gothic Neo', 'NanumGothic',
                 'Noto Sans CJK KR', 'Noto Sans KR', 'UnDotum']:
        if name in have:
            plt.rcParams['font.family'] = name
            break
    plt.rcParams['axes.unicode_minus'] = False
