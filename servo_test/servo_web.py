"""STS 버스 서보 웹 콘솔 - 여러 서보 실행/구동/실시간 측정

사용법:
    파일 더블클릭 또는 start_web.bat / start_web.sh   # 어댑터 자동 연결 + ID 0~20 검색 + 브라우저 열기
    python servo_web.py --port COM13 --scan 0-20      # Linux: /dev/ttyUSB0, macOS: /dev/cu.usbserial-xxx
    python servo_web.py --no-browser                  # 브라우저 자동 실행 안 함

브라우저: http://127.0.0.1:8765  (이 PC에서만 접속 가능, --http 로 포트 변경)

안전:
    - 종료(Ctrl+C), 연결 해제, 브라우저 창을 모두 닫은 뒤 3초가 지나면 모든 서보 토크 OFF
    - 토크 ON 시 목표를 현재 위치로 먼저 맞춰서 튀지 않게 함
    - EEPROM(ID, 보드레이트, 각도 제한, PID 등)은 쓰지 않음
"""
import argparse, collections, csv, json, os, sys, threading, time, webbrowser
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from scservo_sdk import COMM_SUCCESS, BROADCAST_ID
from servo_common import (STEP_DEG, POS_MIN, POS_MAX, SPEED_MAX, ACC_MAX, DOUBLE_CLICKED,
                          list_serial_ports as list_ports, find_port, port_hint, open_bus,
                          read_state, status_names)

HERE = os.path.dirname(os.path.abspath(__file__))
LOG_FIELDS = ['t_s', 'id', 'torque', 'goal', 'pos', 'speed', 'load_pct', 'volt', 'temp', 'current_mA', 'moving', 'status']


class Bus:
    """시리얼 버스 하나를 소유. 폴링 스레드와 HTTP 명령이 같은 락을 공유한다."""

    def __init__(self, hz=20):
        self.lock = threading.RLock()
        self.port = self.s = None
        self.port_name, self.baud = None, 1000000
        self.ids, self.info = [], {}
        self.state, self.fails = {}, {}
        self.moves, self.results = {}, {}
        self.hz, self.hz_actual = hz, 0.0
        self.log_file = self.log_writer = None
        self.log_name = None
        self.t0 = time.perf_counter()
        self.msg = '연결 안 됨'
        self.clients, self.last_client_t = 0, time.time()
        threading.Thread(target=self._poll_loop, daemon=True).start()
        threading.Thread(target=self._watchdog, daemon=True).start()

    # ---------- 연결 ----------
    @property
    def connected(self):
        return self.port is not None

    def connect(self, port_name, baud=1000000):
        with self.lock:
            self.disconnect()
            try:
                self.port, self.s = open_bus(port_name, baud)
            except RuntimeError as e:
                return False, str(e)
            self.port_name, self.baud = port_name, int(baud)
            self.msg = f"{port_name} @ {baud} 연결됨"
            return True, self.msg

    def disconnect(self):
        with self.lock:
            if not self.connected:
                return True, '이미 연결 해제됨'
            self.torque_off_all()
            self.port.closePort()
            self.port = self.s = None
            self.ids, self.state, self.moves = [], {}, {}
            self.msg = '연결 해제됨 (토크 OFF 후)'
            return True, self.msg

    def scan(self, lo, hi):
        if not self.connected:
            return False, '먼저 연결하세요'
        lo, hi = max(0, int(lo)), min(253, int(hi))
        found = []
        with self.lock:
            for i in range(lo, hi + 1):
                model, res, _ = self.s.ping(i)
                if res == COMM_SUCCESS:
                    fw_main, _, _ = self.s.read1ByteTxRx(i, 0)
                    fw_sub, _, _ = self.s.read1ByteTxRx(i, 1)
                    vmax, _, _ = self.s.read1ByteTxRx(i, 14)       # EEPROM 읽기만 (전압 경고 기준)
                    vmin, _, _ = self.s.read1ByteTxRx(i, 15)
                    self.info[i] = {'model': model, 'fw': f"{fw_main}.{fw_sub}",
                                    'max_volt': vmax / 10 if vmax else None,
                                    'min_volt': vmin / 10 if vmin else None}
                    found.append(i)
            self.ids = found
            for i in found:
                self.results.setdefault(i, collections.deque(maxlen=8))
                self.fails.setdefault(i, 0)
            self.msg = f"ID {lo}~{hi} 검색: {len(found)}개 발견 {found}"
        return True, self.msg

    # ---------- 읽기 ----------
    def read_state(self, i):
        return read_state(self.s, i)

    def _poll_loop(self):
        while True:
            t_cycle = time.perf_counter()
            if not (self.connected and self.ids):
                time.sleep(0.1)
                continue
            for i in list(self.ids):
                with self.lock:
                    if not self.connected:
                        break
                    d = self.read_state(i)
                    now = time.perf_counter()
                    if d is None:
                        self.fails[i] = self.fails.get(i, 0) + 1
                        self.state[i] = None
                        continue
                    d['t'] = round(now - self.t0, 3)
                    self.state[i] = d
                    self._track_move(i, d, now)
                    if self.log_writer:
                        self.log_writer.writerow({'t_s': d['t'], 'id': i, **{k: d[k] for k in LOG_FIELDS[2:]}})
            dt = time.perf_counter() - t_cycle
            time.sleep(max(0, 1 / self.hz - dt))
            total = time.perf_counter() - t_cycle
            self.hz_actual = 0.8 * self.hz_actual + 0.2 * (1 / total if total else 0)

    def _track_move(self, i, d, now):
        m = self.moves.get(i)
        if m is None:
            return
        elapsed = now - m['t0']
        m['peak_mA'] = max(m['peak_mA'], abs(d['current_mA']))
        if d['moving']:
            m['seen_moving'], m['settle'] = True, None
        elif m['settle'] is None and (m['seen_moving'] or elapsed > 0.3):
            m['settle'] = now + 0.2           # 멈춘 뒤 0.2초 안정화 후 기록
        timeout = elapsed > 10.0
        if (m['settle'] is not None and now >= m['settle']) or timeout:
            err = d['pos'] - m['goal']
            self.results[i].appendleft({
                'start': m['start'], 'goal': m['goal'], 'pos': d['pos'], 'err': err,
                'err_deg': round(err * STEP_DEG, 3),
                'ms': round((elapsed - (0 if timeout else 0.2)) * 1000),
                'peak_mA': round(m['peak_mA']), 'speed': m['speed'], 'acc': m['acc'],
                'timeout': timeout,
            })
            del self.moves[i]

    # ---------- 쓰기 ----------
    def _need(self, i):
        if not self.connected:
            return '연결 안 됨'
        if i not in self.ids:
            return f'ID {i}는 검색된 서보가 아닙니다'
        return None

    def torque(self, i, on, speed=0, acc=0):
        err = self._need(i)
        if err:
            return False, err
        with self.lock:
            if on:
                d = self.read_state(i)
                if d is None:
                    return False, f'ID {i} 읽기 실패'
                self.s.WritePosEx(i, d['pos'], speed, acc)    # 목표 = 현재 위치 → 켜져도 움직이지 않음
                self.s.write1ByteTxRx(i, 40, 1)
                return True, f'ID {i} 토크 ON (위치 {d["pos"]} 유지)'
            self.s.write1ByteTxRx(i, 40, 0)
            self.moves.pop(i, None)
            return True, f'ID {i} 토크 OFF'

    def move(self, i, goal, speed, acc):
        err = self._need(i)
        if err:
            return False, err
        goal, speed, acc = int(goal), int(speed), int(acc)
        if not POS_MIN <= goal <= POS_MAX:
            return False, f'목표 {goal}가 {POS_MIN}~{POS_MAX} 밖입니다'
        if not 0 <= speed <= SPEED_MAX:
            return False, f'속도는 0~{SPEED_MAX}'
        if not 0 <= acc <= ACC_MAX:
            return False, f'가속도는 0~{ACC_MAX}'
        with self.lock:
            d = self.read_state(i)
            if d is None:
                return False, f'ID {i} 읽기 실패'
            if not d['torque']:
                return False, f'ID {i} 토크가 OFF입니다. 먼저 토크를 켜세요'
            self.s.WritePosEx(i, goal, speed, acc)
            self.moves[i] = {'t0': time.perf_counter(), 'start': d['pos'], 'goal': goal,
                             'speed': speed, 'acc': acc, 'seen_moving': False,
                             'settle': None, 'peak_mA': 0.0}
            return True, f'ID {i}: {d["pos"]} → {goal} ({goal - d["pos"]:+d} 스텝)'

    def stop(self, i):
        err = self._need(i)
        if err:
            return False, err
        with self.lock:
            d = self.read_state(i)
            if d is None:
                return False, f'ID {i} 읽기 실패'
            if d['torque']:
                self.s.WritePosEx(i, d['pos'], 0, 0)
            return True, f'ID {i} 정지 (위치 {d["pos"]})'

    def torque_off_all(self):
        with self.lock:
            if not self.connected:
                return False, '연결 안 됨'
            self.s.write1ByteTxOnly(BROADCAST_ID, 40, 0)   # 응답 없는 브로드캐스트
            time.sleep(0.005)
            bad = []
            for i in self.ids:
                self.s.write1ByteTxRx(i, 40, 0)
                v, r, _ = self.s.read1ByteTxRx(i, 40)
                if r != COMM_SUCCESS or v != 0:
                    bad.append(i)
            self.moves.clear()
            return (not bad), ('전체 토크 OFF 확인' if not bad else f'토크 OFF 확인 실패: {bad}')

    # ---------- 기록 ----------
    def set_logging(self, on):
        with self.lock:
            if on and not self.log_writer:
                os.makedirs(os.path.join(HERE, 'logs'), exist_ok=True)
                self.log_name = os.path.join('logs', time.strftime('servo_log_%Y%m%d_%H%M%S.csv'))
                self.log_file = open(os.path.join(HERE, self.log_name), 'w', newline='')
                self.log_writer = csv.DictWriter(self.log_file, LOG_FIELDS)
                self.log_writer.writeheader()
                return True, f'기록 시작: {self.log_name}'
            if not on and self.log_writer:
                self.log_file.close()
                self.log_file = self.log_writer = None
                return True, f'기록 저장: {self.log_name}'
            return True, '변경 없음'

    def snapshot(self):
        with self.lock:
            return self._snapshot()

    def _snapshot(self):
        servos = []
        for i in self.ids:
            d = self.state.get(i)
            servos.append({
                'id': i, **self.info.get(i, {}), 'state': d, 'fails': self.fails.get(i, 0),
                'moving_cmd': i in self.moves,
                'status_text': None if d is None else status_names(d['status']),
                'results': list(self.results.get(i, [])),
            })
        return {'connected': self.connected, 'port': self.port_name, 'baud': self.baud,
                'hz': round(self.hz_actual, 1), 'hz_target': self.hz, 'msg': self.msg,
                'logging': self.log_name if self.log_writer else None, 'servos': servos}

    # ---------- 안전 ----------
    def _watchdog(self):
        """브라우저가 모두 닫히고 3초가 지나면 토크 OFF."""
        armed = False
        while True:
            time.sleep(0.5)
            if self.clients > 0:
                armed = True
            elif armed and time.time() - self.last_client_t > 3 and self.connected:
                self.torque_off_all()
                self.msg = '브라우저가 닫혀 전체 토크 OFF'
                print(self.msg)
                armed = False

    def shutdown(self):
        if self.connected:
            ok, msg = self.torque_off_all()
            print(msg)
        self.set_logging(False)
        if self.connected:
            self.port.closePort()


bus = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        body = json.dumps(obj, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            with open(os.path.join(HERE, 'web', 'index.html'), 'rb') as f:
                body = f.read()
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)
        elif self.path == '/api/ports':
            self._json({'ports': list_ports()})
        elif self.path == '/api/state':
            self._json(bus.snapshot())
        elif self.path == '/events':
            self._events()
        else:
            self.send_error(404)

    def _events(self):
        self.send_response(200)
        self.send_header('Content-Type', 'text/event-stream')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        bus.clients += 1
        try:
            while True:
                data = json.dumps(bus.snapshot(), ensure_ascii=False)
                self.wfile.write(f"data: {data}\n\n".encode('utf-8'))
                self.wfile.flush()
                time.sleep(1 / max(bus.hz, 1))
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass
        finally:
            bus.clients -= 1
            bus.last_client_t = time.time()

    def do_POST(self):
        n = int(self.headers.get('Content-Length', 0))
        try:
            req = json.loads(self.rfile.read(n) or b'{}')
        except json.JSONDecodeError:
            return self._json({'ok': False, 'msg': 'JSON 오류'}, 400)
        name = self.path.removeprefix('/api/')
        try:
            ok, msg = self._dispatch(name, req)
        except Exception as e:
            ok, msg = False, f'{type(e).__name__}: {e}'
        bus.msg = msg
        self._json({'ok': ok, 'msg': msg})

    def _dispatch(self, name, r):
        ids = r.get('ids') or ([r['id']] if 'id' in r else [])
        if name == 'connect':
            return bus.connect(r['port'], r.get('baud', 1000000))
        if name == 'disconnect':
            return bus.disconnect()
        if name == 'scan':
            return bus.scan(r.get('lo', 0), r.get('hi', 20))
        if name == 'torque_all_off':
            return bus.torque_off_all()
        if name == 'logging':
            return bus.set_logging(bool(r.get('on')))
        if name == 'rate':
            bus.hz = max(1, min(int(r.get('hz', 20)), 100))
            return True, f'갱신 주기 {bus.hz} Hz'
        if not ids:
            return False, '대상 ID가 없습니다'
        results = []
        for i in ids:
            i = int(i)
            if name == 'torque':
                results.append(bus.torque(i, bool(r.get('on')), int(r.get('speed', 0)), int(r.get('acc', 0))))
            elif name == 'move':
                goal = r.get('goal')
                if goal is None:                       # 상대 이동
                    d = bus.state.get(i)
                    if not d:
                        results.append((False, f'ID {i} 상태 없음')); continue
                    goal = (d['goal'] if d['torque'] else d['pos']) + int(r.get('delta', 0))
                results.append(bus.move(i, goal, r.get('speed', 500), r.get('acc', 30)))
            elif name == 'stop':
                results.append(bus.stop(i))
            else:
                return False, f'알 수 없는 명령: {name}'
        return all(ok for ok, _ in results), ' / '.join(m for _, m in results)


def main():
    global bus
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', help='예: COM13, /dev/ttyUSB0 (생략하면 어댑터 자동 검색)')
    ap.add_argument('--baud', type=int, default=1000000)
    ap.add_argument('--scan', default='0-20', help='시작 시 검색할 ID 범위, 예: 0-20')
    ap.add_argument('--http', type=int, default=8765)
    ap.add_argument('--hz', type=int, default=20)
    ap.add_argument('--no-browser', action='store_true')
    args = ap.parse_args()

    try:
        server = ThreadingHTTPServer(('127.0.0.1', args.http), Handler)
    except OSError as e:
        raise OSError(f'{e} - 이미 실행 중이거나 {args.http} 포트를 다른 프로그램이 쓰고 있습니다 (--http 로 변경 가능)') from None
    server.daemon_threads = True

    bus = Bus(args.hz)
    port = args.port or find_port()
    if port:
        ok, msg = bus.connect(port, args.baud)
        print(msg)
        if ok:
            lo, hi = (int(x) for x in args.scan.split('-'))
            print(bus.scan(lo, hi)[1])
    else:
        print('어댑터를 찾지 못했습니다. 연결한 뒤 브라우저에서 ↻ 를 눌러 포트를 선택하세요.\n' + port_hint())

    url = f'http://127.0.0.1:{args.http}'
    print(f'웹 콘솔: {url}   (종료: 이 창에서 Ctrl+C → 전체 토크 OFF)')
    if not args.no_browser:
        threading.Timer(0.5, webbrowser.open, [url]).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        print('\n종료 중...')
        bus.shutdown()
        server.server_close()


if __name__ == '__main__':
    try:
        main()
    except OSError as e:
        print(f'오류: {e}')
    finally:
        if DOUBLE_CLICKED and sys.stdin and sys.stdin.isatty():
            input('\nEnter를 누르면 창이 닫힙니다...')
