# Single file FRC driver station targetting linux with 0 dependencies
# Written Feb 2026 <gabriel@nakamoto.ca> + Claude Opus 4.5
import curses, socket, time, threading, struct, fcntl, os, re, select, sys
from datetime import datetime, timezone

DS_SEND_PORT, DS_RECV_PORT, SEND_INTERVAL = 1110, 1150, 0.05
AXIS_REMAP = (0, 1, 2, 4, 5, 3)  # Swap axes 3,4,5 -> 4,5,3

class Mode:    TELEOP, TEST, AUTO = 0x00, 0x01, 0x02
class Request: NORMAL, RESTART, REBOOT = 0x00, 0x04, 0x08

def ctrl_packet(num, mode, enabled, estopped=False, req=0): 
    return struct.pack('>HBBBB', num, 0x01, mode | (0x04 if enabled else 0) | (0x80 if estopped else 0), req, 0)

def datetime_tags():
    now = datetime.now(timezone.utc)
    d = struct.pack('>IBBBBB', now.microsecond, now.second, now.minute, now.hour, now.day, now.month - 1) + bytes([now.year - 1900])
    tz = time.tzname[0].encode()
    return bytes([len(d) + 1, 0x0f]) + d + bytes([len(tz) + 1, 0x10]) + tz

class Joystick:
    def __init__(self, dev='/dev/input/js0'):
        self.mu, self.file, self.connected = threading.Lock(), None, False
        self.num_axes, self.num_buttons, self.axes, self.buttons = 0, 0, [], []
        try:
            self.file = open(dev, 'rb'); buf = bytearray(1)
            fcntl.ioctl(self.file, 0x80016a11, buf); self.num_axes = buf[0]
            fcntl.ioctl(self.file, 0x80016a12, buf); self.num_buttons = buf[0]
            self.axes, self.buttons = [0.0] * self.num_axes, [0] * self.num_buttons
            self.connected = True
            threading.Thread(target=self._read, daemon=True).start()
        except OSError: pass

    def _read(self):
        os.set_blocking(self.file.fileno(), False)
        while select.select([self.file], [], [], 0.1) or True:
            try:
                if (data := self.file.read(8)) and len(data) == 8:
                    _, val, typ, num = struct.unpack('IhBB', data)
                    with self.mu:
                        if typ & 1 and num < self.num_buttons: self.buttons[num] = val
                        if typ & 2 and num < self.num_axes: self.axes[num] = val / 32767.0
            except OSError: break

    def packet(self):
        if not self.connected: return bytes([4, 0x0c, 0, 0, 0, 0])
        with self.mu:
            axes = [self.axes[AXIS_REMAP[i]] if i < len(AXIS_REMAP) else self.axes[i] for i in range(len(self.axes))]
            axis_bytes = bytes(int(max(-128, min(127, a * 127))) & 0xFF for a in axes)
            btn_flags = sum(1 << i for i, b in enumerate(self.buttons) if b)
            btn_bytes = btn_flags.to_bytes((self.num_buttons + 7) // 8, 'big')
            payload = bytes([self.num_axes]) + axis_bytes + bytes([self.num_buttons]) + btn_bytes + b'\x00'
            return bytes([len(payload) + 1, 0x0c]) + payload

class KeyboardMonitor:
    def __init__(self, on_estop):
        self.on_estop, self.active, self.triggered, self.files = on_estop, False, 0.0, []
        try:
            for block in open('/proc/bus/input/devices').read().split('\n\n'):
                if 'kbd' in block.lower() or 'keyboard' in block.lower():
                    if m := re.search(r'event(\d+)', block):
                        try: f = open(f'/dev/input/event{m.group(1)}', 'rb'); os.set_blocking(f.fileno(), False); self.files.append(f)
                        except OSError: pass
            if self.files: self.active = True; threading.Thread(target=self._read, daemon=True).start()
        except OSError: pass

    def _read(self):
        while True:
            for f in select.select(self.files, [], [], 0.1)[0]:
                try:
                    if (data := f.read(24)) and len(data) == 24:
                        _, _, typ, code, val = struct.unpack('llHHI', data)
                        if typ == 1 and code == 57 and val == 1: self.triggered = time.time(); self.on_estop()
                except OSError: pass

class Nexus:
    def __init__(self, team):
        self.js, self.robot_ip = Joystick(), f"roboRIO-{team}-FRC.local"
        self.kb = KeyboardMonitor(self.estop)
        self._ip, self._dns_retry, self.mu = None, 0, threading.Lock()
        self.send_sock, self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM), socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind(("", DS_RECV_PORT))
        self.pkt_num, self.state, self.request, self.enabled, self.estopped, self.mode = 0, None, Request.NORMAL, False, False, Mode.TELEOP
        self._sent_dt = False
        threading.Thread(target=self._recv, daemon=True).start()
        threading.Thread(target=self._send, daemon=True).start()

    def estop(self): self.estopped = True

    def _recv(self):
        self.recv_sock.settimeout(0.1)
        while True:
            try:
                data, _ = self.recv_sock.recvfrom(1024)
                if len(data) >= 8:
                    _, _, _, status, bat_int, bat_frac = struct.unpack('>HBBBB', data[:6])
                    with self.mu: self.state = (bool(status & 0x20), bat_int + bat_frac / 255.0)
            except (socket.timeout, OSError): pass

    def _send(self):
        while True:
            if not self._ip and time.time() > self._dns_retry:
                try: self._ip = socket.gethostbyname(self.robot_ip)
                except socket.gaierror: self._dns_retry = time.time() + 2
            if self._ip:
                try:
                    pkt = ctrl_packet(self.pkt_num, self.mode, self.enabled, self.estopped, self.request)
                    if not self._sent_dt:
                        pkt += datetime_tags()
                        with self.mu:
                            if self.state: self._sent_dt = True
                    pkt += self.js.packet()
                    self.send_sock.sendto(pkt, (self._ip, DS_SEND_PORT))
                    self.request, self.pkt_num = Request.NORMAL, self.pkt_num + 1
                except OSError: self._ip = None
            time.sleep(SEND_INTERVAL)

    def stop(self):
        if self.state and self._ip:
            try: self.send_sock.sendto(ctrl_packet(self.pkt_num, self.mode, False, self.estopped), (self._ip, DS_SEND_PORT))
            except OSError: pass

def ui(win, net):
    curses.curs_set(0); win.nodelay(True)
    keys = {'e': ('enable', lambda: setattr(net, 'enabled', not net.enabled)),
            'a': ('auto', lambda: setattr(net, 'mode', Mode.AUTO)),
            't': ('teleop', lambda: setattr(net, 'mode', Mode.TELEOP)),
            'r': ('restart', lambda: setattr(net, 'request', Request.RESTART))}
    while (ch := win.getch()) != ord('q'):
        if ch == 10 or ch == ord(' '): net.estop()  # Enter or space = e-stop
        if ch > 0 and chr(ch) in keys: keys[chr(ch)][1]()
        with net.mu: s = net.state
        conn, code, v = bool(s), s and s[0], s[1] if s else 0
        win.erase()
        win.addstr(0, 0, "      --- Nexus Robot Connection ---", curses.A_BOLD)
        status = 'ESTOP' if net.estopped else ('EN' if net.enabled else 'DIS')
        win.addstr(1, 0, f"[{'CONN' if conn else 'DISC'}] [{'CODE' if code else 'NO CODE'}] {'AUTO' if net.mode == Mode.AUTO else 'TELEOP'} | {status}")
        win.addstr(2, 0, f"Battery: [{'█' * int(v / 14 * 20)}{'░' * (20 - int(v / 14 * 20))}] {v:.1f}V")
        win.addstr(3, 0, ' '.join(f"{k}:{v[0]}" for k, v in keys.items()) + " q:quit")
        flash = time.time() - net.kb.triggered < 1.0
        win.addstr(4, 0, f"E-Stop: {'TRIGGERED!' if flash else ('ACTIVE' if net.kb.active else 'unavailable')}", curses.A_REVERSE if flash else 0)
        if net.js.connected:
            with net.js.mu: axes, btns = list(net.js.axes), list(net.js.buttons)
            win.addstr(6, 0, f"Axes:  {' '.join(f'{a:+.2f}' for a in axes[:6])}")
            win.addstr(7, 0, f"Btns:  {''.join('●' if b else '○' for b in btns)}")
        else: win.addstr(6, 0, "Joystick: NOT CONNECTED")
        win.refresh(); time.sleep(0.016)

if __name__ == '__main__':
    if len(sys.argv) < 2: print("Usage: nexus <team number>"); sys.exit(1)
    net = Nexus(int(sys.argv[1]))
    try: curses.wrapper(lambda w: ui(w, net))
    finally: net.stop(); os._exit(0)
