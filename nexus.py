import curses, socket, time, threading, struct, fcntl
import sys
from dataclasses import dataclass
from enum import Enum

# Joystick ioctl codes (JSIOCGAXES, JSIOCGBUTTONS)
IOCTL_GET_AXES, IOCTL_GET_BUTTONS = 0x80016a11, 0x80016a12
AXIS_MAX = 32767.0

# FRC Driver Station protocol
DS_SEND_PORT, DS_RECV_PORT = 1110, 1150
SEND_INTERVAL = 0.05  # 20Hz
UI_REFRESH = 0.016    # ~60fps

class Mode(Enum):       TeleOp = 0x00; TestMode = 0x01; Autonomous = 0x02
class Request(Enum):    Unconnected = 0x00; RestartCode = 0x04; RebootRoboRIO = 0x08; Normal = 0x80

@dataclass(slots=True)
class StatePacket:
    code_running: bool
    bat_voltage: float

    @classmethod
    def from_bytes(cls, data: bytes) -> 'StatePacket':
        _, _, _, status, bat_int, bat_frac, *_ = struct.unpack('>HBBBBBB', data)
        return cls(bool(status & 0x20), bat_int + bat_frac / 255.0)

def control_packet(num: int, mode: Mode, enabled: bool, request: Request) -> bytes:
    return struct.pack('>HBBBB', num, 0x01, mode.value | (0x04 if enabled else 0), request.value, 0)

class Joystick:
    def __init__(self, dev: str = '/dev/input/js0'):
        self.mu = threading.Lock()
        self.file, self.connected = None, False
        self.num_axes, self.num_buttons = 0, 0
        self.axes: list[float] = []
        self.buttons: list[int] = []
        try:
            self.file = open(dev, 'rb')
            buf = bytearray(1)
            fcntl.ioctl(self.file, IOCTL_GET_AXES, buf); self.num_axes = buf[0]
            fcntl.ioctl(self.file, IOCTL_GET_BUTTONS, buf); self.num_buttons = buf[0]
            self.axes, self.buttons = [0.0] * self.num_axes, [0] * self.num_buttons
            self.connected = True
            threading.Thread(target=self._read_loop, daemon=True).start()
        except (FileNotFoundError, OSError): pass

    def get_state(self) -> tuple[list[float], list[int]]:
        with self.mu: return list(self.axes), list(self.buttons)
    def get_packet(self) -> bytes:
        if not self.connected: return bytes([3, 0x0c, 0, 0, 0])
        with self.mu:
            axis_bytes = bytes(int(max(-128, min(127, a * 127))) & 0xFF for a in self.axes)
            btn_bytes = bytearray((self.num_buttons + 7) // 8)
            for i, pressed in enumerate(self.buttons):
                if pressed: btn_bytes[i // 8] |= 1 << (i % 8)
            payload = bytes([self.num_axes]) + axis_bytes + bytes([self.num_buttons]) + btn_bytes + b'\x00'
            return bytes([len(payload) + 1, 0x0c]) + payload

    def _read_loop(self) -> None:
        while True:
            if self.file:
                _, val, typ, num = struct.unpack('IhBB', self.file.read(8))
                with self.mu:
                    if typ & 1 and num < self.num_buttons: self.buttons[num] = val
                    if typ & 2 and num < self.num_axes: self.axes[num] = val / AXIS_MAX
        # if self.file: self.file.close()

class Nexus:
    def __init__(self, team_number: int):
        self.js = Joystick()
        self.robot_ip = f"roboRIO-{team_number}-FRC.local"
        self.mu = threading.Lock()
        self.send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.recv_sock.bind(("", DS_RECV_PORT))
        self.recv_sock.settimeout(0.05)
        self.packet_num, self.running = 0, False
        self.last_state: StatePacket | None = None
        self.request, self.enabled, self.mode = Request.Normal, False, Mode.TeleOp
        self.th1 = threading.Thread(target=self._recv_loop, daemon=True)
        self.th2 = threading.Thread(target=self._send_loop, daemon=True)
        self.th1.start(); self.th2.start()

    def stop(self) -> None:
        try: self.send_sock.sendto(control_packet(self.packet_num, self.mode, False, Request.Normal), (self.robot_ip, DS_SEND_PORT))
        except (socket.error, OSError): pass

    def set_mode(self, mode: Mode) -> None: self.mode = mode
    def toggle_enable(self) -> None: self.enabled = not self.enabled
    def send_request(self, request: Request) -> None: self.request = request
    def get_state(self) -> StatePacket | None:
        with self.mu: return self.last_state

    def _recv_loop(self) -> None:
        while True:
            try:
                data, _ = self.recv_sock.recvfrom(1024)
                with self.mu: self.last_state = StatePacket.from_bytes(data)
            except socket.timeout: pass
    def _send_loop(self) -> None:
        while True:
            try:
                packet = control_packet(self.packet_num, self.mode, self.enabled, self.request) + self.js.get_packet()
                self.send_sock.sendto(packet, (self.robot_ip, DS_SEND_PORT))
                self.request, self.packet_num = Request.Normal, self.packet_num + 1
            except (socket.error, OSError): pass
            time.sleep(SEND_INTERVAL)

class UI:
    def __init__(self, net: Nexus):
        self.net = net
        self.keybinds = {
            ord('e'): ('enable',    net.toggle_enable),
            ord('a'): ('auto',      lambda: net.set_mode(Mode.Autonomous)),
            ord('t'): ('teleop',    lambda: net.set_mode(Mode.TeleOp)),
            ord('r'): ('restart',   lambda: net.send_request(Request.RestartCode)),
        }

    def __call__(self, win: curses.window) -> None:
        curses.curs_set(0); win.nodelay(True)
        while (ch := win.getch()) != ord('q'):
            s = self.net.get_state()
            status = f"[{'CONN' if s else 'DISC'}] [{'CODE' if s and s.code_running else 'NO CODE'}] {s.bat_voltage if s else 0:.1f}V | {self.net.mode.name} | {'EN' if self.net.enabled else 'DIS'}"
            win.erase(); win.addstr(0, 0, status)
            win.addstr(1, 0, ' '.join(f"{chr(k)}:{v[0]}" for k, v in self.keybinds.items()) + " q:quit")
            js = self.net.js
            if js.connected:
                axes, buttons = js.get_state()
                win.addstr(3, 0, f"Axes:  {' '.join(f'{a:+.2f}' for a in axes[:6])}")
                win.addstr(4, 0, f"Btns:  {''.join('●' if b else '○' for b in buttons)}")
            else:
                win.addstr(3, 0, "Joystick: NOT CONNECTED")
            win.refresh()
            if ch in self.keybinds: self.keybinds[ch][1]()
            if ch == 10: self.net.toggle_enable()
            time.sleep(UI_REFRESH)

if __name__ == '__main__':
    if len(sys.argv) == 1:
        print("Usage: nexus <team number>")
        exit(1)
    net = Nexus(int(sys.argv[1]))
    try: curses.wrapper(UI(net))
    finally: net.stop()
