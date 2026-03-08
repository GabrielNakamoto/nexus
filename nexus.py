# Single file FRC driver station targetting linux with 0 dependencies
# Written Feb 2026 <gabriel@nakamoto.ca> + Claude Opus 4.5

import curses, socket, time, threading, struct, fcntl
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum

IOCTL_GET_AXES, IOCTL_GET_BUTTONS = 0x80016a11, 0x80016a12
AXIS_MAX = 32767.0
DS_SEND_PORT, DS_RECV_PORT = 1110, 1150
SEND_INTERVAL = 0.05  # 20Hz
UI_REFRESH = 0.016    # ~60fps

class Mode(Enum):       TeleOp = 0x00; TestMode = 0x01; Autonomous = 0x02
class Request(Enum):    Normal = 0x00; RestartCode = 0x04; RebootRoboRIO = 0x08

@dataclass(slots=True)
class StatePacket:
    code_running: bool
    bat_voltage: float

    @classmethod
    def from_bytes(cls, data: bytes) -> 'StatePacket':
        _, _, _, status, bat_int, bat_frac, *_ = struct.unpack('>HBBBBBB', data[:8])
        return cls(bool(status & 0x20), bat_int + bat_frac / 255.0)

def control_packet(num: int, mode: Mode, enabled: bool, request: Request, alliance: int = 0) -> bytes:
    # alliance: 0=Red1, 1=Red2, 2=Red3, 3=Blue1, 4=Blue2, 5=Blue3
    return struct.pack('>HBBBB', num, 0x01, mode.value | (0x04 if enabled else 0), request.value, alliance)

def date_tag() -> bytes:
    # Tag 0x0f: Date - required for roboRIO to accept connection
    now = datetime.now(timezone.utc)
    data = struct.pack('>IBBBBB', now.microsecond, now.second, now.minute, now.hour, now.day, now.month - 1)
    data += struct.pack('B', now.year - 1900)
    return bytes([len(data) + 1, 0x0f]) + data

def timezone_tag() -> bytes:
    # Tag 0x10: Timezone
    tz = time.tzname[0].encode('utf-8')
    return bytes([len(tz) + 1, 0x10]) + tz


AXIS_REMAP = {
    3: 4,
    4: 5,
    5: 3
}

class Joystick:
    def __init__(self, dev: str = '/dev/input/js0'):
        self.dev = dev
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
        if not self.connected: return bytes([4, 0x0c, 0, 0, 0, 0])  # axes=0, buttons=0, povs=0
        with self.mu:
            remapped_axes = [0.0] * len(self.axes)
            for i in range(len(self.axes)): remapped_axes[AXIS_REMAP[i] if i in AXIS_REMAP else i]=self.axes[i]
            axis_bytes = bytes(int(max(-128, min(127, a * 127))) & 0xFF for a in remapped_axes)
            # Build button_flags as integer (LSB = button 0), then send big-endian
            button_flags = sum(1 << i for i, pressed in enumerate(self.buttons) if pressed)
            num_btn_bytes = (self.num_buttons + 7) // 8
            btn_bytes = button_flags.to_bytes(num_btn_bytes, 'big')
            # Format: axes_count, axes..., buttons_count, buttons..., pov_count (0 for now)
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
        self.packet_num, self.running = 0, False
        self.last_state: StatePacket | None = None
        self.request, self.enabled, self.mode = Request.Normal, False, Mode.TeleOp
        self.sent_datetime = False  # Date/timezone only sent once on connect
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
                if len(data) >= 8:  # Minimum valid packet size
                    with self.mu: self.last_state = StatePacket.from_bytes(data)
            except socket.timeout: pass
    def _send_loop(self) -> None:
        while True:
            try:
                packet = control_packet(self.packet_num, self.mode, self.enabled, self.request)
                # Send date/timezone tags until we get a response
                if not self.sent_datetime:
                    packet += date_tag() + timezone_tag()
                    with self.mu:
                        if self.last_state is not None:
                            self.sent_datetime = True
                packet += self.js.get_packet()
                self.send_sock.sendto(packet, (self.robot_ip, DS_SEND_PORT))
                self.request, self.packet_num = Request.Normal, self.packet_num + 1
            except (socket.error, OSError): pass
            time.sleep(SEND_INTERVAL)

class UI:
    def __init__(self, net: Nexus):
        self.net = net
        self.keybinds = {
            ord('e'): ('toggle enable',    net.toggle_enable),
            ord('a'): ('auto',      lambda: net.set_mode(Mode.Autonomous)),
            ord('t'): ('teleop',    lambda: net.set_mode(Mode.TeleOp)),
            ord('r'): ('restart',   lambda: net.send_request(Request.RestartCode)),
        }

    def __call__(self, win: curses.window) -> None:
        curses.curs_set(0); win.nodelay(True)
        while (ch := win.getch()) != ord('q'):
            s = self.net.get_state()
            status = f"[{'CONN' if s else 'DISC'}] [{'CODE' if s and s.code_running else 'NO CODE'}] {s.bat_voltage if s else 0:.1f}V | {self.net.mode.name} | {'EN' if self.net.enabled else 'DIS'}"
            win.erase();
            win.addstr(0, 0, "      --- Nexus Robot Connection ---", curses.A_BOLD)
            win.addstr(1, 0, status)
            win.addstr(2, 0, ' '.join(f"{chr(k)}:{v[0]}" for k, v in self.keybinds.items()) + " q:quit")
            js = self.net.js
            if js.connected:
                axes, buttons = js.get_state()
                win.addstr(4, 0, f"Axes:  {' '.join(f'{a:+.2f}' for a in axes[:6])}")
                win.addstr(5, 0, f"Btns:  {''.join('●' if b else '○' for b in buttons)}")
            else:
                win.addstr(4, 0, "Joystick: NOT CONNECTED")
            win.refresh()
            if ch in self.keybinds: self.keybinds[ch][1]()
            if ch == 10: self.net.enabled = False
            time.sleep(UI_REFRESH)

if __name__ == '__main__':
    if len(sys.argv) == 1:
        print("Usage: nexus <team number>")
        exit(1)
    net = Nexus(int(sys.argv[1]))
    try: curses.wrapper(UI(net))
    finally: net.stop()
