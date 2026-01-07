import time
import serial

class PicoLink:
    def __init__(self, port="/dev/ttyACM0", baud=115200, timeout=0.2):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        self.seq = 0
        time.sleep(1.0)
        self._drain()

    def _drain(self):
        time.sleep(0.05)
        while self.ser.in_waiting:
            self.ser.read(self.ser.in_waiting)

    def _next_seq(self):
        self.seq = (self.seq + 1) % 10000
        return self.seq

    def cmd(self, cmd_str, expect_ack=True, retries=2):
        seq = self._next_seq()
        line = f"@{seq} {cmd_str}\n".encode()
        for _ in range(retries + 1):
            self.ser.write(line)
            self.ser.flush()
            if not expect_ack:
                return None
            t0 = time.time()
            while time.time() - t0 < 0.5:
                resp = self.ser.readline().decode(errors="replace").strip()
                if not resp:
                    continue
                # Expect: !<seq> OK ...
                if resp.startswith("!"):
                    parts = resp[1:].split()
                    if len(parts) >= 2 and int(parts[0]) == seq:
                        status = parts[1]
                        rest = parts[2:]
                        if status == "OK":
                            return ("OK", rest)
                        else:
                            # ERR
                            return ("ERR", rest)
        raise TimeoutError(f"No ACK for seq {seq} cmd={cmd_str}")

    # Convenience API
    def ping(self): return self.cmd("PING")
    def hello(self): return self.cmd("HELLO")
    def estop(self, on: bool): return self.cmd(f"ESTOP {1 if on else 0}")

    def set_servo(self, sid: int, angle: int):
        return self.cmd(f"SV {sid} ANG {angle}")

    def esc_arm(self, eid: int): return self.cmd(f"ESC {eid} ARM")
    def esc_stop(self, eid: int): return self.cmd(f"ESC {eid} STOP")
    def esc_us(self, eid: int, us: int): return self.cmd(f"ESC {eid} US {us}")

    def stepper_enable(self, sid: int, en: bool):
        return self.cmd(f"ST {sid} EN {1 if en else 0}")

    def stepper_vel(self, sid: int, sps: int):
        return self.cmd(f"ST {sid} VEL {sps}")

    def laser(self, on: bool): return self.cmd(f"DO LASER {1 if on else 0}")
    def led(self, on: bool): return self.cmd(f"DO LED {1 if on else 0}")

    def get_all(self): return self.cmd("GET ALL")

    def get_pos(self):
        """
        Returns tuple (st0, st1) of step counts from the Pico, or None on error.
        """
        resp = self.cmd("GET POS")
        if not resp or resp[0] != "OK":
            return None
        vals = {}
        for part in resp[1]:
            if "=" in part:
                k, v = part.split("=", 1)
                vals[k] = int(v)
        if "ST0" in vals and "ST1" in vals:
            return vals["ST0"], vals["ST1"]
        return None

    def get_limits(self):
        """
        Returns tuple (ls0, ls1) where each is 0/1, or None on error.
        """
        resp = self.cmd("GET LIMITS")
        if not resp or resp[0] != "OK":
            return None
        vals = {}
        for part in resp[1]:
            if "=" in part:
                k, v = part.split("=", 1)
                vals[k] = int(v)
        if "LS0" in vals and "LS1" in vals:
            return vals["LS0"], vals["LS1"]
        return None
