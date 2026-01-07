# main.py (Raspberry Pi Pico 2 / MicroPython)
from machine import Pin, PWM, Timer
import sys, time

# ---------------------------
# Pin configuration (EDIT ME)
# ---------------------------

# Servos (PWM)
SERVO_PINS = [27, 26]      # SV0=dart pusher on GPIO27, SV1=homing popper on GPIO26

# ESCs (PWM) - choose two PWM-capable pins
ESC_PINS   = [16, 17]      # ESC0, ESC1  (EDIT)

# Steppers (TMC2209 in STEP/DIR mode)
ST_STEP_PINS = [2, 4]      # ST0 pan STEP, ST1 tilt STEP (EDIT)
ST_DIR_PINS  = [3, 5]      # ST0 pan DIR,  ST1 tilt DIR  (EDIT)
ST_EN_PINS   = [6, 7]      # ST0 EN,       ST1 EN        (EDIT)

# Limit switches (digital inputs)
LS_PINS = [8, 9]           # LS0 pan, LS1 tilt (EDIT)

# Diagnostics + laser
LED_PIN   = 25             # Pico onboard LED (good default)
LASER_PIN = 15             # (EDIT)

# ---------------------------
# Behavior configuration
# ---------------------------

FIRMWARE_VERSION = "NT2-PICO-0.1"

# Limit switch wiring:
# Recommended: NC to GND, COM to pin with PULL_UP.
# In that wiring, pressed -> circuit opens -> pin reads 1 (HIGH).
# If you wired NO-to-GND (pressed pulls pin LOW), set this to False.
LIMIT_ACTIVE_HIGH = True    # set True for NC-to-GND, False for NO-to-GND (pressed=0)

# Servo pulse range (typical)
SERVO_MIN_US = 500
SERVO_MAX_US = 2500

# ESC range (typical)
ESC_STOP_US  = 1000
ESC_MIN_US   = 900
ESC_MAX_US   = 2100
ESC_ARM_US   = 1000         # idle pulse during arming

# Failsafe
COMMS_TIMEOUT_MS = 250       # no valid command -> safe state

# Stepper velocity limits (steps/s)
ST_MAX_ABS_VEL = 5000

# Step pulse timing (us). 10–20 us works for most drivers.
STEP_PULSE_US = 10

# ---------------------------
# Helpers
# ---------------------------

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def angle_to_duty_u16(angle):
    a = clamp(int(angle), 0, 180)
    pulse_us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * (a / 180.0)
    return int((pulse_us / 20000.0) * 65535)

def us_to_duty_u16(pulse_us):
    pulse_us = clamp(int(pulse_us), 0, 20000)
    return int((pulse_us / 20000.0) * 65535)

# ---------------------------
# Hardware init
# ---------------------------

# Servos
servos = []
servo_angles = [90, 90]
for p in SERVO_PINS:
    pwm = PWM(Pin(p))
    pwm.freq(50)
    servos.append(pwm)

# ESCs
escs = []
esc_us = [ESC_STOP_US, ESC_STOP_US]
esc_armed = [False, False]
for p in ESC_PINS:
    pwm = PWM(Pin(p))
    pwm.freq(50)
    escs.append(pwm)

# Steppers
st_step = [Pin(p, Pin.OUT) for p in ST_STEP_PINS]
st_dir  = [Pin(p, Pin.OUT) for p in ST_DIR_PINS]
st_en   = [Pin(p, Pin.OUT) for p in ST_EN_PINS]

# NOTE: Enable polarity depends on your TMC2209 module wiring.
# Common: EN pin is active-low (0 enables). We'll default to active-low.
ST_EN_ACTIVE_LOW = True

st_enabled = [False, False]
st_vel_sps = [0, 0]           # signed steps per second
st_pos = [0, 0]               # simple position estimate from step pulses

# Limit switches
ls = []
for p in LS_PINS:
    ls.append(Pin(p, Pin.IN, Pin.PULL_UP))

# Define which velocity sign drives *toward* each limit switch.
# v>0 means DIR=1 (see stepper_tick). If your wiring is opposite, flip signs here.
LIMIT_TOWARD_HOME = [ -1, -1 ]   # ST0 pan: -vel hits limit; ST1 tilt: -vel hits limit

# Digital outputs
led = Pin(LED_PIN, Pin.OUT)
laser = Pin(LASER_PIN, Pin.OUT)
laser.value(0)

# ESTOP flag
estop = False

# Comms watchdog
last_cmd_ms = time.ticks_ms()

# ---------------------------
# Low-level control funcs
# ---------------------------

def st_set_enable(i, en):
    global st_enabled
    st_enabled[i] = bool(en)
    if ST_EN_ACTIVE_LOW:
        st_en[i].value(0 if st_enabled[i] else 1)
    else:
        st_en[i].value(1 if st_enabled[i] else 0)

def sv_set_angle(i, ang):
    ang = clamp(int(ang), 0, 180)
    servo_angles[i] = ang
    servos[i].duty_u16(angle_to_duty_u16(ang))

def esc_set_us(i, pulse):
    pulse = clamp(int(pulse), ESC_MIN_US, ESC_MAX_US)
    esc_us[i] = pulse
    escs[i].duty_u16(us_to_duty_u16(pulse))

def esc_stop(i):
    esc_us[i] = ESC_STOP_US
    escs[i].duty_u16(us_to_duty_u16(ESC_STOP_US))

def limit_is_active(i):
    v = ls[i].value()
    return (v == 1) if LIMIT_ACTIVE_HIGH else (v == 0)

def safe_state(reason=""):
    # Stop flywheels, stop steppers, laser off; LED blink pattern handled elsewhere
    for i in range(2):
        esc_stop(i)
    for i in range(2):
        st_vel_sps[i] = 0
    laser.value(0)

# ---------------------------
# Stepper pulse generator (velocity mode)
# ---------------------------

# We generate step pulses in a timer ISR-like callback.
# For each stepper, we accumulate phase and emit pulses at the requested frequency.
accum = [0.0, 0.0]   # fractional steps accumulator

def stepper_tick(_t):
    # fail-safe check is done in main loop; here we only generate steps.
    for i in range(2):
        v = st_vel_sps[i]
        if (not st_enabled[i]) or v == 0 or estop:
            continue

        # Limit handling moved to host (Turret_Control). Always allow motion here.

        # Timer frequency is fixed; we accumulate based on v
        # tick_hz set below.
        step_per_tick = v / TICK_HZ
        accum[i] += step_per_tick

        # emit steps while accumulator crosses ±1
        while accum[i] >= 1.0:
            st_dir[i].value(1)      # DIR=1
            st_step[i].value(1); time.sleep_us(STEP_PULSE_US); st_step[i].value(0)
            st_pos[i] += 1
            accum[i] -= 1.0
        while accum[i] <= -1.0:
            st_dir[i].value(0)      # DIR=0
            st_step[i].value(1); time.sleep_us(STEP_PULSE_US); st_step[i].value(0)
            st_pos[i] -= 1
            accum[i] += 1.0

# Choose a tick rate (higher = smoother velocity). 2 kHz is a decent start.
TICK_HZ = 2000
timer = Timer()
timer.init(freq=TICK_HZ, mode=Timer.PERIODIC, callback=stepper_tick)

# ---------------------------
# Serial protocol parsing
# ---------------------------

def reply_ok(seq, *parts):
    sys.stdout.write("!{} OK {}\n".format(seq, " ".join(str(p) for p in parts)).rstrip() + "\n")

def reply_err(seq, code, *parts):
    sys.stdout.write("!{} ERR {} {}\n".format(seq, code, " ".join(str(p) for p in parts)).rstrip() + "\n")

def parse_line(line):
    # Expect: @<seq> <CMD> ...
    line = line.strip()
    if not line or not line.startswith("@"):
        return None
    try:
        after = line[1:]
        sp = after.split()
        seq = int(sp[0])
        cmd = sp[1].upper()
        args = sp[2:]
        return seq, cmd, args
    except Exception:
        return None

# ---------------------------
# Boot defaults
# ---------------------------

# Start safe
safe_state("boot")
for i in range(2):
    sv_set_angle(i, servo_angles[i])
for i in range(2):
    st_set_enable(i, 0)

led.value(1)
time.sleep(0.2)
led.value(0)

sys.stdout.write("READY {}\n".format(FIRMWARE_VERSION))

# ---------------------------
# Main loop
# ---------------------------

while True:
    # Watchdog: comms timeout -> safe state & disarm ESC
    now = time.ticks_ms()
    if time.ticks_diff(now, last_cmd_ms) > COMMS_TIMEOUT_MS:
        # disarm + safe
        for i in range(2):
            esc_armed[i] = False
        safe_state("timeout")
        # blink LED fast to indicate comm loss
        led.toggle()
        time.sleep_ms(100)

    line = sys.stdin.readline()
    if not line:
        continue

    msg = parse_line(line)
    if msg is None:
        continue

    last_cmd_ms = time.ticks_ms()
    seq, cmd, args = msg

    try:
        if cmd == "PING":
            reply_ok(seq, "ALIVE", FIRMWARE_VERSION)

        elif cmd == "HELLO":
            reply_ok(seq, "FW", FIRMWARE_VERSION, "SV=2", "ESC=2", "ST=2", "LS=2")

        elif cmd == "ESTOP":
            v = int(args[0])
            estop = (v == 1)
            if estop:
                for i in range(2):
                    esc_armed[i] = False
                safe_state("estop")
                reply_ok(seq, "ESTOP", 1)
            else:
                reply_ok(seq, "ESTOP", 0)
            if estop:
                for i in range(2):
                    esc_armed[i] = False
                safe_state("estop")
                reply_ok(seq, "ESTOP", 1)
            else:
                reply_ok(seq, "ESTOP", 0)

        elif cmd == "SV":
            sid = int(args[0])
            mode = args[1].upper()
            if sid not in (0,1):
                reply_err(seq, "BAD_ID")
            elif mode == "ANG":
                ang = int(args[2])
                sv_set_angle(sid, ang)
                reply_ok(seq, "SV", sid, servo_angles[sid])
            else:
                reply_err(seq, "BAD_ARG")

        elif cmd == "ESC":
            eid = int(args[0])
            sub = args[1].upper()
            if eid not in (0,1):
                reply_err(seq, "BAD_ID")
            elif sub == "ARM":
                # Keep at idle pulse for arming; mark armed
                escs[eid].duty_u16(us_to_duty_u16(ESC_ARM_US))
                esc_us[eid] = ESC_ARM_US
                esc_armed[eid] = True
                reply_ok(seq, "ESC", eid, "ARMED")
            elif sub == "STOP":
                esc_armed[eid] = False
                esc_stop(eid)
                reply_ok(seq, "ESC", eid, "STOP")
            elif sub == "US":
                pulse = int(args[2])
                if estop:
                    reply_err(seq, "ESTOP")
                elif not esc_armed[eid]:
                    reply_err(seq, "NOT_ARMED")
                else:
                    esc_set_us(eid, pulse)
                    reply_ok(seq, "ESC", eid, esc_us[eid])
            else:
                reply_err(seq, "BAD_ARG")

        elif cmd == "ST":
            sid = int(args[0])
            sub = args[1].upper()
            if sid not in (0,1):
                reply_err(seq, "BAD_ID")
            elif sub == "EN":
                en = int(args[2])
                st_set_enable(sid, en)
                reply_ok(seq, "ST", sid, "EN", 1 if st_enabled[sid] else 0)
            elif sub == "VEL":
                v = int(args[2])
                v = clamp(v, -ST_MAX_ABS_VEL, ST_MAX_ABS_VEL)
                if estop:
                    st_vel_sps[sid] = 0
                    reply_err(seq, "ESTOP")
                else:
                    st_vel_sps[sid] = v
                    reply_ok(seq, "ST", sid, "VEL", st_vel_sps[sid])
            elif sub == "STOP":
                st_vel_sps[sid] = 0
                reply_ok(seq, "ST", sid, "STOP")
            else:
                reply_err(seq, "BAD_ARG")

        elif cmd == "DO":
            name = args[0].upper()
            val = int(args[1])
            if name == "LED":
                led.value(1 if val else 0)
                reply_ok(seq, "DO", "LED", val)
            elif name == "LASER":
                if estop:
                    laser.value(0)
                    reply_err(seq, "ESTOP")
                else:
                    laser.value(1 if val else 0)
                    reply_ok(seq, "DO", "LASER", val)
            else:
                reply_err(seq, "BAD_ARG")

        elif cmd == "GET":
            what = args[0].upper()
            if what == "LIMITS":
                reply_ok(seq, f"LS0={1 if limit_is_active(0) else 0}", f"LS1={1 if limit_is_active(1) else 0}")
            elif what == "POS":
                reply_ok(seq, f"ST0={st_pos[0]}", f"ST1={st_pos[1]}")
            elif what == "ALL":
                reply_ok(
                    seq,
                    f"LS0={1 if limit_is_active(0) else 0}",
                    f"LS1={1 if limit_is_active(1) else 0}",
                    f"ST0={st_pos[0]}",
                    f"ST1={st_pos[1]}",
                    f"ESC0={esc_us[0]}",
                    f"ESC1={esc_us[1]}",
                    f"SV0={servo_angles[0]}",
                    f"SV1={servo_angles[1]}",
                    f"ESTOP={1 if estop else 0}",
                )
            else:
                reply_err(seq, "BAD_ARG")

        else:
            reply_err(seq, "UNKNOWN_CMD")

    except Exception as e:
        reply_err(seq, "EXC", str(e))
