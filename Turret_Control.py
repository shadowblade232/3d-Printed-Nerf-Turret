# Turret_Control.py
import time
import pygame
import sys
from pico_link import PicoLink

# =======================
# CONFIG
# =======================

PORT = "/dev/ttyACM0"

# -----------------------
# Controller mapping
# -----------------------
RT_AXIS = 5                       # fixed RT axis
LEFT_STICK_AXES = (0, 1)          # left stick X,Y

# -----------------------
# Mechanical conversion (NO microstepping)
# -----------------------
# If your drivers are in microstep mode, set *_MICROSTEP to that factor (e.g., 8, 16).
# The default assumes full-step (1).
PAN_MICROSTEP = 8.0
TILT_MICROSTEP = 8.0
MOTOR_DEG_PER_FULL_STEP = 1.8     # full step mechanical motor angle

# Motor on SMALL gear
# PAN: motor 15T -> output 80T
PAN_MOTOR_T = 15
PAN_OUT_T = 80

# TILT: motor 10T -> output 55T
TILT_MOTOR_T = 10
TILT_OUT_T = 55

# Output deg per motor step:
# output_rev_per_motor_rev = motorT/outT
# output_deg_per_step = (motor_deg_per_full_step / microstep) * (motorT/outT)
PAN_DEG_PER_STEP = (MOTOR_DEG_PER_FULL_STEP / PAN_MICROSTEP) * (PAN_MOTOR_T / PAN_OUT_T)
TILT_DEG_PER_STEP = (MOTOR_DEG_PER_FULL_STEP / TILT_MICROSTEP) * (TILT_MOTOR_T / TILT_OUT_T)

# Convenience:
PAN_STEPS_PER_DEG = 1.0 / PAN_DEG_PER_STEP
TILT_STEPS_PER_DEG = 1.0 / TILT_DEG_PER_STEP

# -----------------------
# Angle limits
# -----------------------
PAN_MIN_DEG = 0.0
PAN_MAX_DEG = 360.0

TILT_MIN_DEG = -45.0
TILT_MAX_DEG = 45.0

# -----------------------
# Steppers (commanded in steps/s)
# -----------------------
PAN_ID = 0
TILT_ID = 1
ST_DEADZONE = 0.12
SPEED_MODIFIER = 1.25
ST_MAX_SPS = int(2500 * SPEED_MODIFIER)
ST_SEND_MIN_INTERVAL = 0.05

# Stepper acceleration limits (trapezoidal velocity smoothing)
PAN_ACCEL_SPS2  = 8000
PAN_DECEL_SPS2  = 10000
TILT_ACCEL_SPS2 = 6000
TILT_DECEL_SPS2 = 8000

# "Go home" tuning (degrees-based controller that outputs steps/s)
HOME_KP_DEG = 8.0                 # (steps/s) per degree of error (tune)
HOME_MAX_SPS = int(ST_MAX_SPS * 0.85)
HOME_MIN_SPS = 250
HOME_TOL_DEG = 1.0                # consider at home within this many degrees
HOME_SETTLE_S = 0.10

# -----------------------
# Flywheels
# -----------------------
DESIRED_VELOCITY_DEFAULT = 25.0
DESIRED_VELOCITY_MIN = 10.0
DESIRED_VELOCITY_MAX = 36.0
DESIRED_VELOCITY_STEP = 1.0
V_TO_T = 0.027066

ESC0_ID = 0
ESC1_ID = 1
ESC_IDLE_US = 1000
MAX_US_PER_SEC = 5000

# -----------------------
# Servos
# -----------------------
PUSHER_SERVO_ID = 0      # SV0
POPPER_SERVO_ID = 1      # SV1

PUSHER_MIN_ANGLE = 0
PUSHER_MAX_ANGLE = 140
PUSH_HOLD_SECONDS = 0.15
RETRACT_SETTLE_SECONDS = 0.05

POPPER_DEFAULT_ANGLE = 90
POPPER_HOME_ANGLE = 180

# -----------------------
# Loop timing
# -----------------------
TRIGGER_DEADBAND = 0.08
UPDATE_HZ = 60
DT = 1.0 / UPDATE_HZ

STATUS_HZ = 10
STATUS_DT = 1.0 / STATUS_HZ
POS_POLL_HZ = 10
POS_POLL_DT = 1.0 / POS_POLL_HZ
LIMIT_POLL_HZ = 30
LIMIT_POLL_DT = 1.0 / LIMIT_POLL_HZ

# Auto-home parameters
AUTO_PAN_LIMIT_DEG = 135.0           # physical angle of pan limit from forward (deg)
AUTO_PAN_SEARCH_SPS = 1000           # search speed magnitude for pan homing (steps/s, direction set by PAN_LIMIT_TOWARD_SIGN)
AUTO_PAN_POP_ANGLE = 180             # SV1 angle to raise popper to hit pan limit
AUTO_PAN_CLEAR_DEG = 3.0             # back off this many output degrees after hitting pan limit

AUTO_TILT_PAN_TARGET_DEG = 0.0       # pan angle to use while tilt homing
AUTO_TILT_LIMIT_DEG = -37.0          # physical angle of tilt limit from level (deg, negative = down)
AUTO_TILT_SEARCH_SPS = -800          # search speed for tilt homing (steps/s, negative to go down)
AUTO_SETTLE_S = 0.20

# Limit switch direction (host-side guard). Sign of velocity that drives *toward* the switch.
PAN_LIMIT_TOWARD_SIGN = -1
TILT_LIMIT_TOWARD_SIGN = -1
# Limit switch polarity (host-side). True if Pico reports 1 when pressed, False if Pico reports 0 when pressed.
PAN_LIMIT_ACTIVE_HIGH = False
TILT_LIMIT_ACTIVE_HIGH = False

# Buttons (Xbox typical in pygame)
A_BTN = 0
B_BTN = 1
X_BTN = 2              # hold for slow mode
Y_BTN = 3              # auto-home
BACK_BTN = 6
START_BTN = 7
LB_BTN = 4              # set home (zero angles)
RB_BTN = 5              # go home (homing mode)

QUIT_HOLD_SECONDS = 0.6


# =======================
# Helpers
# =======================

def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x

def safe_get_button(js, idx):
    return js.get_button(idx) if js.get_numbuttons() > idx else 0

def apply_deadzone(x: float, dz: float) -> float:
    return 0.0 if abs(x) < dz else x

def axis_to_01(v):
    if v < -0.05:
        return (v + 1.0) * 0.5
    return clamp(v, 0.0, 1.0)

def read_trigger_axis(js, ax):
    if ax < js.get_numaxes():
        return axis_to_01(js.get_axis(ax))
    return 0.0

def slew_limit(current_us, target_us, max_us_per_sec, dt):
    max_step = max_us_per_sec * dt
    if target_us > current_us + max_step:
        return int(current_us + max_step)
    if target_us < current_us - max_step:
        return int(current_us - max_step)
    return int(target_us)

def stop_all_flywheels(pico):
    for eid in (ESC0_ID, ESC1_ID):
        try:
            pico.esc_stop(eid)
        except Exception:
            pass
        try:
            pico.esc_us(eid, ESC_IDLE_US)
        except Exception:
            pass

def normalize_limits(raw_limits):
    """
    Apply polarity settings so the rest of the code can treat 1 as 'pressed'.
    """
    if not raw_limits:
        return (0, 0)
    ls0, ls1 = raw_limits
    if not PAN_LIMIT_ACTIVE_HIGH:
        ls0 = 1 - ls0
    if not TILT_LIMIT_ACTIVE_HIGH:
        ls1 = 1 - ls1
    return (ls0, ls1)

def maybe_send_stepper_vel(link, sid, vel, state):
    now = time.time()
    last_v = state["last_sent_vel"].get(sid, None)
    last_t = state["last_send_t"].get(sid, 0.0)

    should_send = (last_v is None) or (vel != last_v) or ((now - last_t) >= ST_SEND_MIN_INTERVAL)
    if should_send:
        link.stepper_vel(sid, int(vel))
        state["last_sent_vel"][sid] = int(vel)
        state["last_send_t"][sid] = now

def ramp_trap_vel(current_sps: float, target_sps: float,
                  accel_sps2: float, decel_sps2: float, dt: float) -> float:
    dv = target_sps - current_sps
    use_decel = (abs(target_sps) < abs(current_sps)) or (current_sps * target_sps < 0)
    a = decel_sps2 if use_decel else accel_sps2
    max_dv = a * dt

    if dv > max_dv:
        return current_sps + max_dv
    if dv < -max_dv:
        return current_sps - max_dv
    return target_sps

def velocity_to_flywheel_targets(desired_v_mps: float):
    v = clamp(float(desired_v_mps), DESIRED_VELOCITY_MIN, DESIRED_VELOCITY_MAX)
    T = V_TO_T * v
    T = max(0.2, min(T, 1.0))
    esc_max = int(1000 + 1000 * T)

    spinup_s = abs(esc_max - ESC_IDLE_US) / float(MAX_US_PER_SEC)
    spindown_s = spinup_s + 0.1
    return v, T, esc_max, spinup_s, spindown_s

def status_print(line: str, state: dict):
    prev_len = state.get("prev_len", 0)
    if len(line) < prev_len:
        line = line + (" " * (prev_len - len(line)))
    sys.stdout.write("\r" + line)
    sys.stdout.flush()
    state["prev_len"] = len(line)

def wrap_0_360(deg: float) -> float:
    # normalize to [0,360)
    deg = deg % 360.0
    if deg < 0:
        deg += 360.0
    return deg

def clamp_tilt_deg(deg: float) -> float:
    return clamp(deg, TILT_MIN_DEG, TILT_MAX_DEG)

def apply_tilt_deg_limits(tilt_vel_sps: float, tilt_deg: float, decel_sps2: float) -> float:
    """
    Block and/or brake commands that would drive beyond the tilt angle limits.
    Uses remaining distance to the limit and decel to cap velocity so we can stop in time.
    """
    if tilt_vel_sps > 0:
        dist_deg = TILT_MAX_DEG - tilt_deg
    elif tilt_vel_sps < 0:
        dist_deg = tilt_deg - TILT_MIN_DEG
    else:
        return 0.0

    if dist_deg <= 0:
        return 0.0

    dist_steps = dist_deg * TILT_STEPS_PER_DEG
    # max velocity that can stop within remaining distance: v = sqrt(2*a*d)
    max_allow = (2.0 * decel_sps2 * dist_steps) ** 0.5
    max_allow *= 0.9  # small safety margin

    if tilt_vel_sps > 0:
        return min(tilt_vel_sps, max_allow)
    return max(tilt_vel_sps, -max_allow)

def guard_limit_switch(vel_sps: float, ls_active: int, toward_sign: int) -> float:
    """
    If limit switch is active and commanded velocity would drive further into it, block.
    """
    if not ls_active:
        return vel_sps
    return 0.0 if (vel_sps * toward_sign) > 0 else vel_sps

def steps_to_pan_deg(steps: float) -> float:
    return steps * PAN_DEG_PER_STEP

def steps_to_tilt_deg(steps: float) -> float:
    return steps * TILT_DEG_PER_STEP

def deg_to_pan_steps(deg: float) -> float:
    return deg * PAN_STEPS_PER_DEG

def deg_to_tilt_steps(deg: float) -> float:
    return deg * TILT_STEPS_PER_DEG

def pan_error_deg(current_deg: float, target_deg: float) -> float:
    """
    Signed shortest-path error between two headings in degrees, normalized to [-180, 180).
    """
    return (target_deg - current_deg + 180.0) % 360.0 - 180.0

def home_controller_deg(current_deg: float, target_deg: float, wrap: bool = False):
    """
    P controller in degrees, output is steps/s.
    """
    if wrap:
        err_deg = pan_error_deg(current_deg, target_deg)
    else:
        err_deg = target_deg - current_deg
    if abs(err_deg) <= HOME_TOL_DEG:
        return 0, err_deg

    v_sps = HOME_KP_DEG * err_deg
    v_sps = clamp(v_sps, -HOME_MAX_SPS, HOME_MAX_SPS)

    if 0 < abs(v_sps) < HOME_MIN_SPS:
        v_sps = HOME_MIN_SPS if v_sps > 0 else -HOME_MIN_SPS

    return int(v_sps), err_deg


# =======================
# Main
# =======================

def main():
    pico = PicoLink(port=PORT)
    try:
        pico._drain()
    except Exception:
        pass
    print("Pico HELLO:", pico.hello())

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise RuntimeError("No controller detected")

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Controller: {js.get_name()}")
    print(f"Buttons: {js.get_numbuttons()}  Axes: {js.get_numaxes()}  Hats: {js.get_numhats()}")

    ax_x, ax_y = LEFT_STICK_AXES
    if js.get_numaxes() <= max(ax_x, ax_y):
        raise RuntimeError(f"Controller does not have required left-stick axes {LEFT_STICK_AXES}")
    print(f"Using LEFT stick axes: X={ax_x}, Y={ax_y}")
    print(f"Using RT axis: {RT_AXIS}")

    # Flywheel setpoint state
    desired_v = float(DESIRED_VELOCITY_DEFAULT)
    desired_v, T, ESC_MAX_US, SPINUP_SECONDS, SPINDOWN_SECONDS = velocity_to_flywheel_targets(desired_v)

    print("\nControls:")
    print("Left stick: pan (X) and tilt (Y)")
    print("Right trigger: ONE-BUTTON FIRE (spin up -> pusher -> spin down)")
    print("A: enable steppers (pan+tilt)")
    print("B: stop everything (flywheels off/disarm, steppers disabled, pusher retract)")
    print("X (hold): SLOW MODE (half pan/tilt speed)")
    print("Y: AUTO-HOME (sequence: go to current home -> pan home via LS0 -> tilt home via LS1)")
    print("LB: SET HOME (zero angles at current position)")
    print("RB: GO HOME (drive angles back to 0,0) + popper up during homing")
    print("D-pad UP/DOWN: adjust desired dart velocity")
    print(f"Pan limits: {PAN_MIN_DEG:.0f}..{PAN_MAX_DEG:.0f} deg (wrap)")
    print(f"Tilt limits: {TILT_MIN_DEG:.0f}..{TILT_MAX_DEG:.0f} deg")
    print(f"PAN_DEG_PER_STEP={PAN_DEG_PER_STEP:.6f} (microstep {PAN_MICROSTEP:g})  "
          f"TILT_DEG_PER_STEP={TILT_DEG_PER_STEP:.6f} (microstep {TILT_MICROSTEP:g})")
    print(f"Hold START+BACK for {QUIT_HOLD_SECONDS:.1f}s to quit\n")

    # ---- Start safe ----
    pico.set_servo(PUSHER_SERVO_ID, PUSHER_MIN_ANGLE)
    pico.set_servo(POPPER_SERVO_ID, POPPER_DEFAULT_ANGLE)
    stop_all_flywheels(pico)

    pico.stepper_enable(PAN_ID, False)
    pico.stepper_enable(TILT_ID, False)
    pico.stepper_vel(PAN_ID, 0)
    pico.stepper_vel(TILT_ID, 0)

    steppers_enabled = False
    estop = False

    esc0_us = ESC_IDLE_US
    esc1_us = ESC_IDLE_US
    flywheels_armed = False

    last_a = last_b = last_y = 0
    last_lb = last_rb = 0
    quit_t0 = None

    last_hat_y = 0

    fire_state = "IDLE"
    fire_t0 = 0.0
    last_rt = 0.0
    last_limits = (0, 0)

    # --- position estimate in *steps* (internal), and home offset in degrees ---
    pos_steps = {PAN_ID: 0.0, TILT_ID: 0.0}  # estimated motor steps
    home_offset_deg = {PAN_ID: 0.0, TILT_ID: 0.0}  # added so current reads as 0,0 after LB
    home_valid = False

    return_state = "IDLE"  # IDLE, RETURN_TILT, RETURN_PAN, SETTLING
    settle_t0 = 0.0

    send_state = {
        "last_sent_vel": {PAN_ID: None, TILT_ID: None},
        "last_send_t": {PAN_ID: 0.0, TILT_ID: 0.0},
    }

    cmd_vel = {PAN_ID: 0.0, TILT_ID: 0.0}

    status_state = {"prev_len": 0}
    last_status_t = 0.0
    last_pos_poll_t = 0.0
    last_lim_poll_t = 0.0
    last_limits_raw = (None, None)

    popper_is_up = False
    last_popper_cmd = None

    def set_home_current():
        nonlocal home_valid, return_state
        # zero angles at current position (uses current pos_steps)
        pan_deg_raw = steps_to_pan_deg(pos_steps[PAN_ID])
        tilt_deg_raw = steps_to_tilt_deg(pos_steps[TILT_ID])
        home_offset_deg[PAN_ID] = wrap_0_360(-pan_deg_raw)
        home_offset_deg[TILT_ID] = clamp_tilt_deg(-tilt_deg_raw)
        home_valid = True
        return_state = "IDLE"
        set_popper(False)

    def set_popper(up: bool, angle_override: int = None, force: bool = False):
        nonlocal popper_is_up, last_popper_cmd
        angle = angle_override if angle_override is not None else (POPPER_HOME_ANGLE if up else POPPER_DEFAULT_ANGLE)

        if force or (last_popper_cmd != angle):
            pico.set_servo(POPPER_SERVO_ID, angle)   # don't hide errors right now
            last_popper_cmd = angle
        popper_is_up = up


    # On load, sync counters from Pico (if available) and declare that position as home (0,0)
    try:
        pos = pico.get_pos()
        if pos:
            pos_steps[PAN_ID], pos_steps[TILT_ID] = map(float, pos)
    except Exception:
        pass
    set_home_current()

    auto = {
        "active": False,
        "phase": "IDLE",
        "t0": 0.0,
        "retract_pending": False,
    }

    def current_angles_deg():
        # Convert estimated steps -> output degrees, then apply home offset
        pan_deg_raw = steps_to_pan_deg(pos_steps[PAN_ID])
        tilt_deg_raw = steps_to_tilt_deg(pos_steps[TILT_ID])

        pan_deg = wrap_0_360(pan_deg_raw + home_offset_deg[PAN_ID])
        tilt_deg = clamp_tilt_deg(tilt_deg_raw + home_offset_deg[TILT_ID])
        return pan_deg, tilt_deg

    try:
        while True:
            loop_t0 = time.time()
            pygame.event.pump()

            # integrate step estimate
            pan_last = send_state["last_sent_vel"][PAN_ID] or 0
            tilt_last = send_state["last_sent_vel"][TILT_ID] or 0
            pos_steps[PAN_ID] += pan_last * DT
            pos_steps[TILT_ID] += tilt_last * DT

            # Periodically refresh from Pico's actual step counters to avoid drift
            t_now = time.time()
            if (t_now - last_pos_poll_t) >= POS_POLL_DT:
                last_pos_poll_t = t_now
                try:
                    pos = pico.get_pos()
                    if pos:
                        pos_steps[PAN_ID], pos_steps[TILT_ID] = map(float, pos)
                except Exception:
                    pass
            if (t_now - last_lim_poll_t) >= LIMIT_POLL_DT:
                last_lim_poll_t = t_now
                try:
                    lm = pico.get_limits()
                    if lm:
                        last_limits_raw = lm
                        last_limits = normalize_limits(lm)
                except Exception:
                    pass

            pan_deg, tilt_deg = current_angles_deg()

            # ---- Quit combo ----
            back = safe_get_button(js, BACK_BTN)
            start = safe_get_button(js, START_BTN)
            if back and start:
                if quit_t0 is None:
                    quit_t0 = time.time()
                elif (time.time() - quit_t0) >= QUIT_HOLD_SECONDS:
                    print("\nQuit combo detected.")
                    break
            else:
                quit_t0 = None

            # ---- Inputs ----
            rt = read_trigger_axis(js, RT_AXIS)
            a = safe_get_button(js, A_BTN)
            b = safe_get_button(js, B_BTN)
            x_held = bool(safe_get_button(js, X_BTN))
            y = safe_get_button(js, Y_BTN)
            lb = safe_get_button(js, LB_BTN)
            rb = safe_get_button(js, RB_BTN)

            # ---- D-pad UP/DOWN adjusts desired velocity ----
            hat_y = 0
            if js.get_numhats() > 0:
                _, hat_y = js.get_hat(0)

            if hat_y != 0 and last_hat_y == 0:
                desired_v += (DESIRED_VELOCITY_STEP * hat_y)
                desired_v, T, ESC_MAX_US, SPINUP_SECONDS, SPINDOWN_SECONDS = velocity_to_flywheel_targets(desired_v)
            last_hat_y = hat_y

            # ---- B = hard stop ----
            if b and not last_b:
                return_state = "IDLE"
                set_popper(False)
                auto["active"] = False
                auto["phase"] = "IDLE"
                auto["retract_pending"] = False

                fire_state = "IDLE"
                flywheels_armed = False
                stop_all_flywheels(pico)
                esc0_us = ESC_IDLE_US
                esc1_us = ESC_IDLE_US

                pico.set_servo(PUSHER_SERVO_ID, PUSHER_MIN_ANGLE)

                cmd_vel[PAN_ID] = 0.0
                cmd_vel[TILT_ID] = 0.0

                pico.stepper_vel(PAN_ID, 0)
                pico.stepper_vel(TILT_ID, 0)
                pico.stepper_enable(PAN_ID, False)
                pico.stepper_enable(TILT_ID, False)
                steppers_enabled = False

                send_state["last_sent_vel"][PAN_ID] = 0
                send_state["last_sent_vel"][TILT_ID] = 0

            last_b = b
            last_y = y
            # ---- Y = auto-home sequence ----
            if y and not auto["active"]:
                if not steppers_enabled:
                    pico.stepper_enable(PAN_ID, True)
                    pico.stepper_enable(TILT_ID, True)
                    steppers_enabled = True
                auto["active"] = True
                auto["phase"] = "GOTO_HOME"
                auto["t0"] = time.time()
                auto["retract_pending"] = False
                return_state = "IDLE"
                set_popper(True, angle_override=AUTO_PAN_POP_ANGLE)  # raise popper pre-emptively
            if not y and last_y and auto["active"]:
                # Y released after triggering; no action
                pass

            # ---- A = enable steppers ----
            if a and not last_a:
                pico.stepper_enable(PAN_ID, True)
                pico.stepper_enable(TILT_ID, True)
                steppers_enabled = True
            last_a = a

            # ---- LB = set home (zero angles) ----
            if lb and not last_lb:
                # We want current pan_deg/tilt_deg to become 0/0.
                # pan_deg = wrap_0_360(steps*deg_per_step + offset) => choose offset so pan becomes 0
                set_home_current()
            last_lb = lb

            # ---- RB = go home ----
            if rb and not last_rb:
                if home_valid:
                    if not steppers_enabled:
                        pico.stepper_enable(PAN_ID, True)
                        pico.stepper_enable(TILT_ID, True)
                        steppers_enabled = True
                    return_state = "RETURN_TILT"
                    settle_t0 = 0.0
            last_rb = rb

            # popper behavior
            # - Auto-home manages popper itself (stays up until pan limit hit).
            # - Manual homing (RB) keeps popper down now per request.
            if not auto["active"]:
                set_popper(False)
            in_homing = (return_state in ("RETURN_TILT", "RETURN_PAN", "SETTLING")) and home_valid and steppers_enabled

            # ==========================
            # Pan/Tilt commands: auto-home > homing > manual
            # ==========================
            speed_scale = 0.5 if x_held else 1.0

            handled_motion = False

            if auto["active"]:
                phase = auto["phase"]
                if phase == "GOTO_HOME":
                    # drive back to current home first
                    pan_cmd_raw, pan_err_deg = home_controller_deg(pan_deg, 0.0, wrap=True)
                    tilt_cmd_raw, tilt_err_deg = home_controller_deg(tilt_deg, 0.0)
                    tilt_cmd = int(tilt_cmd_raw * speed_scale)
                    tilt_cmd = int(apply_tilt_deg_limits(tilt_cmd, tilt_deg, TILT_DECEL_SPS2))
                    pan_cmd = int(pan_cmd_raw * speed_scale)

                    cmd_vel[PAN_ID]  = ramp_trap_vel(cmd_vel[PAN_ID],  pan_cmd,  PAN_ACCEL_SPS2,  PAN_DECEL_SPS2,  DT)
                    cmd_vel[TILT_ID] = ramp_trap_vel(cmd_vel[TILT_ID], tilt_cmd, TILT_ACCEL_SPS2, TILT_DECEL_SPS2, DT)

                    maybe_send_stepper_vel(pico, PAN_ID, int(cmd_vel[PAN_ID]), send_state)
                    maybe_send_stepper_vel(pico, TILT_ID, int(cmd_vel[TILT_ID]), send_state)

                    if (abs(pan_err_deg) <= HOME_TOL_DEG) and (abs(tilt_err_deg) <= HOME_TOL_DEG):
                        auto["phase"] = "PAN_POP"
                        auto["t0"] = time.time()
                        set_popper(True, angle_override=AUTO_PAN_POP_ANGLE)
                        cmd_vel[PAN_ID] = 0.0
                        cmd_vel[TILT_ID] = 0.0
                        pico.stepper_vel(PAN_ID, 0)
                        pico.stepper_vel(TILT_ID, 0)
                        send_state["last_sent_vel"][PAN_ID] = 0
                        send_state["last_sent_vel"][TILT_ID] = 0
                    handled_motion = True

                elif phase == "PAN_POP":
                    # wait briefly for popper to rise
                    set_popper(True, angle_override=AUTO_PAN_POP_ANGLE)
                    if (time.time() - auto["t0"]) >= 0.2:
                        auto["phase"] = "PAN_FIND_LIMIT"
                    handled_motion = True

                elif phase == "PAN_FIND_LIMIT":
                    # drive pan toward limit until LS0 trips
                    set_popper(True, angle_override=AUTO_PAN_POP_ANGLE)
                    pan_search_sps = PAN_LIMIT_TOWARD_SIGN * abs(AUTO_PAN_SEARCH_SPS)
                    pan_target = int(pan_search_sps)
                    tilt_target = 0

                    cmd_vel[PAN_ID]  = ramp_trap_vel(cmd_vel[PAN_ID],  pan_target,  PAN_ACCEL_SPS2,  PAN_DECEL_SPS2,  DT)
                    cmd_vel[TILT_ID] = ramp_trap_vel(cmd_vel[TILT_ID], tilt_target, TILT_ACCEL_SPS2, TILT_DECEL_SPS2, DT)

                    maybe_send_stepper_vel(pico, PAN_ID, int(cmd_vel[PAN_ID]), send_state)
                    maybe_send_stepper_vel(pico, TILT_ID, int(cmd_vel[TILT_ID]), send_state)

                    ls0, _ = last_limits
                    try:
                        lm = pico.get_limits()
                        if lm:
                            last_limits_raw = lm
                            last_limits = normalize_limits(lm)
                            ls0, _ = last_limits
                    except Exception:
                        pass
                    if ls0:
                        cmd_vel[PAN_ID] = 0.0
                        pico.stepper_vel(PAN_ID, 0)
                        send_state["last_sent_vel"][PAN_ID] = 0

                        pan_limit_steps = pos_steps[PAN_ID]
                        pan_deg_raw = steps_to_pan_deg(pan_limit_steps)
                        home_offset_deg[PAN_ID] = wrap_0_360(AUTO_PAN_LIMIT_DEG - pan_deg_raw)

                        # keep popper up for now; retract after unloading in PAN_CLEAR
                        auto["retract_pending"] = True
                        clear_dir = -PAN_LIMIT_TOWARD_SIGN
                        auto["pan_clear_target_steps"] = pan_limit_steps + clear_dir * deg_to_pan_steps(AUTO_PAN_CLEAR_DEG)
                        auto["phase"] = "PAN_CLEAR"
                        auto["t0"] = time.time()

                    handled_motion = True

                elif phase == "PAN_CLEAR":
                    # keep popper up until the switch is released, then retract
                    set_popper(True, angle_override=AUTO_PAN_POP_ANGLE)
                    # back off from the limit switch before continuing
                    target_steps = auto.get("pan_clear_target_steps", pos_steps[PAN_ID])
                    err_steps = target_steps - pos_steps[PAN_ID]
                    ls0, _ = last_limits
                    if auto.get("retract_pending", False) and (not ls0):
                        set_popper(False)
                        auto["retract_pending"] = False

                    if abs(err_steps) <= 1.0 and not ls0:
                        cmd_vel[PAN_ID] = 0.0
                        pico.stepper_vel(PAN_ID, 0)
                        send_state["last_sent_vel"][PAN_ID] = 0
                        auto["retract_pending"] = False
                        auto["phase"] = "PAN_DONE"
                        auto["t0"] = time.time()
                    else:
                        dir_sign = 1 if err_steps > 0 else -1
                        pan_target = dir_sign * abs(AUTO_PAN_SEARCH_SPS)
                        cmd_vel[PAN_ID] = ramp_trap_vel(cmd_vel[PAN_ID], pan_target, PAN_ACCEL_SPS2, PAN_DECEL_SPS2, DT)
                        cmd_vel[TILT_ID] = ramp_trap_vel(cmd_vel[TILT_ID], 0, TILT_ACCEL_SPS2, TILT_DECEL_SPS2, DT)
                        maybe_send_stepper_vel(pico, PAN_ID, int(cmd_vel[PAN_ID]), send_state)
                        maybe_send_stepper_vel(pico, TILT_ID, int(cmd_vel[TILT_ID]), send_state)
                    handled_motion = True

                elif phase == "PAN_DONE":
                    set_popper(False)
                    auto["retract_pending"] = False
                    # move pan to desired tilt-homing angle
                    pan_cmd_raw, pan_err_deg = home_controller_deg(pan_deg, AUTO_TILT_PAN_TARGET_DEG, wrap=True)
                    pan_cmd = int(pan_cmd_raw * speed_scale)
                    tilt_cmd = 0

                    cmd_vel[PAN_ID]  = ramp_trap_vel(cmd_vel[PAN_ID],  pan_cmd,  PAN_ACCEL_SPS2,  PAN_DECEL_SPS2,  DT)
                    cmd_vel[TILT_ID] = ramp_trap_vel(cmd_vel[TILT_ID], tilt_cmd, TILT_ACCEL_SPS2, TILT_DECEL_SPS2, DT)

                    maybe_send_stepper_vel(pico, PAN_ID, int(cmd_vel[PAN_ID]), send_state)
                    maybe_send_stepper_vel(pico, TILT_ID, int(cmd_vel[TILT_ID]), send_state)

                    if abs(pan_err_deg) <= HOME_TOL_DEG:
                        auto["phase"] = "TILT_FIND_LIMIT"
                        auto["t0"] = time.time()
                    handled_motion = True

                elif phase == "TILT_FIND_LIMIT":
                    pan_target = 0
                    tilt_target = int(AUTO_TILT_SEARCH_SPS)

                    cmd_vel[PAN_ID]  = ramp_trap_vel(cmd_vel[PAN_ID],  pan_target,  PAN_ACCEL_SPS2,  PAN_DECEL_SPS2,  DT)
                    # bypass braking limiter so we can reach the physical switch
                    cmd_vel[TILT_ID] = ramp_trap_vel(cmd_vel[TILT_ID], tilt_target, TILT_ACCEL_SPS2, TILT_DECEL_SPS2, DT)

                    maybe_send_stepper_vel(pico, PAN_ID, int(cmd_vel[PAN_ID]), send_state)
                    maybe_send_stepper_vel(pico, TILT_ID, int(cmd_vel[TILT_ID]), send_state)

                    _, ls1 = last_limits
                    try:
                        lm = pico.get_limits()
                        if lm:
                            last_limits_raw = lm
                            last_limits = normalize_limits(lm)
                            _, ls1 = last_limits
                    except Exception:
                        pass
                    if ls1:
                        cmd_vel[PAN_ID] = 0.0
                        cmd_vel[TILT_ID] = 0.0
                        pico.stepper_vel(PAN_ID, 0)
                        pico.stepper_vel(TILT_ID, 0)
                        send_state["last_sent_vel"][PAN_ID] = 0
                        send_state["last_sent_vel"][TILT_ID] = 0
                        tilt_deg_raw = steps_to_tilt_deg(pos_steps[TILT_ID])
                        home_offset_deg[TILT_ID] = clamp_tilt_deg(AUTO_TILT_LIMIT_DEG - tilt_deg_raw)
                        auto["phase"] = "SETTLE"
                        auto["t0"] = time.time()
                    handled_motion = True

                elif phase == "SETTLE":
                    if (time.time() - auto["t0"]) >= AUTO_SETTLE_S:
                        home_valid = True
                        auto["active"] = False
                        auto["phase"] = "IDLE"
                        return_state = "IDLE"
                        set_popper(False)
                    handled_motion = True

            if not handled_motion:
                if in_homing:
                    # target is (0 deg, 0 deg)
                    pan_cmd_raw, pan_err_deg = home_controller_deg(pan_deg, 0.0, wrap=True)
                    tilt_cmd_raw, tilt_err_deg = home_controller_deg(tilt_deg, 0.0)

                    tilt_at_home = abs(tilt_err_deg) <= HOME_TOL_DEG
                    pan_at_home = abs(pan_err_deg) <= HOME_TOL_DEG

                    if return_state == "RETURN_TILT":
                        pan_cmd_raw = 0
                        if tilt_at_home:
                            return_state = "RETURN_PAN"
                    elif return_state == "RETURN_PAN":
                        if tilt_at_home and pan_at_home:
                            return_state = "SETTLING"
                            settle_t0 = time.time()
                    elif return_state == "SETTLING":
                        if not (tilt_at_home and pan_at_home):
                            return_state = "RETURN_TILT" if not tilt_at_home else "RETURN_PAN"
                        elif (time.time() - settle_t0) >= HOME_SETTLE_S:
                            cmd_vel[PAN_ID] = 0.0
                            cmd_vel[TILT_ID] = 0.0
                            pico.stepper_vel(PAN_ID, 0)
                            pico.stepper_vel(TILT_ID, 0)
                            send_state["last_sent_vel"][PAN_ID] = 0
                            send_state["last_sent_vel"][TILT_ID] = 0
                            return_state = "IDLE"
                            set_popper(False)

                    pan_cmd = int(pan_cmd_raw * speed_scale)
                    tilt_cmd = int(tilt_cmd_raw * speed_scale)

                    # enforce tilt angle limits (with braking cap) and limit switch guard
                    tilt_cmd = int(apply_tilt_deg_limits(tilt_cmd, tilt_deg, TILT_DECEL_SPS2))
                    tilt_cmd = int(guard_limit_switch(tilt_cmd, last_limits[1], TILT_LIMIT_TOWARD_SIGN))
                    pan_cmd = int(guard_limit_switch(pan_cmd, last_limits[0], PAN_LIMIT_TOWARD_SIGN))

                    cmd_vel[PAN_ID]  = ramp_trap_vel(cmd_vel[PAN_ID],  pan_cmd,  PAN_ACCEL_SPS2,  PAN_DECEL_SPS2,  DT)
                    cmd_vel[TILT_ID] = ramp_trap_vel(cmd_vel[TILT_ID], tilt_cmd, TILT_ACCEL_SPS2, TILT_DECEL_SPS2, DT)

                    maybe_send_stepper_vel(pico, PAN_ID, int(cmd_vel[PAN_ID]), send_state)
                    maybe_send_stepper_vel(pico, TILT_ID, int(cmd_vel[TILT_ID]), send_state)
                else:
                    # Manual control (left stick)
                    stick_x = apply_deadzone(js.get_axis(ax_x), ST_DEADZONE)
                    stick_y = apply_deadzone(js.get_axis(ax_y), ST_DEADZONE)
                    stick_y = -stick_y  # up => +tilt

                    pan_target = int(stick_x * ST_MAX_SPS * speed_scale)
                    tilt_target = int(stick_y * ST_MAX_SPS * speed_scale)

                    if not steppers_enabled:
                        pan_target = 0
                        tilt_target = 0

                    # enforce tilt angle limits (with braking cap) and limit switch guard
                    tilt_target = int(apply_tilt_deg_limits(tilt_target, tilt_deg, TILT_DECEL_SPS2))
                    tilt_target = int(guard_limit_switch(tilt_target, last_limits[1], TILT_LIMIT_TOWARD_SIGN))
                    pan_target = int(guard_limit_switch(pan_target, last_limits[0], PAN_LIMIT_TOWARD_SIGN))

                    cmd_vel[PAN_ID]  = ramp_trap_vel(cmd_vel[PAN_ID],  pan_target,  PAN_ACCEL_SPS2,  PAN_DECEL_SPS2,  DT)
                    cmd_vel[TILT_ID] = ramp_trap_vel(cmd_vel[TILT_ID], tilt_target, TILT_ACCEL_SPS2, TILT_DECEL_SPS2, DT)

                    maybe_send_stepper_vel(pico, PAN_ID, int(cmd_vel[PAN_ID]), send_state)
                    maybe_send_stepper_vel(pico, TILT_ID, int(cmd_vel[TILT_ID]), send_state)

            # ==========================
            # ONE-BUTTON FIRE (RT)
            # ==========================
            rt_pressed = rt >= TRIGGER_DEADBAND
            rt_edge = rt_pressed and (last_rt < TRIGGER_DEADBAND)

            if fire_state == "IDLE" and rt_edge and not estop:
                try:
                    pico.esc_arm(ESC0_ID)
                    pico.esc_arm(ESC1_ID)
                    flywheels_armed = True
                except Exception:
                    flywheels_armed = False

                pico.set_servo(PUSHER_SERVO_ID, PUSHER_MIN_ANGLE)
                fire_state = "SPINUP"
                fire_t0 = time.time()

            if (not flywheels_armed) or estop or fire_state == "IDLE":
                target0 = ESC_IDLE_US
                target1 = ESC_IDLE_US
            else:
                if fire_state in ("SPINUP", "PUSH", "RETRACT"):
                    target0 = ESC_MAX_US
                    target1 = ESC_MAX_US
                elif fire_state == "SPINDOWN":
                    target0 = ESC_IDLE_US
                    target1 = ESC_IDLE_US
                else:
                    target0 = ESC_IDLE_US
                    target1 = ESC_IDLE_US

            esc0_us = slew_limit(esc0_us, target0, MAX_US_PER_SEC, DT)
            esc1_us = slew_limit(esc1_us, target1, MAX_US_PER_SEC, DT)

            if flywheels_armed and not estop:
                try:
                    pico.esc_us(ESC0_ID, esc0_us)
                    pico.esc_us(ESC1_ID, esc1_us)
                except Exception:
                    flywheels_armed = False

            now = time.time()
            if fire_state == "SPINUP":
                if (now - fire_t0) >= SPINUP_SECONDS:
                    fire_state = "PUSH"
                    fire_t0 = now
                    pico.set_servo(PUSHER_SERVO_ID, PUSHER_MAX_ANGLE)
            elif fire_state == "PUSH":
                if (now - fire_t0) >= PUSH_HOLD_SECONDS:
                    fire_state = "RETRACT"
                    fire_t0 = now
                    pico.set_servo(PUSHER_SERVO_ID, PUSHER_MIN_ANGLE)
            elif fire_state == "RETRACT":
                if (now - fire_t0) >= RETRACT_SETTLE_SECONDS:
                    fire_state = "SPINDOWN"
                    fire_t0 = now
            elif fire_state == "SPINDOWN":
                if (now - fire_t0) >= SPINDOWN_SECONDS:
                    stop_all_flywheels(pico)
                    flywheels_armed = False
                    fire_state = "IDLE"

            last_rt = rt

            # ==========================
            # Status line (rate-limited)
            # ==========================
            tnow = time.time()
            if (tnow - last_status_t) >= STATUS_DT:
                last_status_t = tnow
                line = (
                    f"pan={pan_deg:6.1f}deg tilt={tilt_deg:6.1f}deg "
                    f"V={desired_v:4.1f} T={T:.3f} escMax={ESC_MAX_US:4d} "
                    f"rt={rt:.2f} fire={fire_state:<8} esc0={esc0_us:4d} "
                    f"cmdPan={cmd_vel[PAN_ID]:5.0f} cmdTilt={cmd_vel[TILT_ID]:5.0f} "
                    f"popper={'UP' if popper_is_up else 'DN'} "
                    f"home={'Y' if home_valid else 'N'} ret={return_state:<9} "
                    f"auto={auto['phase'] if auto['active'] else 'OFF'} "
                    f"ls0={last_limits[0]} ls1={last_limits[1]} "
                    f"rawLS0={last_limits_raw[0] if last_limits_raw[0] is not None else '-'} "
                    f"rawLS1={last_limits_raw[1] if last_limits_raw[1] is not None else '-'}"
                )
                status_print(line, status_state)

            # Timing
            elapsed = time.time() - loop_t0
            sleep_s = DT - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)

    except KeyboardInterrupt:
        print("\nCtrl+C exiting...")

    finally:
        try:
            stop_all_flywheels(pico)
        except Exception:
            pass
        try:
            pico.set_servo(PUSHER_SERVO_ID, PUSHER_MIN_ANGLE)
        except Exception:
            pass
        try:
            pico.set_servo(POPPER_SERVO_ID, POPPER_DEFAULT_ANGLE)
        except Exception:
            pass
        try:
            pico.stepper_vel(PAN_ID, 0)
            pico.stepper_vel(TILT_ID, 0)
            pico.stepper_enable(PAN_ID, False)
            pico.stepper_enable(TILT_ID, False)
        except Exception:
            pass

        pygame.quit()
        print("\nExited safe (flywheels stopped, steppers disabled, pusher retracted, popper=90).")


if __name__ == "__main__":
    main()
