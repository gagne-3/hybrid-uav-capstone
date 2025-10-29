import time
import math
import odrive

from odrive.enums import AxisState, ControlMode

import signal

# ---------- CONFIG ----------
AXIS_INDEX = 0                # 0 or 1 depending which axis on the ODrive this ICE is connected to
CRANK_TORQUE_NM = -5.0        # negative torque (Nm) applied during cranking
ENGINE_TORQUE_PEAK_NM = 15.0  # steady positive torque after startup
START_RPM_THRESHOLD = 200.0   # detect start when RPM exceeds this
START_HOLD_SECONDS = 0.5      # must hold above RPM threshold for this long
DETECT_TORQUE_DROP_NM = 2.0   # optional: detect when measured torque drops by this (Nm)
SAMPLE_INTERVAL = 0.02        # seconds
MAX_CURRENT_A = 40.0          # safety
MAX_RPM = 4500.0              # safety (motor RPM)
STARTUP_TIMEOUT = 10.0        # fallback: if not started within this many seconds, stop
TORQUE_RAMP_TIME = 2.0        # time to ramp engine torque up after start, seconds
# ----------------------------

running = True


def sigint_handler(sig, frame):
    global running
    running = False


signal.signal(signal.SIGINT, sigint_handler)
signal.signal(signal.SIGTERM, sigint_handler)


def find_odrive_and_axis():
    print("Finding an ODrive... (this script expects to run on a machine connected to one ODrive)")
    odrv0 = odrive.find_any(timeout=10)
    print("Found ODrive:", odrv0.serial_number)
    axis = odrv0.axis0
    return odrv0, axis

def nm_from_measured_current(axis):
    # torque = Iq_measured * motor.config.torque_constant
    try:
        t = axis.motor.torque_estimate
        return t
    except Exception:
        return 0.0
    
def rpm_from_velocity(axis):
    # axis.encoder.vel_estimate is in rev/s per docs; convert to RPM
    try:
        revs_per_sec = axis.encoder.vel_estimate
        return revs_per_sec * 60.0
    except Exception:
        return 0.0


def set_torque(axis, torque_nm):
    # ODrive controller accepts torque setpoint in Nm via axis.controller.input_torque
    axis.controller.input_torque = float(torque_nm)


def safe_check(axis):
    # read some safety telemetry and return False if unsafe
    try:
        iq = abs(axis.motor.foc.Iq_measured)
        rpm = abs(rpm_from_velocity(axis))
        if iq > MAX_CURRENT_A + 2.0:
            print("SAFETY: measured current too high:", iq)
            return False
        if rpm > MAX_RPM + 100.0:
            print("SAFETY: RPM too high:", rpm)
            return False
    except Exception as e:
        print("Safety read error:", e)
    return True


def main_loop():
    odrv0, axis = find_odrive_and_axis()

    # Ensure closed loop
    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
    print("Entered closed loop control. Starting ICE state machine.")

    axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL

    state = "CRANKING"
    cranking_start_time = time.time()
    start_detected_time = None
    torque_before = None

    while running:
        if not safe_check(axis):
            print("Unsafe condition — stopping torque and exiting.")
            set_torque(axis, 0.0)
            break

        rpm = rpm_from_velocity(axis)
        measured_torque = nm_from_measured_current(axis)
        # instantaneous mechanical power (W): torque (Nm) * angular velocity (rad/s)
        omega = rpm * 2.0 * math.pi / 60.0
        mech_power_W = measured_torque * omega

        if state == "CRANKING":
            set_torque(axis, CRANK_TORQUE_NM)    # hold negative torque to simulate cranking resistance
            if torque_before is None:
                torque_before = measured_torque

            # detect start primarily by RPM sustained above threshold
            if rpm >= START_RPM_THRESHOLD:
                if start_detected_time is None:
                    start_detected_time = time.time()
                elif (time.time() - start_detected_time) >= START_HOLD_SECONDS:
                    print(f"Start detected via RPM >= {START_RPM_THRESHOLD} RPM for {START_HOLD_SECONDS}s.")
                    state = "ENGINE_RUNNING_RAMP"
                    engine_ramp_start = time.time()
            else:
                start_detected_time = None

            # optional: detect torque drop (starter sees torque fall)
            if abs(torque_before - measured_torque) >= DETECT_TORQUE_DROP_NM:
                print("Start detected via torque drop.")
                state = "ENGINE_RUNNING_RAMP"
                engine_ramp_start = time.time()

            # timeout fallback
            if (time.time() - cranking_start_time) > STARTUP_TIMEOUT:
                print("Startup timeout reached, stopping torque.")
                set_torque(axis, 0.0)
                break

        elif state == "ENGINE_RUNNING_RAMP":
            # Ramp torque up smoothly to ENGINE_TORQUE_PEAK_NM over TORQUE_RAMP_TIME
            elapsed = time.time() - engine_ramp_start
            frac = min(1.0, elapsed / TORQUE_RAMP_TIME)
            torque_cmd = CRANK_TORQUE_NM + (ENGINE_TORQUE_PEAK_NM - CRANK_TORQUE_NM) * frac
            set_torque(axis, torque_cmd)
            if frac >= 1.0:
                state = "ENGINE_RUNNING"
                print("Engine running: applying positive torque.")

        elif state == "ENGINE_RUNNING":
            set_torque(axis, ENGINE_TORQUE_PEAK_NM)
            # remain here indefinitely; you can implement more complex engine torque curves if desired.

        # Logging telemetry occasionally
        print(f"[{state}] RPM={rpm:.1f} RPM, T_meas={measured_torque:.2f} Nm, P={mech_power_W:.1f} W")
        time.sleep(SAMPLE_INTERVAL)

    # cleanup
    try:
        set_torque(axis, 0.0)
        axis.requested_state = AxisState.IDLE
    except Exception:
        pass
    print("ICE controller exiting cleanly.")


if __name__ == "__main__":
    main_loop()