import time
import math
import odrive
from odrive.enums import AxisState
import signal

# ---------- CONFIG ----------
AXIS_INDEX = 1                # axis of the ODrive for starter/generator
ASSIST_TORQUE_NM = 8.0        # positive torque applied while cranking to assist starter
GEN_MAX_NEG_TORQUE_NM = -12.0 # maximum allowed negative torque (generator braking)
GEN_TORQUE_LIMIT_NEG = -20.0  # safety hard limit
ASSIST_RPM_TARGET = 150.0     # optionally aim to maintain this RPM while cranking
DETECTION_RPM_INCREASE = 50.0 # if RPM increases by this much (from baseline) detect reduced load
SAMPLE_INTERVAL = 0.02
MEASURE_WINDOW = 0.2
MAX_CURRENT_A = 40.0
MAX_RPM = 6000.0
# ----------------------------

running = True

def sigint_handler(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT, sigint_handler)
signal.signal(signal.SIGTERM, sigint_handler)


def find_odrive_and_axis(timeout=10.0):
    print("Finding an ODrive for starter... ")
    odrv0 = odrive.find_any(timeout=timeout)
    print("Found ODrive:", odrv0.serial_number)
    if AXIS_INDEX == 0:
        axis = odrv0.axis0
    else:
        axis = odrv0.axis1
    return odrv0, axis


def ensure_calibrated(axis):
    # Check if already calibrated
    if axis.motor.is_calibrated and axis.encoder.is_ready:
        return
    print("Starting full calibration sequence...")
    axis.requested_state = AxisState.FULL_CALIBRATION_SEQUENCE
    # Wait for calibration to finish
    while True:
        st = axis.current_state
        if st == AxisState.CLOSED_LOOP_CONTROL:
            break
        if not running:
            raise SystemExit("Aborted during calibration")
        time.sleep(0.1)
    print("Calibration complete.")


def rpm_from_velocity(axis):
    try:
        return axis.encoder.vel_estimate * 60.0
    except Exception:
        return 0.0


def nm_from_measured_current(axis):
    try:
        iq = axis.motor.current_control.Iq_measured
        tc = axis.motor.config.torque_constant
        return iq * tc
    except Exception:
        return 0.0


def set_torque(axis, torque_nm):
    # Using torque control input
    axis.controller.input_torque = float(torque_nm)


def safe_check(axis):
    try:
        iq = abs(axis.motor.current_control.Iq_measured)
        rpm = abs(rpm_from_velocity(axis))
        if iq > (MAX_CURRENT_A + 2.0):
            print("SAFETY: measured current too high:", iq)
            return False
        if rpm > (MAX_RPM + 100.0):
            print("SAFETY: RPM too high:", rpm)
            return False
    except Exception as e:
        print("Safety read error:", e)
    return True


def main_loop():
    odrv0, axis = find_odrive_and_axis()
    ensure_calibrated(axis)
    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
    print("Starter axis in closed loop. Starting state machine.")

    state = "ASSIST"
    rpm_baseline = None
    last_window = []
    gen_engaged_time = None

    while running:
        if not safe_check(axis):
            print("Unsafe condition — stopping torque and exiting.")
            set_torque(axis, 0.0)
            break

        rpm = rpm_from_velocity(axis)
        measured_torque = nm_from_measured_current(axis)
        omega = rpm * 2.0 * math.pi / 60.0
        mech_power_W = measured_torque * omega

        last_window.append(rpm)
        if len(last_window) > int(MEASURE_WINDOW / SAMPLE_INTERVAL):
            last_window.pop(0)

        if rpm_baseline is None:
            rpm_baseline = rpm

        if state == "ASSIST":
            set_torque(axis, ASSIST_TORQUE_NM)

            rpm_delta = rpm - rpm_baseline
            if rpm_delta > DETECTION_RPM_INCREASE:
                print(f"Detection: RPM increased by {rpm_delta:.1f} -> assuming ICE started / resistance reduced")
                state = "GENERATE"
                gen_engaged_time = time.time()

            rpm_baseline = rpm_baseline * 0.995 + rpm * 0.005

        elif state == "GENERATE":
            set_torque(axis, GEN_MAX_NEG_TORQUE_NM)

            if rpm < (rpm_baseline + DETECTION_RPM_INCREASE * 0.5):
                if (time.time() - gen_engaged_time) > 0.5:
                    print("Generator: load re-appeared or RPM fell -> switching back to ASSIST")
                    state = "ASSIST"
                    rpm_baseline = rpm

        print(f"[{state}] RPM={rpm:.1f} T_meas={measured_torque:.2f} Nm P={mech_power_W:.1f} W")
        time.sleep(SAMPLE_INTERVAL)

    try:
        set_torque(axis, 0.0)
    except Exception:
        pass
    print("Starter controller exiting cleanly.")


if __name__ == "__main__":
    main_loop()
