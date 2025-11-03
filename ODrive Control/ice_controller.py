import math
import odrive
import keyboard
from odrive.enums import AxisState, ControlMode, InputMode

ODRV_SN = "3348373D3432" # Generic Serial Number, Change This

MAX_CURRENT = 20.0           # Absolute maximum input/output current to/from the motor
CRANK_TORQUE = -5.0          # negative torque (Nm) applied during cranking
MAX_RPM = 3000               # Absolute maximum RPM of motor
STARTING_RPM_THRESHOLD = 1   # Speed at which ICE can be determined to be starting
RUNNING_RPM_THRESHOLD = 1500 # Speed at which ICE can be determined to be running
RUNNING_RPM = 2000           # Speed at which the ICE runs at
SHUTDOWN_RPM_THRESHOLD = 100 # Speed at which ICE can be determined to be stopped

running = True
mode = "IDLE"

def find_odrive():
    print(f"Searching for ODrive: {ODRV_SN}")
    odrv = odrive.find_sync(serial_number=ODRV_SN, timeout=10)
    print(f"Found ODrive: {odrv.serial_number}")

    axis = odrv.axis0
    axis.requested_state = AxisState.IDLE

    return odrv, axis
    
def get_rpm(axis):
    rpm = axis.vel_estimate * 60.0 # convert Rev/s to RPM
    return rpm

def get_rad(axis):
    rad = axis.vel_estimate * 2.0 * math.pi # convert Rev/s to rad/s
    return rad

def get_current(axis):
    Iq = axis.motor.foc.Iq_measured
    return Iq

def get_torque(axis):
    T = axis.motor.torque_estimate
    return T

def set_rpm(axis, rpm):
    if axis.controller.config.control_mode == ControlMode.VELOCITY_CONTROL:
        axis.controller.input_mode = InputMode.VEL_RAMP
        axis.controller.input_vel = rpm
        print(f"Axis velocity set to {rpm} RPM")
    else:
        print("Warning: axis not in velocity control mode. Unable to set desired velocity.")
    return

def set_torque(axis, T):
    if axis.controller.config.control_mode == ControlMode.TORQUE_CONTROL:
        axis.controller.input_torque = T
        print(f"Axis torque set to {T}")
    else:
        print("Warning: axis not in torque control mode. Unable to set desired torque.")
    return

def start_closed_loop_rpm_control(axis, rpm):
    axis.controller.config.control_mode = ControlMode.VELOCITY_CONTROL
    set_rpm(axis, rpm)
    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
    print("Axis in closed-loop velocity control mode")
    return

def start_closed_loop_torque_control(axis, T):
    axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    set_torque(axis, T)
    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
    print("Axis in closed-loop torque control mode")
    return

def end_closed_loop_control(axis):
    set_rpm(axis, 0.0)
    set_torque(axis, 0.0)
    axis.requested_state = AxisState.IDLE
    print("Axis in idle mode")
    return

def is_safe():
    try:
        if abs(get_current(axis)) > MAX_CURRENT:
            print("Unsafe condition: current exceeds maximum current")
            return False
        
        if abs(get_rpm(axis)) > MAX_RPM:
            print("Unsafe condition: RPM exceeds maximum RPM")
            return False
        
    except Exception as e:
        print("Unsafe Condition:", e)
        return False
    
    return True

if __name__ == "__main__":
    print("Intenal Combustion(ICE) Unit Control Program")
    print("Initializing...")
    odrv, axis = find_odrive()
    print("Initialization Complete")

    print("Starting state machine...")
    print("Mode: STARTING")
    mode = "STARTING"
    start_closed_loop_torque_control(axis, 0.0)

    while running:
        if not is_safe():
            raise SystemExit("Unsafe condition: exiting now...")

        if mode == "STARTING":
            if get_rpm(axis) >= STARTING_RPM_THRESHOLD:
                set_torque(axis, CRANK_TORQUE)
            else:
                set_torque(axis, 0.0)
            if get_rpm(axis) >= RUNNING_RPM_THRESHOLD:
                start_closed_loop_rpm_control(axis, RUNNING_RPM)
                print(f"ICE startup detected at {get_rpm(axis)} rpm, switching to running mode...")
                print("Mode: RUNNING")
                mode = "RUNNING"

        elif mode == "RUNNING":
            if get_rpm(axis) <= SHUTDOWN_RPM_THRESHOLD or keyboard.is_pressed('q'):
                print(f"ICE Shutdown initiated at {get_rpm(axis)} rpm")
                set_rpm(axis, 0.0)
                end_closed_loop_control(axis)
                raise SystemExit("Shutdown complete: exiting now...")

        else:
            raise SystemExit("Undefined state: check program, exiting now...")

    end_closed_loop_control(axis)
    print("Exited Intenal Combustion(ICE) Unit Control Program: End of Program")
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