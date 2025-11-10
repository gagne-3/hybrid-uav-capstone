import math
import odrive
import keyboard
from odrive.enums import AxisState, ControlMode, InputMode
from odrive_log import odrive_log

ODRV_SN = "003cf674534b"

MAX_CURRENT = 20.0           # Absolute maximum input/output current to/from the motor
CRANK_TORQUE = -5.0          # negative torque (Nm) applied during cranking
MAX_RPM = 3000               # Absolute maximum RPM of motor
STARTING_RPM_THRESHOLD = 150   # Speed at which ICE can be determined to be starting
RUNNING_RPM_THRESHOLD = 1500 # Speed at which ICE can be determined to be running
RUNNING_RPM = 2000           # Speed at which the ICE runs at
SHUTDOWN_RPM_THRESHOLD = 300 # Speed at which ICE can be determined to be stopped

log = odrive_log("ice")

running = True

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
        axis.controller.input_vel = (rpm / 60)
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
    print("Internal Combustion Engine (ICE) Control Program")
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
        
        log.logData(mode, odrv, axis)

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