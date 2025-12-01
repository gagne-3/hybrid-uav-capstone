import odrive
import time
from odrive.enums import AxisState, ControlMode, InputMode

ODRV_SN = "003CF674534B"

MAX_CURRENT = 20.0           # Absolute maximum input/output current to/from the motor
MAX_RPM = 3000               # Absolute maximum RPM of motor

running = True

def find_odrive():
    print(f"Searching for ODrive: {ODRV_SN}")
    odrv = odrive.find_sync(serial_number=ODRV_SN, timeout=10)
    print(f"Found ODrive: {odrv.serial_number}")

    axis = odrv.axis0
    axis.requested_state = AxisState.IDLE

    return odrv, axis

def get_torque(axis):
    T = axis.motor.torque_estimate
    return T

def get_rpm(axis):
    rpm = axis.vel_estimate * 60.0 # convert Rev/s to RPM
    return rpm

def get_current(axis):
    Iq = axis.motor.foc.Iq_measured
    return Iq

def set_torque(axis, T):
    if axis.controller.config.control_mode == ControlMode.TORQUE_CONTROL:
        axis.controller.input_torque = T
        # print(f"Axis torque set to {T}")
    else:
        print("Warning: axis not in torque control mode. Unable to set desired torque.")
    return

def start_closed_loop_torque_control(axis, T):
    axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    set_torque(axis, T)
    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
    print("Axis in closed-loop torque control mode")
    return

def end_closed_loop_control(axis):
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
    print("Tourque Testing Control Program")
    print("Initializing...")
    odrv, axis = find_odrive()
    print("Initialization Complete")

    print("Running")
    print("Mode: RUNNING")
    mode = "RUNNING"
    start_closed_loop_torque_control(axis, 0.0)

    while running:
        if not is_safe():
            raise SystemExit("Unsafe condition: exiting now...")
        if mode == "RUNNING":
            for SET_TORQUE in [0.5, 1.0, 2.0, 4.0]:
                print(f"Setting torque to {SET_TORQUE} A")
                set_torque(axis, SET_TORQUE)
                time.sleep(5)
            running = False

    end_closed_loop_control(axis)
    print("Exited Torque Test Control Program: End of Program")