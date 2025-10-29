import math
import odrive
from odrive.enums import AxisState, ControlMode

ODRV_SN = "3348373D3432" # Generic Serial Number, Change This
MAX_CURRENT = 20
MAX_RPM = 3000

running = True
state = "IDLE"

def find_odrive():
    print(f"Searching for ODrive: {ODRV_SN}")
    odrv = odrive.find_sync(serial_number=ODRV_SN, timeout=10)
    print(f"Found ODrive: {odrv.serial_number}")

    axis = odrv.axis0

    return odrv, axis

def get_rpm(axis):
    rpm = axis.vel_estimate * 60.0 # convert Rev/s to RPM
    return rpm

def get_rad(axis):
    rad = (get_rpm(axis) * 2.0 * math.pi) / 60.0 # convert RPM to rad/s
    return rad

def get_current(axis):
    Iq = axis.motor.foc.Iq_measured
    return Iq

def get_torque(axis):
    T = axis.motor.torque_estimate
    return T

def set_torque(axis, T):
    if axis.controller.config.control_mode == ControlMode.TORQUE_CONTROL:
        axis.controller.input_torque = T
    else:
        print("Warning: axis not in torque control mode. Unable to set desired torque.")
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
    print("Starter/Generator Unit Control Program")
    print("Initializing...")
    odrv, axis = find_odrive()
    print("Initialization Complete")

    axis.requested_state = AxisState.CLOSED_LOOP_CONTROL
    axis.controller.config.control_mode = ControlMode.TORQUE_CONTROL
    set_torque(axis, 0.0)
    print("Axis in closed-loop torque control mode")

    state = "IDLE"
    print("Starting state machine...")
    print("State: IDLE")

    while running:
        if not is_safe():
            raise SystemExit("Unsafe condition: exiting now...")

        if state == "IDLE":
            print("Please select next step:")
            print("1) Begin startup routine")
            print("2) Exit")
            user_input = input()
            if user_input == "1":
                print("Startup routine beginning...")
                print("State: START")
            elif user_input == "2":
                print("Exiting now...")
                break;
            else:
                print("Invalid input: please try again")

        elif state == "START":
            print()

        elif state == "GENERATE":
            print()

        else:
            print("Undefined state: check program, exiting now...")
            break

    set_torque(axis, 0.0)
    axis.requested_state = AxisState.IDLE
    print("Exited Starter/Generator Unit Control Program")