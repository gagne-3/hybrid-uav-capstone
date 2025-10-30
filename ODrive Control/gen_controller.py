import math
import odrive
from odrive.enums import AxisState, ControlMode

ODRV_SN = "3348373D3432" # Generic Serial Number, Change This

MAX_CURRENT = 20.0 # Absolute maximum input/output current to/from the motor
CRANKING_TORQUE = 15.0 # Applied torque when cranking ICE, Amps
GENERATING_TORQUE = -10.0 # Applied torque when generating, Amps

MAX_RPM = 3000 # Absolute maximum RPM of motor
STARTUP_RPM_THRESHOLD = 1500 # Speed at which ICE can be determined to be running
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

def set_torque(axis, T):
    if axis.controller.config.control_mode == ControlMode.TORQUE_CONTROL:
        axis.controller.input_torque = T
        print(f"Axis torque set to {T}")
    else:
        print("Warning: axis not in torque control mode. Unable to set desired torque.")
    return

def start_closed_loop_control(axis, T):
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
    print("Starter/Generator Unit Control Program")
    print("Initializing...")
    odrv, axis = find_odrive()
    print("Initialization Complete")

    print("Starting state machine...")
    print("Mode: IDLE")
    mode = "IDLE"

    while running:
        if not is_safe():
            raise SystemExit("Unsafe condition: exiting now...")

        if mode == "IDLE":
            print("Please select next step:")
            print("1) Begin startup routine")
            print("2) Exit")
            user_input = input()
            if user_input == "1":
                print("Startup routine beginning...")
                print("Mode: STARTER")
                mode = "STARTER"
                start_closed_loop_control(axis, CRANKING_TORQUE)
            elif user_input == "2":
                print("Exiting now...")
                raise SystemExit("Exited Starter Generator Unit Control Program: User Requested Exit")
            else:
                print("Invalid input: please try again")

        elif mode == "STARTER":
            if get_rpm(axis) >= STARTUP_RPM_THRESHOLD:
                print(f"ICE startup detected at {get_rpm(axis)} rpm, switching to generator mode...")
                print("Mode: GENERATOR")
                mode = "GENERATOR"
                set_torque(axis, GENERATING_TORQUE)

        elif mode == "GENERATOR":
            if get_rpm(axis) <= SHUTDOWN_RPM_THRESHOLD:
                print(f"ICE Shutdown detected at {get_rpm(axis)} rpm, switching to idle mode")
                print("Mode: IDLE")
                mode = "IDLE"
                end_closed_loop_control(axis)

        else:
            raise SystemExit("Undefined state: check program, exiting now...")

    end_closed_loop_control(axis)
    print("Exited Starter/Generator Unit Control Program: End of Program")