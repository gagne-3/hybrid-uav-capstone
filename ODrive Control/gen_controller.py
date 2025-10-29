import odrive
from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL

ODRV_SN = "3348373D3432" # Generic Serial Number, Change This

running = True
state = "IDLE"

def find_odrive():
    print(f"Searching for ODrive: {ODRV_SN}")
    odrv = odrive.find_sync(serial_number=ODRV_SN, timeout=10)

    print(f"Found ODrive: {odrv.serial_number}")
    axis = odrv.axis0

    return odrv, axis

def check_calibration():
    print("Checking motor calibration...")
    if axis.motor.is_calibrated and axis.encoder.is_ready:
        print("Calibration check success!")
        return
    raise SystemExit("Calibration check failure: please check motor calibration")

def is_safe():
    return True

if __name__ == "__main__":
    print("Starter/Generator Unit Control Program")
    print("Initializing...")
    odrv, axis = find_odrive()
    check_calibration()
    print("Initialization Complete")

    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    print("Axis in closed loop control")

    state = "IDLE"
    print("Starting state machine...")
    print("State: IDLE")

    while running:
        if not is_safe():
            print("Unsafe condition: exiting now...")
            break

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

    print("Exited Starter/Generator Unit Control Program")