import math
import time

import odrive
from odrive.enums import *
from odrive.utils import *
from odrive.device_manager import *

# Find a connected ODrive (this will block until you connect one)
print("waiting for ODrive...")
odrv0 = odrive.find_any()
print(f"found ODrive {odrv0._dev.serial_number}")

# Clear errors
odrv0.clear_errors()

# Enter closed loop control
odrv0.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
odrv0.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
request_state(odrv0.axis0, AxisState.CLOSED_LOOP_CONTROL)

# Reset Current Position to 0
odrv0.axis0.pos_estimate = 0

# Live Plotter
#start_liveplotter(properties=[odrv0.axis0._pos_estimate_property])


# Ramp up velocity over time
try: 
    for i in range(50):
        target_pos = i # Increase velocity gradually
        odrv0.axis0.controller.input_pos = target_pos
        print(target_pos)
        time.sleep(0.1)
        

    for i in range(50, 0, -1):
        new_target_pos = i # Increase velocity gradually
        odrv0.axis0.controller.input_pos = new_target_pos
        print(new_target_pos)
        time.sleep(0.1)

finally:
    request_state(odrv0.axis0, AxisState.IDLE)

# Show errors
dump_errors(odrv0)