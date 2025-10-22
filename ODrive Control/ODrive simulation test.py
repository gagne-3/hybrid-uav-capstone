import math
import time

import odrive
from odrive.enums import AxisState, ControlMode, InputMode
from odrive.utils import dump_errors, request_state

# Find a connected ODrive (this will block until you connect one)
print("waiting for ODrive...")
odrv0 = odrive.find_sync()
print(f"found ODrive {odrv0._dev.serial_number}")

# Enter closed loop control
odrv0.axis0.controller.config.input_mode = InputMode.PASSTHROUGH
odrv0.axis0.controller.config.control_mode = ControlMode.POSITION_CONTROL
request_state(odrv0.axis0, AxisState.CLOSED_LOOP_CONTROL)

# Ramp up velocity over time
try: 
    for i in range(100):
        target_velocity = i * 0.1  # Increase velocity gradually
        odrv0.axis0.controller.input_vel = target_velocity
        time.sleep(0.1)  # Wait 100ms between updates

finally:
    request_state(odrv0.axis0, AxisState.IDLE)

# Show errors
dump_errors(odrv0)