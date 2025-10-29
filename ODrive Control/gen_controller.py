import odrive
import time
import math
import signal

from odrive.enums import *
from odrive.utils import *
from odrive.device_manager import *

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
MAX_RPM = 5000.0              # safety(Based off previous groups work)
# ----------------------------

running = True

def sigint_handler(sig, frame):
    global running
    running = False

signal.signal(signal.SIGINT, sigint_handler)
signal.signal(signal.SIGTERM, sigint_handler)

def find_odrive_and_axis():
    print("Finding an ODrive for starter... ")
    odrv0 = odrive.find_any(timeout=10)
    print("Found ODrive:", odrv0._dev.serial_number)
    axis = odrv0.axis0 if AXIS_INDEX == 0 else odrv0.axis1
    return odrv0, axis

