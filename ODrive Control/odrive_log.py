import odrive
from datetime import datetime
import csv
import math

axis_states = ["Undefined", "Idle", "Startup Sequence", "Full Calibration Sequence", "Motor Calibration", "Encoder Index Search", "Encoder Offset Calibration", "Closed Loop Control",
               "Lockin Spin", "Encoder Dir Find", "Homing", "Encoder Hall Phase Calibration", "Anticogging Calibration", "Harmonic Calibration", "Harmonic Calibration Commutation"]

control_modes = ["Voltage Control", "Torque Control", "Velocity Control", "Position Control"]

headings = ['Time (s)', 'Mode', 'Axis State', 'Control Mode', 'Velocity (rpm)', 'Velocity Setpoint (rpm)', 'Torque (A)', 'Torque Setpoint (A)', 'Current (A)', 'Mechanical Power (W)', 
            'Electrical Power (W)', 'Bus Voltage (V)', 'Bus Current (A)', 'Brake Resistor Current (A)']

class odrive_log:
    
    def __init__(self, drive_type):
        self.startTime = datetime.now()
        logDate = self.startTime.strftime('%m%d%Y-%H%M%S%f')
        self.filename = drive_type + '_log_' + logDate + '.csv'

        self.log = open(self.filename, 'a', newline='')
        self.logWriter = csv.writer(self.log)
        self.writeRow(headings)

        self.lastTime = datetime.now()

    def writeRow(self, row):
        self.logWriter.writerow(row)
    
    def getTime(self):
        dT = datetime.now() - self.startTime
        return dT.total_seconds()
    
    def checkAxisState(axis):
        state = axis_states[axis.current_state]
        return state
    
    def checkControlMode(axis):
        mode = control_modes[axis.controller.config.control_mode]
        return mode
    
    def get_rpm(axis):
        rpm = axis.vel_estimate * 60.0 # convert Rev/s to RPM
        return rpm
    
    def get_vel_setpoint(axis):
        vel = axis.controller.vel_setpoint
        return vel
    
    def get_torque(axis):
        T = axis.motor.torque_estimate
        return T
    
    def get_torque_setpoint(axis):
        T = axis.controller.input_torque
        return T
    
    def get_current(axis):
        Iq = axis.motor.foc.Iq_measured
        return Iq
    
    def get_mech_power(axis):
        P = axis.motor.mechanical_power
        return P
    
    def get_elec_power(axis):
        P = axis.motor.electrical_power
        return P
    
    def get_bus_voltage(odrv):
        V = odrv.vbus_voltage
        return V
    
    def get_bus_current(odrv):
        I = odrv.ibus
        return I
    
    def get_br_current(odrv):
        I = odrv.brake_resistor0.current
        return I
        
    def logData(self, mode, odrv, axis):
        dT = datetime.now() - self.lastTime

        # Throttle Log Rate
        if dT > 0.01:
            data = [self.getTime(), mode, self.checkAxisState(axis), self.checkControlMode(axis), self.get_rpm(axis), self.get_vel_setpoint(axis), self.get_torque(axis), self.get_torque_setpoint(axis),
                    self.get_current(axis), self.get_mech_power(axis), self.get_elec_power(axis), self.get_bus_voltage(odrv), self.get_bus_current(odrv), self.get_br_current(odrv)]
            self.writeRow(data)
            self.lastTime = datetime.now()
            return
        else:
            return