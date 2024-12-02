import threading
import time
import minimalmodbus
import serial

class MotorController:
    _lock = threading.RLock()  # Class-level lock for all instances

    def __init__(self, 
                port='/dev/ttyUSB0', 
                slave_address=1, 
                baudrate=115200, 
                pulses_per_unit=None, 
                upper_limit_units=None,
                lower_limit_units=None,
                home_position=0,
                velocity=450, 
                acceleration=100, 
                deceleration=100,
                jog_velocity=100,
                current_limit_amperes=4.0,
                homing_current=1.0):
        
        self.port = port
        self.slave_address = slave_address
        self.home_position = home_position #TODO consider when we need to home to the upper limit
        self.is_homed = False
        
        self._setup_drive(port, slave_address, baudrate)
        self._setup_motor_parameters(pulses_per_unit, upper_limit_units, lower_limit_units, velocity, acceleration, deceleration, jog_velocity, current_limit_amperes, homing_current)

    # Setup methods (private)
    def _setup_drive(self, port, slave_address, baudrate):
        self.drive = minimalmodbus.Instrument(port, slave_address)
        
        self.drive.serial.baudrate = baudrate
        self.drive.serial.bytesize = 8
        self.drive.serial.parity = serial.PARITY_NONE
        self.drive.serial.stopbits = 1
        self.drive.serial.timeout = 1

    def _setup_motor_parameters(self, pulses_per_unit, upper_limit_units, lower_limit_units, velocity, acceleration, deceleration, jog_velocity, current_limit_amperes, homing_current):
        self.pulses_per_unit = pulses_per_unit
        self.lower_limit = lower_limit_units
        self.upper_limit = upper_limit_units
        self.homing_direction = None
        self.current_limit = current_limit_amperes
        self.homing_current = homing_current
        
        self.set_current_limit(self.current_limit)
        self.set_velocity(velocity)
        self.set_jog_velocity(jog_velocity)
        self.set_acceleration(acceleration)
        self.set_deceleration(deceleration)

    # Register read/write methods (private)
    def _write_register(self, address, value, functioncode=6):
        with self._lock:
            self.drive.write_register(address, value, functioncode=functioncode)

    def _read_register(self, address, functioncode=3):
        with self._lock:
            return self.drive.read_register(address, functioncode=functioncode)

    # Private methods for internal operations
    def _check_alarm(self):
        try:
            alarm_status = self._read_register(0x2203)
            if alarm_status != 0:
                raise RuntimeError(f"Motor alarm is active! Alarm status: {alarm_status:#04X}")
        except Exception as e:
            raise RuntimeError(f"Error checking alarm status: {str(e)}")
    
    def _check_homed(self):
        if not self.is_homed:
            raise RuntimeError("Motors not homed")
    
    def _set_current_position_as_zero(self):
        try:
            self._write_register(0x6002, 0x21)
        except Exception as e:
            raise RuntimeError(f"Failed to set current position as zero: {e}")

    def _read_feedback_position(self):
        try:
            high_feedback = self._read_register(0x1014)
            low_feedback = self._read_register(0x1015)
            return (high_feedback << 16) | (low_feedback & 0xFFFF)
        except Exception as e:
            raise RuntimeError(f"Failed to read feedback position: {e}")

    def _read_current_position(self):
        try:
            high_position = self._read_register(0x602C)
            low_position = self._read_register(0x602D)
            position = (high_position << 16) | (low_position & 0xFFFF)
            if position > 0x7FFFFFFF:  # If the value is negative (2's complement)
                position -= 0x100000000  # Convert to signed integer
            return position
        except Exception as e:
            raise RuntimeError(f"Failed to read current position: {e}")

    def _jog_motor_towards_home(self):
        command = 0x4002 
        try:
            self._write_register(0x1801, command)
        except Exception as e:
            raise RuntimeError(f"Failed to execute jog command: {e}")

    def _jog_to_mechanical_limit(self,tolerance):
        try:
            previous_positions = []
            while True:
                self._jog_motor_towards_home()
                current_position = self._read_feedback_position()
                
                if len(previous_positions) >= 5:
                    if all(abs(current_position - pos) <= tolerance for pos in previous_positions):
                        break
                    previous_positions.pop(0)
                
                previous_positions.append(current_position)
        except Exception as e:
            raise RuntimeError(f"Error during jogging to limit: {e}")

    def _wait_for_motion_complete(self):
        try:
            while True:
                status = self._read_register(0x1003)
                if status & 0x10:  # Check Bit 4 (0x10 is binary 10000)
                    break
                time.sleep(0.1)
        except Exception as e:
            raise RuntimeError(f"Error checking motion status: {e}")

    def _move_relative_pulses(self, pulses):
        try: 
            pulses = pulses * 2  
            high_16_bits = (pulses >> 16) & 0xFFFF
            low_16_bits = pulses & 0xFFFF
            self._write_register(0x6200, 0x41)
            self._write_register(0x6201, high_16_bits)
            self._write_register(0x6202, low_16_bits)
            self._write_register(0x6002, 0x10)
        except Exception as e:
            raise RuntimeError(f"Failed to move relative pulses: {e}")
        self._wait_for_motion_complete()

    def _move_absolute_pulses(self, pulses):
        try: 
            pulses = pulses * 2  
            high_16_bits = (pulses >> 16) & 0xFFFF
            low_16_bits = pulses & 0xFFFF
            self._write_register(0x6200, 0x01)
            self._write_register(0x6201, high_16_bits)
            self._write_register(0x6202, low_16_bits)
            self._write_register(0x6002, 0x10)
        except Exception as e:
            raise RuntimeError(f"Failed to move to absolute position: {e}")
        self._wait_for_motion_complete()

    # Public methods for motor control
    @property
    def current_position(self):
        return self._read_current_position() / (self.pulses_per_unit * 2)

    def home_to_mechanical(self, direction, tolerance=5):
        self._check_alarm()
        self.set_homing_direction(direction)
        try:
            self.set_current_limit(self.homing_current)
            self._jog_to_mechanical_limit(tolerance)

            self.set_current_limit(self.current_limit)
            self._set_current_position_as_zero()  
            self.is_homed = True
        except Exception as e:
            raise RuntimeError(f"Error during homing to mechanical limit: {e}")

    def home_to_sensor(self, direction):
        self._check_alarm()
        direction = direction.upper()
        
        try:
            self.set_current_limit(self.homing_current)
            while not self.home_sensor:
                self._jog_motor_towards_home()
                time.sleep(0.05)
            self.home_sensor = False
            self.lower_limit = self.home_position
            self.set_current_limit(self.current_limit)
            self._set_current_position_as_zero()
            self.is_homed = True
        except Exception as e:
            raise RuntimeError(f"Error during homing to sensor: {e}")

    def move_absolute(self, target):
        self._check_alarm()
        self._check_homed()
        if self.lower_limit is None or self.upper_limit is None:
            raise ValueError("Limits not set. Please set limits first.")
        if not self.lower_limit <= target <= self.upper_limit:
            raise ValueError(f"Target position {target} is out of bounds of {self.lower_limit} to {self.upper_limit}")
        
        move_pulses = int(target * self.pulses_per_unit)
        self._move_absolute_pulses(move_pulses)

    def move_relative(self, distance):
        self._check_alarm()
        self._check_homed()
        if self.lower_limit is None or self.upper_limit is None:
            raise ValueError("Limits not set. Please set limits first.")
        target = self.current_position + distance
        if not self.lower_limit <= target <= self.upper_limit:
            raise ValueError(f"Target position {target} is out of bounds of {self.lower_limit} to {self.upper_limit}")
        
        distance = float(distance)
        move_pulses = int(distance * self.pulses_per_unit)
        self._move_relative_pulses(move_pulses)

    def stop(self):
        try:
            self._write_register(0x6002, 0x40)
            print("Motor e-stop engaged")
        except Exception as e:
            raise RuntimeError(f"Failed to stop motor: {e}")

    def clear_alarm(self):
        try:
            self._write_register(0x1801, 0x1111)
        except Exception as e:
            raise RuntimeError(f"Failed to clear alarm: {e}")

    # Public methods for motor parameter setting
    def set_velocity(self, velocity_rpm):
        self._write_register(0x6203, velocity_rpm)

    def set_jog_velocity(self, jog_vel):
        self._write_register(0x01E1, jog_vel)

    def set_acceleration(self, acceleration_ms_per_1000rpm):
        self._write_register(0x6204, acceleration_ms_per_1000rpm)

    def set_deceleration(self, deceleration_ms_per_1000rpm):
        self._write_register(0x6205, deceleration_ms_per_1000rpm)

    def set_current_limit(self, current_limit_amperes):
        register_value = int(current_limit_amperes * 10)
        self._write_register(0x0191, register_value)

    def set_homing_direction(self, homing_direction):
        homing_direction = homing_direction.upper()
        if homing_direction not in ['CW', 'CCW']:
            raise ValueError("Invalid direction. Please enter 'CW' or 'CCW'.")
        self.homing_direction = homing_direction
        if homing_direction == "CW":
            self._write_register(0x0007, 1) # intentionally flipped
        elif homing_direction == "CCW":
            self._write_register(0x0007, 0) # intentionally flipped