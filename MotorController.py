import threading
import time
import minimalmodbus
import serial
import serial.tools.list_ports
import glob
import os
import subprocess
from config import MOTOR_CONTROLLER_USB
from lib.EmailService import EmailService
import logging
from gpio_utils import reset_usb_connection

logger = logging.getLogger(__name__)

class MotorController:
    _lock = threading.RLock()  # Class-level lock for all instances
    _max_retries = 5  # Maximum number of retries for communication

    @classmethod
    def list_usb_devices(cls):
        """List all available USB serial devices."""
        # Method 1: Using glob to find all ttyUSB devices
        tty_devices = glob.glob('/dev/ttyUSB*')
        
        # Method 2: Using pyserial to get detailed port info
        detailed_ports = []
        for port in serial.tools.list_ports.comports():
            if 'USB' in port.device:
                port_info = f"{port.device} - {port.description}"
                if port.vid is not None and port.pid is not None:
                    port_info += f" (VID:{port.vid:04x} PID:{port.pid:04x})"
                detailed_ports.append(port_info)
        
        print("==== Available USB devices ====")
        if tty_devices:
            print("Simple device listing:")
            for dev in tty_devices:
                print(f"  {dev} - Exists: {os.path.exists(dev)}")
        else:
            print("No /dev/ttyUSB* devices found")
            
        print("\nDetailed port information:")
        if detailed_ports:
            for port_info in detailed_ports:
                print(f"  {port_info}")
        else:
            print("  No USB serial ports detected by pyserial")
        print("===============================")
        
        return tty_devices

    @classmethod
    def force_reset_connection(cls, device_path=None, motor_number=None):
        # Get all available devices before reset for comparison
        before_devices = cls.list_usb_devices()
        
        reset_message = f"Performing system-level USB reset for motor {motor_number or 'unknown'}"
        print(reset_message)
        
        # If no specific device provided, try to find motor controller devices
        if device_path is None:
            # Try to find any ttyUSB device
            usb_devices = glob.glob('/dev/ttyUSB*')
            print(f"Found {usb_devices} USB devices")
            if usb_devices:
                device_path = usb_devices[0]
                print(f"No device specified, using first available: {device_path}")
            else:
                print("WARNING: No USB devices found - cannot attempt any reset methods")
                return False
        
        success = False
        
        if device_path and os.path.exists(device_path):
            try:
                # Find the USB bus and device ID
                result = subprocess.run(
                    f"udevadm info -q path -n {device_path}",
                    shell=True, capture_output=True, text=True
                )
                if result.returncode != 0:
                    print(f"Failed to get device info: {result.stderr}")
                    return False
                
                device_path = result.stdout.strip()
                print(f"USB device path: {device_path}")
                
                # Only use usb_modeswitch method since the other methods don't work
                print("Attempting USB reset using usb_modeswitch")
                try:
                    # Check if usb_modeswitch is available
                    result = subprocess.run(
                        "which usb_modeswitch",
                        shell=True, capture_output=True, text=True
                    )
                    if result.returncode == 0:
                        # Extract bus and device number from path
                        # This is complex and may need adjustment for your system
                        print(f"Attempting reset with usb_modeswitch")
                        result = subprocess.run(
                            f"usb_modeswitch -R -v {MOTOR_CONTROLLER_USB['vendor_id']} -p {MOTOR_CONTROLLER_USB['product_id']}",
                            shell=True, capture_output=True, text=True
                        )
                        if result.returncode == 0:
                            print(f"USB device reset with usb_modeswitch")
                            success = True
                        else:
                            print(f"usb_modeswitch failed: {result.stderr}")
                    else:
                        print("USB reset skipped: usb_modeswitch not found")
                except Exception as e:
                    print(f"Failed to reset using usb_modeswitch: {e}")
                
                # Wait for device to re-enumerate
                time.sleep(3)
                
            except Exception as e:
                print(f"Failed during USB device reset: {e}")
        else:
            print(f"Device path {device_path} not found or not specified - cannot attempt any reset methods")
        
        # Get devices after reset and compare
        after_devices = cls.list_usb_devices()
        
        # Log changes in devices
        added = [d for d in after_devices if d not in before_devices]
        removed = [d for d in before_devices if d not in after_devices]
        
        if added:
            print(f"New devices after reset: {added}")
        if removed:
            print(f"Devices removed after reset: {removed}")
        
        # Try to reconnect a few times after reset
        if success:
            print("USB reset completed, attempting to reconnect...")
        else:
            print("USB reset methods failed, still attempting to reconnect...")
        
        # Send email notification about the reset with all collected logs
        try:
            email = EmailService(log_file_path="output.log")
            email.send_notification(
                subject=f"Motor Controller USB Reset - Motor {motor_number or 'unknown'}",
                message=f"""
                Motor controller communication failure detected.
                Performing system-level USB reset at {time.strftime('%Y-%m-%d %H:%M:%S')}.

                Device path: {device_path or 'unknown'}
                Motor number: {motor_number or 'unknown'}
                Reset success: {success}

                Devices added: {added if added else 'None'}
                Devices removed: {removed if removed else 'None'}

                This email was sent automatically by the motor control system.
                """,
                include_log_lines=True
            )
        except Exception as e:
            print(f"Failed to send email notification: {e}")
        
        # Return the success status of the reset operation
        return success

    def __init__(self, 
                port=None, 
                slave_address=1, 
                baudrate=115200, 
                pulses_per_unit=None, 
                upper_limit_units=None,
                lower_limit_units=None,
                home_position=0,
                velocity=450, 
                acceleration=100, 
                deceleration=100,
                jog_velocity=300,
                current_limit_amperes=0.5,
                homing_current=0.5,
                reset_pin=12):
        
        # If port is not provided, find it automatically
        self.port = port if port is not None else self.find_port()
        if self.port is None:
            raise ValueError("Motor controller port not found")
            
        self.slave_address = slave_address
        self.baudrate = baudrate
        self.home_position = home_position #TODO consider when we need to home to the upper limit
        self.is_homed = False
        self.reset_pin = reset_pin  # Store reset pin
        
        self._setup_drive(self.port, slave_address, baudrate)
        self._setup_motor_parameters(pulses_per_unit, upper_limit_units, lower_limit_units, velocity, acceleration, deceleration, jog_velocity, current_limit_amperes, homing_current)

    @classmethod
    def find_port(cls):
        """Find the serial port for the motor controller."""
        logger.info(f"Searching for motor controller with VID:{MOTOR_CONTROLLER_USB['vendor_id']:04x} PID:{MOTOR_CONTROLLER_USB['product_id']:04x}")
        
        # List all available ports for debugging
        available_ports = []
        for port in serial.tools.list_ports.comports():
            port_info = f"{port.device} - {port.description}"
            if hasattr(port, 'vid') and port.vid is not None and hasattr(port, 'pid') and port.pid is not None:
                port_info += f" (VID:{port.vid:04x} PID:{port.pid:04x})"
                available_ports.append(port_info)
                
                if port.vid == MOTOR_CONTROLLER_USB['vendor_id'] and port.pid == MOTOR_CONTROLLER_USB['product_id']:
                    logger.info(f"Found motor controller at {port.device}")
                    return port.device
            else:
                available_ports.append(port_info)
        
        # If we reach here, no matching device was found
        logger.warning(f"Motor controller not found. Available ports: {available_ports}")
        return None

    # Setup methods (private)
    def _setup_drive(self, port, slave_address, baudrate):
        """Setup the drive with continuous retry until successful."""
        attempt = 1
        while attempt <= self._max_retries:
            try:
                # Check if port exists before attempting connection
                if port is None:
                    # Try to find a valid port
                    port = self.find_port()
                    if port is None:
                        print(f"No valid motor controller port found on attempt {attempt}")
                        # List available USB devices to help diagnose the issue
                        usb_devices = self.list_usb_devices()
                        
                        # Check if any USB devices exist at all
                        if not glob.glob('/dev/ttyUSB*') and attempt == self._max_retries:
                            print("No USB devices detected - aborting connection attempts")
                            raise ConnectionError(f"Failed to connect to motor {slave_address} - no USB devices detected")
                            
                        attempt += 1
                        time.sleep(1)
                        continue

                self.drive = minimalmodbus.Instrument(port, slave_address)
                self.drive.serial.baudrate = baudrate
                self.drive.serial.bytesize = 8
                self.drive.serial.parity = serial.PARITY_NONE
                self.drive.serial.stopbits = 1
                self.drive.serial.timeout = 1
                
                # Test communication
                self._read_register(0x2203)
                print(f"Successfully connected to motor {slave_address}")
                return  # Success
            except Exception as e:
                print(f"Connection attempt {attempt} failed: {str(e)}")
                print(f"Retrying connection to motor {slave_address} in 1 second...")
                # List available USB devices when connection fails
                self.list_usb_devices()
                attempt += 1
                time.sleep(1)
                try:
                    self.drive.serial.close()
                except:
                    pass
                
                # Always find port dynamically on failure
                # This ensures we don't keep trying a disconnected port
                port = self.find_port()
                if port:
                    print(f"Found motor controller on port: {port}")
                    self.port = port  # Update the instance port
                else:
                    print(f"No motor controller found - will retry in 1 second")
                    port = None  # Reset port to force new discovery
                    
        # All normal attempts failed, check for USB devices before trying reset
        usb_devices = glob.glob('/dev/ttyUSB*')
        if not usb_devices:
            print(f"No USB devices detected - skipping system-level reset and further connection attempts")
            raise ConnectionError(f"Failed to connect to motor {slave_address} - no USB devices detected")
            
        # Try nuclear option as last resort
        print(f"All {self._max_retries} connection attempts failed. Attempting system-level USB reset...")
        self.force_reset_connection(device_path=port, motor_number=slave_address)
        
        # After reset, try one final connection attempt
        time.sleep(2)  # Give system time to re-enumerate USB devices
        final_port = self.find_port()
        if final_port:
            print(f"Found motor controller on port {final_port} after reset. Making final connection attempt...")
            try:
                self.drive = minimalmodbus.Instrument(final_port, slave_address)
                self.drive.serial.baudrate = baudrate
                self.drive.serial.bytesize = 8
                self.drive.serial.parity = serial.PARITY_NONE
                self.drive.serial.stopbits = 1
                self.drive.serial.timeout = 1
                
                # Test communication
                self._read_register(0x2203)
                print(f"Successfully connected to motor {slave_address} after system reset")
                self.port = final_port  # Update the instance port
                return  # Success after reset
            except Exception as e:
                print(f"Final connection attempt after reset failed: {str(e)}")
        else:
            print("No motor controller found even after system reset")
            
        # If we reach here, even the nuclear option failed
        raise ConnectionError(f"Failed to connect to motor {slave_address} after {self._max_retries} attempts and system reset")

    def _reconnect(self):
        """Attempt to reconnect to the drive."""
        try:
            self.drive.serial.close()
        except:
            pass
        self._setup_drive(self.port, self.slave_address, self.baudrate)

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
        """Write to register with continuous retry until successful."""
        with self._lock:
            attempt = 1
            while attempt <= 3:  # Limit to 3 attempts
                try:
                    self.drive.write_register(address, value, functioncode=functioncode)
                    time.sleep(0.01)
                    return
                except Exception as e:
                    error_msg = str(e)
                    logger.error(f"Write attempt {attempt}/3 failed: {error_msg}")
                    
                    # Check if this is a USB communication error
                    if "No such file or directory" in error_msg or "No such device" in error_msg or "Input/output error" in error_msg or "could not open port" in error_msg:
                        logger.info(f"USB communication error detected, resetting USB via pin {self.reset_pin}")
                        reset_usb_connection(self.reset_pin)
                        # Allow additional time for complete device enumeration after reset
                        logger.info("Waiting additional time for device enumeration after reset...")
                        time.sleep(2)
                    
                    # Always try to reconnect regardless of error type
                    attempt += 1
                    if attempt <= 3:
                        logger.info(f"Retrying write to motor {self.slave_address}")
                        self._reconnect()
                    else:
                        raise  # Re-raise the last exception after all attempts

    def _read_register(self, address, functioncode=3):
        """Read from register with continuous retry until successful."""
        with self._lock:
            attempt = 1
            while attempt <= 3:  # Limit to 3 attempts
                try:
                    response = self.drive.read_register(address, functioncode=functioncode)
                    time.sleep(0.01)
                    return response
                except Exception as e:
                    error_msg = str(e)
                    logger.error(f"Read attempt {attempt}/3 failed: {error_msg}")
                    
                    # Check if this is a USB communication error
                    if "No such file or directory" in error_msg or "No such device" in error_msg or "Input/output error" in error_msg or "could not open port" in error_msg:
                        logger.info(f"USB communication error detected, resetting USB via pin {self.reset_pin}")
                        reset_usb_connection(self.reset_pin)
                        # Allow additional time for complete device enumeration after reset
                        logger.info("Waiting additional time for device enumeration after reset...")
                        time.sleep(2)
                    
                    # Always try to reconnect regardless of error type
                    attempt += 1
                    if attempt <= 3:
                        logger.info(f"Retrying read from motor {self.slave_address}")
                        self._reconnect()
                    else:
                        raise  # Re-raise the last exception after all attempts

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
        command = 0x4001 
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

    def save_parameters_to_eeprom(self):
        self._write_register(0x1801, 0x2211)

    def set_homing_direction(self, homing_direction):
        homing_direction = homing_direction.upper()
        if homing_direction not in ['CW', 'CCW']:
            raise ValueError("Invalid direction. Please enter 'CW' or 'CCW'.")
        self.homing_direction = homing_direction
        if homing_direction == "CW":
            self._write_register(0x0007, 1) # intentionally flipped
        elif homing_direction == "CCW":
            self._write_register(0x0007, 0) # intentionally flipped