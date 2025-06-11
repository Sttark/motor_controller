from MotorController import MotorController
import sys
import curses
import traceback

class CursesMenu:
    def __init__(self, stdscr):
        self.stdscr = stdscr
        self.motors = {}
        self.current_motor = None
        
        # Initialize curses settings
        curses.curs_set(0)  # Hide cursor
        curses.start_color()
        curses.use_default_colors()
        
        # Define color pairs
        curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_WHITE)  # Highlighted
        curses.init_pair(2, curses.COLOR_GREEN, -1)  # Success
        curses.init_pair(3, curses.COLOR_RED, -1)    # Error
        curses.init_pair(4, curses.COLOR_YELLOW, -1) # Warning
        curses.init_pair(5, curses.COLOR_CYAN, -1)   # Info
        
        self.highlight = curses.color_pair(1)
        self.success = curses.color_pair(2)
        self.error = curses.color_pair(3)
        self.warning = curses.color_pair(4)
        self.info = curses.color_pair(5)

    def clear_screen(self):
        """Clear the screen and reset cursor position."""
        self.stdscr.clear()
        self.stdscr.refresh()

    def show_message(self, message, color=None, wait=True):
        """Show a message at the bottom of the screen."""
        height, width = self.stdscr.getmaxyx()
        
        # Clear the message area
        self.stdscr.move(height-2, 0)
        self.stdscr.clrtoeol()
        
        # Display message
        attr = color if color else curses.A_NORMAL
        self.stdscr.addstr(height-2, 0, message[:width-1], attr)
        
        if wait:
            self.stdscr.addstr(height-1, 0, "Press any key to continue...", curses.A_DIM)
            self.stdscr.refresh()
            self.stdscr.getch()
        else:
            self.stdscr.refresh()

    def get_input(self, prompt, y_offset=0):
        """Get user input with a prompt."""
        height, width = self.stdscr.getmaxyx()
        y = height - 4 + y_offset
        
        # Clear input area
        for i in range(3):
            self.stdscr.move(y + i, 0)
            self.stdscr.clrtoeol()
        
        self.stdscr.addstr(y, 0, prompt)
        self.stdscr.refresh()
        
        curses.curs_set(1)  # Show cursor
        curses.echo()       # Echo input
        
        try:
            user_input = self.stdscr.getstr(y + 1, 0, width - 1).decode('utf-8')
        except:
            user_input = ""
        
        curses.noecho()     # Disable echo
        curses.curs_set(0)  # Hide cursor
        
        return user_input.strip()

    def draw_menu(self, title, options, selected_idx):
        """Draw a menu with the given title and options."""
        self.clear_screen()
        height, width = self.stdscr.getmaxyx()
        
        # Draw title
        title_y = 2
        self.stdscr.addstr(title_y, (width - len(title)) // 2, title, curses.A_BOLD | self.info)
        
        # Draw separator
        separator = "=" * min(len(title), width - 4)
        self.stdscr.addstr(title_y + 1, (width - len(separator)) // 2, separator, self.info)
        
        # Draw options
        start_y = title_y + 3
        for i, option in enumerate(options):
            y = start_y + i
            if y >= height - 6:  # Leave space for messages and input
                break
            
            attr = self.highlight if i == selected_idx else curses.A_NORMAL
            self.stdscr.addstr(y, 4, f"{option}", attr)
        
        # Draw instructions
        instructions = "Use ↑/↓ arrows to navigate, ENTER to select, 'q' to quit"
        self.stdscr.addstr(height - 6, 0, instructions, curses.A_DIM)
        
        self.stdscr.refresh()

    def select_from_menu(self, title, options):
        """Display a menu and return the selected index."""
        selected = 0
        max_idx = len(options) - 1
        
        while True:
            self.draw_menu(title, options, selected)
            
            key = self.stdscr.getch()
            
            if key == curses.KEY_UP and selected > 0:
                selected -= 1
            elif key == curses.KEY_DOWN and selected < max_idx:
                selected += 1
            elif key == ord('\n') or key == ord('\r'):
                return selected
            elif key == ord('q') or key == ord('Q'):
                return -1  # Quit signal

    def configure_motors(self):
        """Configure multiple motors with shared parameters."""
        try:
            num_motors = self.get_input("Enter the number of motors you want to configure:")
            if not num_motors:
                return False
            num_motors = int(num_motors)
            
            port = self.get_input("Enter port for all motors (e.g., COM7 or /dev/ttyUSB0):")
            if not port:
                return False
            
            pulses_per_unit = self.get_input("Enter pulses per unit (same for all motors):")
            if not pulses_per_unit:
                return False
            pulses_per_unit = float(pulses_per_unit)
            
            upper_limit_units = self.get_input("Enter upper limit units (same for all motors):")
            if not upper_limit_units:
                return False
            upper_limit_units = float(upper_limit_units)
            
            lower_limit_units = self.get_input("Enter lower limit units (same for all motors):")
            if not lower_limit_units:
                return False
            lower_limit_units = float(lower_limit_units)
            
            # Get optional parameters, let MotorController use defaults if not provided
            velocity_input = self.get_input("Enter velocity in RPM (press Enter for default):")
            velocity = float(velocity_input) if velocity_input.strip() else None
            
            acceleration_input = self.get_input("Enter acceleration in ms/1000rpm (press Enter for default):")
            acceleration = float(acceleration_input) if acceleration_input.strip() else None
            
            deceleration_input = self.get_input("Enter deceleration in ms/1000rpm (press Enter for default):")
            deceleration = float(deceleration_input) if deceleration_input.strip() else None
            
            current_limit_input = self.get_input("Enter current limit in amperes (press Enter for default):")
            current_limit_amperes = float(current_limit_input) if current_limit_input.strip() else None
            
            homing_current_input = self.get_input("Enter homing current in amperes (press Enter for default):")
            homing_current = float(homing_current_input) if homing_current_input.strip() else None
            
            # Build MotorController kwargs, only including non-None values
            motor_kwargs = {
                'port': port,
                'pulses_per_unit': pulses_per_unit,
                'upper_limit_units': upper_limit_units,
                'lower_limit_units': lower_limit_units,
                'home_position': 0
            }
            
            # Only add optional parameters if user provided them
            if velocity is not None:
                motor_kwargs['velocity'] = velocity
            if acceleration is not None:
                motor_kwargs['acceleration'] = acceleration
            if deceleration is not None:
                motor_kwargs['deceleration'] = deceleration
            if current_limit_amperes is not None:
                motor_kwargs['current_limit_amperes'] = current_limit_amperes
            if homing_current is not None:
                motor_kwargs['homing_current'] = homing_current
            
            self.motors.clear()
            for i in range(1, num_motors + 1):
                self.motors[str(i)] = MotorController(
                    slave_address=i,
                    **motor_kwargs
                )
            
            # Show configuration summary
            summary_lines = [
                f"Successfully configured {num_motors} motors:",
                f"Port: {port}",
                f"Pulses per unit: {pulses_per_unit}",
                f"Upper limit: {upper_limit_units}",
                f"Lower limit: {lower_limit_units}"
            ]
            
            # Only show parameters that were explicitly set
            if velocity is not None:
                summary_lines.append(f"Velocity: {velocity} RPM")
            if acceleration is not None:
                summary_lines.append(f"Acceleration: {acceleration} ms/1000rpm")
            if deceleration is not None:
                summary_lines.append(f"Deceleration: {deceleration} ms/1000rpm")
            if current_limit_amperes is not None:
                summary_lines.append(f"Current limit: {current_limit_amperes} A")
            if homing_current is not None:
                summary_lines.append(f"Homing current: {homing_current} A")
                
            summary_lines.append("(Other parameters using MotorController defaults)")
            
            summary = "\n".join(summary_lines)
            self.show_message(summary, self.success)
            return True
            
        except ValueError as e:
            self.show_message(f"Invalid input: {e}", self.error)
            return False
        except Exception as e:
            self.show_message(f"Error: {e}", self.error)
            return False

    def home_all_motors(self):
        """Home all motors to mechanical position."""
        if not self.motors:
            self.show_message("No motors configured. Please configure motors first.", self.error)
            return
        
        direction = self.get_input("Enter homing direction for all motors (CW/CCW):")
        if direction.upper() not in ['CW', 'CCW']:
            self.show_message("Invalid direction. Please enter CW or CCW.", self.error)
            return
        
        try:
            for motor_num, controller in self.motors.items():
                home_pos = self.get_input(f"Enter home position for Motor {motor_num} (or press Enter for 0):")
                home_position = float(home_pos) if home_pos.strip() else 0
                controller.home_position = home_position
                
                self.show_message(f"Homing Motor {motor_num}...", self.info, wait=False)
                controller.home_to_mechanical(direction.upper())
            
            self.show_message("All motors homed successfully", self.success)
        except Exception as e:
            self.show_message(f"Error homing motors: {e}", self.error)

    def motor_control_menu(self, motor_num):
        """Control menu for a specific motor."""
        controller = self.motors[motor_num]
        
        options = [
            "Back to main menu",
            "Move Clockwise (Continuous)",
            "Move Counter-clockwise (Continuous)",
            "Home to mechanical",
            "Read position",
            "Read register",
            "Write register", 
            "Clear alarm",
            "Move relative",
            "Move absolute",
            "Set current position as origin",
            "Set current limit",
            "Save parameters to EEPROM"
        ]
        
        while True:
            title = f"MOTOR {motor_num} CONTROL MENU"
            choice = self.select_from_menu(title, options)
            
            if choice == -1 or choice == 0:  # Quit or back
                break
            
            try:
                if choice == 1:  # Clockwise continuous
                    self.continuous_movement_menu(controller, motor_num, "Clockwise", 0x4001)
                elif choice == 2:  # Counter-clockwise continuous
                    self.continuous_movement_menu(controller, motor_num, "Counter-clockwise", 0x4002)
                elif choice == 3:  # Home to mechanical
                    direction = self.get_input("Give direction (CW/CCW):")
                    if direction.upper() in ['CW', 'CCW']:
                        controller.home_to_mechanical(direction.upper())
                        self.show_message("Homing completed", self.success)
                    else:
                        self.show_message("Invalid direction", self.error)
                elif choice == 4:  # Read position
                    pos = controller.current_position
                    self.show_message(f"Current position: {pos}", self.info)
                elif choice == 5:  # Read register
                    reg_addr = self.get_input("Enter register address (hex, e.g., 0x1801):")
                    try:
                        addr = int(reg_addr, 16) if reg_addr.startswith('0x') else int(reg_addr, 16)
                        value = controller._read_register(addr)
                        self.show_message(f"Register {reg_addr}: {value} (0x{value:04X})", self.info)
                    except ValueError:
                        self.show_message("Invalid register address", self.error)
                elif choice == 6:  # Write register
                    reg_addr = self.get_input("Enter register address (hex, e.g., 0x1801):")
                    reg_value = self.get_input("Enter value to write (hex, e.g., 0x4001):")
                    try:
                        addr = int(reg_addr, 16) if reg_addr.startswith('0x') else int(reg_addr, 16)
                        value = int(reg_value, 16) if reg_value.startswith('0x') else int(reg_value)
                        controller._write_register(addr, value)
                        self.show_message(f"Written {value} to register {reg_addr}", self.success)
                    except ValueError:
                        self.show_message("Invalid address or value", self.error)
                elif choice == 7:  # Clear alarm
                    controller.clear_alarm()
                    self.show_message("Alarm cleared", self.success)
                elif choice == 8:  # Move relative
                    distance = self.get_input("Enter relative distance to move:")
                    try:
                        dist = float(distance)
                        controller.move_relative(dist)
                        self.show_message(f"Moving relative distance: {dist}", self.success)
                    except ValueError:
                        self.show_message("Invalid distance", self.error)
                elif choice == 9:  # Move absolute
                    position = self.get_input("Enter absolute position to move to:")
                    try:
                        pos = float(position)
                        controller.move_absolute(pos)
                        self.show_message(f"Moving to absolute position: {pos}", self.success)
                    except ValueError:
                        self.show_message("Invalid position", self.error)
                elif choice == 10:  # Set current position as origin
                    controller._set_current_position_as_zero()
                    self.show_message("Current position set as zero", self.success)
                elif choice == 11:  # Set current limit
                    current_limit = self.get_input("Enter new current limit (in amperes):")
                    try:
                        limit = float(current_limit)
                        controller.set_current_limit(limit)
                        self.show_message(f"Current limit set to {limit} A", self.success)
                    except ValueError:
                        self.show_message("Invalid current limit", self.error)
                elif choice == 12:  # Save parameters to EEPROM
                    controller.save_parameters_to_eeprom()
                    self.show_message("Parameters saved to EEPROM", self.success)
                    
            except Exception as e:
                self.show_message(f"Error: {e}", self.error)

    def continuous_movement_menu(self, controller, motor_num, direction, command):
        """Continuous movement menu for clockwise/counter-clockwise."""
        options = [
            "Back to motor menu",
            f"Move {direction}"
        ]
        
        while True:
            title = f"MOTOR {motor_num} - {direction.upper()} MODE"
            choice = self.select_from_menu(title, options)
            
            if choice == -1 or choice == 0:  # Quit or back
                break
            elif choice == 1:  # Move
                try:
                    controller._write_register(0x1801, command)
                    self.show_message(f"Moving {direction.lower()}...", self.info, wait=False)
                except Exception as e:
                    self.show_message(f"Error: {e}", self.error)

    def select_motor_menu(self):
        """Menu to select which motor to control."""
        if not self.motors:
            self.show_message("No motors configured. Please configure motors first.", self.error)
            return
        
        motor_options = [f"Motor {num}" for num in self.motors.keys()]
        motor_options.append("Back to main menu")
        
        choice = self.select_from_menu("SELECT MOTOR", motor_options)
        
        if choice == -1 or choice == len(motor_options) - 1:  # Quit or back
            return
        
        motor_num = list(self.motors.keys())[choice]
        self.motor_control_menu(motor_num)

    def main_menu(self):
        """Main menu loop."""
        # Initial motor configuration
        self.show_message("Welcome! Please configure your motors first.", self.info)
        while not self.motors:
            if not self.configure_motors():
                continue
        
        options = [
            "Reconfigure motors",
            "Home all motors to mechanical", 
            "Select a motor",
            "Quit program"
        ]
        
        while True:
            choice = self.select_from_menu("MAIN MENU", options)
            
            if choice == -1 or choice == 3:  # Quit
                break
            elif choice == 0:  # Reconfigure motors
                self.configure_motors()
            elif choice == 1:  # Home all motors
                self.home_all_motors()
            elif choice == 2:  # Select a motor
                self.select_motor_menu()

def curses_main(stdscr):
    """Main curses function."""
    try:
        menu = CursesMenu(stdscr)
        menu.main_menu()
    except Exception as e:
        # Ensure we can see errors even if curses fails
        curses.endwin()
        print(f"Error in curses application: {e}")
        print(traceback.format_exc())
        raise

def main():
    """Main entry point."""
    try:
        curses.wrapper(curses_main)
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"Fatal error: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()
