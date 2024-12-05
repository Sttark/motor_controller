from MotorController import MotorController
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def interactive_mode(motors):
    current_motor = 'X'  # Default to X motor
    while True:
        try:
            controller = motors[current_motor]
            prompt = f"Motor {current_motor} | Enter command (X/Y/Z=Switch Motor, C=Clockwise, A=CCW, T=Home to Mechanical, P=Read Position, R=Read Register, W=Write Register, F=Clear Alarm, M=Move relative, L=Move absolute, S=Set current position as origin, Q=Quit): "
            command = input(prompt).upper()

            if command in ['X', 'Y', 'Z']:
                current_motor = command
                logging.info(f"Switched to Motor {current_motor}")
                continue
            elif command == 'Q':
                logging.info("Exiting program.")
                break
            elif command == 'T':
                dir = input("Give direction (CW/CCW): ").upper()
                controller.home_to_mechanical(dir)
            elif command == 'P':
                print(f"Current position: {controller.current_position}")
            elif command == 'M':
                distance = float(input("Enter relative distance to move: "))
                controller.move_relative(distance)
            elif command == 'R':
                reg_address = int(input("Enter the Modbus register address (in hexadecimal): "), 16)
                value = controller._read_register(reg_address)
                print(f"Register value: {value}")
            elif command == 'W':
                reg_address = int(input("Enter the Modbus register address to write to (in hexadecimal): "), 16)
                value_input = input("Enter the value to write to the register (in hexadecimal): ")
                value = int(value_input, 16) if value_input.startswith("0x") else int(value_input)
                controller._write_register(reg_address, value)
            elif command == 'F':
                controller.clear_alarm()
            elif command in ['C', 'A']:
                controller._write_register(0x1801, 0x4001) if command == 'C' else controller._write_register(0x1801, 0x4002)
            elif command == 'L':
                position = float(input("Enter absolute position to move to: "))
                controller.move_absolute(position)
            elif command == 'I':
                current_limit = float(input("Enter new current limit (in amperes): "))
                controller.set_current_limit(current_limit)
                logging.info(f"Current limit set to {current_limit} A")
            elif command == 'S':
                controller._set_current_position_as_zero()
                logging.info("Current position set as zero.")
            else:
                logging.error("Invalid command. Please try again.")
        except ValueError as ve:
            logging.error(f"Invalid input: {ve}. Please try again.")
        except RuntimeError as re:
            logging.error(f"Runtime error: {re}. Please try again.")
        except Exception as e:
            logging.error(f"An error occurred: {e}. Please try again.")

def main():
    motor_x = MotorController(port='COM7', slave_address=1, pulses_per_unit=500, upper_limit_units=350, lower_limit_units=0, home_position=0)
    motor_y = MotorController(port='COM7', slave_address=2, pulses_per_unit=500, upper_limit_units=300, lower_limit_units=0, home_position=0)
    motor_z = MotorController(port='COM7', slave_address=3, pulses_per_unit=500, upper_limit_units=50, lower_limit_units=0, home_position=0)

    motors = {'X': motor_x, 'Y': motor_y, 'Z': motor_z}

    motor_z.home_to_mechanical('CW')
    motor_x.home_to_mechanical('CW')
    motor_y.home_to_mechanical('CW')

    print("Test update for Nathan and Cole - 2")

    interactive_mode(motors)

if __name__ == "__main__":
    main()
