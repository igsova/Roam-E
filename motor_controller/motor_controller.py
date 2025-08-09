import serial
import time

class MotorController:
    def __init__(self, port='/dev/ttyUSB1', baudrate=115200):
        """
        Initializes the connection to the Arduino.
        The port might be /dev/ttyACM0 on some systems.
        Check your Raspberry Pi's connected devices with `ls /dev/tty*`
        """
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2) # Wait for the connection to establish
            print(f"Successfully connected to Arduino on {port}")
        except serial.SerialException as e:
            print(f"Error connecting to Arduino: {e}")
            self.ser = None

    def send_command(self, command):
        if not self.ser:
            print("Serial not connected.")
            return None
        self.ser.write((command + '\n').encode('utf-8'))
        response = self.ser.readline().decode('utf-8').strip()
        return response

    def get_encoders(self):
        """Requests and returns the current encoder counts."""
        response = self.send_command("GET_ENCODERS")
        if response:
            try:
                pos_a, pos_b = map(int, response.split(','))
                return pos_a, pos_b
            except (ValueError, IndexError):
                print(f"Could not parse encoder data: {response}")
                return None, None
        return None, None

    def set_motor(self, motor, speed, direction):
        """
        Sets the speed and direction of a motor.
        motor: 'A' or 'B'
        speed: 0-255
        direction: 1 for forward, -1 for backward, 0 for stop
        """
        command = f"SET_MOTOR,{motor.upper()},{int(speed)},{int(direction)}"
        self.send_command(command)

    def stop_motors(self):
        """Stops both motors."""
        self.set_motor('A', 0, 0)
        self.set_motor('B', 0, 0)

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Serial connection closed.")


if __name__ == '__main__':
    # Find the correct port for your Arduino Nano
    # Run `ls /dev/tty*` on your Pi to see available ports. It's often /dev/ttyUSB0 or /dev/ttyACM0
    controller = MotorController(port='/dev/ttyUSB1')

    if controller.ser:
        try:
            print("Reading initial encoder values...")
            encoders = controller.get_encoders()
            if encoders[0] is not None:
                print(f"Initial Encoders -> A: {encoders[0]}, B: {encoders[1]}")

            print("\nRotate the motor shafts by hand and see the values change.")
            for _ in range(5):
                encoders = controller.get_encoders()
                if encoders[0] is not None:
                    print(f"Encoders -> A: {encoders[0]}, B: {encoders[1]}")
                time.sleep(1)

            print("\n--- Motor Test ---")
            print("This will only work if the 12V motor power is connected.")

            print("Running Motor A forward at half speed for 2 seconds...")
            controller.set_motor('A', 128, 1)
            time.sleep(2)

            print("Running Motor B backward at full speed for 2 seconds...")
            controller.set_motor('B', 255, -1)
            time.sleep(2)

            print("Stopping motors.")
            controller.stop_motors()
            time.sleep(1)

            final_encoders = controller.get_encoders()
            if final_encoders[0] is not None:
                print(f"Final Encoders -> A: {final_encoders[0]}, B: {final_encoders[1]}")

        except KeyboardInterrupt:
            print("\nExiting program.")
        finally:
            controller.stop_motors()
            controller.close()

