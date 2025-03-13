from bluetooth_hc05 import BluetoothHC05
import pyb
from time import sleep_ms

# Initialize Bluetooth in normal mode (module already set to 115200)
bt = BluetoothHC05(
    at_baud=115200,    # Since we're already configured, use new baud here
    new_baud=115200,   
    name="MyRomiBot",
    password="4321"
)

# Main loop: read sensor values and send them over Bluetooth
while True:
    # Replace these with actual sensor reading functions or variables
    motor_speed = 50           # e.g., motor PWM value or RPM
    encoder_count = 1234       # e.g., cumulative encoder ticks
    ir_sensor = 1              # e.g., 1 if an object is detected, 0 otherwise

    # Format the sensor data as a string (could also use JSON or CSV)
    data_str = "motor:{}, encoder:{}, ir:{}\r\n".format(motor_speed, encoder_count, ir_sensor)

    # Send the data over Bluetooth
    bt.send(data_str)
    print("Sent:", data_str.strip())

    # Optionally, check if there's any data coming in over Bluetooth
    if bt.any():
        incoming = bt.recv(64)
        print("Received:", incoming)

    # Wait a bit before sending the next set of data
    sleep_ms(500)
