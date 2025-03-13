# test_imu_debug.py
import utime
from pyb import I2C
from imu import BNO055

# Create an I2C instance on bus 2 (make sure your wiring is correct).
i2c = I2C(2, I2C.MASTER, baudrate=400000)

# Instantiate the BNO055 IMU.
try:
    imu = BNO055(i2c, address=0x28)
except Exception as e:
    print("Failed to initialize IMU:", e)
    raise

# Run debug output every second.
while True:
    imu.debug()
    utime.sleep(1)
