"""
lab5_main.py

Lab 0x05 – Inertial Measurement Units and I2C

Overview:
  This program uses the BNO055 IMU to pivot Romi until it is pointing due-north.
  The IMU communicates over I2C (using I2C bus 2 with SCL on PB10 and SDA on PB11).
  Two tasks are created:
    - The IMU task reads Euler angles (specifically the heading) and applies a 180° offset,
      then converts the heading into the range [-180, 180], updating a shared variable.
    - The Pivot task uses a proportional controller to command the motors to pivot until the
      corrected heading is within a 5° tolerance of 0°.
      
Pin Assignments:
  - Left Motor: PWM: "A9", Direction: "C7", Enable: "B6"
  - Right Motor: PWM: "B1", Direction: "B15", Enable: "B2"
  - IMU (BNO055): Uses I2C Bus 2, SCL on "PB10", SDA on "PB11"
"""

import gc
import utime
import cotask
import task_share
from motor import Motor
from imu import BNO055
from pyb import Pin, Timer, I2C

def angle_diff(target, current):
    """
    Computes the minimal difference between two angles (in degrees), accounting for wrap-around.
    """
    diff = target - current
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360
    return diff

def wrap_to_180(angle):
    """
    Wraps any angle (in degrees) to the range [-180, 180].
    For example, an input of 270 becomes -90.
    """
    return ((angle + 180) % 360) - 180

# --- Shared Variable ---
imu_heading = task_share.Share("f", name="IMUHeading")

# --- Hardware Instantiation ---
# Create an I2C instance on bus 2 (SCL on PB10, SDA on PB11).
i2c = I2C(2, I2C.MASTER, baudrate=400000)
# Instantiate the BNO055 IMU.
imu = BNO055(i2c, address=0x28)
# Instantiate motors.
left_motor = Motor(timer=Timer(1, freq=25000), channel=2, pwm_pin=Pin("A9"), dir_pin=Pin("C7"), nslp_pin=Pin("B6"))
right_motor = Motor(timer=Timer(3, freq=25000), channel=4, pwm_pin=Pin("B1"), dir_pin=Pin("B15"), nslp_pin=Pin("B2"))

# --- Heading Offset ---
# Our IMU appears to be mounted "backwards" so that it reads ~180° off.
# We subtract 180° and then wrap the result to the range [-180, 180].
heading_offset = 180

# --- Task Functions ---
def imu_task():
    """
    Reads Euler angles from the IMU every 50 ms, applies a 180° offset,
    and converts the result into the range [-180, 180]. The corrected heading
    is then stored in a shared variable.
    """
    while True:
        heading, pitch, roll = imu.get_euler()
        # Apply the 180° offset.
        corrected = heading - heading_offset
        # Wrap the corrected heading into [-180, 180].
        corrected_heading = wrap_to_180(corrected)
        imu_heading.put(corrected_heading)
        yield 0

def pivot_task():
    """
    Pivots Romi in place until the corrected heading is within a 5° tolerance of 0° (due-north).
    
    Process:
      1. Retrieve the corrected heading from the shared variable.
      2. Compute the error (difference from 0°) using angle_diff().
      3. Compute a proportional command with reduced gain for slower pivoting.
      4. Clamp the command to a limited range.
      5. Command the left motor with the positive command and the right motor with the negative.
    """
    desired_heading = 0.0  # Target heading (due-north after correction)
    tolerance = 5.0        # 5° tolerance
    k_p = 0.5              # Proportional gain (reduced for a slower response)
    max_cmd = 20           # Maximum command value
    while True:
        current_heading = imu_heading.get()
        error = angle_diff(desired_heading, current_heading)
        if abs(error) < tolerance:
            cmd = 0
        else:
            cmd = k_p * error
            if cmd > max_cmd:
                cmd = max_cmd
            elif cmd < -max_cmd:
                cmd = -max_cmd
        left_motor.set_effort(int(cmd))
        right_motor.set_effort(int(-cmd))
        print("Corrected Heading: {:.2f}°, Error: {:.2f}°, Command: {:.2f}".format(current_heading, error, cmd))
        yield 0

# --- Task Creation ---
imu_task_obj = cotask.Task(imu_task, name="IMUTask", priority=3, period=50, profile=True)
pivot_task_obj = cotask.Task(pivot_task, name="PivotTask", priority=2, period=50, profile=True)

cotask.task_list.append(imu_task_obj)
cotask.task_list.append(pivot_task_obj)

# --- Main Execution ---
print("Press Enter to start pivot control...")
input()  # Wait for user input to start
print("Starting pivot control. Romi will pivot until it is pointing due-north.")

left_motor.enable()
right_motor.enable()
gc.collect()

while True:
    cotask.task_list.pri_sched()
    gc.collect()
