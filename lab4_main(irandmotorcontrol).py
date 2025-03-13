"""
main.py

Lab 0x04 – Line Following (No IMU)

Overview:
  This program uses motor and encoder drivers from previous labs along with a new driver
  for the QTRX-MD-08A IR Sensor Array and a closed-loop controller to enable line following.
  The robot drives in reverse by default; when the IR sensor detects a deviation from the center,
  the closed-loop controller adjusts motor commands so that the robot follows the line indefinitely.
  
  Note: Calibration has been performed on the IR sensor and the calibration parameters below
        are used to normalize the raw ADC values into a 0–1 range.
        
File Organization:
  - motor.py: Motor driver class.
  - encoder.py: Encoder driver class.
  - ir_sensor.py: Driver for the 8-channel IR sensor array (computes a normalized centroid).
  - closed_loop.py: Implements a simple PID controller for line following.
  - main.py: Instantiates drivers, creates tasks, and runs the cooperative scheduler indefinitely.
            The program waits for the user to press Enter to start.
            
Pin Assignments:
  - Left Motor: PWM: "A9", Direction: "C7", Enable: "B6"
  - Left Encoder: Channel A: "A15", Channel B: "B3"
  - Right Motor: PWM: "B1", Direction: "B15", Enable: "B2"
  - Right Encoder: Channel A: "A0", Channel B: "A1"
  - IR Sensor Array:
       Channels: "PC4", "PA5", "PC5", "PA6", "PB0", "PC1", "PA4", "PC0"
       Vcc: 3.3V, GND: Ground

Note: The IMU is not used in this lab.
"""

import gc
import utime
import cotask
import task_share
from motor import Motor
from encoder import Encoder
from ir_sensor import IRSensor
from closed_loop import ClosedLoopController
from pyb import Pin, Timer

# --- Calibration Parameters for IR Sensor ---
# These values are obtained from your calibration procedure.
calibration_min = [268, 217, 239, 217, 239, 243, 244, 247]   # White surface readings
calibration_max = [3609, 2114, 3031, 3314, 3037, 3148, 3147, 3157]   # Black surface readings

# --- Shared Variables ---
left_encoder_count = task_share.Share("i", name="LeftEncoderCount")
left_velocity      = task_share.Share("i", name="LeftVelocity")
left_motor_command = task_share.Share("i", name="LeftMotorCommand")
right_encoder_count = task_share.Share("i", name="RightEncoderCount")
right_velocity      = task_share.Share("i", name="RightVelocity")
right_motor_command = task_share.Share("i", name="RightMotorCommand")
ir_line_centroid    = task_share.Share("f", name="IRLineCentroid")

# --- Hardware Instantiation ---
left_motor = Motor(timer=Timer(1, freq=25000), channel=2, pwm_pin=Pin("A9"), dir_pin=Pin("C7"), nslp_pin=Pin("B6"))
left_encoder = Encoder(tim=2, chA_pin=Pin("A15"), chB_pin=Pin("B3"))
right_motor = Motor(timer=Timer(3, freq=25000), channel=4, pwm_pin=Pin("B1"), dir_pin=Pin("B15"), nslp_pin=Pin("B2"))
right_encoder = Encoder(tim=5, chA_pin=Pin("A0"), chB_pin=Pin("A1"))
ir_sensor = IRSensor(channels=[Pin("PC4"), Pin("PA5"), Pin("PC5"), Pin("PA6"),
                                Pin("PB0"), Pin("PC1"), Pin("PA4"), Pin("PC0")],
                     calibration_min=calibration_min,
                     calibration_max=calibration_max)
# Instantiate the PID controller (proportional only in this case).
controller = ClosedLoopController(setpoint=0.5, kp=10, ki=0, kd=0)

# --- Task Functions ---

def encoder_task_left():
    while True:
        left_encoder.update()
        left_encoder_count.put(left_encoder.get_position())
        left_velocity.put(int(left_encoder.get_velocity()))
        yield 0

def encoder_task_right():
    while True:
        right_encoder.update()
        right_encoder_count.put(right_encoder.get_position())
        right_velocity.put(int(right_encoder.get_velocity()))
        yield 0

def motor_control_task_left():
    while True:
        cmd = left_motor_command.get()
        left_motor.set_effort(cmd)
        yield 0

def motor_control_task_right():
    while True:
        cmd = right_motor_command.get()
        right_motor.set_effort(cmd)
        yield 0

def ir_sensor_task():
    """
    Reads the IR sensor array and updates the IRLineCentroid shared variable.
    Runs every 50 ms.
    """
    while True:
        centroid = ir_sensor.compute_centroid()
        ir_line_centroid.put(centroid)
        yield 0

def closed_loop_task():
    """
    Implements closed-loop line following with a low-pass filter and a deadband.
    
    Process:
      1. Read the raw IR centroid and apply a low-pass filter to smooth out noise.
      2. Compute the error as the difference between the desired setpoint (0.5) and the filtered reading.
      3. If the error is within a small deadband, no correction is applied.
      4. Otherwise, compute a PID correction (proportional only here).
      5. Multiply the PID correction by a multiplier to amplify its effect.
      6. Adjust motor commands using separate base efforts and additional offsets:
           left_motor_command = base_left + left_offset + amplified_correction
           right_motor_command = base_right + right_offset - amplified_correction
         This allows independent tuning for each motor.
    
    Tuning Parameters:
      - alpha: Low-pass filter coefficient.
      - deadband: Minimum error threshold.
      - correction_multiplier: Factor to amplify the PID output.
      - base_left, base_right: Base motor commands when no correction is needed.
      - left_offset, right_offset: Additional offsets to balance the motors.
    """
    # Initialize filter and tuning parameters.
    filtered_centroid = 0.5  # Start at the desired setpoint.
    alpha = 0.8              # Filter coefficient.
    deadband = 0.01          # Deadband threshold.
    correction_multiplier = 3.0  # Amplify the PID output.
    
    # Define base motor commands (assumed to be in reverse) and additional offsets.
    base_left = -10    # Base command for left motor.
    base_right = -20   # Base command for right motor.
    left_offset = -5   # Additional offset to make left motor spin faster.
    right_offset = 5   # Additional offset to make right motor spin slower.
    
    while True:
        # 1. Read raw centroid from the IR sensor.
        centroid_raw = ir_line_centroid.get()
        # 2. Apply low-pass filter.
        filtered_centroid = alpha * filtered_centroid + (1 - alpha) * centroid_raw
        # 3. Compute error.
        error = controller.setpoint - filtered_centroid
        # 4. Apply deadband.
        if abs(error) < deadband:
            correction = 0
        else:
            correction = controller.compute(filtered_centroid)
        # 5. Amplify correction.
        amplified_correction = correction_multiplier * correction
        
        # Debug print.
        print("Filtered IR Centroid: {:.3f}, Amplified Correction: {:.2f}".format(filtered_centroid, amplified_correction))
        
        # 6. Compute motor commands using separate base commands and offsets.
        left_command = base_left + left_offset + amplified_correction
        right_command = base_right + right_offset - amplified_correction
        
        left_motor_command.put(int(left_command))
        right_motor_command.put(int(right_command))
        yield 0

# --- Task Creation ---
encoder_task_L = cotask.Task(encoder_task_left, name="EncoderTaskL", priority=3, period=20, profile=True)
encoder_task_R = cotask.Task(encoder_task_right, name="EncoderTaskR", priority=3, period=20, profile=True)
motor_control_task_L = cotask.Task(motor_control_task_left, name="MotorControlTaskL", priority=2, period=20, profile=True)
motor_control_task_R = cotask.Task(motor_control_task_right, name="MotorControlTaskR", priority=2, period=20, profile=True)
ir_sensor_task_obj = cotask.Task(ir_sensor_task, name="IRSensorTask", priority=2, period=50, profile=True)
closed_loop_task_obj = cotask.Task(closed_loop_task, name="ClosedLoopTask", priority=2, period=50, profile=True)

cotask.task_list.append(encoder_task_L)
cotask.task_list.append(encoder_task_R)
cotask.task_list.append(motor_control_task_L)
cotask.task_list.append(motor_control_task_R)
cotask.task_list.append(ir_sensor_task_obj)
cotask.task_list.append(closed_loop_task_obj)

# --- Main Execution ---
print("Press Enter to start closed-loop line following...")
input()  # Wait for user input to start
print("Starting closed-loop line following...")

left_motor.enable()
left_motor_command.put(0)
right_motor.enable()
right_motor_command.put(0)
gc.collect()

while True:
    cotask.task_list.pri_sched()
    gc.collect()
