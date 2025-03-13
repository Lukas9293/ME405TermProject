"""
term_project_main.py

Term Project 0x02 – Romi Time‑trials on a 6ft x 3ft Track

Overview:
  This program integrates our previously-developed drivers:
    - Motor and Encoder (for wheel control and odometry)
    - IRSensor (for line following using calibrated IR sensor readings)
    - BNO055 (IMU for heading and orientation via I2C)
    - Bumper (to detect collisions and trigger a backup maneuver)
  In addition, it uses our cooperative multitasking framework (cotask and task_share)
  to coordinate multiple tasks. The behavior is divided into 9 sections based on encoder ticks
  (instead of elapsed time). In addition, if a bumper is pressed or the robot is stationary 
  (motors active but no encoder change) for 2 seconds, the robot will execute a backup maneuver.
  
Pin Assignments:
  - Left Motor: PWM: "A9", Direction: "C7", Enable: "B6"
  - Left Encoder: Channel A: "A15", Channel B: "B3"
  - Right Motor: PWM: "B1", Direction: "B15", Enable: "B2"
  - Right Encoder: Channel A: "A0", Channel B: "A1"
  - IR Sensor Array:
       Channels: "PC4", "PA5", "PC5", "PA6", "PB0", "PC1", "PA4", "PC0"
       Vcc: 3.3V, GND: Ground
  - Bumpers:
       Left: Three sensors on ["PB4", "PB14", "PB13"]  
              (Note: Order adjusted so that BMP5 = PB13, BMP4 = PB14, BMP3 = PB4)
       Right: Three sensors on ["PB7", "PC2", "PC3"] (active low)

Note: The IMU is not used.
"""

import gc
import utime
import cotask
import task_share
from pyb import Pin, Timer

# Import driver classes.
from motor import Motor
from encoder import Encoder
from ir_sensor import IRSensor
from closed_loop import ClosedLoopController
from bumper import Bumper

# --- Calibration Parameters for IR Sensor ---
calibration_min = [337, 326, 317, 325, 311, 316, 318, 324]   # White surface readings
calibration_max = [3869, 2099, 3218, 3340, 3353, 3237, 3325, 3471]  # Black surface readings

# --- Shared Variables ---
left_encoder_count  = task_share.Share("i", name="LeftEncoderCount")
left_velocity       = task_share.Share("i", name="LeftVelocity")
left_motor_command  = task_share.Share("i", name="LeftMotorCommand")
right_encoder_count = task_share.Share("i", name="RightEncoderCount")
right_velocity      = task_share.Share("i", name="RightVelocity")
right_motor_command = task_share.Share("i", name="RightMotorCommand")
ir_line_centroid    = task_share.Share("f", name="IRLineCentroid")
switch_state        = task_share.Share("i", name="SwitchState")

# Global flag for backup mode.
backup_active = False

# Global variable for encoder baseline (to “reset” our count for closed-loop sections).
encoder_baseline = 0

# --- Hardware Instantiation ---
left_motor = Motor(timer=Timer(1, freq=25000), channel=2,
                   pwm_pin=Pin("A9"), dir_pin=Pin("C7"), nslp_pin=Pin("B6"))
left_encoder = Encoder(tim=2, chA_pin=Pin("A15"), chB_pin=Pin("B3"))

right_motor = Motor(timer=Timer(3, freq=25000), channel=4,
                    pwm_pin=Pin("B1"), dir_pin=Pin("B15"), nslp_pin=Pin("B2"))
right_encoder = Encoder(tim=5, chA_pin=Pin("A0"), chB_pin=Pin("A1"))

ir_sensor = IRSensor(channels=[Pin("PC4"), Pin("PA5"), Pin("PC5"), Pin("PA6"),
                               Pin("PB0"), Pin("PC1"), Pin("PA4"), Pin("PC0")],
                     calibration_min=calibration_min,
                     calibration_max=calibration_max)

# Instantiate bumpers using three sensors per side.
# For left bumper, order so that BMP5 = "PB13", BMP4 = "PB14", BMP3 = "PB4"
left_bumper = Bumper(["PB13", "PB14", "PB4"])
# For right bumper, order: BMP0 = "PB7", BMP1 = "PC2", BMP2 = "PC3"
right_bumper = Bumper(["PB7", "PC2", "PC3"])

# Instantiate the PID controller with initial gains.
# The setpoint is 0.5 (the center of the normalized sensor range).
controller = ClosedLoopController(setpoint=0.5, kp=15, ki=0, kd=0)

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
    Reads the IR sensor array every 50 ms and updates the shared IRLineCentroid.
    """
    while True:
        centroid = ir_sensor.compute_centroid()
        ir_line_centroid.put(centroid)
        yield 0

def closed_loop_task():
    """
    Implements closed-loop line following with sections determined by left encoder ticks.
    The section thresholds are based on the change in left encoder ticks.
    If backup_active is True, this task skips updating motor commands.
    """
    global backup_active, encoder_baseline
    filtered_centroid = 0.5  # Initialize low-pass filter variable.
    while True:
        # If a backup maneuver is active, skip closed-loop processing.
        if backup_active:
            yield 0
            continue

        # Check if the robot is enabled.
        if switch_state.get() == 0:
            left_motor_command.put(0)
            right_motor_command.put(0)
            yield 0
            continue

        current_ticks = left_encoder_count.get()
        elapsed_ticks = current_ticks - encoder_baseline  # Compute elapsed ticks relative to baseline

        # Determine section based on elapsed ticks.
        if elapsed_ticks < 6700:  # straight: l=-10, r=-21
            alpha = 0.8; deadband = 0.01; correction_multiplier = 3.0
            base_left = -10; base_right = -21; left_offset = -5; right_offset = 5
            controller.kp = 12; controller.ki = 0; controller.kd = 0.01
            section = 1
            section_name = 'straight'
        elif elapsed_ticks < 7725:
            alpha = 0.8; deadband = 0.01; correction_multiplier = 3.0
            base_left = -10; base_right = -21; left_offset = -5; right_offset = 5
            controller.kp = 0; controller.ki = 0.000001; controller.kd = 0.001
            section = 2
            section_name = 'diamond'
        elif elapsed_ticks < 30100:  # straight: l=-10, r=-21
            alpha = 0.8; deadband = 0.01; correction_multiplier = 3.0
            base_left = -10; base_right = -22; left_offset = -5; right_offset = 5
            controller.kp = 14; controller.ki = 0; controller.kd = 0.001
            section = 3    
            section_name = 'curve and structure'
        elif elapsed_ticks < 30850:  # right turn: l=-20, r=15, ticks = 850
            alpha = 0; deadband = 0.01; correction_multiplier = 3.0
            base_left = -20; base_right = 15; left_offset = -5; right_offset = 5
            controller.kp = 10; controller.ki = 0; controller.kd = 0
            section = 4 
            section_name = 'structure right turn'
        elif elapsed_ticks < 35000:  
            alpha = 0; deadband = 0.01; correction_multiplier = 3.0
            base_left = -10; base_right = -21; left_offset = -5; right_offset = 5
            controller.kp = 10; controller.ki = 0; controller.kd = 0
            section = 5 
            section_name = 'wall bump'  
        elif elapsed_ticks < 35850:  # right turn: l=-20, r=15, ticks = 850
            alpha = 0; deadband = 0.01; correction_multiplier = 3.0
            base_left = -20; base_right = 15; left_offset = -5; right_offset = 5
            controller.kp = 10; controller.ki = 0; controller.kd = 0
            section = 6
            section_name = 'wall right'
        elif elapsed_ticks < 37225:  # straight: l=-10, r=-21
            alpha = 0.8; deadband = 0.01; correction_multiplier = 3.0
            base_left = -10; base_right = -21; left_offset = -5; right_offset = 5
            controller.kp = 10; controller.ki = 0; controller.kd = 0
            section = 7
            section_name = 'wall straight'
        elif elapsed_ticks < 3915:  # left turn: l=-5, r=-20, offsets: 0, 5; duration: 690 ticks
            alpha = 0; deadband = 0.01; correction_multiplier = 3.0
            base_left = -5; base_right = -20; left_offset = 0; right_offset = 5
            controller.kp = 10; controller.ki = 0; controller.kd = 0
            section = 8
            section_name = 'wall left'
        
        elif elapsed_ticks < 41500:  # straight: l=-10, r=-21
            alpha = 0.8; deadband = 0.01; correction_multiplier = 3.0
            base_left = -10; base_right = -20; left_offset = -5; right_offset = 5
            controller.kp = 10; controller.ki = 0; controller.kd = 0
            section = 10
            section_name = 'Straight to Finish'
        else:
            alpha = 0.8; deadband = 0.01; correction_multiplier = 3.0
            base_left = 0; base_right = 0; left_offset = 0; right_offset = 0
            controller.kp = 10; controller.ki = 0; controller.kd = 0
            section = 11
            section_name = 'Stop'

        # 1. Read raw centroid.
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
        # 5. Amplify the correction.
        amplified_correction = correction_multiplier * correction
        
        print("Section {}: {}, Encoder Ticks: {}".format(section, section_name, elapsed_ticks))
        
        # 6. Compute motor commands.
        left_command = base_left + left_offset + amplified_correction
        right_command = base_right + right_offset - amplified_correction
        
        left_motor_command.put(int(left_command))
        right_motor_command.put(int(right_command))
        
        yield 0

def bumper_task():
    """
    Monitors the bumper switches and checks for a stationary condition.
    If either the bumper sensors are activated or the robot is stationary 
    (motors commanded but no encoder velocity) for 2 seconds,
    the robot executes a backup maneuver with a variable turn to steer away from the bumper.
    In section 5.5 (based on encoder ticks), bumper input is ignored.
    """
    global backup_active
    backup_duration = 300  # Backup for 300 ms.
    velocity_threshold = 5  # Threshold for detecting near-zero velocity.
    stationary_start = None

    while True:
        left_cmd = left_motor_command.get()
        right_cmd = right_motor_command.get()
        lv = left_velocity.get()
        rv = right_velocity.get()
        
        stationary = (abs(lv) < velocity_threshold and abs(rv) < velocity_threshold and 
                      (abs(left_cmd) > 0 or abs(right_cmd) > 0))
        
        # Check current section using encoder ticks.
        current_ticks = left_encoder_count.get()
        elapsed_ticks = current_ticks - encoder_baseline
        
        # Determine if we are in section 5.5 (example range: 32000 <= ticks < 33175).
        in_section_5_5 = (33000 <= elapsed_ticks < 36000)
        
        # Only consider bumper input if not in section 5.5.
        bumper_triggered = False
        if not in_section_5_5:
            bumper_triggered = left_bumper.is_pressed() or right_bumper.is_pressed()
        
        current_time = utime.ticks_ms()
        if bumper_triggered:
            print("Bumper condition triggered backup maneuver.")
            stationary_start = None
            backup_active = True
        elif stationary:
            if stationary_start is None:
                stationary_start = current_time
            elif utime.ticks_diff(current_time, stationary_start) >= 2000:
                print("Stationary condition persisted for 2 seconds, triggering backup maneuver.")
                backup_active = True
        else:
            stationary_start = None
        
        if backup_active:
            # Compute turn offset to steer away from the bumper.
            turn_offset = 0
            if left_bumper.is_pressed():
                pressed = left_bumper.get_pressed_sensors()
                for sensor in pressed:
                    if sensor == "PB13":
                        offset = -5
                    elif sensor == "PB14":
                        offset = -8
                    elif sensor == "PB4":
                        offset = -12
                    else:
                        offset = 0
                    if offset < turn_offset:
                        turn_offset = offset
            if right_bumper.is_pressed():
                pressed = right_bumper.get_pressed_sensors()
                for sensor in pressed:
                    if sensor == "PB7":
                        offset = 5
                    elif sensor == "PC2":
                        offset = 8
                    elif sensor == "PC3":
                        offset = 12
                    else:
                        offset = 0
                    if offset > turn_offset:
                        turn_offset = offset
                        
            backup_effort = 30
            left_motor_command.put(backup_effort + turn_offset)
            right_motor_command.put(backup_effort - turn_offset)
            start_backup = utime.ticks_ms()
            while utime.ticks_diff(utime.ticks_ms(), start_backup) < backup_duration:
                yield 0
            left_motor_command.put(0)
            right_motor_command.put(0)
            backup_active = False
            stationary_start = None
            utime.sleep_ms(100)
        yield 0


def switch_task():
    # Use working button logic without args and update encoder baseline.
    global encoder_baseline
    switch_pin = Pin('C13', Pin.IN, Pin.PULL_DOWN)  # Initialize the pin once.
    while True:
        if switch_pin.value() == 0:  # Button pressed (active low).
            current_switch_state = switch_state.get()
            new_state = 1 - current_switch_state
            switch_state.put(new_state)  # Toggle state.
            # When enabling the robot (switching to Running), update encoder_baseline.
            if new_state == 1:
                encoder_baseline = left_encoder_count.get()
            print("Button Pressed! Robot " + ("Running" if switch_state.get() else "Stopped"))
            utime.sleep_ms(200)  # Debounce delay.
        yield  # Yield control to the scheduler.

# --- Task Creation and Scheduling ---
encoder_task_L = cotask.Task(encoder_task_left, name="EncoderTaskL", priority=3, period=15, profile=True)#was 20
encoder_task_R = cotask.Task(encoder_task_right, name="EncoderTaskR", priority=3, period=15, profile=True)#was 20
motor_control_task_L = cotask.Task(motor_control_task_left, name="MotorControlTaskL", priority=2, period=15, profile=True)
motor_control_task_R = cotask.Task(motor_control_task_right, name="MotorControlTaskR", priority=2, period=15, profile=True)
ir_sensor_task_obj = cotask.Task(ir_sensor_task, name="IRSensorTask", priority=2, period=5, profile=True)#was 10
closed_loop_task_obj = cotask.Task(closed_loop_task, name="ClosedLoopTask", priority=2, period=10, profile=True)
bumper_task_obj = cotask.Task(bumper_task, name="BumperTask", priority=4, period=50, profile=True)
switch_task_obj = cotask.Task(switch_task, name="SwitchTask", priority=2, period=20, profile=True)

cotask.task_list.append(encoder_task_L)
cotask.task_list.append(encoder_task_R)
cotask.task_list.append(motor_control_task_L)
cotask.task_list.append(motor_control_task_R)
cotask.task_list.append(ir_sensor_task_obj)
cotask.task_list.append(closed_loop_task_obj)
cotask.task_list.append(bumper_task_obj)
cotask.task_list.append(switch_task_obj)

# --- Main Execution ---
print("Press Enter to start closed-loop line following with 3 parameter sections and bumper backup...")
#input()  # Wait for user input to start.
print("Starting closed-loop line following...")

left_motor.enable()
left_motor_command.put(0)
right_motor.enable()
right_motor_command.put(0)
gc.collect()

while True:
    cotask.task_list.pri_sched()
    gc.collect()
