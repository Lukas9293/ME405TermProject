# ME 405 Term Project
Welcome to our ME 405 Term Project repository! We are **Lukas Moreau** and **Patrick Michael**, and this project documents our journey in designing, building, and programming an autonomous Romi robot for a time-trial challenge.  

Our Romi robot integrates multiple subsystems, including:  
- **Motor and Encoder Drivers** for precise wheel control and odometry.  
- **IR Sensor Array** for line following with calibrated readings.  
- **Bumper Sensors** to detect collisions and trigger recovery maneuvers.  
- **Closed-Loop Control** using a PID-based system for accurate path correction.  
- **Task-Based Multitasking** for efficient coordination between system components.  

This repository includes our full **codebase**, **circuit diagrams**, **state machine logic**, and **test results**. Below, you’ll find a structured breakdown of our implementation, challenges, and optimizations.  

## Table of Contents
- [Required Parts and Materials](#required-parts-and-materials)
- [Wiring Diagram](#wiring-diagram)
- [Finite State Machines](#finite-state-machines)
- [State Transition Diagrams](#state-transition-diagrams)
- [What Each Class Does](#what-each-class-does)

## Required Parts and Materials

| Quantity | Item                                    | Purchase Link |
|----------|-----------------------------------------|--------------|
| 1x       | 120pcs 20cm Dupont Ribbon             | [Amazon](https://www.amazon.com/dp/B07GCY6CH7) |
| 1x       | HC-05 Bluetooth Module                | [Amazon](https://www.amazon.com/dp/B01MQKX7VP) |
| 6x       | NiMH AA Battery                        | [Amazon](https://www.amazon.com/dp/B0D2JCY87L) |
| 1x       | NiMH Battery Charger                   | [Amazon](https://www.amazon.com/dp/B00JHKSLM8) |
| 2x       | 1x6 Pin Dupont Housing                 | [Pololu](https://www.pololu.com/product/1905) |
| 2x       | 2x6 Pin Dupont Housing                 | [Pololu](https://www.pololu.com/product/1914) |
| 2x       | 2x13 Pin Dupont Housing                | [Pololu](https://www.pololu.com/product/1969) |
| 1x       | IR Reflectance Sensors                 | [Pololu](https://www.pololu.com/category/123) |
| 1x       | USB-Mini-B to USB-C Cable              | [Amazon](https://www.amazon.com/dp/B082F3M1HW) |
| -        | Heat Shrink Tubing                     | [Amazon](https://www.amazon.com/dp/B01MFA3OFA) |

# Miscellaneous Components

| Quantity | Component                              |
|----------|----------------------------------------|
| 4x       | M2.5 x 8mm Standoff                    |
| 4x       | M2.5 x 10mm Standoff                   |
| 4x       | M2.5 x 30mm Standoff                   |
| 4x       | M2.5 x 6mm Socket Head Cap Screw       |
| 4x       | M2.5 x 8mm Socket Head Cap Screw       |
| 4x       | M2.5 x 10mm Socket Head Cap Screw      |
| 8x       | M2.5 Nylon Lock Nuts                   |
| 8x       | M2.5 Nylon Washer                      |
| 1x       | Acrylic Romi-to-Shoe Adapter           |
| 1x       | BNO055 IMU Breakout Board              |
| 1x       | Modified Shoe of Brian                 |
| 1x       | Extra Nucleo L476RG                    |
| 1x       | Romi Chassis w/ Wheels and Casters     |

# Tools Required

| Tool                                          |
|-----------------------------------------------|
| 2mm Hex Driver (for M2.5 SHCS)               |
| 5mm Nut Driver (for M2.5 Nuts)               |
| Small Adjustable Wrench or Parallel Pliers   |
| Slotted/Flat Blade Screwdriver or Small Pry Bar |

## Wiring Diagram
![image](https://github.com/user-attachments/assets/d3d4d17b-7e4a-4be0-a88a-0e2bdd16e1dd)

## System Overview

The system is composed of several key components:

- **Hardware Drivers:**
  - **Motor Driver:** Controls motors using PWM signals along with direction and enable/sleep pins.
  - **Encoder:** Reads wheel rotations via quadrature encoding to provide cumulative counts and velocity.
  - **IR Sensor Array:** Reads eight analog channels to compute a weighted "centroid" of the line.
  - **Bumper Sensors:** Detect collisions using one or more digital inputs.
  - **IMU (Not used):** A BNO055 IMU driver is available but not used in the current application.
  - **Closed-Loop Controller:** Implements a PID controller (mainly proportional in use) to correct the robot’s course based on the IR sensor centroid.

- **Shared Variables (via `task_share.py`):**
  - Encoder counts and velocities for both left and right wheels.
  - Motor command shares for left and right motors.
  - IR sensor line centroid.
  - A switch state indicating whether the robot is running or stopped.

- **Cooperative Tasks (via `cotask.py`):**
  - **Encoder Tasks:** Continuously update encoder counts and velocities.
  - **Motor Control Tasks:** Read motor commands and update PWM outputs accordingly.
  - **IR Sensor Task:** Computes the centroid of the line from sensor readings.
  - **Closed-Loop Task:** Uses encoder data and the IR sensor centroid to determine the robot’s “section” of the track, update PID parameters, and generate motor commands.
  - **Bumper Task:** Monitors for collisions or prolonged stationary conditions and initiates a backup maneuver if needed.
  - **Switch Task:** Toggles between "Running" and "Stopped" modes using a physical button and resets the encoder baseline.

## Task Diagram & Descriptions

### Hardware Interfaces and Drivers

- **Motor Driver (`motor.py`):**
  - Uses PWM for speed control.
  - Sets motor direction via a digital pin.
  - Enables/disables the motor driver via a sleep pin.

- **Encoder (`encoder.py`):**
  - Uses hardware timers configured in encoder mode.
  - Updates cumulative encoder counts and computes instantaneous velocities.
  
- **IR Sensor (`ir_sensor.py`):**
  - Reads and normalizes eight ADC channels.
  - Applies a sensitivity adjustment and computes a weighted centroid for line position.

- **Bumper (`bumper.py`):**
  - Reads one or more digital bumper sensors.
  - Provides functions to check for pressed sensors.

- **Closed-Loop Controller (`closed_loop.py`):**
  - Implements a PID algorithm for line following.
  - Primarily used as a proportional controller with possible tuning of integral and derivative gains.

### Shared Variables

- **Encoder Data:**  
  - `left_encoder_count`, `right_encoder_count` store cumulative counts.
  - `left_velocity`, `right_velocity` store the computed velocities.
- **Motor Commands:**  
  - `left_motor_command` and `right_motor_command` hold the latest motor effort values.
- **IR Sensor Centroid:**  
  - `ir_line_centroid` holds the computed weighted average (line position).
- **Switch State:**  
  - `switch_state` toggles between 0 (Stopped) and 1 (Running).

### Task Scheduler

All tasks are organized in a cooperative multitasking system where each task periodically yields control. The scheduler runs tasks based on priorities and timing constraints defined by each task's period.

## Finite State Machines (FSMs)

### 1. Closed-Loop Task FSM (Line-Following Sections)

This task divides the robot’s journey into sections based on elapsed left encoder ticks. Each section corresponds to a state with its own set of motor commands and PID tuning:

- **State 1 ("Straight"):**
  - **Condition:** Elapsed ticks < 6700.
  - **Parameters:**  
    - Low-pass filter constant (α) = 0.8  
    - Deadband = 0.01  
    - Base motor commands: left = –10, right = –21  
    - Offsets: left –5, right +5  
    - PID gains: kp = 12, ki = 0, kd = 0  
    - Correction multiplier = 3.0  
  - **Transition:** Move to State 2 when elapsed ticks ≥ 6700.

- **State 2 ("Diamond"):**
  - **Condition:** 6700 ≤ elapsed ticks < 7725.
  - **Parameters:**  
    - Same base commands as State 1.
    - PID gains: kp = 0, ki = 0.000001, kd = 0.001  
  - **Transition:** When elapsed ticks ≥ 7725, transition to State 3.

- **State 3 ("Curve and Structure"):**
  - **Condition:** 7725 ≤ elapsed ticks < 30500.
  - **Parameters:**  
    - α = 0.7, deadband = 0.01  
    - Base commands: left = –10, right = –22  
    - Offsets: left –5, right +5  
    - PID gains: kp = 11, ki = 0, kd = 0  
  - **Transition:** When elapsed ticks reaches 30500, transition to State 4.

- **State 4 ("Structure Right Turn"):**
  - **Condition:** 30500 ≤ elapsed ticks < 31350.
  - **Parameters:**  
    - Base commands: left = –20, right = 15  
    - Offsets: left –5, right +5  
    - PID gains: kp = 10, ki = 0, kd = 0  
  - **Transition:** When elapsed ticks ≥ 31350, transition to State 5.

- **State 5 ("Wall Bump"):**
  - **Condition:** 31350 ≤ elapsed ticks < 33775.
  - **Parameters:**  
    - Base commands: left = –10, right = –21  
    - PID gains: kp = 10, ki = 0, kd = 0  
  - **Transition:** When elapsed ticks ≥ 33775, transition to State 6.

- **State 6 ("Wall Right"):**
  - **Condition:** 33775 ≤ elapsed ticks < 34625.
  - **Parameters:**  
    - Base commands: left = –20, right = 15  
    - PID gains: kp = 10, ki = 0, kd = 0  
  - **Transition:** When elapsed ticks ≥ 34625, transition to State 7.

- **State 7 ("Wall Straight"):**
  - **Condition:** 34625 ≤ elapsed ticks < 36625.
  - **Parameters:**  
    - Base commands: left = –10, right = –21  
  - **Transition:** When elapsed ticks ≥ 36625, transition to State 8.

- **State 8 ("Wall Left"):**
  - **Condition:** 36625 ≤ elapsed ticks < 37375.
  - **Parameters:**  
    - Base commands: left = –5, right = –20  
    - Offsets: left offset = 0, right offset = +5  
  - **Transition:** When elapsed ticks ≥ 37375, move directly to State 10.

- **State 9 ("Straight to Finish"):**
  - **Condition:** 37375 ≤ elapsed ticks < 40000.
  - **Parameters:**  
    - Base commands: left = –10, right = –20  
    - Offsets: left –5, right +5  
  - **Transition:** When elapsed ticks ≥ 40000, transition to State 11.

- **State 10 ("Stop"):**
  - **Condition:** Elapsed ticks ≥ 40000.
  - **Action:** Set both motor commands to 0 (stop the robot).

### 2. Bumper Task FSM (Backup Maneuver)

The bumper task monitors the bumper sensors and the encoder velocities:

- **Normal Operation:**
  - **Action:** Continuously check:
    - If any bumper sensor is pressed.
    - If the robot’s velocity is below a threshold while nonzero motor commands are active.
  - **Transition:**  
    - If a bumper is pressed or the stationary condition persists for 2 seconds, set the `backup_active` flag to true and transition to the Backup Maneuver state.

- **Backup Maneuver:**
  - **Action:**  
    - Compute a turning offset based on which bumper sensor(s) are activated.
    - Override normal motor commands by applying a fixed backup effort (e.g., 30) adjusted with the computed offset.
    - Maintain this backup effort for 300 ms.
  - **Transition:** After 300 ms, clear the backup flag, reset motor commands to 0, and return to Normal Operation.

### 3. Switch Task FSM (Operational State Toggle)

The switch task monitors a button to toggle the robot's operational state:

- **Stopped (switch_state = 0):**
  - **Action:** Robot remains inactive (closed-loop task forces motor commands to 0).
  - **Transition:** On button press, switch to Running state and update the encoder baseline.

- **Running (switch_state = 1):**
  - **Action:** Normal operation with active closed-loop control.
  - **Transition:** On button press, toggle back to Stopped.

## Using our Code

1. **Hardware Setup:**  
   - Ensure proper connections for motors, encoders, IR sensor array, bumpers, and the switch as defined abve.
   - Confirm the pin assignments in the source files match your hardware configuration.

2. **Software Setup:**  
   - Load the MicroPython files (`bumper.py`, `closed_loop.py`, `cotask.py`, `encoder.py`, `imu.py`, `ir_sensor.py`, `motor.py`, `task_share.py`, and the main project file) onto your board.
   - Make any necessary calibration adjustments for the IR sensor (calibration_min and calibration_max values).

3. **Running the Project:**  
   - Reset or run the main project file (e.g., `V9_modificationofV7postV8shitstorm.py`).
   - Use the physical button (on pin C13) to toggle between the Running and Stopped states.

## Conclusion

This project demonstrates a modular design for robotic control using cooperative multitasking. The clear separation of hardware interfaces, shared data, and FSM-based control in the closed-loop, bumper, and switch tasks allows for easy modification and extension. Adjust the PID gains and task parameters as needed to fine-tune the robot’s behavior on different track sections.
