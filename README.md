# ME 405 Term Project
Welcome to our ME 405 Term Project! We are Lukas Moreau and Patrick Michael, and this repository documents our work on the Romi robot time-trial challenge. Our goal is to design, build, and program an autonomous Romi robot to navigate a track efficiently and reliably. 

This repository contains all the necessary code, wiring diagrams, and documentation for our project. Below, you'll find a structured breakdown of our work, including state machine design, sensor integration, and control strategies.

## Table of Contents
- [Required Parts and Materials](#required-parts-and-materials)
- [Wiring Diagram](#wiring-diagram)
- [Finite State Machines](#finite-state-machines)
- [State Transition Diagrams](#state-transition-diagrams)
- [What Each Class Does](#what-each-class-does)

## Required Parts and Materials

## Wiring Diagram
![image](https://github.com/user-attachments/assets/d3d4d17b-7e4a-4be0-a88a-0e2bdd16e1dd)

## Shared Variable Table
Shared Variables (via task_share.py):

Encoder Counts and Velocities:
• left_encoder_count and right_encoder_count store cumulative encoder values updated by the encoder tasks.
• left_velocity and right_velocity capture the instantaneous speed (counts per second), also updated by the encoder tasks.

Motor Commands:
• left_motor_command and right_motor_command hold the latest effort values to be applied by the motor control tasks.
• These values are calculated by the closed-loop task based on line-following corrections or overridden during backup maneuvers.

IR Line Centroid:
• ir_line_centroid holds the computed weighted average from the IR sensor array.
• Updated continuously by the IR sensor task and used by the closed-loop task to drive the PID controller.

Switch State:
• switch_state toggles between “Stopped” (0) and “Running” (1).
• Controlled by the switch task and used by the closed-loop task to either process or suspend motor commands.
## Task Diagrams and Descriptions 

Tasks & Their Interactions:

Encoder Tasks (Left and Right):
• Each encoder task calls the respective encoder’s update() method to compute the latest count and velocity.
• They then update the corresponding shared variables so that subsequent tasks have real-time odometry data.

Motor Control Tasks (Left and Right):
• These tasks continuously read the motor command shared variables.
• They translate these commands into effort values, adjusting PWM output accordingly.

IR Sensor Task:
• Periodically reads the sensor array and computes the centroid.
• Writes the adjusted, normalized centroid into the shared variable, ensuring that the closed-loop task receives an up-to-date measure of line position.

Closed-Loop Task:
• Acts as the “brain” for line following. It first checks for a valid running condition (using switch_state and the encoder baseline).
• It then computes the difference in left encoder ticks relative to a baseline and uses this “elapsed_ticks” to decide which section of the course the robot is in.
• For each section, it adjusts parameters such as PID gains, base motor commands, offsets, filtering constant (α), deadband thresholds, and a correction multiplier.
• It applies a low-pass filter to the IR centroid, computes the error with respect to the setpoint, and uses the PID controller to compute a correction that is then scaled and added/subtracted to generate left and right motor commands.
• Finally, it updates the shared motor commands.

Bumper Task:
• Monitors bumper sensor states as well as current motor commands and velocities.
• Checks for two conditions: either a bumper sensor is pressed, or the robot is commanded to move yet remains “stationary” (velocities below a threshold) for more than 2 seconds.
• When either condition is met, it sets a global flag (backup_active) and enters a backup maneuver where motor commands are overridden with reverse efforts and a turning offset computed from the specific bumper sensor(s) pressed.
• After a fixed backup duration, it clears the backup flag and resumes normal operations.

Switch Task:
• Monitors a physical button to toggle the operational state.
• When the button is pressed, it toggles switch_state.
• If transitioning from “Stopped” to “Running”, it also resets the encoder baseline, effectively restarting the closed-loop section logic.

Task Scheduler (cotask):
• Organizes all tasks in a prioritized list.
• Implements a cooperative multitasking model where each task yields control (usually yielding “0”) so that the scheduler can cycle through them based on timing (periods) or flags (go_flag).

## Finite State Machines and Descriptions

#A. Closed-Loop Task FSM (Line-Following & Section Logic)
This task uses the elapsed encoder ticks to divide the robot’s run into distinct “sections” that tailor motor commands and PID controller parameters. Each section is like a state with defined entry conditions, operational parameters, and transitions:

State 1 ("Straight"):

Entry Condition: Elapsed ticks (current left encoder count – baseline) < 6700.
Actions/Parameters:
• Apply a low-pass filter with α = 0.8.
• Deadband threshold set to 0.01 (small errors are ignored).
• Base motor commands: left = –10, right = –21.
• Offsets: left offset = –5, right offset = +5.
• PID gains: kp = 12, ki = 0, kd = 0 (pure proportional control).
• Correction is multiplied by 3.0 to adjust responsiveness.
Transition: When elapsed_ticks reaches or exceeds 6700, move to State 2.
State 2 ("Diamond"):

Entry Condition: 6700 ≤ elapsed_ticks < 7725.
Actions/Parameters:
• Maintain the same base motor parameters as State 1.
• Modify PID gains to a very low kp (0) with a slight integral and derivative contribution (ki = 0.000001, kd = 0.001) to possibly deal with more dynamic error responses.
Transition: When elapsed_ticks ≥ 7725, transition to State 3.
State 3 ("Curve and Structure"):

Entry Condition: 7725 ≤ elapsed_ticks < 30500.
Actions/Parameters:
• Use a slightly reduced filter constant (α = 0.7) for a smoother response.
• Adjust base motor commands to left = –10, right = –22.
• PID gains set to kp = 11, keeping it proportional without integral or derivative components.
Transition: When elapsed_ticks reaches 30500, transition to State 4.
State 4 ("Structure Right Turn"):

Entry Condition: 30500 ≤ elapsed_ticks < 31350.
Actions/Parameters:
• Change base commands to a turning profile: left = –20, right = 15.
• Offsets remain (left offset = –5, right offset = +5) to fine-tune the turn.
• PID controller gains are lowered (kp = 10) to reduce aggressiveness during the turn.
Transition: When elapsed_ticks reaches 31350, move to State 5.
State 5 ("Wall Bump"):

Entry Condition: 31350 ≤ elapsed_ticks < 33775.
Actions/Parameters:
• Revert to a “straight” profile with left = –10 and right = –21.
• Maintain consistent offsets and PID gains (kp = 10).
Transition: When elapsed_ticks reaches 33775, transition to State 6.
State 6 ("Wall Right"):

Entry Condition: 33775 ≤ elapsed_ticks < 34625.
Actions/Parameters:
• Configure for a right turn: left = –20, right = 15.
• Maintain similar offsets and PID gains as in State 4.
Transition: When elapsed_ticks reaches 34625, transition to State 7.
State 7 ("Wall Straight"):

Entry Condition: 34625 ≤ elapsed_ticks < 36625.
Actions/Parameters:
• Return to a “straight” mode with left = –10, right = –21.
• Offsets and gains continue as in previous “straight” states.
Transition: When elapsed_ticks reaches 36625, transition to State 8.
State 8 ("Wall Left"):

Entry Condition: 36625 ≤ elapsed_ticks < 37375.
Actions/Parameters:
• Switch the turning profile: left = –5, right = –20 with offsets (left offset = 0, right offset = +5).
• PID gains remain at kp = 10.
Transition: When elapsed_ticks reaches 37375, skip any commented-out intermediate state and move directly to State 10.
State 10 ("Straight to Finish"):

Entry Condition: 37375 ≤ elapsed_ticks < 40000.
Actions/Parameters:
• Set a “final approach” profile: left = –10, right = –20 with offsets (left offset = –5, right offset = +5).
• PID gains remain at kp = 10.
Transition: When elapsed_ticks reaches or exceeds 40000, transition to the final state.
State 11 ("Stop"):

Entry Condition: elapsed_ticks ≥ 40000.
Action:
• Both motor commands are set to 0 to bring the robot to a stop.

In this system, the data flow is tightly integrated through shared variables and a cooperative task scheduler. The closed-loop task’s FSM uses encoder counts to switch between different driving “profiles,” adapting PID gains and motor commands for various sections of a course. Simultaneously, the bumper task provides safety by initiating backup maneuvers under collision or stall conditions, and the switch task offers a manual override to start or stop the robot. Together, these components create a robust control system for a line-following, collision-aware robot.

#B. Bumper Task FSM (Backup Maneuver)
The bumper task monitors both the bumper sensors and the robot’s velocity to detect a collision or a prolonged stationary condition. Its FSM can be further detailed as follows:

Normal Operation State:

Monitoring:
• Continuously read the left/right motor commands and the corresponding velocities.
• Use the Bumper driver’s methods to check if any sensor is activated.
Stationary Detection:
• If both left and right velocities are below a defined threshold (e.g., < 5 counts/sec) while nonzero motor commands are active, start a timer.
• If this condition persists for 2 seconds, it is treated as a stall.
Transition:
• If either a bumper sensor is pressed or the stationary condition persists, set the global flag backup_active to True and transition to the Backup state.
Backup Maneuver State:

Actions:
• Determine a turn offset based on which specific bumper sensors are pressed.
– For example, on the left side, sensors with pin "PB13", "PB14", or "PB4" may each contribute different offsets to steer the robot away.
– Similarly, the right side sensors contribute positive offsets.
• Override normal commands by applying a fixed backup effort (e.g., 30) plus or minus the computed turn offset to the left and right motor commands respectively.
• Maintain this reverse/turn maneuver for a fixed backup duration (300 ms).
Transition:
• After 300 ms, clear the backup flag, set motor commands to 0, and return to the Normal Operation state.

#C. Switch Task FSM (Operational State Toggle)
The switch task’s FSM is relatively simple but critical for overall operation:

Stopped State (switch_state = 0):

Action:
• The closed-loop task interprets this state as “robot disabled,” so it forces motor commands to 0.
Transition:
• When the physical button (connected to pin C13) is pressed, toggle the state to “Running” (switch_state becomes 1) and update the encoder baseline to the current left encoder count.
Running State (switch_state = 1):

Action:
• The closed-loop task actively computes motor commands based on the current IR sensor readings and encoder data.
Transition:
• On another button press, toggle back to “Stopped,” halting motor command updates.

#Summary
In this system, the data flow is tightly integrated through shared variables and a cooperative task scheduler. The closed-loop task’s FSM uses encoder counts to switch between different driving “profiles,” adapting PID gains and motor commands for various sections of a course. Simultaneously, the bumper task provides safety by initiating backup maneuvers under collision or stall conditions, and the switch task offers a manual override to start or stop the robot. Together, these components create a robust control system for a line-following, collision-aware robot.

## Driver Descriptions
Hardware Interfaces and Drivers:

Motor Driver (motor.py):
• Manages the PWM output for driving the motors.
• Uses a direction (DIR) pin to set the forward or reverse direction and an nSLP (sleep/enable) pin to activate the driver.
• Receives “effort” commands (in the range –100 to 100) from the control tasks.

Encoder (encoder.py):
• Uses hardware timers in encoder mode to track wheel rotations.
• Computes both the cumulative position (as an encoder count) and instantaneous velocity.
• Provides data to the encoder tasks which update shared variables.

IR Sensor (ir_sensor.py):
• Reads analog values from an 8-channel sensor array.
• Normalizes these raw ADC values using calibration parameters (min/max per channel) and applies a sensitivity adjustment.
• Computes a weighted “centroid” which represents the line’s position relative to the sensor array.

Bumper (bumper.py):
• Interfaces with one or more digital bumper sensors that are active low.
• Can report whether any sensor is pressed and list which sensors are active.

Closed-Loop Controller (closed_loop.py):
• Implements a PID (proportional–integral–derivative) algorithm.
• Currently functions mainly as a proportional controller (with ki and kd usually set to 0) but can be tuned to include integral and derivative actions.
• Computes a “correction” based on the error between the desired line position (setpoint) and the measured, filtered IR centroid.
