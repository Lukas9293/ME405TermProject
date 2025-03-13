# closed_loop.py
import utime

class ClosedLoopController:
    """
    A simple PID controller for line following.
    
    The controller calculates a correction value based on the error between the desired setpoint
    (the target IR sensor centroid, typically 0.5 for center) and the measured value.
    
    In our current use, we set the integral (ki) and derivative (kd) gains to 0, meaning the controller
    behaves as a proportional controller. If you wish to incorporate integral and derivative action,
    set ki and/or kd to nonzero values.
    
    Attributes:
      setpoint (float): Desired normalized IR sensor centroid.
      kp (float): Proportional gain (determines immediate response to error).
      ki (float): Integral gain (accumulates error over time; currently 0).
      kd (float): Derivative gain (predicts future error; currently 0).
    """
    
    def __init__(self, setpoint=0.5, kp=50, ki=0, kd=0):
        self.setpoint = setpoint
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error = 0
        self.integral = 0
        self.last_time = None

    def compute(self, measurement):
        """
        Computes the PID correction based on the measurement.
        
        Parameters:
          measurement (float): The current filtered sensor reading (IR centroid).
          
        Returns:
          float: The computed correction.
          
        Steps:
          1. Calculate error = setpoint - measurement.
          2. Compute time difference dt.
          3. Update integral (ki * error * dt) and derivative ((error - last_error) / dt).
          4. Return the weighted sum of P, I, and D terms.
          
        Note: With ki and kd set to 0, this returns a pure proportional correction.
        """
        error = self.setpoint - measurement
        current_time = utime.ticks_ms()
        if self.last_time is None:
            dt = 20  # Default dt in ms if first run.
        else:
            dt = utime.ticks_diff(current_time, self.last_time)
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.last_error = error
        self.last_time = current_time
        return output

    def debug(self, measurement):
        """
        Debug method: Computes and prints the correction for a given measurement.
        """
        correction = self.compute(measurement)
        print("Closed-Loop Controller Debug Info:")
        print("Setpoint:", self.setpoint)
        print("Measurement:", measurement)
        print("Correction:", correction)
        return correction
