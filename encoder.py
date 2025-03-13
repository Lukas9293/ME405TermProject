from pyb import Pin, Timer
from utime import ticks_us, ticks_diff

class Encoder:
    """
    A quadrature encoder interface that uses a timer configured in encoder mode.
    This class accumulates encoder counts and computes velocity.
    """
    def __init__(self, tim, chA_pin, chB_pin, period=0xFFFF, prescaler=0):
        """
        Initializes an Encoder object.
        
        :param tim: The timer number to use (e.g., 2 for left encoder, 5 for right encoder).
        :param chA_pin: The pin for encoder channel A.
        :param chB_pin: The pin for encoder channel B.
        :param period: The timer period (default 0xFFFF).
        :param prescaler: The timer prescaler (default 0).
        """
        self.tim = Timer(tim, period=period, prescaler=prescaler)
        self.tim.channel(1, pin=Pin(chA_pin), mode=Timer.ENC_AB)
        self.tim.channel(2, pin=Pin(chB_pin), mode=Timer.ENC_AB)
        self.position = 0          # Accumulated encoder count.
        self.prev_count = self.tim.counter()
        self.delta = 0             # Change in count since last update.
        self.dt = 1e-3             # Time between updates in seconds (initial guess).
        self.prev_time = ticks_us()

    def update(self):
        """
        Updates the encoder's count and computes the change (delta) and dt.
        Corrects for timer overflow/underflow.
        """
        current_count = self.tim.counter()
        current_time = ticks_us()
        # Compute the elapsed time in seconds.
        self.dt = ticks_diff(current_time, self.prev_time) / 1e6
        self.prev_time = current_time

        max_val = self.tim.period()
        diff = current_count - self.prev_count
        
        # Correct for overflow or underflow.
        if diff > max_val / 2:
            diff -= max_val
        elif diff < -max_val / 2:
            diff += max_val

        self.delta = diff
        self.position += diff
        self.prev_count = current_count

    def get_position(self):
        """
        Returns the accumulated encoder position.
        """
        return self.position

    def get_velocity(self):
        """
        Returns the computed velocity (counts per second).
        """
        if self.dt > 0:
            return self.delta / self.dt
        return 0

    def zero(self):
        """
        Resets the encoder's accumulated position to zero.
        """
        self.position = 0
        self.prev_count = self.tim.counter()
        self.prev_time = ticks_us()
