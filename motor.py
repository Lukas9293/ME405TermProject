from pyb import Pin, Timer

class Motor:
    """
    A motor driver interface for DRV8838-based drivers.
    Controls motor effort via PWM, a direction (DIR) pin, and a sleep/enable (nSLP) pin.
    """
    def __init__(self, timer, channel, pwm_pin, dir_pin, nslp_pin, pwm_freq=20000):
        """
        Initializes a Motor object.
        
        :param timer: A Timer object to use for PWM.
        :param channel: The timer channel number to use.
        :param pwm_pin: The pin used for PWM output.
        :param dir_pin: The pin used to set motor direction.
        :param nslp_pin: The pin used to enable/sleep the driver.
        :param pwm_freq: The PWM frequency in Hz (default 20,000 Hz).
        """
        self.dir = Pin(dir_pin, Pin.OUT_PP)
        self.nslp = Pin(nslp_pin, Pin.OUT_PP, value=0)  # Initially disabled.
        self.timer = timer
        self.channel = self.timer.channel(channel, Timer.PWM, pin=Pin(pwm_pin))
        self.set_effort(0)

    def set_effort(self, effort):
        """
        Sets the motor effort.
        
        :param effort: A signed integer between -100 and 100.
                       Positive values command one direction; negative values command the other.
        """
        # Clamp effort to [-100, 100]
        if effort > 100:
            effort = 100
        elif effort < -100:
            effort = -100

        # Set direction and duty.
        if effort >= 0:
            self.dir.high()  # For example, HIGH means forward.
            duty = effort
        else:
            self.dir.low()   # LOW means reverse.
            duty = -effort

        self.channel.pulse_width_percent(duty)

    def enable(self):
        """
        Enables the motor driver (exits sleep mode) and sets the motor to brake mode.
        """
        self.nslp.high()
        self.set_effort(0)

    def disable(self):
        """
        Disables the motor driver (puts it to sleep).
        """
        self.nslp.low()
