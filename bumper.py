import pyb

class Bumper:
    """
    Driver for a bumper sensor (or sensors).

    This class interfaces with one or more bumper sensors connected to digital input pins.
    It is assumed that the bumper sensor(s) are active low (i.e., when pressed, the pin reads 0).
    """
    
    def __init__(self, pin_names):
        """
        Initialize the bumper sensor(s).

        Parameters:
          pin_names (str or list of str): The name(s) of the pin(s) to which the bumper is connected.
            For a single sensor, provide a string (e.g., "PD0").
            For multiple sensors, provide a list of strings.
        """
        if isinstance(pin_names, list):
            self.pin_names = pin_names
            self.pins = [pyb.Pin(pin_name, pyb.Pin.IN, pull=pyb.Pin.PULL_UP) for pin_name in pin_names]
        else:
            self.pin_names = [pin_names]
            self.pins = [pyb.Pin(pin_names, pyb.Pin.IN, pull=pyb.Pin.PULL_UP)]
    
    def is_pressed(self):
        """
        Check if any of the bumper sensor(s) is pressed.

        Returns:
          bool: True if any sensor is pressed, False otherwise.
        """
        return any(pin.value() == 0 for pin in self.pins)
    
    def get_pressed_sensors(self):
        """
        Get a list of bumper sensor pin names that are currently pressed.

        Returns:
          list of str: A list of the pin names for which the sensor is pressed.
        """
        pressed = []
        for name, pin in zip(self.pin_names, self.pins):
            if pin.value() == 0:
                pressed.append(name)
        return pressed

# Example usage (for debugging):
if __name__ == "__main__":
    # Create a left bumper with three sensors.
    left_bumper = Bumper(["PB4", "PB14", "PB13"])
    # Create a right bumper with three sensors.
    right_bumper = Bumper(["PB7", "PC2", "PC3"])
    
    while True:
        if left_bumper.is_pressed():
            pressed_left = left_bumper.get_pressed_sensors()
            print("Left bumper pressed! Sensors pressed:", pressed_left)
        else:
            print("Left bumper is not pressed.")
        
        if right_bumper.is_pressed():
            pressed_right = right_bumper.get_pressed_sensors()
            print("Right bumper pressed! Sensors pressed:", pressed_right)
        else:
            print("Right bumper is not pressed.")
        
        pyb.delay(500)
