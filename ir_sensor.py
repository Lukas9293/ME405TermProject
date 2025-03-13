# ir_sensor.py
import pyb

class IRSensor:
    def __init__(self, channels, calibration_min=None, calibration_max=None, sensitivity=2.0):
        """
        Initialize the IR sensor array driver.
        
        Parameters:
          channels: List of pyb.Pin objects corresponding to the 8 analog outputs.
                    Example:
                      [pyb.Pin("PC4"), pyb.Pin("PA5"), pyb.Pin("PC5"), pyb.Pin("PA6"),
                       pyb.Pin("PB0"), pyb.Pin("PC1"), pyb.Pin("PA4"), pyb.Pin("PC0")]
          calibration_min: Optional list of minimum raw ADC values for each channel.
                           (These should come from a black calibration.)
          calibration_max: Optional list of maximum raw ADC values for each channel.
                           (These should come from a white calibration.)
          sensitivity: A factor to amplify differences in the normalized sensor values.
                       The normalized value is adjusted using:
                           adjusted = 0.5 + sensitivity * (raw_normalized - 0.5)
                       A value > 1 will expand differences.
                       
        If calibration_min and calibration_max are not provided, default values of 0 and 4095 are used.
        
        Note:
          If the sensor’s raw readings do not vary much between black and white surfaces,
          then the calibration_min and calibration_max will be very close to each other.
          This will result in a very narrow normalized range – and even after applying the
          sensitivity adjustment, the adjusted values may not differ significantly.
        """
        # Create an ADC object for each channel
        self.adcs = [pyb.ADC(pin) for pin in channels]
        self.num_channels = len(self.adcs)
        # Use provided calibration values or defaults
        self.calibration_min = calibration_min if calibration_min is not None else [0] * self.num_channels
        self.calibration_max = calibration_max if calibration_max is not None else [4095] * self.num_channels
        self.sensitivity = sensitivity

    def read_all(self):
        """
        Returns:
          A list of raw ADC readings (integers) from all channels.
        """
        return [adc.read() for adc in self.adcs]

    def read_all_normalized(self):
        """
        Normalizes each channel's raw ADC value to a 0–1 range using the calibration parameters.
        
        For each channel:
          1. Calculate the normalized value:
               normalized = (raw_value - calibration_min) / (calibration_max - calibration_min)
          2. Apply sensitivity adjustment:
               adjusted = 0.5 + sensitivity * (normalized - 0.5)
          3. Clamp the adjusted value to ensure it stays within [0, 1].
        
        Returns:
          A list of sensitivity-adjusted normalized sensor values.
          
        Explanation:
          If the calibration range is very narrow (i.e., calibration_min and calibration_max are nearly equal),
          then even small differences in raw readings will result in very small differences after normalization.
          The sensitivity factor is intended to “stretch” these differences. However, if the range is extremely narrow,
          even an amplified difference might remain small.
        """
        raw = self.read_all()
        normalized = []
        for i, value in enumerate(raw):
            # Calculate denominator (range of values)
            denominator = self.calibration_max[i] - self.calibration_min[i]
            if denominator != 0:
                norm_val = (value - self.calibration_min[i]) / denominator
            else:
                norm_val = 0  # Fallback if denominator is 0
            # Apply sensitivity adjustment
            adjusted = 0.5 + self.sensitivity * (norm_val - 0.5)
            # Clamp the value between 0 and 1
            adjusted = max(0, min(1, adjusted))
            normalized.append(adjusted)
        return normalized

    def compute_centroid(self):
        """
        Computes the centroid (weighted average index) of the sensor array based on the normalized values.
        
        The sensor array is assumed to be arranged from left to right.
        Each channel is assigned a normalized index from 0 (leftmost) to 1 (rightmost).
        The centroid is calculated as:
          centroid = (sum(normalized_value_i * index_i)) / (sum(normalized_value_i))
        
        If all channels have the same value, the centroid will be 0.5.
        
        Returns:
          A float representing the normalized centroid.
        """
        normalized = self.read_all_normalized()
        total = sum(normalized)
        if total == 0:
            return 0.5  # If no signal is detected, default to center.
        indices = [i / (self.num_channels - 1) for i in range(self.num_channels)]
        weighted_sum = sum(val * idx for val, idx in zip(normalized, indices))
        centroid = weighted_sum / total
        return centroid

    def debug(self):
        """
        Prints detailed debug information:
          - Raw ADC readings.
          - Sensitivity-adjusted normalized readings.
          - Computed centroid.
        """
        raw = self.read_all()
        normalized = self.read_all_normalized()
        centroid = self.compute_centroid()
        print("IR Sensor Debug Info:")
        print("Raw Readings:", raw)
        print("Adjusted Normalized Readings:", normalized)
        print("Computed Centroid:", centroid)
