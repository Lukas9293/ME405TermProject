# calibrate_ir.py
import utime
import pyb
from ir_sensor import IRSensor

def calibrate_ir(channels, sample_duration=5000, sample_interval=50):
    """
    Calibrates the IR sensor by sampling raw ADC values over a given duration.
    
    Parameters:
      channels: List of pyb.Pin objects for the sensor channels.
      sample_duration: Total sampling time in milliseconds.
      sample_interval: Time between samples in milliseconds.
      
    Returns:
      cal_min: List of minimum readings for each channel during the sampling period.
      cal_max: List of maximum readings for each channel during the sampling period.
      
    How It Works:
      - The function instantiates an IRSensor (without calibration parameters).
      - It then initializes two lists: one for minimum values (starting high) and one for maximum values (starting low).
      - Over the sample_duration, it repeatedly reads all raw ADC values from the sensor.
      - For each channel, it updates the minimum and maximum values if the current reading is lower or higher than the stored value.
      - These min and max values will later be used to normalize the sensor data to a 0–1 range.
    """
    sensor = IRSensor(channels)
    num_channels = sensor.num_channels
    # Initialize calibration arrays with extreme values.
    cal_min = [4095] * num_channels  # Start with the maximum possible ADC value.
    cal_max = [0] * num_channels     # Start with the minimum possible ADC value.

    print("Calibrating... Please hold the sensor steady.")
    start_time = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), start_time) < sample_duration:
        readings = sensor.read_all()  # Get raw ADC readings from each channel.
        for i, value in enumerate(readings):
            if value < cal_min[i]:
                cal_min[i] = value  # Update minimum if current reading is lower.
            if value > cal_max[i]:
                cal_max[i] = value  # Update maximum if current reading is higher.
        utime.sleep_ms(sample_interval)
    return cal_min, cal_max

def main():
    # Define the IR sensor channel pins.
    channels = [pyb.Pin("PC4"), pyb.Pin("PA5"), pyb.Pin("PC5"), pyb.Pin("PA6"),
                pyb.Pin("PB0"), pyb.Pin("PC1"), pyb.Pin("PA4"), pyb.Pin("PC0")]

    print("Place the IR sensor array on a white surface and press Enter.")
    input()  # Wait for the user to press Enter.
    print("Calibrating for white surface... Please keep sensor steady for 5 seconds.")
    white_min, white_max = calibrate_ir(channels, sample_duration=5000, sample_interval=50)
    print("White calibration complete.")
    print("White min readings:", white_min)
    print("White max readings:", white_max)
    
    print("\nNow place the IR sensor array on a black surface and press Enter.")
    input()  # Wait for the user to press Enter.
    print("Calibrating for black surface... Please keep sensor steady for 5 seconds.")
    black_min, black_max = calibrate_ir(channels, sample_duration=5000, sample_interval=50)
    print("Black calibration complete.")
    print("Black min readings:", black_min)
    print("Black max readings:", black_max)

    # In many reflectance sensor applications:
    # - The white surface yields higher ADC values.
    # - The black surface yields lower ADC values.
    # Therefore, for normalization, we can use:
    #   calibration_min = black_min (the minimum values on a black surface)
    #   calibration_max = white_max (the maximum values on a white surface)
    print("\nFinal Calibration Parameters:")
    print("Calibration min (black values):", black_min)
    print("Calibration max (white values):", white_max)
    print("\nUse these values to update your IRSensor instance, e.g.:")
    print("    ir_sensor = IRSensor(channels, calibration_min={}, calibration_max={})".format(black_min, white_max))

if __name__ == "__main__":
    main()
