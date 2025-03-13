# imu.py
import utime
from pyb import I2C

class BNO055:
    """
    Driver class for the BNO055 Inertial Measurement Unit (IMU) using I2C.
    
    This class provides methods to initialize the sensor, change its operating mode,
    read the calibration status, and retrieve sensor data (Euler angles and angular velocities).

    The calibration status is obtained from the CALIB_STAT register (0x35) and parsed as follows:
      sys  = (cal_status >> 6) & 0x03  (System calibration)
      gyro = (cal_status >> 4) & 0x03  (Gyroscope calibration)
      acc  = (cal_status >> 2) & 0x03  (Accelerometer calibration)
      mag  = (cal_status)      & 0x03  (Magnetometer calibration)

    Default I2C address is 0x28 (can be 0x29 if the ADR pin is high).
    """
    # Register addresses (from the datasheet)
    _BNO055_CHIP_ID_ADDR     = 0x00  # Should return 0xA0
    _BNO055_OPR_MODE_ADDR    = 0x3D  # Operating mode register
    _BNO055_CALIB_STAT_ADDR  = 0x35  # Calibration status register
    _BNO055_EULER_H_LSB_ADDR = 0x1A  # Euler angles: Heading LSB
    _BNO055_EULER_P_LSB_ADDR = 0x1C  # Euler angles: Pitch LSB
    _BNO055_EULER_R_LSB_ADDR = 0x1E  # Euler angles: Roll LSB
    _BNO055_GYRO_DATA_X_LSB_ADDR = 0x14  # Gyro X LSB (for angular velocity)

    def __init__(self, i2c, address=0x28):
        """
        Initialize the BNO055 IMU.
        
        Parameters:
          i2c (pyb.I2C): An I2C object configured in controller mode.
          address (int): I2C address of the BNO055 (default 0x28).
        """
        self.i2c = i2c
        self.address = address
        # Verify chip ID
        chip_id = self.i2c.mem_read(1, self.address, self._BNO055_CHIP_ID_ADDR)[0]
        if chip_id != 0xA0:
            raise Exception("BNO055 not found. Chip ID: 0x{:02X}".format(chip_id))
        # Initialize sensor: set to CONFIG mode first, then to NDOF mode for fusion.
        self.set_mode(0x00)  # CONFIG mode
        utime.sleep_ms(20)
        self.set_mode(0x0C)  # NDOF mode (fusion mode)
        utime.sleep_ms(20)

    def set_mode(self, mode):
        """
        Set the operating mode of the BNO055.
        
        Parameters:
          mode (int): Desired operating mode (e.g., 0x0C for NDOF).
        """
        self.i2c.mem_write(mode, self.address, self._BNO055_OPR_MODE_ADDR)
        utime.sleep_ms(30)

    def get_calibration_status(self):
        """
        Reads the calibration status and returns a tuple:
          (sys_cal, gyro_cal, acc_cal, mag_cal)
        Each value is between 0 and 3.
        """
        cal_status = self.i2c.mem_read(1, self.address, self._BNO055_CALIB_STAT_ADDR)[0]
        sys_cal = (cal_status >> 6) & 0x03
        gyro_cal = (cal_status >> 4) & 0x03
        acc_cal = (cal_status >> 2) & 0x03
        mag_cal = cal_status & 0x03
        return sys_cal, gyro_cal, acc_cal, mag_cal

    def _read_word(self, register):
        """
        Reads a 16-bit word from the specified register and converts it to a signed integer.
        """
        data = self.i2c.mem_read(2, self.address, register)
        result = data[0] | (data[1] << 8)
        if result > 32767:
            result -= 65536
        return result

    def get_euler(self):
        """
        Retrieves Euler angles (heading, pitch, roll) from the BNO055.
        The sensor provides angles in 1/16th of a degree.
        
        Returns:
          tuple: (heading, pitch, roll) in degrees.
        """
        heading = self._read_word(self._BNO055_EULER_H_LSB_ADDR) / 16.0
        pitch   = self._read_word(self._BNO055_EULER_P_LSB_ADDR) / 16.0
        roll    = self._read_word(self._BNO055_EULER_R_LSB_ADDR) / 16.0
        return heading, pitch, roll

    def get_angular_velocity(self):
        """
        Retrieves gyroscope data (angular velocities) from the BNO055.
        Returns values in degrees per second.
        
        Returns:
          tuple: (gx, gy, gz)
        """
        gx = self._read_word(self._BNO055_GYRO_DATA_X_LSB_ADDR) / 16.0
        gy = self._read_word(self._BNO055_GYRO_DATA_X_LSB_ADDR + 2) / 16.0
        gz = self._read_word(self._BNO055_GYRO_DATA_X_LSB_ADDR + 4) / 16.0
        return gx, gy, gz

    def debug(self):
        """
        Prints debugging information including calibration status and Euler angles.
        """
        cal_status = self.get_calibration_status()
        heading, pitch, roll = self.get_euler()
        print("BNO055 Debug Info:")
        print("Calibration Status - System: {}, Gyro: {}, Acc: {}, Mag: {}".format(*cal_status))
        print("Euler Angles - Heading: {:.2f}°, Pitch: {:.2f}°, Roll: {:.2f}°".format(heading, pitch, roll))
