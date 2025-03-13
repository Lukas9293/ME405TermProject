# bluetooth_hc05.py
from pyb import Pin, UART
from time import sleep_ms

class BluetoothHC05:
    """
    A helper class to manage an HC-05 Bluetooth module over UART3
    using PC10 (TX) and PC11 (RX) on AF7.

    - By default, it initializes the UART at 'at_baud' (the baud
      the HC-05 is currently using in AT mode).
    - The configure() method sends AT commands to rename the module,
      change its password, and set a new baudrate.
    - After configuration, it re-initializes the UART at 'new_baud'
      for normal data communication.

    Wiring:
        HC-05 TX  -> PC11  (RX on MCU)
        HC-05 RX  -> PC10  (TX on MCU)
        HC-05 VCC -> 5V
        HC-05 GND -> GND
        HC-05 EN/KEY pin -> Pull HIGH on power-up for AT mode (slow blink)
    """

    def __init__(self,
                 at_baud=38400,
                 new_baud=115200,
                 name="HC05",
                 password="1234",
                 stop_bits=1,
                 parity=0):
        """
        :param at_baud:   The current baudrate of the HC-05 in AT mode.
        :param new_baud:  The baudrate you want to set on the HC-05.
        :param name:      The new name for the HC-05.
        :param password:  The new password/PIN (often must be 4 digits).
        :param stop_bits: 1 or 2 for the AT+UART command.
        :param parity:    0=None, 1=Odd, 2=Even for the AT+UART command.
        """

        self.at_baud   = at_baud
        self.new_baud  = new_baud
        self.name      = name
        self.password  = password
        self.stop_bits = stop_bits
        self.parity    = parity

        # If you previously used PC4/PC5 for UART3, deconfigure them:
        # (Only do this if PC4/PC5 were used; otherwise you can skip)
        # Pin(Pin.C4, mode=Pin.ANALOG)
        # Pin(Pin.C5, mode=Pin.ANALOG)

        # Deconfigure PC10/PC11 in case they were used for something else
        Pin(Pin.C10, mode=Pin.ANALOG)
        Pin(Pin.C11, mode=Pin.ANALOG)

        # Now set PC10 as USART3_TX and PC11 as USART3_RX on AF7
        Pin(Pin.C10, mode=Pin.ALT, alt=7)  # TX
        Pin(Pin.C11, mode=Pin.ALT, alt=7)  # RX

        # Create the UART at the "AT mode" baudrate
        self.uart = UART(3, self.at_baud, timeout=1000)

    def configure(self):
        """
        Use AT commands to rename the device, set a new password, and
        change the baudrate. Assumes the HC-05 is in AT mode (slow blink).
        After successful configuration, re-initializes UART at 'new_baud'.
        """
        input("Put the HC-05 in AT mode (EN pin high on power-up) and press Enter...")

        # 1) Rename device
        cmd = f"AT+NAME={self.name}\r\n"
        self._send_cmd(cmd, "OK\r\n", "Rename command not accepted")

        # 2) Set password
        cmd = f"AT+PSWD={self.password}\r\n"
        self._send_cmd(cmd, "OK\r\n", "Password reset command not accepted")

        # 3) Set new baud, stopbits, parity
        cmd = f"AT+UART={self.new_baud},{self.stop_bits},{self.parity}\r\n"
        self._send_cmd(cmd, "OK\r\n", "UART config command not accepted")

        # 4) Reset the module
        cmd = "AT+RESET\r\n"
        self._send_cmd(cmd, "OK\r\n", "Reset failed")

        print("\nConfiguration succeeded. Re-initializing UART to new baud...\n")
        sleep_ms(1000)
        # Re-initialize the UART at the new baud for normal use
        self.uart.init(self.new_baud, timeout=1000)

    def _send_cmd(self, cmd, expected, error_msg):
        """
        Internal helper to send an AT command and check the response.
        """
        print(f"Sending: {repr(cmd)}")
        self.uart.write(cmd)
        resp = self.uart.readline()
        print(f"Response: {repr(resp)}\n")
        if resp != expected.encode('utf-8'):
            raise Exception(error_msg)
        sleep_ms(500)

    def send(self, data):
        """
        Send data (str or bytes) over Bluetooth.
        """
        if isinstance(data, str):
            data = data.encode('utf-8')
        return self.uart.write(data)

    def recv(self, max_bytes=64):
        """
        Read up to 'max_bytes' from the HC-05. Returns bytes or None if empty.
        """
        return self.uart.read(max_bytes)

    def any(self):
        """
        Returns the number of bytes in the UART RX buffer (non-blocking).
        """
        return self.uart.any()
