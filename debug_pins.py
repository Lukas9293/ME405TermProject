"""
debug_pins.py

This file tests the pin assignments for the QTRX-MD-08A IR Sensor Array on the 
ST-Nucleo-L476RG.

For the IR Sensor Array:
  - Channel 1: "PC4"
  - Channel 2: "PA5"
  - Channel 3: "PC5"
  - Channel 4: "PA6"
  - Channel 5: "PB0"
  - Channel 6: "PC1"
  - Channel 7: "PA4"
  - Channel 8: "PC0"
"""

import utime
import pyb
from ir_sensor import IRSensor

def test_ir_sensor():
    print("----- Testing IR Sensor Array -----")
    channels = [pyb.Pin("PC4"), pyb.Pin("PA5"), pyb.Pin("PC5"), pyb.Pin("PA6"),
                pyb.Pin("PB0"), pyb.Pin("PC1"), pyb.Pin("PA4"), pyb.Pin("PC0")]
    ir = IRSensor(channels)
    for _ in range(5):
        ir.debug()
        utime.sleep_ms(500)

print("Starting IR sensor debug for pin assignments...")
test_ir_sensor()
print("IR sensor debug complete.")
