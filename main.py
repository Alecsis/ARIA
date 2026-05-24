from machine import Pin, I2C
from time import sleep, sleep_ms
from BME280 import BME280

# -------------------
# I2C SETUP
# -------------------
i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=400000)
bme = BME280(i2c=i2c)

print("Sensors ready")

# -------------------
# CALIBRATION (IMPROVED)
# -------------------
print("Calibrating ground pressure...")

bme.calibrate(samples=100, delay_ms=20)

print("Baseline:", bme.baseline_pressure)
print("Altitude zeroed!\n")

# -------------------
# FILTER STATE
# -------------------
last_alt = 0

def smooth(new, old):
    return (0.8 * old) + (0.2 * new)

# -------------------
# MAIN LOOP
# -------------------
while True:
    temp, press, hum = bme.read_compensated_data()

    alt = bme.altitude(press)
    
    alt = smooth(alt, last_alt)
    last_alt = alt
    
    if abs(alt) < 0.5:
        alt = 0
    
    alt_ft = alt * 3.28084
    
    print("---- BME ----")
    print("Temp:", temp)
    print("Pressure:", press)
    print("Humidity:", hum)
    print("Altitude:", "{:.3f} ft".format(alt_ft))
    print("\n")

    sleep(0.5)