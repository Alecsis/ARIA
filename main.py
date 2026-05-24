from radio import Radio
import time
import struct # <-- ADD THIS
from machine import Pin, I2C
from time import sleep_ms
from lib.MPU6050 import MPU6050
from lib.BME280 import BME280

radio = Radio()
print("TX ready")

i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=400000)

mpu = MPU6050(i2c)
bme = BME280(i2c=i2c)

print("Sensors initialized")

# ---------- calibration ----------
print("Calibrating gyro...")

gx_off = gy_off = gz_off = 0
samples = 200

for _ in range(samples):
    g = mpu.read_gyro_data()
    gx_off += g['x']
    gy_off += g['y']
    gz_off += g['z']
    sleep_ms(5)

gx_off /= samples
gy_off /= samples
gz_off /= samples

bme.calibrate(samples=100, delay_ms=20)

# ---------- helpers ----------
def smooth(new, old):
    return (0.8 * old) + (0.2 * new)

def to_feet(m):
    return m * 3.28084

last_alt = 0.0
counter = 0

while True:
    accel = mpu.read_accel_data()
    gyro = mpu.read_gyro_data()

    gx = gyro['x'] - gx_off
    gy = gyro['y'] - gy_off
    gz = gyro['z'] - gz_off

    temp, press, hum = bme.read_compensated_data()
    alt_m = bme.altitude(press)

    if abs(alt_m) < 0.5:
        alt_m = 0

    alt_m = smooth(alt_m, last_alt)
    last_alt = alt_m
    alt_ft = to_feet(alt_m)

    # PACK DATA INTO 32 BYTES
    # '<I7f' means: Little-Endian ('<'), 1 Unsigned Int ('I'), 7 Floats ('7f')
    msg = struct.pack('<I7f', 
        counter, 
        gx, gy, gz, 
        alt_ft, 
        accel['x'], accel['y'], accel['z']
    )

    print(f"TX: Packet {counter} -> {len(msg)} bytes")
    
    if not radio.send(msg):
        print("FAILED TO SEND")
    
    counter += 1
    sleep_ms(100) # Reverted to single sleep block for clean timing