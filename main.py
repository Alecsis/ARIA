from radio import Radio
import time
from machine import Pin, I2C
from time import sleep_ms
from lib.MPU6050 import MPU6050
from lib.BME280 import BME280

# =========================
# RADIO
# =========================
radio = Radio()
print("TX ready")

# =========================
# I2C SETUP
# =========================
i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=400000)

mpu = MPU6050(i2c)
bme = BME280(i2c=i2c)

print("Sensors initialized")

# =========================
# CALIBRATION
# =========================
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

print("Gyro offsets:", gx_off, gy_off, gz_off)

print("Calibrating pressure...")
bme.calibrate(samples=100, delay_ms=20)

last_alt = 0.0

def smooth(new, old):
    return (0.8 * old) + (0.2 * new)

def to_feet(m):
    return m * 3.28084

# =========================
# MAIN LOOP
# =========================
counter = 0

while True:

    # -------- MPU --------
    accel = mpu.read_accel_data()
    gyro = mpu.read_gyro_data()

    gx = gyro['x'] - gx_off
    gy = gyro['y'] - gy_off
    gz = gyro['z'] - gz_off

    # -------- BME --------
    temp, press, hum = bme.read_compensated_data()
    alt_m = bme.altitude(press)

    if abs(alt_m) < 0.5:
        alt_m = 0

    alt_m = smooth(alt_m, last_alt)
    last_alt = alt_m

    alt_ft = to_feet(alt_m)

    # =========================
    # PACKET
    # =========================
    msg = "AVO,{:.2f},{:.2f},{:.2f},{:.3f},{:.2f},{:.2f},{:.2f}".format(
        gx, gy, gz,
        alt_ft,
        accel['x'],
        accel['y'],
        accel['z']
    )

    print("Sending:", msg)

    radio.send(msg.encode())

    counter += 1
    sleep_ms(50)