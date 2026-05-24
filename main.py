from machine import Pin, I2C
from time import sleep_ms
from lib.MPU6050 import MPU6050
from lib.BME280 import BME280

# =========================
# I2C SETUP
# =========================
i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=400000)

print("I2C Scan:", i2c.scan())

mpu = MPU6050(i2c)
bme = BME280(i2c=i2c)

print("MPU6050 + BME280 initialized")

# =========================
# GYRO CALIBRATION
# =========================
print("Calibrating gyro... keep sensor still")

gx_off = 0
gy_off = 0
gz_off = 0

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

# =========================
# BARO CALIBRATION
# =========================
print("Calibrating ground pressure...")
bme.calibrate(samples=100, delay_ms=20)

print("Baseline:", bme.baseline_pressure)
print("Altitude zeroed!\n")

# =========================
# FILTERS
# =========================
last_alt = 0.0

def smooth(new, old):
    return (0.8 * old) + (0.2 * new)

def to_feet(m):
    return m * 3.28084

# =========================
# MAIN LOOP
# =========================
while True:

    # -------- MPU --------
    accel = mpu.read_accel_data()
    gyro = mpu.read_gyro_data()

    gx = gyro['x'] - gx_off
    gy = gyro['y'] - gy_off
    gz = gyro['z'] - gz_off

    # -------- BME --------
    temp, press, hum = bme.read_compensated_data()

    # altitude (meters)
    alt_m = bme.altitude(press)

    # deadband FIRST (important fix)
    if abs(alt_m) < 0.5:
        alt_m = 0

    # smoothing AFTER deadband
    alt_m = smooth(alt_m, last_alt)
    last_alt = alt_m

    # convert to feet
    alt_ft = to_feet(alt_m)

    # for display only
    alt_ft_print = round(alt_ft, 3)

    # =========================
    # PRINT DEBUG
    # =========================
    print("---- BME ----")
    print("Temp:", temp)
    print("Pressure:", press)
    print("Humidity:", hum)
    print("Altitude:", "{:.3f} ft".format(alt_ft_print))
    print()

    # =========================
    # PACKET
    # =========================
    packet = "AVO,{:.2f},{:.2f},{:.2f},{:.3f},{:.2f},{:.2f},{:.2f}".format(
        gx, gy, gz,
        alt_ft,
        accel['x'],
        accel['y'],
        accel['z']
    )

    print(packet)
    print("-------------------\n")

    sleep_ms(50)