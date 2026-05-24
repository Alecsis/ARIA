from machine import Pin, I2C
from time import sleep_ms, ticks_ms
from lib.MPU6050 import MPU6050

# =========================
# I2C SETUP
# =========================
i2c = I2C(
    1,
    scl=Pin(7),
    sda=Pin(6),
    freq=100000
)

print("I2C Scan:", i2c.scan())

mpu = MPU6050(i2c)
print("MPU6050 initialized")

# =========================
# GYRO CALIBRATION (IMPORTANT)
# =========================
print("Calibrating gyro... keep sensor still")

gx_off = 0
gy_off = 0
gz_off = 0

samples = 200

for i in range(samples):
    g = mpu.read_gyro_data()
    gx_off += g['x']
    gy_off += g['y']
    gz_off += g['z']
    sleep_ms(5)

gx_off /= samples
gy_off /= samples
gz_off /= samples

print("Gyro offsets:")
print(gx_off, gy_off, gz_off)

print("Calibration complete.\n")

# =========================
# FLIGHT STATE
# =========================
t = 0
is_launched = False

altitude = 0.0  # placeholder until you add barometer

# =========================
# MAIN LOOP
# =========================
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

    accel = mpu.read_accel_data()
    gyro = mpu.read_gyro_data()
    temp = mpu.read_temperature()

    # -------------------------
    # APPLY GYRO ZEROING
    # -------------------------
    gx = gyro['x'] - gx_off
    gy = gyro['y'] - gy_off
    gz = gyro['z'] - gz_off

    # -------------------------
    # SIMPLE LAUNCH DETECTION (TEMP LOGIC)
    # replace later with real sensor or threshold
    # -------------------------
    if t > 50:
        is_launched = True

    # -------------------------
    # ALTITUDE SIMULATION (TEMP PLACEHOLDER)
    # replace with barometer later
    # -------------------------
    if is_launched:
        altitude += 0.5
    else:
        altitude = max(0, altitude - 0.02)

    # -------------------------
    # PACKET (MATCHS PROCESSING FORMAT)
    # AVO,pitch,roll,yaw,alt,ax,ay,az
    # -------------------------
    packet = "AVO,{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f},{:.2f}".format(
        gx, gy, gz,
        altitude,
        accel['x'], # type: ignore
        accel['y'], # type: ignore
        accel['z'] # type: ignore
    )

    print(packet)

    # -------------------------
    # DEBUG OUTPUT (OPTIONAL)
    # -------------------------
    print("TEMP:", temp)
    print("ACC:", accel)
    print("GYRO:", {"x": gx, "y": gy, "z": gz})
    print("-------------------\n")

    t += 1
    sleep_ms(50)  # ~20 Hz