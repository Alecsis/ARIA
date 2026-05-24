from radio import Radio
import time
import struct
from machine import Pin, I2C
from time import sleep_ms
from lib.MPU6050 import _ACC_RNG_2G, _GYR_RNG_250DEG, MPU6050
from lib.BME280 import BME280

radio = Radio()
print("TX ready")

# Global hardware configurations
SCL_PIN = 7
SDA_PIN = 6
FREQ = 400000

def init_sensors():
    """Initializes or resets the hardware connection to the sensors"""
    global i2c, mpu, bme
    print("Initializing I2C Bus and Sensors...")
    try:
        i2c = I2C(1, scl=Pin(SCL_PIN), sda=Pin(SDA_PIN), freq=FREQ)
        mpu = MPU6050(i2c)
        bme = BME280(i2c=i2c)
        mpu.set_accel_range(_ACC_RNG_2G)   
        mpu.set_gyro_range(_GYR_RNG_250DEG)
        print("Sensors successfully bound.")
        return True
    except Exception as e:
        print("Sensor initialization failed:", e)
        return False

# Initial hardware startup
init_sensors()

# ---------- calibration ----------
print("Calibrating gyro...")
gx_off = gy_off = gz_off = 0
samples = 200

for _ in range(samples):
    try:
        g = mpu.read_gyro_data()
        gx_off += g['x']
        gy_off += g['y']
        gz_off += g['z']
    except:
        pass
    sleep_ms(5)

gx_off /= samples
gy_off /= samples
gz_off /= samples

try:
    bme.calibrate(samples=100, delay_ms=20)
except:
    print("BME calibration failed, skipping...")

# ---------- helpers ----------
def smooth(new, old):
    return (0.8 * old) + (0.2 * new)

def to_feet(m):
    return m * 3.28084

last_alt = 0.0
counter = 0
consecutive_dead_packets = 0

while True:
    accel = mpu.read_accel_data(g=True) 
    gyro = mpu.read_gyro_data()

    # Watchdog check
    if abs(accel['x']) == 0.0 and abs(accel['y']) == 0.0 and abs(accel['z']) == 0.0:
        consecutive_dead_packets += 1
        if consecutive_dead_packets >= 5:
            print("WARNING: IMU freeze detected! Attempting hot-reset...")
            init_sensors()
            consecutive_dead_packets = 0
            sleep_ms(50)
            continue
    else:
        consecutive_dead_packets = 0

    # Apply gyro calibration offsets
    raw_gx = gyro['x'] - gx_off
    raw_gy = gyro['y'] - gy_off
    raw_gz = gyro['z'] - gz_off

    # ========== CORRECTED AXIS MAPPING ==========
    # Accelerometer mapping (keep this as is - it's working)
    tx_ax = accel['z']   # Physical Z → Processing accelX (pitch)
    tx_ay = accel['y']   # Physical Y → Processing accelY (roll)
    tx_az = accel['x']   # Physical X → Processing accelZ (vertical)
    
    # Gyroscope mapping (FIXED for yaw)
    tx_gx = raw_gx       # Physical X → pitch rate (matches accelerometer mapping)
    tx_gy = raw_gy       # Physical Y → roll rate (matches accelerometer mapping)
    tx_gz = raw_gz       # Physical Z → yaw rate (matches accelerometer mapping)
    # ============================================

    # Altitude reading
    try:
        temp, press, hum = bme.read_compensated_data()
        alt_m = bme.altitude(press)
    except:
        alt_m = last_alt

    if abs(alt_m) < 0.5:
        alt_m = 0

    alt_m = smooth(alt_m, last_alt)
    last_alt = alt_m
    alt_ft = to_feet(alt_m)

    # Pack and send
    msg = struct.pack('<I7f', 
        counter, 
        tx_gx, tx_gy, tx_gz, 
        alt_ft, 
        tx_ax, tx_ay, tx_az
    )

    # Debug output - should show Z ≈ 1.0G at rest
    print(f"Pack {counter:3d} | Accel: X={tx_ax:5.2f} Y={tx_ay:5.2f} Z={tx_az:5.2f}G | Alt={alt_ft:5.2f}ft")
    
    if not radio.send(msg):
        print("FAILED TO SEND RADIO")
    
    counter += 1
    sleep_ms(50)