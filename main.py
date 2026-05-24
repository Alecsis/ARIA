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

    # --- HARDWARE WATCHDOG LAYER ---
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
    # -------------------------------

    # 1. Apply your calibrated calibration offsets
    raw_gx = gyro['x'] - gx_off
    raw_gy = gyro['y'] - gy_off
    raw_gz = gyro['z'] - gz_off

    # 2. THE REMAPPING FIX
    # We swap the axes right here so the radio transmits what Processing expects!
    # Processing expects Pitch on the 1st gyro slot and 1st accel slot.
    
    tx_gx = raw_gy       # Force Processing's Pitch loop to listen to physical Gyro Y
    tx_gy = raw_gx       # Route Gyro X to the Roll channel slot
    tx_gz = raw_gz       # Keep Yaw on Yaw
    
    # 2. Accelerometer Swaps (Aligns gravity changes with the calculation)
    # Because your movement shifts physical Accel Z, we must pass it to the 
    # position Processing uses for the Pitch angle trig matrix (Position 1).
    tx_ax = accel['z']   # Route physical Accel Z into Processing's 'accelX' slot
    tx_ay = accel['y']   # Route physical Accel Y into Processing's 'accelY' slot
    tx_az = accel['x']   # Route physical Accel X into Processing's 'accelZ' slot
    # =========================================================================

    # --- BME280 Altitude Logic ---
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

    # 3. PACK DATA INTO THE SAME 32 BYTES
    # Processing receives this and has no idea we swapped the wires in code!
    msg = struct.pack('<I7f', 
        counter, 
        tx_gx, tx_gy, tx_gz, 
        alt_ft, 
        tx_ax, tx_ay, tx_az
    )

    print(f"TX Pack {counter} -> Sent Pitch Gyro: {tx_gx:.2f}°/s | Sent Pitch Accel: {tx_ax:.2f}g")
    
    if not radio.send(msg):
        print("FAILED TO SEND RADIO")
    
    counter += 1
    sleep_ms(100)