from radio import Radio
import time
import struct
from machine import Pin, I2C
from time import sleep_ms, ticks_us, ticks_diff
from lib.MPU6050 import _ACC_RNG_2G, _GYR_RNG_250DEG, MPU6050
from lib.BME280 import BME280
import math

radio = Radio()
print("TX ready")

# Global hardware configurations
SCL_PIN = 7
SDA_PIN = 6
FREQ = 400000

# IMU update period (400 Hz = 2500 microseconds)
IMU_PERIOD_US = 2500
last_update_us = 0

# Filtered angles (what we'll send to Processing)
roll = 0.0
pitch = 0.0
yaw = 0.0

# Offsets for zeroing
roll_offset = 0.0
pitch_offset = 0.0
yaw_offset = 0.0

# Gyro bias (calibration)
gyro_bias_x = 0.0
gyro_bias_y = 0.0
gyro_bias_z = 0.0

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

def wrap_angle(angle):
    """Wrap angle to -180 to 180 degrees"""
    while angle > 180.0:
        angle -= 360.0
    while angle < -180.0:
        angle += 360.0
    return angle

def compute_accel_angles(ax, ay, az):
    """Calculate pitch and roll from accelerometer data"""
    # Roll: rotation around X axis (using Y and Z)
    roll_acc = math.atan2(ay, az) * 180.0 / math.pi
    
    # Pitch: rotation around Y axis (using X, Y, Z)
    pitch_acc = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 180.0 / math.pi
    
    return roll_acc, pitch_acc

def calibrate_gyro():
    """Calibrate gyro by taking 500 samples while stationary"""
    global gyro_bias_x, gyro_bias_y, gyro_bias_z
    samples = 500
    sum_x = 0.0
    sum_y = 0.0
    sum_z = 0.0
    
    print("Calibrating gyro... keep device still")
    
    for i in range(samples):
        gyro = mpu.read_gyro_data()
        sum_x += gyro['x']
        sum_y += gyro['y']
        sum_z += gyro['z']
        sleep_ms(3)
    
    gyro_bias_x = sum_x / samples
    gyro_bias_y = sum_y / samples
    gyro_bias_z = sum_z / samples
    
    print(f"Gyro calibration done. Biases: X={gyro_bias_x:.3f}, Y={gyro_bias_y:.3f}, Z={gyro_bias_z:.3f}")

def smooth(new, old):
    return (0.8 * old) + (0.2 * new)

def to_feet(m):
    return m * 3.28084

# Initial hardware startup
init_sensors()

# Calibrate gyro
calibrate_gyro()

# Get initial accelerometer angles for offset
accel = mpu.read_accel_data(g=True)
roll_acc, pitch_acc = compute_accel_angles(accel['x'], accel['y'], accel['z'])
roll_offset = roll_acc
pitch_offset = pitch_acc

try:
    bme.calibrate(samples=100, delay_ms=20)
except:
    print("BME calibration failed, skipping...")

last_alt = 0.0
counter = 0
consecutive_dead_packets = 0

# Initialize timing
last_update_us = ticks_us()
print("Starting main loop at 400Hz...")

while True:
    # Timing control - maintain 400Hz update rate
    now_us = ticks_us()
    dt_us = ticks_diff(now_us, last_update_us)
    
    if dt_us < IMU_PERIOD_US:
        sleep_ms(1)
        continue
    
    dt = dt_us / 1000000.0  # Convert to seconds
    if dt > 0.01:  # Clamp dt to reasonable value
        dt = 0.0025
    
    last_update_us = now_us
    
    # Read sensors
    accel = mpu.read_accel_data(g=True)
    gyro = mpu.read_gyro_data()
    
    # Apply gyro bias correction
    gyro_x = gyro['x'] - gyro_bias_x
    gyro_y = gyro['y'] - gyro_bias_y
    gyro_z = gyro['z'] - gyro_bias_z
    
    # Calculate accelerometer angles (absolute)
    roll_acc, pitch_acc = compute_accel_angles(accel['x'], accel['y'], accel['z'])
    
    # Apply offsets (relative to starting position)
    roll_acc_rel = wrap_angle(roll_acc - roll_offset)
    pitch_acc_rel = wrap_angle(pitch_acc - pitch_offset)
    
    # Complementary filter (98% gyro, 2% accelerometer)
    # Same as the working C++ example
    filter_coeff = 0.98
    
    roll = filter_coeff * (roll + gyro_x * dt) + (1 - filter_coeff) * roll_acc_rel
    pitch = filter_coeff * (pitch + gyro_y * dt) + (1 - filter_coeff) * pitch_acc_rel
    yaw += gyro_z * dt
    
    # Wrap angles to -180..180
    roll = wrap_angle(roll)
    pitch = wrap_angle(pitch)
    yaw = wrap_angle(yaw)
    
    # Altitude reading (at ~10Hz)
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
    
    # Send data at ~40Hz (every 10th iteration)
    if counter % 10 == 0:
        # Format for Processing - simple slash-separated like the C++ example
        # Processing expects: roll/pitch/yaw
        print(f"{roll:.3f}/{pitch:.3f}/{yaw:.3f}")
        
        # Also send altitude occasionally
        if counter % 20 == 0:
            print(f"Alt(ft): {alt_ft:.2f}")
    
    # Optional: Send binary packet for radio (keep this if you need radio)
    if counter % 4 == 0:  # Send radio at ~100Hz
        msg = struct.pack('<I3f', counter, roll, pitch, yaw)
        if not radio.send(msg):
            print("FAILED TO SEND RADIO")
    
    counter += 1