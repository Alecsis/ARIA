from radio import Radio
import time
import struct
from machine import Pin, I2C, Timer
from time import sleep_ms, ticks_us, ticks_diff
from lib.MPU6050 import _ACC_RNG_2G, _GYR_RNG_250DEG, MPU6050
from lib.BME280 import BME280
import math
from ws2812 import WS2812  

led_power = Pin(11, Pin.OUT)
led_power.value(1) 

pixel = WS2812(12, 1)

COLOR_RED    = (30, 0, 0)    # Soft Red
COLOR_GREEN  = (0, 30, 0)    # Soft Green
COLOR_ORANGE = (35, 10, 0)   # Soft Orange/Yellow
COLOR_BLACK  = (0, 0, 0)     # Off

system_status = "STARTUP"  
current_color = COLOR_BLACK
blink_toggle = False

def led_timer_callback(timer):
    """Safe, ultra-fast background color switcher"""
    global system_status, current_color, blink_toggle
    
    if system_status == "STARTUP":
        current_color = COLOR_RED
        
    elif system_status == "CALIBRATING":
        blink_toggle = not blink_toggle
        current_color = COLOR_ORANGE if blink_toggle else COLOR_BLACK
        
    elif system_status == "LINK_OK":
        current_color = COLOR_GREEN
        
    elif system_status == "LINK_FAIL":
        blink_toggle = not blink_toggle
        current_color = COLOR_RED if blink_toggle else COLOR_BLACK

led_timer = Timer(-1)
led_timer.init(period=200, mode=Timer.PERIODIC, callback=led_timer_callback)

system_status = "STARTUP"
radio = Radio()
print("TX ready")

SCL_PIN = 7
SDA_PIN = 6
FREQ = 400000

IMU_PERIOD_US = 2500
last_update_us = 0

roll = 0.0; pitch = 0.0; yaw = 0.0
roll_offset = 0.0; pitch_offset = 0.0; yaw_offset = 0.0
gyro_bias_x = 0.0; gyro_bias_y = 0.0; gyro_bias_z = 0.0

def init_sensors():
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
    while angle > 180.0: angle -= 360.0
    while angle < -180.0: angle += 360.0
    return angle

def compute_accel_angles(ax, ay, az):
    roll_acc = math.atan2(ay, az) * 180.0 / math.pi
    pitch_acc = math.atan2(-ax, math.sqrt(ay*ay + az*az)) * 180.0 / math.pi
    return roll_acc, pitch_acc

def calibrate_gyro():
    global gyro_bias_x, gyro_bias_y, gyro_bias_z, system_status
    system_status = "CALIBRATING"
    
    samples = 500
    sum_x = 0.0; sum_y = 0.0; sum_z = 0.0
    
    print("Calibrating gyro... keep device still")
    for i in range(samples):
        pixel.pixels_fill(current_color)
        pixel.pixels_show()
        
        gyro = mpu.read_gyro_data()
        sum_x += gyro['x']
        sum_y += gyro['y']
        sum_z += gyro['z']
        sleep_ms(3)
    
    gyro_bias_x = sum_x / samples
    gyro_bias_y = sum_y / samples
    gyro_bias_z = sum_z / samples
    print("Gyro calibration done.")

def smooth(new, old): return (0.8 * old) + (0.2 * new)
def to_feet(m): return m * 3.28084

init_sensors()
calibrate_gyro()

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

system_status = "LINK_OK"
last_update_us = ticks_us()
print("Starting main loop at 400Hz...")


while True:
    now_us = ticks_us()
    dt_us = ticks_diff(now_us, last_update_us)
    
    if dt_us < IMU_PERIOD_US:
        sleep_ms(1)
        continue
    
    dt = dt_us / 1000000.0
    if dt > 0.01: dt = 0.0025
    last_update_us = now_us
    
    accel = mpu.read_accel_data(g=True)
    gyro = mpu.read_gyro_data()
    
    gyro_x = gyro['x'] - gyro_bias_x
    gyro_y = gyro['y'] - gyro_bias_y
    gyro_z = gyro['z'] - gyro_bias_z
    
    roll_acc, pitch_acc = compute_accel_angles(accel['x'], accel['y'], accel['z'])
    roll_acc_rel = wrap_angle(roll_acc - roll_offset)
    pitch_acc_rel = wrap_angle(pitch_acc - pitch_offset)
    
    filter_coeff = 0.98
    roll = filter_coeff * (roll + gyro_x * dt) + (1 - filter_coeff) * roll_acc_rel
    pitch = filter_coeff * (pitch + gyro_y * dt) + (1 - filter_coeff) * pitch_acc_rel
    yaw += gyro_z * dt
    
    roll = wrap_angle(roll); pitch = wrap_angle(pitch); yaw = wrap_angle(yaw)
    
    try:
        temp, press, hum = bme.read_compensated_data()
        alt_m = bme.altitude(press)
    except:
        alt_m = last_alt
        
    if abs(alt_m) < 0.5: alt_m = 0
    alt_m = smooth(alt_m, last_alt)
    last_alt = alt_m
    alt_ft = to_feet(alt_m)
    
    # Send data via radio at ~40Hz
    if counter % 10 == 0:
        message = f"{roll:.3f}/{pitch:.3f}/{yaw:.3f}\n"
        
        if not radio.send(message.encode()):
            print("FAILED TO SEND RADIO")
            consecutive_dead_packets += 1
            if consecutive_dead_packets > 5:
                system_status = "LINK_FAIL"
        else:
            consecutive_dead_packets = 0
            system_status = "LINK_OK"
        
        print(message.strip())
        
        if counter % 20 == 0:
            alt_msg = f"Alt(ft): {alt_ft:.2f}\n"
            radio.send(alt_msg.encode())
            print(alt_msg.strip())
            
        pixel.pixels_fill(current_color)
        pixel.pixels_show()
    
    counter += 1