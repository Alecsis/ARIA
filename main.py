from machine import Pin, I2C
from time import sleep_ms
from lib.MPU6050 import MPU6050  # your file name
import os

i2c = I2C(
    0,
    scl=Pin(5), 
    sda=Pin(4),
    freq=400000
)
print("I2C Scan:", i2c.scan())
mpu = MPU6050(i2c)
print("MPU6050 initialized")
while True:
    accel = mpu.read_accel_data()   # m/s^2
    gyro = mpu.read_gyro_data()     # deg/s
    temp = mpu.read_temperature()   # °C
    print("----- MPU6050 -----")
    print("Accel (m/s^2):", accel)
    print("Gyro  (deg/s):", gyro)
    print("Temp  (C):", temp)
    angle = mpu.read_angle()
    print("Angle (rad):", angle)
    print("-------------------\n")
    sleep_ms(100)