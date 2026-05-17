from machine import Pin, I2C
from time import sleep
#from mpu6050 import MPU6050
from BME280 import BME280

i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)

#mpu = MPU6050(i2c)
bme = BME280(i2c=i2c)

print("Sensors ready")

while True:
    # accel = mpu.read_accel_data()
    # gyro = mpu.read_gyro_data()

    temp, press, hum = bme.read_compensated_data()
    alt = bme.altitude

   ## print("---- MPU ----")
   ## print("Accel:", accel)
   ## print("Gyro :", gyro)

    print("---- BME ----")
    print("Temp:", temp)
    print("Pressure:", press)
    print("Humidity:", hum)
    print("Altitude:", alt, "m")

    print("\n")

    sleep(1)