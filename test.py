from machine import Pin, I2C

i2c = I2C(
    1,
    scl=Pin(7),
    sda=Pin(6),
    freq=100000
)

print(i2c.scan())