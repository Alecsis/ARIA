from machine import I2C
import time
from ustruct import unpack

BME280_I2CADDR = 0x76

_REG_CALIB_00 = 0x88
_REG_CALIB_26 = 0xE1
_REG_DATA = 0xF7
_REG_CTRL_HUM = 0xF2
_REG_CTRL = 0xF4
_REG_STATUS = 0xF3

class BME280:
    def __init__(self, i2c, address=BME280_I2CADDR):
        self.i2c = i2c
        self.address = address

        self.t_fine = 0
        self.sea_level_pressure = 101325.0
        self.baseline_pressure = 101325.0  # Set default safe fallback value

        cal1 = self.i2c.readfrom_mem(self.address, _REG_CALIB_00, 26)
        cal2 = self.i2c.readfrom_mem(self.address, _REG_CALIB_26, 7)

        self.dig_T1, self.dig_T2, self.dig_T3, \
        self.dig_P1, self.dig_P2, self.dig_P3, self.dig_P4, \
        self.dig_P5, self.dig_P6, self.dig_P7, self.dig_P8, self.dig_P9, \
        _, self.dig_H1 = unpack("<HhhHhhhhhhhhBB", cal1)

        self.dig_H2, self.dig_H3, self.dig_H4, self.dig_H5, self.dig_H6 = unpack("<hBbhb", cal2)
        self.dig_H4 = (self.dig_H4 * 16) | (self.dig_H5 & 0x0F)
        self.dig_H5 >>= 4

        self.i2c.writeto_mem(self.address, _REG_CTRL, b"\x00")

    def _read_raw(self):
        self.i2c.writeto_mem(self.address, _REG_CTRL_HUM, b"\x01")
        self.i2c.writeto_mem(self.address, _REG_CTRL, b"\x25") 

        while self.i2c.readfrom_mem(self.address, _REG_STATUS, 1)[0] & 0x08:
            time.sleep_ms(5)

        buf = bytearray(8)
        self.i2c.readfrom_mem_into(self.address, _REG_DATA, buf)

        raw_press = ((buf[0] << 16) | (buf[1] << 8) | buf[2]) >> 4
        raw_temp = ((buf[3] << 16) | (buf[4] << 8) | buf[5]) >> 4
        raw_hum = (buf[6] << 8) | buf[7]

        return raw_temp, raw_press, raw_hum

    def read_compensated_data(self):
        raw_temp, raw_press, raw_hum = self._read_raw()

        var1 = ((raw_temp / 16384.0) - (self.dig_T1 / 1024.0)) * self.dig_T2
        var2 = (((raw_temp / 131072.0) - (self.dig_T1 / 8192.0)) ** 2) * self.dig_T3
        self.t_fine = var1 + var2
        temp = self.t_fine / 5120.0

        var1 = (self.t_fine / 2.0) - 64000.0
        var2 = var1 * var1 * self.dig_P6 / 32768.0
        var2 += var1 * self.dig_P5 * 2.0
        var2 = (var2 / 4.0) + (self.dig_P4 * 65536.0)

        var1 = (self.dig_P3 * var1 * var1 / 524288.0 + self.dig_P2 * var1) / 524288.0
        var1 = (1.0 + var1 / 32768.0) * self.dig_P1

        if var1 == 0:
            pressure = 0
        else:
            p = 1048576.0 - raw_press
            p = ((p - (var2 / 4096.0)) * 6250.0) / var1
            var1 = self.dig_P9 * p * p / 2147483648.0
            var2 = p * self.dig_P8 / 32768.0
            pressure = p + var1 + var2 + self.dig_P7

        h = self.t_fine - 76800.0
        h = (raw_hum - (self.dig_H4 * 64.0 + self.dig_H5 / 16384.0 * h)) * \
            (self.dig_H2 / 65536.0 * (1.0 + self.dig_H6 / 67108864.0 * h *
            (1.0 + self.dig_H3 / 67108864.0 * h)))

        humidity = h * (1.0 - self.dig_H1 * h / 524288.0)
        humidity = max(0.0, min(100.0, humidity))

        return temp, pressure, humidity

    def calibrate(self, samples=20, delay_ms=50):
        total = 0
        valid_samples = 0
        
        # Discard the first 5 readings to let the sensor's internal calculation stabilize
        for _ in range(5):
            self.read_compensated_data()
            time.sleep_ms(delay_ms)

        for _ in range(samples):
            try:
                _, p, _ = self.read_compensated_data()
                if p > 0:
                    total += p
                    valid_samples += 1
            except:
                pass
            time.sleep_ms(delay_ms)

        if valid_samples > 0:
            self.baseline_pressure = total / valid_samples
        else:
            self.baseline_pressure = self.sea_level_pressure
        return self.baseline_pressure

    def altitude(self, pressure=None):
        if pressure is None:
            _, pressure, _ = self.read_compensated_data()

        if pressure == 0:
            return 0.0

        # Calculate altitude relative to ground calibration baseline
        alt = 44330.0 * (1.0 - (pressure / self.baseline_pressure) ** 0.1903)
        
        # Ground clamping: No negative altitudes at launch pad
        if alt < 0.0:
            return 0.0
        return alt