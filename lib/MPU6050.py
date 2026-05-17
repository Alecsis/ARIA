from machine import I2C
import time
from math import pow

BME280_ADDRESS = 0x76

REG_DATA = 0xF7
REG_CONTROL = 0xF4
REG_CONTROL_HUM = 0xF2
REG_CONFIG = 0xF5


class BME280:
    def __init__(self, i2c, address=BME280_ADDRESS):
        self.i2c = i2c
        self.addr = address

        # calibration data
        self.dig_T1 = self._read16(0x88)
        self.dig_T2 = self._readS16(0x8A)
        self.dig_T3 = self._readS16(0x8C)

        self.dig_P1 = self._read16(0x8E)
        self.dig_P2 = self._readS16(0x90)
        self.dig_P3 = self._readS16(0x92)
        self.dig_P4 = self._readS16(0x94)
        self.dig_P5 = self._readS16(0x96)
        self.dig_P6 = self._readS16(0x98)
        self.dig_P7 = self._readS16(0x9A)
        self.dig_P8 = self._readS16(0x9C)
        self.dig_P9 = self._readS16(0x9E)

        self.dig_H1 = self._read8(0xA1)

        self.t_fine = 0

        # init sensor
        self.i2c.writeto_mem(self.addr, REG_CONTROL_HUM, b"\x01")
        self.i2c.writeto_mem(self.addr, REG_CONTROL, b"\x27")
        self.i2c.writeto_mem(self.addr, REG_CONFIG, b"\xA0")

        self.sea_level = 101325  # Pa

    # ---------- raw reads ----------
    def _read8(self, reg):
        return self.i2c.readfrom_mem(self.addr, reg, 1)[0]

    def _read16(self, reg):
        data = self.i2c.readfrom_mem(self.addr, reg, 2)
        return data[0] | (data[1] << 8)

    def _readS16(self, reg):
        val = self._read16(reg)
        if val > 32767:
            val -= 65536
        return val

    def read_raw(self):
        data = self.i2c.readfrom_mem(self.addr, REG_DATA, 8)

        adc_p = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        adc_t = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        adc_h = (data[6] << 8) | data[7]

        return adc_t, adc_p, adc_h

    # ---------- main API ----------
    def read_compensated_data(self):
        adc_t, adc_p, adc_h = self.read_raw()

        # temp
        var1 = (((adc_t >> 3) - (self.dig_T1 << 1)) * self.dig_T2) >> 11
        var2 = (((((adc_t >> 4) - self.dig_T1) *
                 ((adc_t >> 4) - self.dig_T1)) >> 12) *
                self.dig_T3) >> 14

        self.t_fine = var1 + var2
        temp = (self.t_fine * 5 + 128) >> 8
        temp_c = temp / 100.0

        # pressure
        var1 = self.t_fine - 128000
        var2 = var1 * var1 * self.dig_P6
        var2 += (var1 * self.dig_P5) << 17
        var2 += self.dig_P4 << 35
        var1 = ((var1 * var1 * self.dig_P3) >> 8) + ((var1 * self.dig_P2) << 12)
        var1 = (((1 << 47) + var1) * self.dig_P1) >> 33

        if var1 == 0:
            pressure = 0
        else:
            p = 1048576 - adc_p
            p = ((p << 31) - var2) * 3125 // var1
            var1 = (self.dig_P9 * (p >> 13) * (p >> 13)) >> 25
            var2 = (self.dig_P8 * p) >> 19
            pressure = ((p + var1 + var2) >> 8)

        hum = adc_h

        return temp, pressure, hum

    # ---------- altitude ----------
    @property
    def altitude(self):
        _, pressure, _ = self.read_compensated_data()
        return 44330 * (1.0 - pow(pressure / self.sea_level, 0.1903))