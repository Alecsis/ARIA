from math import sqrt, atan2
from machine import Pin, I2C
from time import sleep_ms
import struct  # Efficient internal byte conversions

error_msg = "\nError \n"
i2c_err_str = "MPU6050 communication error at address 0x{:02X}"

# Global Constants
_GRAVITIY_MS2 = 9.80665

# Scale Modifiers
_ACC_SCLR_2G = 16384.0
_ACC_SCLR_4G = 8192.0
_ACC_SCLR_8G = 4096.0
_ACC_SCLR_16G = 2048.0

_GYR_SCLR_250DEG = 131.0
_GYR_SCLR_500DEG = 65.5
_GYR_SCLR_1000DEG = 32.8
_GYR_SCLR_2000DEG = 16.4

# Pre-defined configurations
_ACC_RNG_2G = 0x00
_ACC_RNG_4G = 0x08
_ACC_RNG_8G = 0x10
_ACC_RNG_16G = 0x18

_GYR_RNG_250DEG = 0x00
_GYR_RNG_500DEG = 0x08
_GYR_RNG_1000DEG = 0x10
_GYR_RNG_2000DEG = 0x18

# MPU-6050 Registers
_PWR_MGMT_1 = 0x6B
_ACCEL_XOUT0 = 0x3B
_TEMP_OUT0 = 0x41
_GYRO_XOUT0 = 0x43
_ACCEL_CONFIG = 0x1C
_GYRO_CONFIG = 0x1B

_maxFails = 3
_MPU6050_ADDRESS = 0x68

class MPU6050(object):

    def __init__(self, i2c=None, addr=_MPU6050_ADDRESS):
        self.addr = addr

        if i2c is None:
            self.i2c = I2C(0, scl=Pin(5), sda=Pin(4), freq=400000)
        else:
            self.i2c = i2c

        try:
            # Wake up MPU6050 (Clear sleep register)
            self.i2c.writeto_mem(self.addr, _PWR_MGMT_1, bytes([0x00]))
            sleep_ms(10)
        except Exception as e:
            print(i2c_err_str.format(self.addr))
            print(error_msg)
            raise e

        # FIXED: Explicitly safe baseline setup instead of querying uninitialized registers
        self.set_accel_range(_ACC_RNG_2G)
        self.set_gyro_range(_GYR_RNG_250DEG)
        
        self._failCount = 0
        self._terminatingFailCount = 0
    
    def _readData(self, register):
        failCount = 0
        while failCount < _maxFails:
            try:
                # Read 6 sequential raw bytes from register target
                data = self.i2c.readfrom_mem(self.addr, register, 6)
                
                # Optimized Native Unpacking: 
                # '>hhh' parses 6 bytes as 3 Big-Endian Signed 16-bit Integers (x, y, z)
                x, y, z = struct.unpack('>hhh', data)
                return {"x": float(x), "y": float(y), "z": float(z)}
            except Exception:
                failCount += 1
                self._failCount += 1
                sleep_ms(2) # Reduced from 10ms to keep flight software responsive
                
        self._terminatingFailCount += 1
        print(i2c_err_str.format(self.addr))
        return {"x": float("NaN"), "y": float("NaN"), "z": float("NaN")}

    def read_temperature(self):
        try:
            rawData = self.i2c.readfrom_mem(self.addr, _TEMP_OUT0, 2)
            raw_temp = struct.unpack('>h', rawData)[0]
            return (raw_temp / 340.0) + 36.53
        except Exception:
            print(i2c_err_str.format(self.addr))
            return float("NaN")

    def set_accel_range(self, accel_range):
        self.i2c.writeto_mem(self.addr, _ACCEL_CONFIG, bytes([accel_range]))
        self._accel_range = accel_range

    def get_accel_range(self, raw = False):
        try:
            raw_data = self.i2c.readfrom_mem(self.addr, _ACCEL_CONFIG, 1)[0]
            if raw:
                return raw_data
            if raw_data == _ACC_RNG_2G:   return 2
            if raw_data == _ACC_RNG_4G:   return 4
            if raw_data == _ACC_RNG_8G:   return 8
            if raw_data == _ACC_RNG_16G:  return 16
            return -1
        except Exception:
            return -1

    def read_accel_data(self, g = False):         
        accel_data = self._readData(_ACCEL_XOUT0)
        
        # Match current range setting to appropriate scaling divider
        if self._accel_range == _ACC_RNG_2G:     scaler = _ACC_SCLR_2G
        elif self._accel_range == _ACC_RNG_4G:   scaler = _ACC_SCLR_4G
        elif self._accel_range == _ACC_RNG_8G:   scaler = _ACC_SCLR_8G
        elif self._accel_range == _ACC_RNG_16G:  scaler = _ACC_SCLR_16G
        else:                                    scaler = _ACC_SCLR_2G

        x = accel_data["x"] / scaler
        y = accel_data["y"] / scaler
        z = accel_data["z"] / scaler

        if g:
            return {"x": x, "y": y, "z": z}
        else:
            return {"x": x * _GRAVITIY_MS2, "y": y * _GRAVITIY_MS2, "z": z * _GRAVITIY_MS2}

    def read_accel_abs(self, g=False):
        d = self.read_accel_data(g)
        return sqrt(d["x"]**2 + d["y"]**2 + d["z"]**2)

    def set_gyro_range(self, gyro_range):
        self.i2c.writeto_mem(self.addr, _GYRO_CONFIG, bytes([gyro_range]))
        self._gyro_range = gyro_range

    def get_gyro_range(self, raw = False):
        try:
            raw_data = self.i2c.readfrom_mem(self.addr, _GYRO_CONFIG, 1)[0]
            if raw:
                return raw_data
            if raw_data == _GYR_RNG_250DEG:  return 250
            if raw_data == _GYR_RNG_500DEG:  return 500
            if raw_data == _GYR_RNG_1000DEG: return 1000
            if raw_data == _GYR_RNG_2000DEG: return 2000
            return -1
        except Exception:
            return -1

    def read_gyro_data(self):
        gyro_data = self._readData(_GYRO_XOUT0)
        
        if self._gyro_range == _GYR_RNG_250DEG:    scaler = _GYR_SCLR_250DEG
        elif self._gyro_range == _GYR_RNG_500DEG:  scaler = _GYR_SCLR_500DEG
        elif self._gyro_range == _GYR_RNG_1000DEG: scaler = _GYR_SCLR_1000DEG
        elif self._gyro_range == _GYR_RNG_2000DEG: scaler = _GYR_SCLR_2000DEG
        else:                                      scaler = _GYR_SCLR_250DEG

        return {
            "x": gyro_data["x"] / scaler,
            "y": gyro_data["y"] / scaler,
            "z": gyro_data["z"] / scaler
        }

    def read_angle(self): 
        a = self.read_accel_data(g=True)
        x = atan2(a["y"], a["z"])
        y = atan2(-a["x"], a["z"])
        return {"x": x, "y": y}