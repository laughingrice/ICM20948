import time
import spidev

from .defines import *

def twos_comp(val, bits):
    """
    Returns two's complement of value given a number of bits
    """          
    return val - (1 << bits) if val & (1 << (bits - 1)) else val 
    
class ICM20948:
    """
    Class for managing the ICM20948 IMU
    """

    def __init__(self, client=SPI_CLIENT):
        # ============
        # Setup device
        # ============

        self.spi = spidev.SpiDev()
        self.spi.open(0, client)
        self.spi.max_speed_hz = SPI_DATA_RATE

        # Reset device
        self.WriteReg(PWR_MGMT_1, 0x80)
        time.sleep(0.1)
        self.WriteReg(PWR_MGMT_1, 0x01)
        time.sleep(0.1)

        # Set User mode
        # disable i2c and reset dmp and sram
        self.WriteReg(USER_CTRL, (1 << 4) | (1 << 3) | (1 << 2))
        time.sleep(0.1)



        # ===============
        # Check device ID
        # ===============

        res = self.ReadReg(WHO_AM_I_ICM20948)
        if DEBUG_MSGS:
            print('Who am I returned 0x{:02X}, expected 0xEA'.format(res))

        if res != 0xEA:
            raise('Who am I returned 0x{:02X}, expected 0xEA'.format(res))

    def __del__(self):
        self.spi.close()


    def measure(self):
        data = self.ReadACC() + self.ReadGyro() + self.ReadTemp()
        return data

    def ReadACC(self):
        data = self.ReadRegs(ACCEL_XOUT_H, 6)
        data = [twos_comp(x[1] + (x[0] << 8), 16) for x in zip(data[0::2], data[1::2])]

        return data

    def ReadGyro(self):
        data = self.ReadRegs(GYRO_XOUT_H, 6)
        data = [twos_comp(x[1] + (x[0] << 8), 16) for x in zip(data[0::2], data[1::2])]

        return data

    def ReadCompas(self):
        pass

    def ReadTemp(self):
        data = self.ReadRegs(TEMP_OUT_H, 2)
        data = [twos_comp(x[1] + (x[0] << 8), 16) for x in zip(data[0::2], data[1::2])]

        return data

    def WriteReg(self, reg, data):
        return self.WriteRegs(reg, [data])[0]

    def WriteRegs(self, reg, data):
        msg = [reg] + data
        res = self.spi.xfer2(msg)

        return res[1:]

    def ReadReg(self, reg):
        return self.ReadRegs(reg, 1)[0]

    def ReadRegs(self, reg, cnt):
        msg = [reg | READ_FLAG] + [0x00] * cnt
        res = self.spi.xfer2(msg)

        return res[1:]
