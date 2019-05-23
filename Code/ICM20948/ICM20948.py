import time
import spidev

from .defines import *

class ICM20948:
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

        res = ReadReg(WHO_AM_I_ICM20948)
        if DEBUG_MSGS:
            print('Who am I returned 0x{:02X}, expected 0xEA'.format(res[1]))

        if res[1] != 0xEA:
            raise('Who am I returned 0x{:02X}, expected 0xEA'.format(res[1]))

    def __del__(self):
        self.spi.close()


    def measure(self):
        pass

    def ReadACC(self):
        acc_x_l = 0
        
    def WriteReg(self, reg, data):
        msg = [reg, data]
        res = self.spi.xfer2(msg)

        return res

    def ReadReg(self, reg):
        msg = [reg | READ_FLAG, 0x00]
        res = self.spi.xfer2(msg)

        return res

