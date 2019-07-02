from .defines import *

try:
    import smbus

    class I2C_Bus:
        def __init__(self):
            '''
            Setup I2C
            '''
            
            self._acc = smbus.SMBus(I2C_PORT)


        def ReadReg(self, reg: int) -> int:
            '''
            Read register value

            :param reg: register to read

            :returns: register value
            '''

            # TODO: Looks like READ_FLAG is not needed for I2C
            return self._acc.read_byte_data(ICM20948_I2C_ADDRESS, reg | READ_FLAG)


        def ReadRegs(self, reg: int, cnt: int) -> list:
            '''
            Read consecutive cnt registers

            :param reg: First register to read
            :param cnt: number of register values to read

            :returns: list of register values
            '''

            return self._acc.read_i2c_block_data(ICM20948_I2C_ADDRESS, reg | READ_FLAG, cnt)


        def WriteReg(self, reg: int, data: int) -> None:
            '''
            Write registers value

            :param reg: register to write to
            :oaram data: data byte to write
            '''

            self._acc.write_byte_data(ICM20948_I2C_ADDRESS, reg, data)


        def WriteRegs(self, reg: int, data: list) -> None:
            '''
            Write consecutive cnt registers

            :param reg: First register to write to
            :oaram data: list of data byte to write
            '''

            self._acc.write_i2c_block_data(ICM20948_I2C_ADDRESS, reg, data)

except ImportError:
    print("Warning: smbus not installed")


try:
    import spidev

    class SPI_Bus:
        def __init__(self) -> None:
            '''
            Setup SPI
            '''

            self._acc = None

            acc = spidev.SpiDev()
            acc.open(0, SPI_CLIENT)
            acc.max_speed_hz = SPI_DATA_RATE # Seems to be required for the device to function

            self._acc = acc


        def __del__(self) -> None:
            '''
            Destructor
            '''

            if self._acc:
                self._acc.close()


        def ReadReg(self, reg: int) -> int:
            '''
            Read register value

            :param reg: register to read

            :returns: register value
            '''

            return self.ReadRegs(reg, 1)[0]


        def ReadRegs(self, reg: int, cnt: int) -> list:
            '''
            Read consecutive cnt registers

            :param reg: First register to read
            :param cnt: number of register values to read

            :returns: list of register values (leading zero is stripped)
            '''

            msg = [reg | READ_FLAG] + [0x00] * cnt
            res = self._acc.xfer2(msg)

            return res[1:]


        def WriteReg(self, reg: int, data: list) -> None:
            '''
            Write registers value

            :param reg: register to write to
            :oaram data: data byte to write
            '''

            self.WriteRegs(reg, [data])


        def WriteRegs(self, reg: int, data: list) -> None:
            '''
            Write consecutive cnt registers

            :param reg: First register to write to
            :param cnt: number of register values to write
            :oaram data: list of data byte to write
            '''

            msg = [reg] + data
            self._acc.xfer2(msg)

except ImportError:
    print("Warning: spidev not installed")