import spidev

class I2C_Bus:
    def __init__(self):
        pass


class SPI_Bus:
    def __init__(self) -> None:
        '''
        Setup SPI
        '''

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


    def ReadRegs(self, reg: int, cnt: int) -> None:
        '''
        Read consecutive cnt registers

        :param reg: First register to read
        :param cnt: number of register values to read

        :returns: list of register values (leading zero is stripped)
        '''

        msg = [reg | READ_FLAG] + [0x00] * cnt
        res = self._acc.xfer2(msg)

        return res[1:]


    def WriteRegs(self, reg, data):
        '''
        Write consecutive cnt registers

        :param reg: First register to write to
        :param cnt: number of register values to write
        :oaram data: list of data byte to write
        '''

        msg = [reg] + data
        res = self._acc.xfer2(msg)

        return res[1:]