import time
from .defines import *
from .bus import SPI_Bus

def twos_comp(val: int, bits: int) -> int:
    '''
    Returns two's complement of value given a number of bits

    :param val: value to convert
    :param bits: number of bits assumed for the conversion

    :returns: two's complement value
    '''          
    return val - (1 << bits) if val & (1 << (bits - 1)) else val 


class ICM20948:
    '''
    Class for managing the ICM20948 IMU
    '''

    def __init__(self) -> None:
        '''
        Initialize class

        :param client: SPI client to to use (0 or 1 on the PI, coresponding to CS pin)
        '''

        self._acc = None
        self.ICM20948_initialized = False
        self.AK09916_initialized = False

        self.user_ctrl = 0

    def _setup(self, bus=None) -> None:
        '''
        Perform actual setup and and intialization
        '''

        if bus is not None:
            self._acc = bus

        self._setup_ICM20948()
        self._setup_AK09916()

    def _setup_ICM20948(self) -> bool:
        '''
        Setup ICM20948
        '''

        if self._acc is None:
            raise Exception('SPI not initialized yet')

        if self.ICM20948_initialized:
            return

        self._acc.WriteReg(PWR_MGMT_1, 0x80) # Reset device
        time.sleep(0.1)

        # Verify device ID
        res = self._acc.ReadReg(WHO_AM_I_ICM20948)
        if res != ICM20948_ID:
            raise Exception('ICM20948 who am I returned 0x{:02X}, expected 0x{:02X}'.format(res, ICM20948_ID))

        self._acc.WriteReg(PWR_MGMT_1, 0x01) # Auto select clock
        self._acc.WriteReg(PWR_MGMT_2, 0x00) # Enable all devices

        # Disable i2c slave if we are in SPI mode, to avoid accidental switch
        self.user_ctrl = self._acc.ReadReg(USER_CTRL)
        if type(self._acc) is SPI_Bus:
            self.user_ctrl |= (1 << 4)
            self._acc.WriteReg(USER_CTRL, self.user_ctrl)

        self.SelectBank(REG_BANK_2)

        # Setup sampling rate, rate divider if LPF is enabled, rate = 1125Hz / (1 + divider)
        self._acc.WriteReg(ACCEL_SMPLRT_DIV_2, 4)
        self._acc.WriteReg(GYRO_SMPLRT_DIV, 4)

        # Accelerometer range and LPF: +-4g, 70Hz NBW low pass
        self._acc.WriteReg(ACCEL_CONFIG, 1 | (1 << 1) | (3 << 3))
        self.acc_scale = 4.0 / (1 << 15)
        # Gyro range and LPF: +-500 dps, 73.3 NBW Hz low pass
        self._acc.WriteReg(GYRO_CONFIG_1, 1 | (1 << 1) | (3 << 3))
        self.gyro_scale = 500.0 / (1 << 15)
        # Temp LPF 65.9 NBW low pass 
        self._acc.WriteReg(TEMP_CONFIG, 3)

        self.SelectBank(REG_BANK_0)

        self.ICM20948_initialized = True


    def _setup_AK09916(self):
        '''
        Setup magnetometer

        Requires that ICM20948 be initialized first
        '''

        if self._acc is None:
            raise Exception('SPI not initialized yet')

        if self.AK09916_initialized:
            return

        # Disable interupt bypass
        self._acc.WriteReg(INT_PIN_CFG, 0x30)

        self.user_ctrl |= 0x20 # enable i2c master mode (to control magnetometer)
        self._acc.WriteReg(USER_CTRL, self.user_ctrl)

        # Configure the i2c master
        self.SelectBank(REG_BANK_3)

        self._acc.WriteReg(I2C_MST_CTRL, 0x1D)
        self._acc.WriteReg(I2C_MST_DELAY_CTRL, 0x01)

        # External device sampling rate if Gyro and ACC are disabled, 1.1KHz/2^val
        # self._acc.WriteReg(I2C_MST_ODR_CONFIG, 2)

        # Sensor ID
        res = self.ReadMagReg(WHO_AM_I_AK09916)
        if res != AK09916_ID:
            raise Exception('AK09916 who am I returned 0x{:02X}, expected 0x{:02X}'.format(res, AK09916_ID))

       # Soft reset
        self.WriteMagReg(AK09916_CTRL_3, 0x01)
        while self.ReadMagReg(AK09916_CTRL_3) == 0x01:
            time.sleep(0.0001)

        self.compass_scale = 0.15

        self.AK09916_initialized = True


    def InitDPM(self) -> None:
        '''
        Load the DPM firmware
        '''

        # Reset memories, enable DPM and FIFO
        self._acc.WriteReg(USER_CTRL, 0xFE)
        time.sleep(0.1)

        from .icm20948_img_dmp3a import dmp_img

        mem_bank = 0
        start_address = DMP_LOAD_START
        data_pos = 0

        # Write dpm firmware to memory
        while data_pos < len(dmp_img):
            write_len = min((256 - start_address, len(dmp_img[data_pos:])))
            self.WriteMems(mem_bank, start_address, dmp_img[data_pos:data_pos + write_len])

            data_pos += write_len
            mem_bank += 1
            start_address = 0


    def ValidateDPM(self) -> None:
        '''
        Validate the DPM firmware
        '''

        from .icm20948_img_dmp3a import dmp_img  # So that we know how much to read  

        read_img = [0] * len(dmp_img)

        mem_bank = 0
        start_address = DMP_LOAD_START
        data_pos = 0

        # Write dpm firmware to memory
        while data_pos < len(dmp_img):
            read_len = min((256 - start_address, len(dmp_img[data_pos:])))
            read_img[data_pos:data_pos + read_len] = self.ReadMems(mem_bank, start_address, read_len)

            data_pos += read_len
            mem_bank += 1
            start_address = 0

        diff = 0
        for z in zip(dmp_img, read_img):
            diff += abs(z[1] - z[0])

        return diff == 0


    def measure(self) -> list:
        '''
        Read sensor data

        :returns: list of accelerometer + gyro + compass + temperature values
        '''

        data = self.ReadACC() + self.ReadGyro()+ self.ReadCompas() + self.ReadTemp()

        return data


    def ReadACC(self) -> list:
        '''
        Read accelerometer data

        :returns: list of three accelerometer values 
        '''

        if not self.ICM20948_initialized:
            return ['NaN', 'NaN', 'NaN']

        data = self._acc.ReadRegs(ACCEL_XOUT_H, 6)
        data = [twos_comp(x[1] + (x[0] << 8), 16) * self.acc_scale for x in zip(data[0::2], data[1::2])]

        return data


    def ReadGyro(self) -> list:
        '''
        Read gyro data

        :returns: list of three gyro values 
        '''

        if not self.ICM20948_initialized:
            return ['NaN', 'NaN', 'NaN']

        data = self._acc.ReadRegs(GYRO_XOUT_H, 6)
        data = [twos_comp(x[1] + (x[0] << 8), 16) * self.gyro_scale for x in zip(data[0::2], data[1::2])]

        return data


    def ReadCompas(self) -> list:
        '''
        Read magnetometer data

        :returns: list of three magnetometr values 
        '''

        if not self.AK09916_initialized:
            return ['NaN', 'NaN', 'NaN']

        # Trigger measurement
        self.WriteMagReg(AK09916_CTRL_2, 0x08)

        # Give the sensor a change if it is not ready, but do not wait too long not to lock up the system
        if not self.IsMagReady():
            time.sleep(0.0001)

        data = self.ReadMagRegs(AK09916_XOUT_L, 8)[:6] # Also reads ST2, dump the value as we don't use it now
        data = [twos_comp(x[1] + (x[0] << 8), 16) * self.compass_scale for x in zip(data[0::2], data[1::2])]

        return data


    def ReadTemp(self) -> float:
        '''
        Read temperature data

        :returns: temperature in celsius
        '''

        if not self.ICM20948_initialized:
            return ['NaN']

        data = self._acc.ReadRegs(TEMP_OUT_H, 2)
        data = [(twos_comp(x[1] + (x[0] << 8), 16) - 21) * 0.003 + 21.0 for x in zip(data[0::2], data[1::2])]

        return data


    def SelectBank(self, bank: int) -> None:
        '''
        Select device register bank

        :param bank: register bank to set
        '''

        self._acc.WriteReg(BANK_SEL, bank)


    def WriteMems(self, bank: int, address: int, data: list) -> None:
        '''
        Write data to device memory

        :param bank: Memory bank to write to
        :param address: memory address to write to
        :param data: list of byte values to write
        '''

        self.SelectBank(bank)
        
        # TODO: it might be possiple to write INV_MAX_SERIAL_WRITE bytes at a time
        for d in data:
            self._acc.WriteReg(MEM_START_ADDR, address)
            self._acc.WriteReg(MEM_R_W, d)
            address += 1


    def ReadMems(self, bank: int, address: int, len: int):
        '''
        Read data from device memory

        :param bank: Memory bank to read from
        :param address: memory address to read from
        :param len: length of data to read

        :returns: list of byte data values
        '''

        read_data = [0] * len

        self.SelectBank(bank)
        
        # TODO: it might be possiple to read INV_MAX_SERIAL_WRITE bytes at a time
        for i in range(len):
            self._acc.WriteReg(MEM_START_ADDR, address)
            read_data[i] = self._acc.ReadReg(MEM_R_W)
            address += 1

        return read_data


    def ReadMagReg(self, reg: int) -> int:
        '''
        Read magnetometer register
        # Note: disables continuous reading

        :param reg: register to read

        :returns: register value
        '''

        return self.ReadMagRegs(reg, 1)[0]


    def ReadMagRegs(self, reg: int, length: int=1) -> list:
        '''
        Read magnetometer registers
        # Note: disables continuous reading

        :param reg: first register to read
        :param length: number of registers to read

        :returns: register values
        '''

        self.SelectBank(REG_BANK_3)

        self._acc.WriteReg(I2C_SLV0_CTRL, 0x00) # Disable while setting up
        self._acc.WriteReg(I2C_SLV0_ADDR, 0x80 | AK09916_I2C_ADDRESS)
        self._acc.WriteReg(I2C_SLV0_REG, reg)
        self._acc.WriteReg(I2C_SLV0_DO, 0xff)
        self._acc.WriteReg(I2C_SLV0_CTRL, 0xD0 | length)

        self.SelectBank(REG_BANK_0)

        time.sleep(1e-3)

        data = self._acc.ReadRegs(EXT_SENS_DATA_00, length)

        return data


    def WriteMagReg(self, reg:int, data:int) -> None:
        '''
        Write magnetometer register
        Note: disables continuous reading

        :param reg: register to write
        '''

        self.SelectBank(REG_BANK_3)

        self._acc.WriteReg(I2C_SLV0_CTRL, 0x00) # Disable while setting up
        self._acc.WriteReg(I2C_SLV0_ADDR, AK09916_I2C_ADDRESS)
        self._acc.WriteReg(I2C_SLV0_REG, reg)
        self._acc.WriteReg(I2C_SLV0_DO, data)
        self._acc.WriteReg(I2C_SLV0_CTRL, 0x81)

        self.SelectBank(REG_BANK_0)


    def IsMagReady(self):
        '''
        Check the magnetometer status self.ready bit
        '''
        return (self.ReadMagReg(AK09916_ST1) & 0x01) > 0

