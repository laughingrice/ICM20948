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
        self.InitICM20948(client)


    def __del__(self):
        self.spi.close()


    def InitICM20948(self, client):
        """
        Setup device
        """

        # ==============
        # Setup ICM20948
        # ==============

        self.spi = spidev.SpiDev()
        self.spi.open(0, client)
        self.spi.max_speed_hz = SPI_DATA_RATE

        # Reset device
        self.WriteReg(PWR_MGMT_1, 0x80)
        time.sleep(0.1)
        self.WriteReg(PWR_MGMT_1, 0x01)
        time.sleep(0.1)

        # Set User mode
        # disable i2c slave, reset dmp and sram, enable i2c master
        self.WriteReg(USER_CTRL, 0x3C)

        # Setup ranges
        self.SelectBank(REG_BANK_2)

        self.WriteReg(GYRO_SMPLRT_DIV, 4) # Rate divider if LPF is enabled, rate = 1125Hz / (1 + GYRO_SMPLRT_DIV)
        self.WriteReg(ACCEL_SMPLRT_DIV_2, 4) # Rate divider if LPF is enabled, rate = 1125Hz / (1 + ACCEL_SMPLRT_DIV)

        self.WriteReg(ACCEL_CONFIG, 1 | (1 << 1) | (3 << 3)) # +-4g, 70Hz NBW low pass filter
        self.WriteReg(GYRO_CONFIG_1, 1 | (1 << 1) | (3 << 3)) # +-500 dps, 73.3 NBW Hz low pass
        self.WriteReg(TEMP_CONFIG, 3) # 65.9 NBW low pass

        self.SelectBank(REG_BANK_0)

        self.acc_scale = 4.0 / (1 << 15)
        self.gyro_scale = 500.0 / (1 << 15)
        self.temp_scale = 0.01

        time.sleep(0.1)

        # Check device ID

        res = self.ReadReg(WHO_AM_I_ICM20948)
        if res != ICM20948_ID:
            raise Exception('ICM20948 who am I returned 0x{:02X}, expected 0x{:02X}'.format(res, ICM20948_ID))

        # ==================
        # Setup Magnetometer
        # ==================

        self.SelectBank(REG_BANK_3)

        # External device sampling rate if Gyro and ACC are disabled, 1.1KHz/2^val
        self.WriteReg(I2C_MST_ODR_CONFIG, 2)
        # Stop between reads (instead of reset)
        self.WriteReg(I2C_MST_CTRL, 0x10)

        # Soft reset
        self.WriteReg(I2C_SLV0_ADDR, AK09916_ADDRESS)
        self.WriteReg(I2C_SLV0_REG, AK09916_CNTL3)
        self.WriteReg(I2C_SLV0_DO, 0x01)
        self.WriteReg(I2C_SLV0_CTRL, 0x81)

        time.sleep(0.5)

        # Sensor ID
        self.WriteReg(I2C_SLV0_ADDR, AK09916_ADDRESS | 0x80)
        self.WriteReg(I2C_SLV0_REG, WHO_AM_I_AK09916)
        self.WriteReg(I2C_SLV0_CTRL, 0x91)

        self.SelectBank(REG_BANK_0)
        time.sleep(0.2)

        res = self.ReadReg(EXT_SENS_DATA_00)
        if res != AK09916_ID:
            raise Exception('AK09916 who am I returned 0x{:02X}, expected 0x{:02X}'.format(res, AK09916_ID))

        self.SelectBank(REG_BANK_3)

        # Set to 100Hz data collection rate
        self.WriteReg(I2C_SLV0_ADDR, AK09916_ADDRESS)
        self.WriteReg(I2C_SLV0_REG, AK09916_CNTL2)
        self.WriteReg(I2C_SLV0_DO, 0x08)
        self.WriteReg(I2C_SLV0_CTRL, 0x81)

        time.sleep(0.2)

        # Collect data from slave at GYRO sampling rate
        # We also read the data overflow register ST1 and magnetic overflow ST2 (& 0x08) but do not use them at the moment
        self.WriteReg(I2C_SLV0_ADDR, AK09916_ADDRESS | 0x80)
        self.WriteReg(I2C_SLV0_REG, AK09916_ST1)
        self.WriteReg(I2C_SLV0_CTRL, 0x99)

        self.SelectBank(REG_BANK_0)

        self.compass_scale = 0.15

        time.sleep(0.2)


    def InitDPM(self):
        # Reset memories, enable DPM and FIFO
        self.WriteReg(USER_CTRL, 0xFE)
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


    def ValidateDPM(self):
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


    def measure(self):
        data = self.ReadACC() + self.ReadGyro() + self.ReadCompas() + self.ReadTemp()
        return data


    def ReadACC(self):
        data = self.ReadRegs(ACCEL_XOUT_H, 6)
        data = [twos_comp(x[1] + (x[0] << 8), 16) * self.acc_scale for x in zip(data[0::2], data[1::2])]

        return data


    def ReadGyro(self):
        data = self.ReadRegs(GYRO_XOUT_H, 6)
        data = [twos_comp(x[1] + (x[0] << 8), 16) * self.gyro_scale for x in zip(data[0::2], data[1::2])]

        return data


    def ReadCompas(self):
        data = self.ReadRegs(EXT_SENS_DATA_01, 6)
        data = [twos_comp(x[0] + (x[1] << 8), 16) * self.compass_scale for x in zip(data[0::2], data[1::2])]

        return data


    def ReadTemp(self):
        data = self.ReadRegs(TEMP_OUT_H, 2)
        data = [twos_comp(x[1] + (x[0] << 8), 16) * self.temp_scale for x in zip(data[0::2], data[1::2])]

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


    def SelectBank(self, bank):
        self.WriteReg(BANK_SEL, bank)


    def WriteMems(self, bank, address, data):
        self.WriteReg(MEM_BANK_SEL, bank)
        
        # TODO: it might be possiple to write INV_MAX_SERIAL_WRITE bytes at a time
        for d in data:
            self.WriteReg(MEM_START_ADDR, address)
            self.WriteReg(MEM_R_W, d)
            address += 1


    def ReadMems(self, bank, address, len):
        read_data = [0] * len

        self.WriteReg(MEM_BANK_SEL, bank)
        
        # TODO: it might be possiple to read INV_MAX_SERIAL_WRITE bytes at a time
        for i in range(len):
            self.WriteReg(MEM_START_ADDR, address)
            read_data[i] = self.ReadReg(MEM_R_W)
            address += 1

        return read_data


    def SelfTest(self):
        """
        Perform self test
        TODO: Not translated from C yet
        """
        pass


    def SelfCalibrate(self):
        """
        Perform self calibration (at rest)
        TODO: Not translated from C yet
        """
        pass

# Accelerometer and gyroscope self test; check calibration wrt factory settings
# Should return percent deviation from factory trim values, +/- 14 or less
# deviation is a pass.
# void ICM20948::ICM20948SelfTest(float *destination)
# {
#   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
#   uint8_t selfTest[6];
#   int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
#   float factoryTrim[6];
#   uint8_t FS = 0;

#   // Get stable time source
#   // Auto select clock source to be PLL gyroscope reference if ready else
#   writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
#   delay(200);

#   // Switch to user bank 2
#   writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);
#   // Set gyro sample rate to 1 kHz
#   writeByte(ICM20948_ADDRESS, GYRO_SMPLRT_DIV, 0x00);
#   // Set gyro sample rate to 1 kHz, DLPF to 119.5 Hz and FSR to 250 dps
#   writeByte(ICM20948_ADDRESS, GYRO_CONFIG_1, 0x11);
#   // Set accelerometer rate to 1 kHz and bandwidth to 111.4 Hz
#   // Set full scale range for the accelerometer to 2 g
#   writeByte(ICM20948_ADDRESS, ACCEL_CONFIG, 0x11);
#   // Switch to user bank 0
#   writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);

#   // Get average current values of gyro and acclerometer
#   for (int ii = 0; ii < 200; ii++)
#   {
#     Serial.print("BHW::ii = ");
#     Serial.println(ii);
#     // Read the six raw data registers into data array
#     readBytes(ICM20948_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
#     // Turn the MSB and LSB into a signed 16-bit value
#     aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
#     aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
#     aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

#     // Read the six raw data registers sequentially into data array
#     readBytes(ICM20948_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
#     // Turn the MSB and LSB into a signed 16-bit value
#     gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
#     gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
#     gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
#   }

#   // Get average of 200 values and store as average current readings
#   for (int ii = 0; ii < 3; ii++)
#   {
#     aAvg[ii] /= 200;
#     gAvg[ii] /= 200;
#   }

#   // Switch to user bank 2
#   writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);

#   // Configure the accelerometer for self-test
#   // Enable self test on all three axes and set accelerometer range to +/- 2 g
#   writeByte(ICM20948_ADDRESS, ACCEL_CONFIG_2, 0x1C);
#   // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
#   writeByte(ICM20948_ADDRESS, GYRO_CONFIG_2, 0x38);
#   delay(25); // Delay a while to let the device stabilize

#   // Switch to user bank 0
#   writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);

#   // Get average self-test values of gyro and acclerometer
#   for (int ii = 0; ii < 200; ii++)
#   {
#     // Read the six raw data registers into data array
#     readBytes(ICM20948_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
#     // Turn the MSB and LSB into a signed 16-bit value
#     aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
#     aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
#     aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);

#     // Read the six raw data registers sequentially into data array
#     readBytes(ICM20948_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
#     // Turn the MSB and LSB into a signed 16-bit value
#     gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
#     gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
#     gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
#   }

#   // Get average of 200 values and store as average self-test readings
#   for (int ii = 0; ii < 3; ii++)
#   {
#     aSTAvg[ii] /= 200;
#     gSTAvg[ii] /= 200;
#   }

#   // Switch to user bank 2
#   writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);

#   // Configure the gyro and accelerometer for normal operation
#   writeByte(ICM20948_ADDRESS, ACCEL_CONFIG_2, 0x00);
#   writeByte(ICM20948_ADDRESS, GYRO_CONFIG_2, 0x00);
#   delay(25); // Delay a while to let the device stabilize

#   // Switch to user bank 1
#   writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x10);

#   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
#   // X-axis accel self-test results
#   selfTest[0] = readByte(ICM20948_ADDRESS, SELF_TEST_X_ACCEL);
#   // Y-axis accel self-test results
#   selfTest[1] = readByte(ICM20948_ADDRESS, SELF_TEST_Y_ACCEL);
#   // Z-axis accel self-test results
#   selfTest[2] = readByte(ICM20948_ADDRESS, SELF_TEST_Z_ACCEL);
#   // X-axis gyro self-test results
#   selfTest[3] = readByte(ICM20948_ADDRESS, SELF_TEST_X_GYRO);
#   // Y-axis gyro self-test results
#   selfTest[4] = readByte(ICM20948_ADDRESS, SELF_TEST_Y_GYRO);
#   // Z-axis gyro self-test results
#   selfTest[5] = readByte(ICM20948_ADDRESS, SELF_TEST_Z_GYRO);

#   // Switch to user bank 0
#   writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);

#   // Retrieve factory self-test value from self-test code reads
#   // FT[Xa] factory trim calculation
#   factoryTrim[0] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)selfTest[0] - 1.0)));
#   // FT[Ya] factory trim calculation
#   factoryTrim[1] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)selfTest[1] - 1.0)));
#   // FT[Za] factory trim calculation
#   factoryTrim[2] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)selfTest[2] - 1.0)));
#   // FT[Xg] factory trim calculation
#   factoryTrim[3] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)selfTest[3] - 1.0)));
#   // FT[Yg] factory trim calculation
#   factoryTrim[4] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)selfTest[4] - 1.0)));
#   // FT[Zg] factory trim calculation
#   factoryTrim[5] = (float)(2620 / 1 << FS) * (pow(1.01, ((float)selfTest[5] - 1.0)));

#   // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim
#   // of the Self-Test Response
#   // To get percent, must multiply by 100
#   for (int i = 0; i < 3; i++)
#   {
#     // Report percent differences
#     destination[i] = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.;
#     // Report percent differences
#     destination[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.;
#   }
# }

# // Function which accumulates gyro and accelerometer data after device
# // initialization. It calculates the average of the at-rest readings and then
# // loads the resulting offsets into accelerometer and gyro bias registers.
# void ICM20948::calibrateICM20948(float *gyroBias, float *accelBias)
# {
#   uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
#   uint16_t ii, packet_count, fifo_count;
#   int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

#   // reset device
#   // Write a one to bit 7 reset bit; toggle reset device
#   writeByte(ICM20948_ADDRESS, PWR_MGMT_1, READ_FLAG);
#   delay(200);

#   // get stable time source; Auto select clock source to be PLL gyroscope
#   // reference if ready else use the internal oscillator, bits 2:0 = 001
#   writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x01);
#   delay(200);

#   // Configure device for bias calculation
#   // Disable all interrupts
#   writeByte(ICM20948_ADDRESS, INT_ENABLE, 0x00);
#   // Disable FIFO
#   writeByte(ICM20948_ADDRESS, FIFO_EN_1, 0x00);
#   writeByte(ICM20948_ADDRESS, FIFO_EN_2, 0x00);
#   // Turn on internal clock source
#   writeByte(ICM20948_ADDRESS, PWR_MGMT_1, 0x00);
#   // Disable I2C master
#   //writeByte(ICM20948_ADDRESS, I2C_MST_CTRL, 0x00); Already disabled
#   // Disable FIFO and I2C master modes
#   writeByte(ICM20948_ADDRESS, USER_CTRL, 0x00);
#   // Reset FIFO and DMP
#   writeByte(ICM20948_ADDRESS, USER_CTRL, 0x08);
#   writeByte(ICM20948_ADDRESS, FIFO_RST, 0x1F);
#   delay(10);
#   writeByte(ICM20948_ADDRESS, FIFO_RST, 0x00);
#   delay(15);

#   // Set FIFO mode to snapshot
#   writeByte(ICM20948_ADDRESS, FIFO_MODE, 0x1F);
#   // Switch to user bank 2
#   writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);
#   // Configure ICM20948 gyro and accelerometer for bias calculation
#   // Set low-pass filter to 188 Hz
#   writeByte(ICM20948_ADDRESS, GYRO_CONFIG_1, 0x01);
#   // Set sample rate to 1 kHz
#   writeByte(ICM20948_ADDRESS, GYRO_SMPLRT_DIV, 0x00);
#   // Set gyro full-scale to 250 degrees per second, maximum sensitivity
#   writeByte(ICM20948_ADDRESS, GYRO_CONFIG_1, 0x00);
#   // Set accelerometer full-scale to 2 g, maximum sensitivity
#   writeByte(ICM20948_ADDRESS, ACCEL_CONFIG, 0x00);

#   uint16_t gyrosensitivity = 131;    // = 131 LSB/degrees/sec
#   uint16_t accelsensitivity = 16384; // = 16384 LSB/g

#   // Switch to user bank 0
#   writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);
#   // Configure FIFO to capture accelerometer and gyro data for bias calculation
#   writeByte(ICM20948_ADDRESS, USER_CTRL, 0x40); // Enable FIFO
#   // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in
#   // ICM20948)
#   writeByte(ICM20948_ADDRESS, FIFO_EN_2, 0x1E);
#   delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

#   // At end of sample accumulation, turn off FIFO sensor read
#   // Disable gyro and accelerometer sensors for FIFO
#   writeByte(ICM20948_ADDRESS, FIFO_EN_2, 0x00);
#   // Read FIFO sample count
#   readBytes(ICM20948_ADDRESS, FIFO_COUNTH, 2, &data[0]);
#   fifo_count = ((uint16_t)data[0] << 8) | data[1];
#   // How many sets of full gyro and accelerometer data for averaging
#   packet_count = fifo_count / 12;

#   for (ii = 0; ii < packet_count; ii++)
#   {
#     int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
#     // Read data for averaging
#     readBytes(ICM20948_ADDRESS, FIFO_R_W, 12, &data[0]);
#     // Form signed 16-bit integer for each sample in FIFO
#     accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);
#     accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
#     accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
#     gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
#     gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
#     gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

#     // Sum individual signed 16-bit biases to get accumulated signed 32-bit
#     // biases.
#     accel_bias[0] += (int32_t)accel_temp[0];
#     accel_bias[1] += (int32_t)accel_temp[1];
#     accel_bias[2] += (int32_t)accel_temp[2];
#     gyro_bias[0] += (int32_t)gyro_temp[0];
#     gyro_bias[1] += (int32_t)gyro_temp[1];
#     gyro_bias[2] += (int32_t)gyro_temp[2];
#   }
#   // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
#   accel_bias[0] /= (int32_t)packet_count;
#   accel_bias[1] /= (int32_t)packet_count;
#   accel_bias[2] /= (int32_t)packet_count;
#   gyro_bias[0] /= (int32_t)packet_count;
#   gyro_bias[1] /= (int32_t)packet_count;
#   gyro_bias[2] /= (int32_t)packet_count;

#   // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
#   if (accel_bias[2] > 0L)
#   {
#     accel_bias[2] -= (int32_t)accelsensitivity;
#   }
#   else
#   {
#     accel_bias[2] += (int32_t)accelsensitivity;
#   }

#   // Construct the gyro biases for push to the hardware gyro bias registers,
#   // which are reset to zero upon device startup.
#   // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input
#   // format.
#   data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF;
#   // Biases are additive, so change sign on calculated average gyro biases
#   data[1] = (-gyro_bias[0] / 4) & 0xFF;
#   data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
#   data[3] = (-gyro_bias[1] / 4) & 0xFF;
#   data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
#   data[5] = (-gyro_bias[2] / 4) & 0xFF;

#   // Switch to user bank 2
#   writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x20);

#   // Push gyro biases to hardware registers
#   writeByte(ICM20948_ADDRESS, XG_OFFSET_H, data[0]);
#   writeByte(ICM20948_ADDRESS, XG_OFFSET_L, data[1]);
#   writeByte(ICM20948_ADDRESS, YG_OFFSET_H, data[2]);
#   writeByte(ICM20948_ADDRESS, YG_OFFSET_L, data[3]);
#   writeByte(ICM20948_ADDRESS, ZG_OFFSET_H, data[4]);
#   writeByte(ICM20948_ADDRESS, ZG_OFFSET_L, data[5]);

#   // Output scaled gyro biases for display in the main program
#   gyroBias[0] = (float)gyro_bias[0] / (float)gyrosensitivity;
#   gyroBias[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
#   gyroBias[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

#   // Construct the accelerometer biases for push to the hardware accelerometer
#   // bias registers. These registers contain factory trim values which must be
#   // added to the calculated accelerometer biases; on boot up these registers
#   // will hold non-zero values. In addition, bit 0 of the lower byte must be
#   // preserved since it is used for temperature compensation calculations.
#   // Accelerometer bias registers expect bias input as 2048 LSB per g, so that
#   // the accelerometer biases calculated above must be divided by 8.

#   // Switch to user bank 1
#   writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x10);
#   // A place to hold the factory accelerometer trim biases
#   int32_t accel_bias_reg[3] = {0, 0, 0};
#   // Read factory accelerometer trim values
#   readBytes(ICM20948_ADDRESS, XA_OFFSET_H, 2, &data[0]);
#   accel_bias_reg[0] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
#   readBytes(ICM20948_ADDRESS, YA_OFFSET_H, 2, &data[0]);
#   accel_bias_reg[1] = (int32_t)(((int16_t)data[0] << 8) | data[1]);
#   readBytes(ICM20948_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
#   accel_bias_reg[2] = (int32_t)(((int16_t)data[0] << 8) | data[1]);

#   // Define mask for temperature compensation bit 0 of lower byte of
#   // accelerometer bias registers
#   uint32_t mask = 1uL;
#   // Define array to hold mask bit for each accelerometer bias axis
#   uint8_t mask_bit[3] = {0, 0, 0};

#   for (ii = 0; ii < 3; ii++)
#   {
#     // If temperature compensation bit is set, record that fact in mask_bit
#     if ((accel_bias_reg[ii] & mask))
#     {
#       mask_bit[ii] = 0x01;
#     }
#   }

#   // Construct total accelerometer bias, including calculated average
#   // accelerometer bias from above
#   // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
#   // (16 g full scale)
#   accel_bias_reg[0] -= (accel_bias[0] / 8);
#   accel_bias_reg[1] -= (accel_bias[1] / 8);
#   accel_bias_reg[2] -= (accel_bias[2] / 8);

#   data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
#   data[1] = (accel_bias_reg[0]) & 0xFF;
#   // preserve temperature compensation bit when writing back to accelerometer
#   // bias registers
#   data[1] = data[1] | mask_bit[0];
#   data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
#   data[3] = (accel_bias_reg[1]) & 0xFF;
#   // Preserve temperature compensation bit when writing back to accelerometer
#   // bias registers
#   data[3] = data[3] | mask_bit[1];
#   data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
#   data[5] = (accel_bias_reg[2]) & 0xFF;
#   // Preserve temperature compensation bit when writing back to accelerometer
#   // bias registers
#   data[5] = data[5] | mask_bit[2];

#   // Apparently this is not working for the acceleration biases in the ICM-20948
#   // Are we handling the temperature correction bit properly?
#   // Push accelerometer biases to hardware registers
#   writeByte(ICM20948_ADDRESS, XA_OFFSET_H, data[0]);
#   writeByte(ICM20948_ADDRESS, XA_OFFSET_L, data[1]);
#   writeByte(ICM20948_ADDRESS, YA_OFFSET_H, data[2]);
#   writeByte(ICM20948_ADDRESS, YA_OFFSET_L, data[3]);
#   writeByte(ICM20948_ADDRESS, ZA_OFFSET_H, data[4]);
#   writeByte(ICM20948_ADDRESS, ZA_OFFSET_L, data[5]);

#   // Output scaled accelerometer biases for display in the main program
#   accelBias[0] = (float)accel_bias[0] / (float)accelsensitivity;
#   accelBias[1] = (float)accel_bias[1] / (float)accelsensitivity;
#   accelBias[2] = (float)accel_bias[2] / (float)accelsensitivity;
#   // Switch to user bank 0
#   writeByte(ICM20948_ADDRESS, REG_BANK_SEL, 0x00);
# }