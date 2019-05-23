DEBUG_MSGS = True 
 
# ==============
# SPI Parameters
# ==============

SPI_DATA_RATE = 1000000
SPI_CLIENT = 0

# =========
# ICM-20948
# =========

# USER BANK 0 REGISTER MAP
WHO_AM_I_ICM20948       = 0x00
USER_CTRL               = 0x03
LP_CONFIG               = 0x05
PWR_MGMT_1              = 0x06
PWR_MGMT_2              = 0x07
INT_PIN_CFG             = 0x0F
INT_ENABLE              = 0x10
INT_ENABLE_1            = 0x11
INT_ENABLE_2            = 0x12
INT_ENABLE_3            = 0x13
I2C_MST_STATUS          = 0x17
INT_STATUS              = 0x19
INT_STATUS_1            = 0x1A
INT_STATUS_2            = 0x1B
INT_STATUS_3            = 0x1C
DELAY_TIMEH             = 0x28
DELAY_TIMEL             = 0x29
ACCEL_XOUT_H            = 0x2D
ACCEL_XOUT_L            = 0x2E
ACCEL_YOUT_H            = 0x2F
ACCEL_YOUT_L            = 0x30
ACCEL_ZOUT_H            = 0x31
ACCEL_ZOUT_L            = 0x32
GYRO_XOUT_H             = 0x33
GYRO_XOUT_L             = 0x34
GYRO_YOUT_H             = 0x35
GYRO_YOUT_L             = 0x36
GYRO_ZOUT_H             = 0x37
GYRO_ZOUT_L             = 0x38
TEMP_OUT_H              = 0x39
TEMP_OUT_L              = 0x3A
EXT_SENS_DATA_00        = 0x3B
EXT_SENS_DATA_01        = 0x3C
EXT_SENS_DATA_02        = 0x3D
EXT_SENS_DATA_03        = 0x3E
EXT_SENS_DATA_04        = 0x3F
EXT_SENS_DATA_05        = 0x40
EXT_SENS_DATA_06        = 0x41
EXT_SENS_DATA_07        = 0x42
EXT_SENS_DATA_08        = 0x43
EXT_SENS_DATA_09        = 0x44
EXT_SENS_DATA_10        = 0x45
EXT_SENS_DATA_11        = 0x46
EXT_SENS_DATA_12        = 0x47
EXT_SENS_DATA_13        = 0x48
EXT_SENS_DATA_14        = 0x49
EXT_SENS_DATA_15        = 0x4A
EXT_SENS_DATA_16        = 0x4B
EXT_SENS_DATA_17        = 0x4C
EXT_SENS_DATA_18        = 0x4D
EXT_SENS_DATA_19        = 0x4E
EXT_SENS_DATA_20        = 0x4F
EXT_SENS_DATA_21        = 0x50
EXT_SENS_DATA_22        = 0x51
EXT_SENS_DATA_23        = 0x52
FIFO_EN_1               = 0x66
FIFO_EN_2               = 0x67
FIFO_RST		        = 0x68
FIFO_MODE		        = 0x69
FIFO_COUNTH             = 0x70
FIFO_COUNTL             = 0x71
FIFO_R_W                = 0x72
DATA_RDY_STATUS	        = 0x74
FIFO_CFG		        = 0x76
REG_BANK_SEL	        = 0x7F

# USER BANK 1 REGISTER MAP
SELF_TEST_X_GYRO        = 0x02
SELF_TEST_Y_GYRO        = 0x03
SELF_TEST_Z_GYRO        = 0x04
SELF_TEST_X_ACCEL       = 0x0E
SELF_TEST_Y_ACCEL       = 0x0F
SELF_TEST_Z_ACCEL       = 0x10
XA_OFFSET_H             = 0x14
XA_OFFSET_L             = 0x15
YA_OFFSET_H             = 0x17
YA_OFFSET_L             = 0x18
ZA_OFFSET_H             = 0x1A
ZA_OFFSET_L             = 0x1B
TIMEBASE_CORRECTION_PLL = 0x28

# USER BANK 2 REGISTER MAP
GYRO_SMPLRT_DIV         = 0x00
GYRO_CONFIG_1           = 0x01
GYRO_CONFIG_2           = 0x02
XG_OFFSET_H             = 0x03
XG_OFFSET_L             = 0x04
YG_OFFSET_H             = 0x05
YG_OFFSET_L             = 0x06
ZG_OFFSET_H             = 0x07
ZG_OFFSET_L             = 0x08
ODR_ALIGN_EN            = 0x09
ACCEL_SMPLRT_DIV_1      = 0x10
ACCEL_SMPLRT_DIV_2      = 0x11
ACCEL_INTEL_CTR         = 0x12
ACCEL_WOM_THR           = 0x13
ACCEL_CONFIG            = 0x14
ACCEL_CONFIG_2          = 0x15
FSYNC_CONFIG            = 0x52
TEMP_CONFIG	            = 0x53
MOD_CTRL_USR            = 0x54

# USER BANK 3 REGISTER MAP
I2C_MST_ODR_CONFIG      = 0x00
I2C_MST_CTRL            = 0x01
I2C_MST_DELAY_CTRL      = 0x02
I2C_SLV0_ADDR           = 0x03
I2C_SLV0_REG            = 0x04
I2C_SLV0_CTRL           = 0x05
I2C_SLV0_DO             = 0x06
I2C_SLV1_ADDR           = 0x07
I2C_SLV1_REG            = 0x08
I2C_SLV1_CTRL           = 0x09
I2C_SLV1_DO             = 0x0A
I2C_SLV2_ADDR           = 0x0B
I2C_SLV2_REG            = 0x0C
I2C_SLV2_CTRL           = 0x0D
I2C_SLV2_DO             = 0x0E
I2C_SLV3_ADDR           = 0x0F
I2C_SLV3_REG            = 0x10
I2C_SLV3_CTRL           = 0x11
I2C_SLV3_DO             = 0x12
I2C_SLV4_ADDR           = 0x13
I2C_SLV4_REG            = 0x14
I2C_SLV4_CTRL           = 0x15
I2C_SLV4_DI             = 0x17
I2C_SLV4_DO             = 0x16

# Other constants
READ_FLAG = 0x80
REG_BANK_0 = (0 << 4)
REG_BANK_1 = (1 << 4)
REG_BANK_2 = (2 << 4)
REG_BANK_3 = (3 << 4)
