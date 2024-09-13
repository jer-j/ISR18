import smbus
import time
from LSM9DS0 import *
from LSM9DS1 import *
from LSM6DSL import *
from LIS3MDL import *
from imu_dict import *


class IMU:
    DEFAULT_SPEED = 0
    NGLE_0_PWM = 850
    NGLE_MAX_PWM = 2150
    RAVEL_RANGE_ANGLE = 130
    WM_FREQ = 100
    RAD_TO_DEG = 57.29578
    M_PI = 3.14159265358979323846
    G_GAIN = 0.070  # [deg/s/LSB]  If you change the dps for gyro, you need     to update this value accordingly
    AA =  0.40      # Complementary filter constant

    def __init__(self):
        self.bus = smbus.SMBus(1)
        self.BerryIMUversion = 99
        self.detectIMU()

    def detectIMU(self):
        try:
            # Check for BerryIMUv1 (LSM9DS0)
            LSM9DS0_WHO_G_response = self.bus.read_byte_data(
                LSM9DS0_GYR_ADDRESS, LSM9DS0_WHO_AM_I_G
            )
            LSM9DS0_WHO_XM_response = self.bus.read_byte_data(
                LSM9DS0_ACC_ADDRESS, LSM9DS0_WHO_AM_I_XM
            )
        except IOError:
            pass
        else:
            if (LSM9DS0_WHO_G_response == 0xD4) and (LSM9DS0_WHO_XM_response == 0x49):
                print("Found BerryIMUv1 (LSM9DS0)")
                self.BerryIMUversion = 1

        try:
            # Check for BerryIMUv2 (LSM9DS1)
            LSM9DS1_WHO_XG_response = self.bus.read_byte_data(
                LSM9DS1_GYR_ADDRESS, LSM9DS1_WHO_AM_I_XG
            )
            LSM9DS1_WHO_M_response = self.bus.read_byte_data(
                LSM9DS1_MAG_ADDRESS, LSM9DS1_WHO_AM_I_M
            )
        except IOError:
            pass
        else:
            if (LSM9DS1_WHO_XG_response == 0x68) and (LSM9DS1_WHO_M_response == 0x3D):
                print("Found BerryIMUv2 (LSM9DS1)")
                self.BerryIMUversion = 2

        try:
            # Check for BerryIMUv3 (LSM6DSL and LIS3MDL)
            LSM6DSL_WHO_AM_I_response = self.bus.read_byte_data(
                LSM6DSL_ADDRESS, LSM6DSL_WHO_AM_I
            )
            LIS3MDL_WHO_AM_I_response = self.bus.read_byte_data(
                LIS3MDL_ADDRESS, LIS3MDL_WHO_AM_I
            )
        except IOError:
            pass
        else:
            if (LSM6DSL_WHO_AM_I_response == 0x6A) and (
                LIS3MDL_WHO_AM_I_response == 0x3D
            ):
                print("Found BerryIMUv3 (LSM6DSL and LIS3MDL)")
                self.BerryIMUversion = 3

        time.sleep(1)

    def write_byte(self, device_address, register, value):
        self.bus.write_byte_data(device_address, register, value)

    def initIMU(self):
        if self.BerryIMUversion == 1:
            self._init_v1()
            self.consts = berryIMUv1
        elif self.BerryIMUversion == 2:
            self._init_v2()
            self.consts = berryIMUv2
        elif self.BerryIMUversion == 3:
            self._init_v3()
            self.consts = berryIMUv3

    def _init_v1(self):
        # initialize accelerometer
        self.write_byte(LSM9DS0_ACC_ADDRESS, LSM9DS0_CTRL_REG1_XM, 0b01100111)
        self.write_byte(LSM9DS0_ACC_ADDRESS, LSM9DS0_CTRL_REG2_XM, 0b00011000)
        # initialize magnetometer
        self.write_byte(LSM9DS0_MAG_ADDRESS, LSM9DS0_CTRL_REG5_XM, 0b11110000)
        self.write_byte(LSM9DS0_MAG_ADDRESS, LSM9DS0_CTRL_REG6_XM, 0b01100000)
        self.write_byte(LSM9DS0_MAG_ADDRESS, LSM9DS0_CTRL_REG7_XM, 0b00000000)
        # initialize gyroscope
        self.write_byte(LSM9DS0_GYR_ADDRESS, LSM9DS0_CTRL_REG1_G, 0b00001111)
        self.write_byte(LSM9DS0_GYR_ADDRESS, LSM9DS0_CTRL_REG4_G, 0b00110000)

    def _init_v2(self):
        # initialize gyroscope
        self.write_byte(LSM9DS1_GYR_ADDRESS, LSM9DS1_CTRL_REG4, 0b00111000)
        self.write_byte(LSM9DS1_GYR_ADDRESS, LSM9DS1_CTRL_REG1_G, 0b10111000)
        self.write_byte(LSM9DS1_GYR_ADDRESS, LSM9DS1_ORIENT_CFG_G, 0b10111000)
        # initialize accelerometer
        self.write_byte(LSM9DS1_ACC_ADDRESS, LSM9DS1_CTRL_REG5_XL, 0b00111000)
        self.write_byte(LSM9DS1_ACC_ADDRESS, LSM9DS1_CTRL_REG6_XL, 0b00111000)
        # initialize magnetometer
        self.write_byte(LSM9DS1_MAG_ADDRESS, LSM9DS1_CTRL_REG1_M, 0b10011100)
        self.write_byte(LSM9DS1_MAG_ADDRESS, LSM9DS1_CTRL_REG2_M, 0b01000000)
        self.write_byte(LSM9DS1_MAG_ADDRESS, LSM9DS1_CTRL_REG3_M, 0b00000000)
        self.write_byte(LSM9DS1_MAG_ADDRESS, LSM9DS1_CTRL_REG4_M, 0b00000000)

    def _init_v3(self):
        # initialize gyroscope and accelerometer
        self.write_byte(LSM6DSL_ADDRESS, LSM6DSL_CTRL1_XL, 0b10011111)
        self.write_byte(LSM6DSL_ADDRESS, LSM6DSL_CTRL8_XL, 0b11001000)
        self.write_byte(LSM6DSL_ADDRESS, LSM6DSL_CTRL3_C, 0b01000100)
        self.write_byte(LSM6DSL_ADDRESS, LSM6DSL_CTRL2_G, 0b10011100)
        # initialize magnetometer
        self.write_byte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG1, 0b11011100)
        self.write_byte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG2, 0b00100000)
        self.write_byte(LIS3MDL_ADDRESS, LIS3MDL_CTRL_REG3, 0b00000000)

    def read_sens(self,sens,axis):
        match sens:
            case 'acc':
                match axis:
                    case 'x':
                        sens_l = self.bus.read_byte_data(self.consts.acc_addr, self.consts.xla_reg)
                        sens_h = self.bus.read_byte_data(self.consts.acc_addr, self.consts.xha_reg)
                    case 'y':
                        sens_l = self.bus.read_byte_data(self.consts.acc_addr, self.consts.yla_reg)
                        sens_h = self.bus.read_byte_data(self.consts.acc_addr, self.consts.yha_reg)
                    case 'z':
                        sens_l = self.bus.read_byte_data(self.consts.acc_addr, self.consts.zla_reg)
                        sens_h = self.bus.read_byte_data(self.consts.acc_addr, self.consts.zha_reg)
            case 'gyr':
                match axis:
                    case 'x':
                        sens_l = self.bus.read_byte_data(self.consts.gyr_addr, self.consts.xlg_reg)
                        sens_h = self.bus.read_byte_data(self.consts.gyr_addr, self.consts.xhg_reg)
                    case 'y':
                        sens_l = self.bus.read_byte_data(self.consts.gyr_addr, self.consts.ylg_reg)
                        sens_h = self.bus.read_byte_data(self.consts.gyr_addr, self.consts.yhg_reg)
                    case 'z':
                        sens_l = self.bus.read_byte_data(self.consts.gyr_addr, self.consts.zlg_reg)
                        sens_h = self.bus.read_byte_data(self.consts.gyr_addr, self.consts.zhg_reg)
            case 'mag':
                match axis:
                    case 'x':
                        sens_l = self.bus.read_byte_data(self.consts.mag_addr, self.consts.xlm_reg)
                        sens_h = self.bus.read_byte_data(self.consts.mag_addr, self.consts.xhm_reg)
                    case 'y':
                        sens_l = self.bus.read_byte_data(self.consts.mag_addr, self.consts.ylm_reg)
                        sens_h = self.bus.read_byte_data(self.consts.mag_addr, self.consts.yhm_reg)
                    case 'z':
                        sens_l = self.bus.read_byte_data(self.consts.mag_addr, self.consts.zlm_reg)
                        sens_h = self.bus.read_byte_data(self.consts.mag_addr, self.consts.zhm_reg)
        
        sens_combined = sens_l | sens_h << 8
        return sens_combined if sens_combined < 32768 else sens_combined - 65536
