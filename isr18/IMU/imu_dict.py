from dataclasses import dataclass

from LSM9DS0 import *
from LSM9DS1 import *
from LSM6DSL import *
from LIS3MDL import *

@dataclass
class imu_consts:
    acc_addr: int
    xla_reg: int
    xha_reg: int
    yla_reg: int
    yha_reg: int
    zla_reg: int
    zha_reg: int

    gyr_addr: int
    xlg_reg: int
    xhg_reg: int
    ylg_reg: int
    yhg_reg: int
    zlg_reg: int
    zhg_reg: int

    mag_addr: int
    xlm_reg: int
    xhm_reg: int
    ylm_reg: int
    yhm_reg: int
    zlm_reg: int
    zhm_reg: int

berryIMUv1 = imu_consts(
    LSM9DS0_ACC_ADDRESS,
    LSM9DS0_OUT_X_L_A,
    LSM9DS0_OUT_X_H_A,
    LSM9DS0_OUT_Y_L_A,
    LSM9DS0_OUT_Y_H_A,
    LSM9DS0_OUT_Z_L_A,
    LSM9DS0_OUT_Z_H_A,
    LSM9DS0_GYR_ADDRESS,
    LSM9DS0_OUT_X_L_G,
    LSM9DS0_OUT_X_H_G,
    LSM9DS0_OUT_Y_L_G,
    LSM9DS0_OUT_Y_H_G,
    LSM9DS0_OUT_Z_L_G,
    LSM9DS0_OUT_Z_H_G,
    LSM9DS0_MAG_ADDRESS,
    LSM9DS0_OUT_X_L_M,
    LSM9DS0_OUT_X_H_M,
    LSM9DS0_OUT_Y_L_M,
    LSM9DS0_OUT_Y_H_M,
    LSM9DS0_OUT_Z_L_M,
    LSM9DS0_OUT_Z_H_M,
)

berryIMUv2 = imu_consts(
    LSM9DS1_ACC_ADDRESS,
    LSM9DS1_OUT_X_L_XL,
    LSM9DS1_OUT_X_H_XL,
    LSM9DS1_OUT_Y_L_XL,
    LSM9DS1_OUT_Y_H_XL,
    LSM9DS1_OUT_Z_L_XL,
    LSM9DS1_OUT_Z_H_XL,
    LSM9DS1_GYR_ADDRESS,
    LSM9DS1_OUT_X_L_G,
    LSM9DS1_OUT_X_H_G,
    LSM9DS1_OUT_Y_L_G,
    LSM9DS1_OUT_Y_H_G,
    LSM9DS1_OUT_Z_L_G,
    LSM9DS1_OUT_Z_H_G,
    LSM9DS1_MAG_ADDRESS,
    LSM9DS1_OUT_X_L_M,
    LSM9DS1_OUT_X_H_M,
    LSM9DS1_OUT_Y_L_M,
    LSM9DS1_OUT_Y_H_M,
    LSM9DS1_OUT_Z_L_M,
    LSM9DS1_OUT_Z_H_M,
)

berryIMUv3 = imu_consts(
    LSM6DSL_ADDRESS,
    LSM6DSL_OUTX_L_XL,
    LSM6DSL_OUTX_H_XL,
    LSM6DSL_OUTY_L_XL,
    LSM6DSL_OUTY_H_XL,
    LSM6DSL_OUTZ_L_XL,
    LSM6DSL_OUTZ_H_XL,
    LSM6DSL_ADDRESS,
    LSM6DSL_OUTX_L_G,
    LSM6DSL_OUTX_H_G,
    LSM6DSL_OUTY_L_G,
    LSM6DSL_OUTY_H_G,
    LSM6DSL_OUTZ_L_G,
    LSM6DSL_OUTZ_H_G,
    LIS3MDL_ADDRESS,
    LIS3MDL_OUT_X_L,
    LIS3MDL_OUT_X_H,
    LIS3MDL_OUT_Y_L,
    LIS3MDL_OUT_Y_H,
    LIS3MDL_OUT_Z_L,
    LIS3MDL_OUT_Z_H,
)
