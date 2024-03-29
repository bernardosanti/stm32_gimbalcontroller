#ifndef _MPU9050_H_
#define _MPU9050_H_

#include <stdint.h>
#include "stm32f4xx.h"

#ifdef __cplusplus
extern "C" {
#endif

//Magnetometer Registers
#define RA_MAG_ADDRESS		0x0C
#define RA_MAG_XOUT_L		0x03
#define RA_MAG_XOUT_H		0x04
#define RA_MAG_YOUT_L		0x05
#define RA_MAG_YOUT_H		0x06
#define RA_MAG_ZOUT_L		0x07
#define RA_MAG_ZOUT_H		0x08

#define ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define DEFAULT_ADDRESS     ADDRESS_AD0_LOW

#define RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define RA_XA_OFFS_L_TC     0x07
#define RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define RA_YA_OFFS_L_TC     0x09
#define RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define RA_ZA_OFFS_L_TC     0x0B
#define RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define RA_XG_OFFS_USRL     0x14
#define RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define RA_YG_OFFS_USRL     0x16
#define RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define RA_ZG_OFFS_USRL     0x18
#define RA_SMPLRT_DIV       0x19
#define RA_CONFIG           0x1A
#define RA_GYRO_CONFIG      0x1B
#define RA_ACCEL_CONFIG     0x1C
#define RA_FF_THR           0x1D
#define RA_FF_DUR           0x1E
#define RA_MOT_THR          0x1F
#define RA_MOT_DUR          0x20
#define RA_ZRMOT_THR        0x21
#define RA_ZRMOT_DUR        0x22
#define RA_FIFO_EN          0x23
#define RA_I2C_MST_CTRL     0x24
#define RA_I2C_SLV0_ADDR    0x25
#define RA_I2C_SLV0_REG     0x26
#define RA_I2C_SLV0_CTRL    0x27
#define RA_I2C_SLV1_ADDR    0x28
#define RA_I2C_SLV1_REG     0x29
#define RA_I2C_SLV1_CTRL    0x2A
#define RA_I2C_SLV2_ADDR    0x2B
#define RA_I2C_SLV2_REG     0x2C
#define RA_I2C_SLV2_CTRL    0x2D
#define RA_I2C_SLV3_ADDR    0x2E
#define RA_I2C_SLV3_REG     0x2F
#define RA_I2C_SLV3_CTRL    0x30
#define RA_I2C_SLV4_ADDR    0x31
#define RA_I2C_SLV4_REG     0x32
#define RA_I2C_SLV4_DO      0x33
#define RA_I2C_SLV4_CTRL    0x34
#define RA_I2C_SLV4_DI      0x35
#define RA_I2C_MST_STATUS   0x36
#define RA_INT_PIN_CFG      0x37
#define RA_INT_ENABLE       0x38
#define RA_DMP_INT_STATUS   0x39
#define RA_INT_STATUS       0x3A
#define RA_ACCEL_XOUT_H     0x3B
#define RA_ACCEL_XOUT_L     0x3C
#define RA_ACCEL_YOUT_H     0x3D
#define RA_ACCEL_YOUT_L     0x3E
#define RA_ACCEL_ZOUT_H     0x3F
#define RA_ACCEL_ZOUT_L     0x40
#define RA_TEMP_OUT_H       0x41
#define RA_TEMP_OUT_L       0x42
#define RA_GYRO_XOUT_H      0x43
#define RA_GYRO_XOUT_L      0x44
#define RA_GYRO_YOUT_H      0x45
#define RA_GYRO_YOUT_L      0x46
#define RA_GYRO_ZOUT_H      0x47
#define RA_GYRO_ZOUT_L      0x48
#define RA_EXT_SENS_DATA_00 0x49
#define RA_EXT_SENS_DATA_01 0x4A
#define RA_EXT_SENS_DATA_02 0x4B
#define RA_EXT_SENS_DATA_03 0x4C
#define RA_EXT_SENS_DATA_04 0x4D
#define RA_EXT_SENS_DATA_05 0x4E
#define RA_EXT_SENS_DATA_06 0x4F
#define RA_EXT_SENS_DATA_07 0x50
#define RA_EXT_SENS_DATA_08 0x51
#define RA_EXT_SENS_DATA_09 0x52
#define RA_EXT_SENS_DATA_10 0x53
#define RA_EXT_SENS_DATA_11 0x54
#define RA_EXT_SENS_DATA_12 0x55
#define RA_EXT_SENS_DATA_13 0x56
#define RA_EXT_SENS_DATA_14 0x57
#define RA_EXT_SENS_DATA_15 0x58
#define RA_EXT_SENS_DATA_16 0x59
#define RA_EXT_SENS_DATA_17 0x5A
#define RA_EXT_SENS_DATA_18 0x5B
#define RA_EXT_SENS_DATA_19 0x5C
#define RA_EXT_SENS_DATA_20 0x5D
#define RA_EXT_SENS_DATA_21 0x5E
#define RA_EXT_SENS_DATA_22 0x5F
#define RA_EXT_SENS_DATA_23 0x60
#define RA_MOT_DETECT_STATUS    0x61
#define RA_I2C_SLV0_DO      0x63
#define RA_I2C_SLV1_DO      0x64
#define RA_I2C_SLV2_DO      0x65
#define RA_I2C_SLV3_DO      0x66
#define RA_I2C_MST_DELAY_CTRL   0x67
#define RA_SIGNAL_PATH_RESET    0x68
#define RA_MOT_DETECT_CTRL      0x69
#define RA_USER_CTRL        0x6A
#define RA_PWR_MGMT_1       0x6B
#define RA_PWR_MGMT_2       0x6C
#define RA_BANK_SEL         0x6D
#define RA_MEM_START_ADDR   0x6E
#define RA_MEM_R_W          0x6F
#define RA_DMP_CFG_1        0x70
#define RA_DMP_CFG_2        0x71
#define RA_FIFO_COUNTH      0x72
#define RA_FIFO_COUNTL      0x73
#define RA_FIFO_R_W         0x74
#define RA_WHO_AM_I         0x75

#define TC_PWR_MODE_BIT     7
#define TC_OFFSET_BIT       6
#define TC_OFFSET_LENGTH    6
#define TC_OTP_BNK_VLD_BIT  0

#define VDDIO_LEVEL_VLOGIC  0
#define VDDIO_LEVEL_VDD     1

#define CFG_EXT_SYNC_SET_BIT    5
#define CFG_EXT_SYNC_SET_LENGTH 3
#define CFG_DLPF_CFG_BIT    2
#define CFG_DLPF_CFG_LENGTH 3

#define EXT_SYNC_DISABLED       0x0
#define EXT_SYNC_TEMP_OUT_L     0x1
#define EXT_SYNC_GYRO_XOUT_L    0x2
#define EXT_SYNC_GYRO_YOUT_L    0x3
#define EXT_SYNC_GYRO_ZOUT_L    0x4
#define EXT_SYNC_ACCEL_XOUT_L   0x5
#define EXT_SYNC_ACCEL_YOUT_L   0x6
#define EXT_SYNC_ACCEL_ZOUT_L   0x7

#define DLPF_BW_256         0x00
#define DLPF_BW_188         0x01
#define DLPF_BW_98          0x02
#define DLPF_BW_42          0x03
#define DLPF_BW_20          0x04
#define DLPF_BW_10          0x05
#define DLPF_BW_5           0x06

#define GCONFIG_FS_SEL_BIT      4
#define GCONFIG_FS_SEL_LENGTH   2

#define GYRO_FS_250         0x00
#define GYRO_FS_500         0x01
#define GYRO_FS_1000        0x02
#define GYRO_FS_2000        0x03

#define ACONFIG_XA_ST_BIT           7
#define ACONFIG_YA_ST_BIT           6
#define ACONFIG_ZA_ST_BIT           5
#define ACONFIG_AFS_SEL_BIT         4
#define ACONFIG_AFS_SEL_LENGTH      2
#define ACONFIG_ACCEL_HPF_BIT       2
#define ACONFIG_ACCEL_HPF_LENGTH    3

#define ACCEL_FS_2          0x00
#define ACCEL_FS_4          0x01
#define ACCEL_FS_8          0x02
#define ACCEL_FS_16         0x03

#define DHPF_RESET          0x00
#define DHPF_5              0x01
#define DHPF_2P5            0x02
#define DHPF_1P25           0x03
#define DHPF_0P63           0x04
#define DHPF_HOLD           0x07

#define TEMP_FIFO_EN_BIT    7
#define XG_FIFO_EN_BIT      6
#define YG_FIFO_EN_BIT      5
#define ZG_FIFO_EN_BIT      4
#define ACCEL_FIFO_EN_BIT   3
#define SLV2_FIFO_EN_BIT    2
#define SLV1_FIFO_EN_BIT    1
#define SLV0_FIFO_EN_BIT    0

#define MULT_MST_EN_BIT     7
#define WAIT_FOR_ES_BIT     6
#define SLV_3_FIFO_EN_BIT   5
#define I2C_MST_P_NSR_BIT   4
#define I2C_MST_CLK_BIT     3
#define I2C_MST_CLK_LENGTH  4

#define CLOCK_DIV_348       0x0
#define CLOCK_DIV_333       0x1
#define CLOCK_DIV_320       0x2
#define CLOCK_DIV_308       0x3
#define CLOCK_DIV_296       0x4
#define CLOCK_DIV_286       0x5
#define CLOCK_DIV_276       0x6
#define CLOCK_DIV_267       0x7
#define CLOCK_DIV_258       0x8
#define CLOCK_DIV_500       0x9
#define CLOCK_DIV_471       0xA
#define CLOCK_DIV_444       0xB
#define CLOCK_DIV_421       0xC
#define CLOCK_DIV_400       0xD
#define CLOCK_DIV_381       0xE
#define CLOCK_DIV_364       0xF

#define I2C_SLV_RW_BIT      7
#define I2C_SLV_ADDR_BIT    6
#define I2C_SLV_ADDR_LENGTH 7
#define I2C_SLV_EN_BIT      7
#define I2C_SLV_BYTE_SW_BIT 6
#define I2C_SLV_REG_DIS_BIT 5
#define I2C_SLV_GRP_BIT     4
#define I2C_SLV_LEN_BIT     3
#define I2C_SLV_LEN_LENGTH  4

#define I2C_SLV4_RW_BIT         7
#define I2C_SLV4_ADDR_BIT       6
#define I2C_SLV4_ADDR_LENGTH    7
#define I2C_SLV4_EN_BIT         7
#define I2C_SLV4_INT_EN_BIT     6
#define I2C_SLV4_REG_DIS_BIT    5
#define I2C_SLV4_MST_DLY_BIT    4
#define I2C_SLV4_MST_DLY_LENGTH 5

#define MST_PASS_THROUGH_BIT    7
#define MST_I2C_SLV4_DONE_BIT   6
#define MST_I2C_LOST_ARB_BIT    5
#define MST_I2C_SLV4_NACK_BIT   4
#define MST_I2C_SLV3_NACK_BIT   3
#define MST_I2C_SLV2_NACK_BIT   2
#define MST_I2C_SLV1_NACK_BIT   1
#define MST_I2C_SLV0_NACK_BIT   0

#define INTCFG_INT_LEVEL_BIT        7
#define INTCFG_INT_OPEN_BIT         6
#define INTCFG_LATCH_INT_EN_BIT     5
#define INTCFG_INT_RD_CLEAR_BIT     4
#define INTCFG_FSYNC_INT_LEVEL_BIT  3
#define INTCFG_FSYNC_INT_EN_BIT     2
#define INTCFG_I2C_BYPASS_EN_BIT    1
#define INTCFG_CLKOUT_EN_BIT        0

#define INTMODE_ACTIVEHIGH  0x00
#define INTMODE_ACTIVELOW   0x01

#define INTDRV_PUSHPULL     0x00
#define INTDRV_OPENDRAIN    0x01

#define INTLATCH_50USPULSE  0x00
#define INTLATCH_WAITCLEAR  0x01

#define INTCLEAR_STATUSREAD 0x00
#define INTCLEAR_ANYREAD    0x01

#define INTERRUPT_FF_BIT            7
#define INTERRUPT_MOT_BIT           6
#define INTERRUPT_ZMOT_BIT          5
#define INTERRUPT_FIFO_OFLOW_BIT    4
#define INTERRUPT_I2C_MST_INT_BIT   3
#define INTERRUPT_PLL_RDY_INT_BIT   2
#define INTERRUPT_DMP_INT_BIT       1
#define INTERRUPT_DATA_RDY_BIT      0

// TODO: figure out what these actually do
// UMPL source code is not very obvious
#define DMPINT_5_BIT            5
#define DMPINT_4_BIT            4
#define DMPINT_3_BIT            3
#define DMPINT_2_BIT            2
#define DMPINT_1_BIT            1
#define DMPINT_0_BIT            0

#define MOTION_MOT_XNEG_BIT     7
#define MOTION_MOT_XPOS_BIT     6
#define MOTION_MOT_YNEG_BIT     5
#define MOTION_MOT_YPOS_BIT     4
#define MOTION_MOT_ZNEG_BIT     3
#define MOTION_MOT_ZPOS_BIT     2
#define MOTION_MOT_ZRMOT_BIT    0

#define DELAYCTRL_DELAY_ES_SHADOW_BIT   7
#define DELAYCTRL_I2C_SLV4_DLY_EN_BIT   4
#define DELAYCTRL_I2C_SLV3_DLY_EN_BIT   3
#define DELAYCTRL_I2C_SLV2_DLY_EN_BIT   2
#define DELAYCTRL_I2C_SLV1_DLY_EN_BIT   1
#define DELAYCTRL_I2C_SLV0_DLY_EN_BIT   0

#define PATHRESET_GYRO_RESET_BIT    2
#define PATHRESET_ACCEL_RESET_BIT   1
#define PATHRESET_TEMP_RESET_BIT    0

#define DETECT_ACCEL_ON_DELAY_BIT       5
#define DETECT_ACCEL_ON_DELAY_LENGTH    2
#define DETECT_FF_COUNT_BIT             3
#define DETECT_FF_COUNT_LENGTH          2
#define DETECT_MOT_COUNT_BIT            1
#define DETECT_MOT_COUNT_LENGTH         2

#define DETECT_DECREMENT_RESET  0x0
#define DETECT_DECREMENT_1      0x1
#define DETECT_DECREMENT_2      0x2
#define DETECT_DECREMENT_4      0x3

#define USERCTRL_DMP_EN_BIT             7
#define USERCTRL_FIFO_EN_BIT            6
#define USERCTRL_I2C_MST_EN_BIT         5
#define USERCTRL_I2C_IF_DIS_BIT         4
#define USERCTRL_DMP_RESET_BIT          3
#define USERCTRL_FIFO_RESET_BIT         2
#define USERCTRL_I2C_MST_RESET_BIT      1
#define USERCTRL_SIG_COND_RESET_BIT     0

#define PWR1_DEVICE_RESET_BIT   7
#define PWR1_SLEEP_BIT          6
#define PWR1_CYCLE_BIT          5
#define PWR1_TEMP_DIS_BIT       3
#define PWR1_CLKSEL_BIT         2
#define PWR1_CLKSEL_LENGTH      3

#define CLOCK_INTERNAL          0x00
#define CLOCK_PLL_XGYRO         0x01
#define CLOCK_PLL_YGYRO         0x02
#define CLOCK_PLL_ZGYRO         0x03
#define CLOCK_PLL_EXT32K        0x04
#define CLOCK_PLL_EXT19M        0x05
#define CLOCK_KEEP_RESET        0x07

#define PWR2_LP_WAKE_CTRL_BIT       7
#define PWR2_LP_WAKE_CTRL_LENGTH    2
#define PWR2_STBY_XA_BIT            5
#define PWR2_STBY_YA_BIT            4
#define PWR2_STBY_ZA_BIT            3
#define PWR2_STBY_XG_BIT            2
#define PWR2_STBY_YG_BIT            1
#define PWR2_STBY_ZG_BIT            0

#define WAKE_FREQ_1P25      0x0
#define WAKE_FREQ_2P5       0x1
#define WAKE_FREQ_5         0x2
#define WAKE_FREQ_10        0x3

#define BANKSEL_PRFTCH_EN_BIT       6
#define BANKSEL_CFG_USER_BANK_BIT   5
#define BANKSEL_MEM_SEL_BIT         4
#define BANKSEL_MEM_SEL_LENGTH      5

#define WHO_AM_I_BIT        6
#define WHO_AM_I_LENGTH     6

#define DMP_MEMORY_BANKS        8
#define DMP_MEMORY_BANK_SIZE    256
#define DMP_MEMORY_CHUNK_SIZE   16

// note: DMP code memory blocks defined at end of header file

#define TIMEOUT 100

class MPU9050 {
    public:
        uint16_t I2CErrorCounter;

        MPU9050();
        MPU9050(uint8_t devAddr, I2C_HandleTypeDef* hi2c);
        ~MPU9050();

		int Init();
        int ReadAccel(float& Ax, float& Ay, float& Az);
        int ReadGyro(float& Gx, float& Gy, float& Gz);
        int ReadAccelGyro(float& Ax, float& Ay, float& Az, float& Gx, float& Gy, float& Gz);


    private:
        uint8_t _devAddr;
        uint16_t devAddr;
        I2C_HandleTypeDef* hi2c;
};

#ifdef __cplusplus
}
#endif

#endif /* _MPU9050_H_ */
