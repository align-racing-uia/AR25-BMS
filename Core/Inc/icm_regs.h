
#ifndef ICM_REGS_H
#define ICM_REGS_H
// Accesible from all user banks
#define ICM_REG_BANK_SEL 0x76

// User Bank 0
#define ICM_UB0_REG_DEVICE_CONFIG 0x11

#define ICM_UB0_REG_DRIVE_CONFIG 0x13
#define ICM_UB0_REG_INT_CONFIG 0x14

#define ICM_UB0_REG_FIFO_CONFIG 0x16

#define ICM_UB0_REG_TEMP_DATA1 0x1D
#define ICM_UB0_REG_TEMP_DATA0 0x1E
#define ICM_UB0_REG_ACCEL_DATA_X1 0x1F
#define ICM_UB0_REG_ACCEL_DATA_X0 0x20
#define ICM_UB0_REG_ACCEL_DATA_Y1 0x21
#define ICM_UB0_REG_ACCEL_DATA_Y0 0x22
#define ICM_UB0_REG_ACCEL_DATA_Z1 0x23
#define ICM_UB0_REG_ACCEL_DATA_Z0 0x24
#define ICM_UB0_REG_GYRO_DATA_X1 0x25
#define ICM_UB0_REG_GYRO_DATA_X0 0x26
#define ICM_UB0_REG_GYRO_DATA_Y1 0x27
#define ICM_UB0_REG_GYRO_DATA_Y0 0x28
#define ICM_UB0_REG_GYRO_DATA_Z1 0x29
#define ICM_UB0_REG_GYRO_DATA_Z0 0x2A
#define ICM_UB0_REG_TMST_FSYNCH 0x2B
#define ICM_UB0_REG_TMST_FSYNCL 0x2C
#define ICM_UB0_REG_INT_STATUS 0x2D
#define ICM_UB0_REG_FIFO_COUNTH 0x2E
#define ICM_UB0_REG_FIFO_COUNTL 0x2F
#define ICM_UB0_REG_FIFO_DATA 0x30
#define ICM_UB0_REG_APEX_DATA0 0x31
#define ICM_UB0_REG_APEX_DATA1 0x32
#define ICM_UB0_REG_APEX_DATA2 0x33
#define ICM_UB0_REG_APEX_DATA3 0x34
#define ICM_UB0_REG_APEX_DATA4 0x35
#define ICM_UB0_REG_APEX_DATA5 0x36
#define ICM_UB0_REG_INT_STATUS2 0x37
#define ICM_UB0_REG_INT_STATUS3 0x38

#define ICM_UB0_REG_SIGNAL_PATH_RESET 0x4B
#define ICM_UB0_REG_INTF_CONFIG0 0x4C
#define ICM_UB0_REG_INTF_CONFIG1 0x4D
#define ICM_UB0_REG_PWR_MGMT0 0x4E
#define ICM_UB0_REG_GYRO_CONFIG0 0x4F
#define ICM_UB0_REG_ACCEL_CONFIG0 0x50
#define ICM_UB0_REG_GYRO_CONFIG1 0x51
#define ICM_UB0_REG_GYRO_ACCEL_CONFIG0 0x52
#define ICM_UB0_REG_ACCEL_CONFIG1 0x53
#define ICM_UB0_REG_TMST_CONFIG 0x54

#define ICM_UB0_REG_APEX_CONFIG0 0x56
#define ICM_UB0_REG_SMD_CONFIG 0x57

#define ICM_UB0_REG_FIFO_CONFIG1 0x5F
#define ICM_UB0_REG_FIFO_CONFIG2 0x60
#define ICM_UB0_REG_FIFO_CONFIG3 0x61
#define ICM_UB0_REG_FSYNC_CONFIG 0x62
#define ICM_UB0_REG_INT_CONFIG0 0x63
#define ICM_UB0_REG_INT_CONFIG1 0x64
#define ICM_UB0_REG_INT_SOURCE0 0x65
#define ICM_UB0_REG_INT_SOURCE1 0x66

#define ICM_UB0_REG_INT_SOURCE3 0x68
#define ICM_UB0_REG_INT_SOURCE4 0x69

#define ICM_UB0_REG_FIFO_LOST_PKT0 0x6C
#define ICM_UB0_REG_FIFO_LOST_PKT1 0x6D

#define ICM_UB0_REG_SELF_TEST_CONFIG 0x70

#define ICM_UB0_REG_WHO_AM_I 0x75

// User Bank 1
#define ICM_UB1_REG_SENSOR_CONFIG0 0x03
// break
#define ICM_UB1_REG_GYRO_CONFIG_STATIC2 0x0B
#define ICM_UB1_REG_GYRO_CONFIG_STATIC3 0x0C
#define ICM_UB1_REG_GYRO_CONFIG_STATIC4 0x0D
#define ICM_UB1_REG_GYRO_CONFIG_STATIC5 0x0E
#define ICM_UB1_REG_GYRO_CONFIG_STATIC6 0x0F
#define ICM_UB1_REG_GYRO_CONFIG_STATIC7 0x10
#define ICM_UB1_REG_GYRO_CONFIG_STATIC8 0x11
#define ICM_UB1_REG_GYRO_CONFIG_STATIC9 0x12
#define ICM_UB1_REG_GYRO_CONFIG_STATIC10 0x13
// break
#define ICM_UB1_REG_XG_ST_DATA 0x5F
#define ICM_UB1_REG_YG_ST_DATA 0x60
#define ICM_UB1_REG_ZG_ST_DATA 0x61
#define ICM_UB1_REG_TMSTVAL0 0x62
#define ICM_UB1_REG_TMSTVAL1 0x63
#define ICM_UB1_REG_TMSTVAL2 0x64
// break
#define ICM_UB1_REG_INTF_CONFIG4 0x7A
#define ICM_UB1_REG_INTF_CONFIG5 0x7B
#define ICM_UB1_REG_INTF_CONFIG6 0x7C

// User Bank 2
#define ICM_UB2_REG_ACCEL_CONFIG_STATIC2 0x03
#define ICM_UB2_REG_ACCEL_CONFIG_STATIC3 0x04
#define ICM_UB2_REG_ACCEL_CONFIG_STATIC4 0x05
// break
#define ICM_UB2_REG_XA_ST_DATA 0x3B
#define ICM_UB2_REG_YA_ST_DATA 0x3C
#define ICM_UB2_REG_ZA_ST_DATA 0x3D

// User Bank 4
#define ICM_UB4_REG_APEX_CONFIG1 0x40
#define ICM_UB4_REG_APEX_CONFIG2 0x41
#define ICM_UB4_REG_APEX_CONFIG3 0x42
#define ICM_UB4_REG_APEX_CONFIG4 0x43
#define ICM_UB4_REG_APEX_CONFIG5 0x44
#define ICM_UB4_REG_APEX_CONFIG6 0x45
#define ICM_UB4_REG_APEX_CONFIG7 0x46
#define ICM_UB4_REG_APEX_CONFIG8 0x47
#define ICM_UB4_REG_APEX_CONFIG9 0x48
// break
#define ICM_UB4_REG_ACCEL_WOM_X_THR 0x4A
#define ICM_UB4_REG_ACCEL_WOM_Y_THR 0x4B
#define ICM_UB4_REG_ACCEL_WOM_Z_THR 0x4C
#define ICM_UB4_REG_INT_SOURCE6 0x4D
#define ICM_UB4_REG_INT_SOURCE7 0x4E
#define ICM_UB4_REG_INT_SOURCE8 0x4F
#define ICM_UB4_REG_INT_SOURCE9 0x50
#define ICM_UB4_REG_INT_SOURCE10 0x51
// break
#define ICM_UB4_REG_OFFSET_USER0 0x77
#define ICM_UB4_REG_OFFSET_USER1 0x78
#define ICM_UB4_REG_OFFSET_USER2 0x79
#define ICM_UB4_REG_OFFSET_USER3 0x7A
#define ICM_UB4_REG_OFFSET_USER4 0x7B
#define ICM_UB4_REG_OFFSET_USER5 0x7C
#define ICM_UB4_REG_OFFSET_USER6 0x7D
#define ICM_UB4_REG_OFFSET_USER7 0x7E
#define ICM_UB4_REG_OFFSET_USER8 0x7F

#endif