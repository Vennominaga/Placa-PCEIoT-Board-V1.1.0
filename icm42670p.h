#ifndef ICM42670P_H
#define ICM42670P_H

#include "hardware/spi.h"

typedef enum
{
    ICM42670_SCALE_ACCEL_16G = 0b00,
    ICM42670_SCALE_ACCEL_8G  = 0b01,
    ICM42670_SCALE_ACCEL_4G  = 0b10,
    ICM42670_SCALE_ACCEL_2G  = 0b11
    
}scale_accel;

typedef enum
{
    ICM42670_SCALE_GYRO_20000DPS = 0b00,
    ICM42670_SCALE_GYRO_1000DPS  = 0b01,
    ICM42670_SCALE_GYRO_500DPS   = 0b10,
    ICM42670_SCALE_GYRO_250DPS   = 0b11
    
}scale_gyro;

typedef enum
{
    ICM42670_FREQ_ACCEL_1K6HZ    = 0b0101,
    ICM42670_FREQ_ACCEL_800HZ    = 0b0110,
    ICM42670_FREQ_ACCEL_400HZ    = 0b0111,
    ICM42670_FREQ_ACCEL_200HZ    = 0b1000,
    ICM42670_FREQ_ACCEL_100HZ    = 0b1001,
    ICM42670_FREQ_ACCEL_50HZ     = 0b1010,
    ICM42670_FREQ_ACCEL_25HZ     = 0b1011,
    ICM42670_FREQ_ACCEL_12_5HZ   = 0b1100,
    ICM42670_FREQ_ACCEL_6_25HZ   = 0b1101,
    ICM42670_FREQ_ACCEL_3_125HZ  = 0b1110,
    ICM42670_FREQ_ACCEL_1_5625HZ = 0b1111
    

}freq_accel;

typedef enum 
{
    ICM42670_FREQ_GYRO_1K6HZ  = 0b0101,
    ICM42670_FREQ_GYRO_800HZ  = 0b0110,
    ICM42670_FREQ_GYRO_400HZ  = 0b0111,
    ICM42670_FREQ_GYRO_200HZ  = 0b1000,
    ICM42670_FREQ_GYRO_100HZ  = 0b1001,
    ICM42670_FREQ_GYRO_50HZ   = 0b1010,
    ICM42670_FREQ_GYRO_25HZ   = 0b1011,
    ICM42670_FREQ_GYRO_12_5HZ = 0b1100

}freq_gyro;

typedef enum
{
    ICM42670_MODE_OFF       = 0x00,
    ICM42670_MODE_STANDBY   = 0x04,
    ICM42670_MODE_ACCEL_LP  = 0x02,
    ICM42670_MODE_ACCEL_LN  = 0x03,
    ICM42670_MODE_GYRO_LN   = 0x0C,
    ICM42670_MODE_6_AXIS_LN = 0x0F

}icm42670p_mode;

typedef enum 
{

    ERROR_ACCEL_CALIB = -1,
    ERROR_GYRO_CALIB  = -1
    
}icm42670p_erro;

typedef struct
{
    spi_inst_t *spi_instance;
    uint8_t cs_pin; 

    float accel_x;
    float accel_y;
    float accel_z;

    float accel_calib;

    icm42670p_mode mode;

    freq_accel f_accel;
    scale_accel rate_accel;

    float gyro_x;
    float gyro_y;
    float gyro_z;

    float gyro_calib;
    
    freq_gyro f_gyro;
    scale_gyro rate_gyro;


}icm42670p;


void icm42670p_pwr_mgmt0_config(uint8_t data);

void icm42670p_read_temperature(void);

void icm42670p_init(icm42670p *device);

void icm42670p_get_accel(icm42670p *device);

void icm42670p_get_gyro(icm42670p *device);

bool icm42670p_sensor_id(icm42670p *device);

#endif