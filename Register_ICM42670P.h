#ifndef REGISTER_ICM42670P_H
#define REGISTER_ICM42670P_H

#include "hardware/spi.h"
#include "pico/stdlib.h"

// =============================================
// DEFINIÇÕES DE PORTAS DO ICM42670P
// =============================================

#define SPI_PORT spi0
#define PIN_MISO 16
#define PIN_CS   10
#define PIN_SCK  18
#define PIN_MOSI 19 


// =============================================
// DEFINIÇÕES DE REGISTRADORES DO ICM42670P
// =============================================

// Registradores básicos

typedef enum
{

    WHO_AM_I_REG = 0x75,
    ICM42670_WHO_AM_I = 0x67,
    ICM42670_REG_PWR_MGMT0 = 0x1F,
    
    ICM42670_REG_GYRO_CONFIG0 = 0x20,
    ICM42670_REG_ACCEL_CONFIG0 = 0x21,
    
    ICM42670_REG_TEMP_DATA1 = 0x09,
    ICM42670_REG_TEMP_DATA0 = 0x0A,
    
    ICM42670_REG_ACCEL_DATA_X1 = 0x0B,
    ICM42670_REG_ACCEL_DATA_X0 = 0x0C,
    ICM42670_REG_ACCEL_DATA_Y1 = 0x0D,
    ICM42670_REG_ACCEL_DATA_Y0 = 0x0E,
    ICM42670_REG_ACCEL_DATA_Z1 = 0x0F,
    ICM42670_REG_ACCEL_DATA_Z0 = 0x10,
    
    ICM42670_REG_GYRO_DATA_X1 = 0x11,
    ICM42670_REG_GYRO_DATA_X0 = 0x12,
    ICM42670_REG_GYRO_DATA_Y1 = 0x13,
    ICM42670_REG_GYRO_DATA_Y0 = 0x14,
    ICM42670_REG_GYRO_DATA_Z1 = 0x15,
    ICM42670_REG_GYRO_DATA_Z0 = 0x16

}Register_ICM42670P;

// =============================================
// FUNÇÕES BÁSICAS DE LEITURA/ESCRITA
// =============================================

uint8_t icm42670p_read_reg(spi_inst_t *spi, uint8_t cs_pin, uint8_t reg);

void icm42670p_write_reg(spi_inst_t *spi, uint8_t cs_pin, uint8_t reg, uint8_t data);

#endif
