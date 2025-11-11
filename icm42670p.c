#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "stdint.h"

#include "icm42670p.h"
#include "Register_ICM42670P.h"

bool icm42670p_sensor_id(icm42670p *device)
{
    //Verificar comunicação com o sensor
    uint8_t who_am_i = icm42670p_read_reg(device->spi_instance, device->cs_pin, WHO_AM_I_REG);
    if (who_am_i != ICM42670_WHO_AM_I) {
        // printf("Erro: WHO_AM_I retornou 0x%02X (esperado 0x%02X)\n", who_am_i, ICM42670_WHO_AM_I);
        return false;
    }
    // printf("ICM42670P detectado com sucesso\n");
    return true;

}

void icm42670p_init(icm42670p *device) 
{
    //Verificação do sensor 
    if (icm42670p_sensor_id(device))
    {

    //Configurar modo de energia - ligar apenas o acelerômetro
     icm42670p_write_reg(device->spi_instance, device->cs_pin, ICM42670_REG_PWR_MGMT0, device->mode ); // Acelerômetro em modo LN

    //Configurar o acelerômetro
    
    //Zerando o Registrador do Acelerômetro
    uint8_t accel_config = 0x00;
    icm42670p_write_reg(device->spi_instance, device->cs_pin, ICM42670_REG_ACCEL_CONFIG0, accel_config);

    // Verificador de Frequências não permitidas 
    if(device->mode == ICM42670_MODE_ACCEL_LP)
    {
        if((device->f_accel == ICM42670_FREQ_ACCEL_1K6HZ) || (device->f_accel == ICM42670_FREQ_ACCEL_800HZ))
        {
            return;
        }
    }

    //Setando a escala e frequência acelerômetro
    accel_config = (device->rate_accel << 5) | device->f_accel;
    icm42670p_write_reg(device->spi_instance, device->cs_pin, ICM42670_REG_ACCEL_CONFIG0, accel_config);

    //Configurando o Giroscópio
    uint8_t gyro_congig = 0x00;
    icm42670p_write_reg(device->spi_instance, device->cs_pin, ICM42670_REG_GYRO_CONFIG0, gyro_congig);

    //Setando escala e frequência do giroscópio

    gyro_congig = (device->rate_gyro << 5) | device->f_gyro;
    icm42670p_write_reg(device->spi_instance, device->cs_pin, ICM42670_REG_GYRO_CONFIG0, gyro_congig);

    }

}

void icm42670p_get_gyro(icm42670p *device)
{
    //Calibração Giroscópio 
    switch (device->rate_gyro) {
        case ICM42670_SCALE_GYRO_20000DPS:
            device->gyro_calib = 16.4f;
            break;
        case ICM42670_SCALE_GYRO_1000DPS:
            device->gyro_calib = 32.8f;
            break;
        case ICM42670_SCALE_GYRO_500DPS:
            device->gyro_calib = 65.5f;
            break;
        case ICM42670_SCALE_GYRO_250DPS:
            device->gyro_calib = 131.0f;
            break;
        default:
            device->gyro_calib = ERROR_GYRO_CALIB; // Escala Invalida 
    }

    // Giroscópio para eixos do sensor 
    int16_t gyro_x = (int16_t)(icm42670p_read_reg(device->spi_instance, device->cs_pin, ICM42670_REG_GYRO_DATA_X1) << 8 | icm42670p_read_reg(device->spi_instance, device->cs_pin, ICM42670_REG_GYRO_DATA_X0));
    int16_t gyro_y = (int16_t)(icm42670p_read_reg(device->spi_instance, device->cs_pin, ICM42670_REG_GYRO_DATA_Y1) << 8 | icm42670p_read_reg(device->spi_instance, device->cs_pin, ICM42670_REG_GYRO_DATA_Y0));
    int16_t gyro_z = (int16_t)(icm42670p_read_reg(device->spi_instance, device->cs_pin, ICM42670_REG_GYRO_DATA_Z1) << 8 | icm42670p_read_reg(device->spi_instance, device->cs_pin, ICM42670_REG_GYRO_DATA_Z0));
    
    // Fator de Escala Giroscópio 
    if(device->gyro_calib != ERROR_GYRO_CALIB)
    {
        device->gyro_x = gyro_x / device->gyro_calib;
        device->gyro_y = gyro_y / device->gyro_calib;
        device->gyro_z = gyro_z / device->gyro_calib;
        
    }

    

}

void icm42670p_get_accel(icm42670p *device) 
{
    //Calibração Acelerômetro 
    switch (device->rate_accel) {
        case ICM42670_SCALE_ACCEL_16G:
            device->accel_calib = 2048.0f;
            break;
        case ICM42670_SCALE_ACCEL_8G:
            device->accel_calib = 4096.0f;
            break;
        case ICM42670_SCALE_ACCEL_4G:
            device->accel_calib = 8192.0f;
            break;
        case ICM42670_SCALE_ACCEL_2G:
            device->accel_calib = 16384.0f;
            break;
        default:
            device->accel_calib = ERROR_ACCEL_CALIB; // Escala Invalida 
    }

    // Aceleração para eixos do sensor 
    int16_t accel_x = (int16_t)(icm42670p_read_reg(device->spi_instance, device->cs_pin, ICM42670_REG_ACCEL_DATA_X1) << 8 | icm42670p_read_reg(device->spi_instance, device->cs_pin, ICM42670_REG_ACCEL_DATA_X0));
    int16_t accel_y = (int16_t)(icm42670p_read_reg(device->spi_instance, device->cs_pin, ICM42670_REG_ACCEL_DATA_Y1) << 8 | icm42670p_read_reg(device->spi_instance, device->cs_pin, ICM42670_REG_ACCEL_DATA_Y0));
    int16_t accel_z = (int16_t)(icm42670p_read_reg(device->spi_instance, device->cs_pin, ICM42670_REG_ACCEL_DATA_Z1) << 8 | icm42670p_read_reg(device->spi_instance, device->cs_pin, ICM42670_REG_ACCEL_DATA_Z0));

    // Fator de Escala Acelerômetro
    if(device->accel_calib != ERROR_ACCEL_CALIB)
    {
        device->accel_x = accel_x / device->accel_calib;
        device->accel_y = accel_y / device->accel_calib;
        device->accel_z = accel_z / device->accel_calib;
        
    } 

    

}

void icm42670p_pwr_mgmt0_config(uint8_t data)
{
    icm42670p_write_reg(SPI_PORT, PIN_CS, ICM42670_REG_PWR_MGMT0, data);
}

void icm42670p_read_temperature(void)
{
    sleep_ms(100);
    // Byte menos significativo
    uint8_t temp_0 = icm42670p_read_reg(SPI_PORT, PIN_CS, ICM42670_REG_TEMP_DATA0);
    sleep_ms(100);
    // Byte mais significativo 
     uint8_t temp_1 = icm42670p_read_reg(SPI_PORT, PIN_CS, ICM42670_REG_TEMP_DATA1);

    uint16_t temp_data = (temp_1 << 8 |temp_0 );
    
    int16_t temp_signed = (int16_t)temp_data;

    float temp =(temp_signed/128.0) + 25.0;
    
    printf("Temperatura de Operacao: %f °C\n", temp);

    sleep_ms(1000);
}

