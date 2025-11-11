#include "Register_ICM42670P.h"

uint8_t icm42670p_read_reg(spi_inst_t *spi, uint8_t cs_pin, uint8_t reg) 
{
    uint8_t tx_buf[2] = {reg | 0x80, 0x00};
    uint8_t rx_buf[2] = {0};
    
    gpio_put(cs_pin, 0);
    spi_write_read_blocking(spi, tx_buf, rx_buf, 2);
    gpio_put(cs_pin, 1);

    sleep_ms(30); // Tempo de repouso entre leituras consecutivas 
    
    return rx_buf[1];
}

void icm42670p_write_reg(spi_inst_t *spi, uint8_t cs_pin, uint8_t reg, uint8_t data) 
{
    uint8_t tx_buf[2] = {reg & 0x7F, data};
    
    gpio_put(cs_pin, 0);
    spi_write_blocking(spi, tx_buf, 2);
    gpio_put(cs_pin, 1);

    sleep_ms(30); // Tempo de repouso entre escritas consecutivas
}
