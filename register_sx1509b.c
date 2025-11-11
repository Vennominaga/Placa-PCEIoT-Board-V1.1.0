/**
 * @file register_sx1509b.c
 * @brief Implementação das funções de comunicação com o expansor de GPIO SX1509B
 * @details Este arquivo contém as funções de baixo nível para leitura e escrita
 * nos registros do SX1509B via interface I2C.
 */

#include "register_sx1509b.h"
#include <hardware/i2c.h>  // Para funções I2C do Raspberry Pi Pico

// =============================================
// Variáveis e Definições Locais
// =============================================



// =============================================
// Implementação das Funções
// =============================================

/**
 * @brief Escreve um valor em um registro do SX1509B
 * @param reg Endereço do registro a ser escrito (8 bits)
 * @param value Valor a ser escrito no registro (8 bits)
 * @note Esta função usa bloqueio I2C e não verifica erros
 * 
 * @example 
 * sx1509b_write_reg(REG_DIR_A, 0x00); // Configura todos os pinos do banco A como saída
 */
void sx1509b_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    
    // Envia o endereço do registro seguido pelo valor
    i2c_write_blocking(i2c0, SX1509B_I2C_ADDR_00, buf, 2, false);
}

/**
 * @brief Lê um valor de um registro do SX1509B
 * @param reg Endereço do registro a ser lido (8 bits)
 * @return Valor lido do registro (8 bits)
 * @note Esta função usa bloqueio I2C e não verifica erros
 * 
 * @example
 * uint8_t port_state = sx1509b_read_reg(REG_DATA_A); // Lê o estado dos pinos do banco A
 */
uint8_t sx1509b_read_reg(uint8_t reg)
{
    uint8_t value;
    
    // Primeiro envia o endereço do registro a ser lido
    i2c_write_blocking(i2c0, SX1509B_I2C_ADDR_00, &reg, 1, true);
    
    // Depois lê o valor do registro
    i2c_read_blocking(i2c0, SX1509B_I2C_ADDR_00, &value, 1, false);

    return value;
}
