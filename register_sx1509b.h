#ifndef REGISTER_SX1509B
#define REGISTER_SX1509B

#include <stdio.h>
#include "hardware/i2c.h"

// =============================================
// Endereços I2C do dispositivo
// =============================================
#define SX1509B_I2C_ADDR_00 0x3E  // Endereço com A1=0, A0=0
#define SX1509B_I2C_ADDR_01 0x3F  // Endereço com A1=0, A0=1
#define SX1509B_I2C_ADDR_10 0x70  // Endereço com A1=1, A0=0
#define SX1509B_I2C_ADDR_11 0x71  // Endereço com A1=1, A0=1

// =============================================
// Definições de Registros
// =============================================
typedef enum {
    // ======================
    // Registros de Banco de E/S
    // ======================
    
    // Configuração de entrada/saída
    REG_INPUT_DISABLE_B = 0x00,  // Desabilita entrada para bancos B (GPIO8-15)
    REG_INPUT_DISABLE_A = 0x01,  // Desabilita entrada para bancos A (GPIO0-7)
    
    // Configuração de slew rate
    REG_LONG_SLEW_B = 0x02,      // Controle de slew rate para banco B
    REG_LONG_SLEW_A = 0x03,      // Controle de slew rate para banco A
    
    // Configuração de drive strength
    REG_LOW_DRIVE_B = 0x04,      // Configura drive baixo para banco B
    REG_LOW_DRIVE_A = 0x05,      // Configura drive baixo para banco A
    
    // Configuração de pull-up/pull-down
    REG_PULL_UP_B = 0x06,        // Habilita pull-up para banco B
    REG_PULL_UP_A = 0x07,        // Habilita pull-up para banco A
    REG_PULL_DOWN_B = 0x08,      // Habilita pull-down para banco B
    REG_PULL_DOWN_A = 0x09,      // Habilita pull-down para banco A
    
    // Configuração de open-drain
    REG_OPEN_DRAIN_B = 0x0A,     // Configura saída como open-drain (banco B)
    REG_OPEN_DRAIN_A = 0x0B,     // Configura saída como open-drain (banco A)
    
    // Inversão de polaridade
    REG_POLARITY_B = 0x0C,       // Inverte polaridade da entrada (banco B)
    REG_POLARITY_A = 0x0D,       // Inverte polaridade da entrada (banco A)
    
    // Direção dos pinos
    REG_DIR_B = 0x0E,            // Configura direção (entrada/saída) banco B
    REG_DIR_A = 0x0F,            // Configura direção (entrada/saída) banco A
    
    // Registros de dados
    REG_DATA_B = 0x10,           // Estado dos pinos (leitura/escrita) banco B
    REG_DATA_A = 0x11,           // Estado dos pinos (leitura/escrita) banco A
    
    // Interrupções
    REG_INTERRUPT_MASK_B = 0x12, // Máscara de interrupção banco B
    REG_INTERRUPT_MASK_A = 0x13, // Máscara de interrupção banco A
    REG_SENSE_HIGH_B = 0x14,     // Sensibilidade alta para interrupção banco B
    REG_SENSE_LOW_B = 0x15,      // Sensibilidade baixa para interrupção banco B
    REG_SENSE_HIGH_A = 0x16,     // Sensibilidade alta para interrupção banco A
    REG_SENSE_LOW_A = 0x17,      // Sensibilidade baixa para interrupção banco A
    REG_INTERRUPT_SOURCE_B = 0x18, // Fonte de interrupção banco B
    REG_INTERRUPT_SOURCE_A = 0x19, // Fonte de interrupção banco A
    REG_EVENT_STATUS_B = 0x1A,   // Status de evento banco B
    REG_EVENT_STATUS_A = 0x1B,   // Status de evento banco A
    
    // ======================
    // Registros de Nível Lógico
    // ======================
    REG_LEVEL_SHIFTER_1 = 0x1C,  // Configuração de level shifter 1
    REG_LEVEL_SHIFTER_2 = 0x1D,  // Configuração de level shifter 2
    
    // ======================
    // Registros de Clock e Miscelânea
    // ======================
    REG_CLOCK = 0x1E,            // Configuração de clock interno
    REG_MISC = 0x1F,             // Registro miscelâneo
    
    // ======================
    // Registros de Driver LED
    // ======================
    REG_LED_DRIVER_ENABLE_B = 0x20, // Habilita modo LED driver banco B
    REG_LED_DRIVER_ENABLE_A = 0x21, // Habilita modo LED driver banco A
    
    // Registros de temporização LED (PWM, blinking, breathing)
    // Padrão: REG_TON_x - Tempo ON, REG_ION_x - Intensidade, REG_OFF_x - Tempo OFF
    // Para alguns pinos: REG_TRISE_x - Tempo de subida, REG_TFALL_x - Tempo de descida
    REG_TON_0 = 0x29, REG_ION_0 = 0x2A, REG_OFF_0 = 0x2B,
    REG_TON_1 = 0x2C, REG_ION_1 = 0x2D, REG_OFF_1 = 0x2E,
    REG_TON_2 = 0x2F, REG_ION_2 = 0x30, REG_OFF_2 = 0x31,
    REG_TON_3 = 0x32, REG_ION_3 = 0x33, REG_OFF_3 = 0x34,
    REG_TON_4 = 0x35, REG_ION_4 = 0x36, REG_OFF_4 = 0x37, REG_TRISE_4 = 0x38, REG_TFALL_4 = 0x39,
    REG_TON_5 = 0x3A, REG_ION_5 = 0x3B, REG_OFF_5 = 0x3C, REG_TRISE_5 = 0x3D, REG_TFALL_5 = 0x3E,
    REG_TON_6 = 0x3F, REG_ION_6 = 0x40, REG_OFF_6 = 0x41, REG_TRISE_6 = 0x42, REG_TFALL_6 = 0x43,
    REG_TON_7 = 0x44, REG_ION_7 = 0x45, REG_OFF_7 = 0x46, REG_TRISE_7 = 0x47, REG_TFALL_7 = 0x48,
    REG_TON_8 = 0x49, REG_ION_8 = 0x4A, REG_OFF_8 = 0x4B,
    REG_TON_9 = 0x4C, REG_ION_9 = 0x4D, REG_OFF_9 = 0x4E,
    REG_TON_10 = 0x4F, REG_ION_10 = 0x50, REG_OFF_10 = 0x51,
    REG_TON_11 = 0x52, REG_ION_11 = 0x53, REG_OFF_11 = 0x54,
    REG_TON_12 = 0x55, REG_ION_12 = 0x56, REG_OFF_12 = 0x57, REG_TRISE_12 = 0x58, REG_TFALL_12 = 0x59,
    REG_TON_13 = 0x5A, REG_ION_13 = 0x5B, REG_OFF_13 = 0x5C, REG_TRISE_13 = 0x5D, REG_TFALL_13 = 0x5E,
    REG_TON_14 = 0x5F, REG_ION_14 = 0x60, REG_OFF_14 = 0x61, REG_TRISE_14 = 0x62, REG_TFALL_14 = 0x63,
    REG_TON_15 = 0x64, REG_ION_15 = 0x65, REG_OFF_15 = 0x66, REG_TRISE_15 = 0x67, REG_TFALL_15 = 0x68,
    
    // ======================
    // Registros de Entrada de Alta Impedância
    // ======================
    REG_HIGH_INPUT_B = 0x69,     // Configura entrada de alta impedância banco B
    REG_HIGH_INPUT_A = 0x6A      // Configura entrada de alta impedância banco A
} SX1509B_Register;

// =============================================
// Protótipos de Função
// =============================================

/**
 * @brief Escreve um valor em um registro do SX1509B
 * @param reg Endereço do registro a ser escrito
 * @param value Valor a ser escrito
 */
void sx1509b_write_reg(uint8_t reg, uint8_t value);

/**
 * @brief Lê um valor de um registro do SX1509B
 * @param reg Endereço do registro a ser lido
 * @return Valor lido do registro
 */
uint8_t sx1509b_read_reg(uint8_t reg);

#endif /* REGISTER_SX1509B */
