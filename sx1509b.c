#include "sx1509b.h"
#include "register_sx1509b.h"


void sx1509b_config_clock(sx1509b_clk_source_t source, sx1509b_oscio_mode_t oscio_mode, uint8_t output_div)
{
    uint8_t config = 0;
    
    /*  Configura fonte do clock */
    config |= (source & 0x03) << 5;
    
    /*  Configura modo OSCIO */
    config |= (oscio_mode & 0x01) << 4;
    
    /*  Configura divisor de saída */
    config |= (output_div & 0x0F);
    /* Escreve no Registrador */
    sx1509b_write_reg(REG_CLOCK,config);
}

void sx1509b_set_pin_direction(uint8_t pin, bool output) 
{
    uint8_t reg;
    uint8_t mask;
    
    if (pin < 8) 
    {
        reg = REG_DIR_A;  /*  RegDirA para pinos 0-7 */
    }
    else 
    {
        reg = REG_DIR_B;  /*  RegDirB para pinos 8-15 */
        pin -= 8;
    }
    
    mask = 1 << pin;
    uint8_t current = sx1509b_read_reg(reg);
    
    if (output) 
    {
        current &= ~mask; /*  Configura como saída (bit 0) */
    } 
    else 
    {
        current |= mask;  /*  Configura como entrada (bit 1) */
    }
    
   sx1509b_write_reg(reg, current);
}
void sx1509b_set_pull(uint8_t pin, bool pull_up, bool pull_down) 
{
    uint8_t reg_pu, reg_pd;
    uint8_t mask;
    
    if (pin < 8) 
    {
        reg_pu = REG_PULL_UP_A;  // RegPullUpA
        reg_pd = REG_PULL_DOWN_A;  // RegPullDownA
    } 
    else 
    {
        reg_pu = REG_PULL_UP_B;  // RegPullUpB
        reg_pd = REG_PULL_DOWN_B; // RegPullDownB
        pin -= 8;
    }
    
    mask = 1 << pin;
    
    // Primeiro desativa ambos para evitar conflito
    uint8_t current_pu = sx1509b_read_reg(reg_pu);
    uint8_t current_pd = sx1509b_read_reg(reg_pd);
    
    current_pu &= ~mask;
    current_pd &= ~mask;
    
    // Depois ativa apenas o solicitado
    if (pull_up && !pull_down) 
    {
        current_pu |= mask;
    } 
    else if (!pull_up && pull_down) 
    {
        current_pd |= mask;
    }
    // Caso ambos sejam true, mantém ambos desativados
    
    sx1509b_write_reg(reg_pu, current_pu);
    sx1509b_write_reg(reg_pd, current_pd);
}

void sx1509b_set_open_drain(uint8_t pin, bool open_drain) 
{
    uint8_t reg;
    uint8_t mask;
    
    if (pin < 8) 
    {
        reg = REG_OPEN_DRAIN_A ; // RegOpenDrainA
    } else {
        reg = REG_OPEN_DRAIN_B; // RegOpenDrainB
        pin -= 8;
    }
    
    mask = 1 << pin;
    uint8_t current = sx1509b_read_reg(reg);
    
    if (open_drain) 
    {
        current |= mask;
    } 
    else 
    {
        current &= ~mask;
    }
    
    sx1509b_write_reg(reg, current);
}

bool sx1509b_read_pin(uint8_t pin) 
{
    uint8_t reg;
    uint8_t mask;
    
    if (pin < 8) 
    {
        reg = REG_DATA_A; // RegDataA para pinos 0-7
    } 
    else 
    {
        reg = REG_DATA_B; // RegDataB para pinos 8-15
        pin -= 8;
    }
    
    mask = 1 << pin;
    uint8_t current = sx1509b_read_reg(reg);
    
    return (current & mask) != 0;
}

void sx1509b_write_pin(uint8_t pin, bool value) 
{
    uint8_t reg;
    uint8_t mask;
    
    if (pin < 8) {
        reg = REG_DATA_A; // RegDataA para pinos 0-7
    } else {
        reg = REG_DATA_B; // RegDataB para pinos 8-15
        pin -= 8;
    }
    
    mask = 1 << pin;
    uint8_t current = sx1509b_read_reg(reg);
    
    if (value) {
        current |= mask;  // Seta o bit (HIGH)
    } else {
        current &= ~mask; // Limpa o bit (LOW)
    }
    
    sx1509b_write_reg(reg, current);
}

void sx1509b_led_enable(uint8_t pin, bool enable) 
{
    // Verifica se o pino é válido
    if (pin > 15) return;

    // 1. Configura o registrador de habilitação do driver LED
    uint8_t reg_led_enable = (pin < 8) ? REG_LED_DRIVER_ENABLE_A : REG_LED_DRIVER_ENABLE_B ; // RegLEDDriverEnableA/B
    uint8_t mask = 1 << (pin % 8);
    
    uint8_t current = sx1509b_read_reg(reg_led_enable);
    
    if (enable) 
    {
        current |= mask;
    } 
    else 
    {
        current &= ~mask;
    }
    
    sx1509b_write_reg(reg_led_enable, current);
}

/**
 * @brief Desativa o buffer de entrada para um pino específico
 * @param pin Número do pino (0-15)
 * @param disable true para desativar, false para ativar o buffer
 * 
 * Nota: Deve ser usado quando o pino está configurado como driver LED
 */
void sx1509b_disable_input_buffer(uint8_t pin, bool disable) {
    // Verificação de segurança
    if (pin > 15) return;

    // Determina o registrador correto
    uint8_t reg;
    if (pin < 8) 
    {
        reg = REG_INPUT_DISABLE_A; // RegInputDisableA (pinos 0-7)
    } 
    else 
    {
        reg = REG_INPUT_DISABLE_B; // RegInputDisableB (pinos 8-15)
        pin -= 8;   // Ajusta o offset
    }

    uint8_t mask = 1 << pin;
    uint8_t current = sx1509b_read_reg(reg);

    // Modifica o bit correspondente
    if (disable) {
        current |= mask;  // Desativa o buffer (1 = desativado)
    } else {
        current &= ~mask; // Ativa o buffer (0 = ativado)
    }

    sx1509b_write_reg(reg, current);
}

/**
 * @brief Configura o modo de fading para os bancos de I/O
 * @param bank_a_log true para modo logarítmico no Bank A (pinos 0-7), false para linear
 * @param bank_b_log true para modo logarítmico no Bank B (pinos 8-15), false para linear
 * 
 * Registrador utilizado: RegMisc (0x1F)
 * Bit 7: Modo Bank B (1=log, 0=linear)
 * Bit 3: Modo Bank A (1=log, 0=linear)
 */
void sx1509b_led_set_fade_mode(bool bank_a_log, bool bank_b_log) 
{
    // Lê o valor atual do registrador RegMisc
    uint8_t current = sx1509b_read_reg(REG_MISC);
    
    // Mantém todos os outros bits inalterados e configura apenas os bits de modo de fading
    current &= ~0x88;  // Limpa os bits 7 e 3
    
    if (bank_b_log) 
    {
        current |= 0x80;  // Seta bit 7 para Bank B logarítmico
    }
    
    if (bank_a_log) 
    {
        current |= 0x08;  // Seta bit 3 para Bank A logarítmico
    }
    
    // Escreve o novo valor no registrador
    sx1509b_write_reg(REG_MISC, current);
}

/**
 * @brief Obtém a frequência atual do ClkX em Hz
 * @return Frequência em Hz (0 se desabilitado)
 */
uint32_t get_current_clkx_freq() 
{
    // Lê a configuração atual do RegMisc
    uint8_t reg_misc = sx1509b_read_reg(REG_MISC);
    uint8_t clk_div = (reg_misc >> 4) & 0x07; // Bits 6:4
    
    if (clk_div == 0) return 0; // Clock desabilitado
    
    // fOSC é tipicamente 2MHz (oscilador interno)
    uint32_t fosc = 2000000;
    
    // Calcula ClkX = fOSC / (2^(N-1))
    return fosc / (1 << (clk_div - 1));
}

uint8_t calculate_ton_value(uint32_t time_ms) 
{
    const uint32_t clkx = get_current_clkx_freq();
    if (clkx == 0) return 1;
    
    const uint32_t product = time_ms * clkx / 1000UL;
    uint32_t value = product / 16320UL; // 255*64
   
    
    if (value <= 15) 
    {
        return value ? value : 1;
    }
    
    value = product / 130560UL; // 255*512
    value+=16;
    
    
    return (value > 31) ? 31 : value;
}

uint8_t calculate_toff_value(uint32_t time_ms) 
{
    const uint32_t clkx = get_current_clkx_freq();
    if (clkx == 0) return 1;
    
    const uint32_t product = time_ms * clkx / 1000UL;
    uint32_t value = product / 16320UL; // 255*64
    
    if (value <= 15) 
    {
        return value ? value : 1;
    }
    
    value = product / 130560UL; // 255*512
    value+=16;
    return (value > 31) ? 31 : value;
}

/**
 * @brief Configura os bits de controle do registrador MISC (0x1F)
 * 
 * @param nreset_mode Modo do pino NRESET (ver sx1509b_nreset_mode_t)
 * @param autoinc_mode Modo de auto-incremento (ver sx1509b_autoinc_mode_t)
 * @param intclr_mode Modo de limpeza de interrupção (ver sx1509b_intclr_mode_t)
 * 
 * @note Esta função preserva as configurações existentes dos bits superiores (7-3)
 */
void sx1509b_config_misc_bits(sx1509b_nreset_mode_t nreset_mode, 
                             sx1509b_autoinc_mode_t autoinc_mode, 
                             sx1509b_intclr_mode_t intclr_mode) {
    // Lê o valor atual do registrador
    uint8_t current_reg = sx1509b_read_reg(REG_MISC);
    
    // Mantém os bits 7-3 e atualiza apenas os bits 2-0
    uint8_t new_value_reg = (current_reg & 0xF8) |       // Mantém bits 7-3
                       ((nreset_mode & 1) << 2) |    // Configura bit 2
                       ((autoinc_mode & 1) << 1) |   // Configura bit 1
                       (intclr_mode & 1);            // Configura bit 0
    
    // Escreve o novo valor no registrador
    sx1509b_write_reg(REG_MISC, new_value_reg);
}
/**
 * @brief Configura a frequência do clock para os drivers LED
 * 
 * @param frequency_div Divisor de frequência (0-7):
 *                     0: Desabilita o clock (LED drivers desativados)
 *                     1: f = IOSC / 1 (2MHz)
 *                     2: f = IOSC / 2 (1MHz)
 *                     3: f = IOSC / 4 (500kHz)
 *                     4: f = IOSC / 8 (250kHz)
 *                     5: f = IOSC / 16 (125kHz)
 *                     6: f = IOSC / 32 (62.5kHz)
 *                     7: f = IOSC / 64 (31.25kHz)
 * @return true se a configuração foi bem sucedida, false se o parâmetro é inválido
 * 
 * @note IOSC é tipicamente 2MHz (oscilador interno)
 */
bool sx1509b_set_led_frequency(uint8_t frequency_div) {
    // Verifica se o valor do divisor é válido (0-7)
    if (frequency_div > 7) {
        return false;
    }

    // Lê o valor atual do registrador MISC
    uint8_t current = sx1509b_read_reg(REG_MISC);
    
    // Mantém todos os outros bits inalterados e configura apenas os bits 6:4
    current &= ~0x70;  // Limpa os bits 6:4 (01110000)
    current |= (frequency_div & 0x07) << 4; // Configura os bits 6:4
    
    // Escreve o novo valor no registrador
    sx1509b_write_reg(REG_MISC, current);
    
    return true;
}

/**
 * @brief Calcula o valor para o registrador RegTRiseX baseado no tempo desejado
 * 
 * @param pin Número do pino (4-7 ou 12-15, que suportam fading)
 * @param rise_time_ms Tempo de fade in desejado em milissegundos
 * @return uint8_t Valor a ser escrito no registrador RegTRiseX (0-31)
 * 
 * @note Retorna 0 se:
 *       - O clock estiver desabilitado (ClkX = 0)
 *       - O pino não suportar fading
 *       - O tempo solicitado for 0
 */
uint8_t sx1509b_calculate_reg_trise(uint8_t pin, uint16_t rise_time_ms) {
    // Verifica se é um pino que suporta fading (4-7 ou 12-15)
    if (!((pin >= 4 && pin <= 7) || (pin >= 12 && pin <= 15))) {
        return 0;
    }
    
    // Se tempo for 0, retorna 0 (OFF)
    if (rise_time_ms == 0) {
        return 0;
    }
    
    // Obtém a frequência do clock
    uint32_t clkx = get_current_clkx_freq();
    if (clkx == 0) {
        return 0; // Clock desabilitado
    }
    
    // Determina os registradores para este pino
    uint8_t reg_ion, reg_off;
    switch (pin) {
        case 4:  reg_ion = REG_ION_4;  reg_off = REG_OFF_4;  break;
        case 5:  reg_ion = REG_ION_5;  reg_off = REG_OFF_5;  break;
        case 6:  reg_ion = REG_ION_6;  reg_off = REG_OFF_6;  break;
        case 7:  reg_ion = REG_ION_7;  reg_off = REG_OFF_7;  break;
        case 12: reg_ion = REG_ION_12; reg_off = REG_OFF_12; break;
        case 13: reg_ion = REG_ION_13; reg_off = REG_OFF_13; break;
        case 14: reg_ion = REG_ION_14; reg_off = REG_OFF_14; break;
        case 15: reg_ion = REG_ION_15; reg_off = REG_OFF_15; break;
        default: return 0; // Nunca deve acontecer devido à verificação inicial
    }
    
    // Lê os valores dos registradores
    uint8_t ion = sx1509b_read_reg(reg_ion);
    uint8_t off = sx1509b_read_reg(reg_off);
    uint8_t off_low_bits = off & 0x07; // Pega apenas os bits 2:0 de RegOffX
    
    // Calcula o termo (RegIOnX - (4 * RegOfIX[2:0]))
    int16_t term = ion - (4 * off_low_bits);
    if (term <= 0) {
        term = 1; // Garante valor mínimo
    }
    
    // Converte o tempo para milissegundos (já está em ms, não precisa multiplicar)
    uint32_t time_ms = rise_time_ms*1000UL;
    
    // Primeiro tenta a equação para 1-15: TRiseX = term * RegTRiseX * 255 / ClkX
    // Isolando RegTRiseX: RegTRiseX = (time_ms * ClkX) / (term * 255)
    uint32_t trise = (time_ms * clkx) / (255UL * term);
    
    if (trise <= 15) {
        return (trise == 0) ? 1 : trise; // Garante mínimo de 1
    }
    
    // Se não couber em 15, usa a segunda equação (16-31)
    // TRiseX = 16 * term * RegTRiseX * 255 / ClkX
    // Isolando RegTRiseX: RegTRiseX = (time_ms * ClkX) / (16 * term * 255)
    trise = (time_ms * clkx) / (255UL * term * 16);
    
    // Aplica os limites para 16-31

    trise+=16;
    return trise>31 ? 31:trise;
}

/**
 * @brief Configura um pino no modo LED estático
 * 
 * @param pin Número do pino (0-15)
 * @param intensity Intensidade do LED (0-255)
 * 
 * @note Esta função realiza todas as configurações necessárias para o modo LED estático:
 *       - Desativa o input buffer
 *       - Desativa pull-up
 *       - Ativa open-drain
 *       - Configura direção como saída
 *       - Habilita o modo LED driver
 *       - Configura o clock LED
 *       - Define registros TOn=0, IOn=intensidade, OFF=0, TRise=0, TFall=0
 *       - Seta RegData como 0
 */
void sx1509b_led_static(uint8_t pin, uint8_t intensity) {
    // Verificação de segurança do pino
    if (pin > 15) return;

    // 1. Configurações básicas do pino
    sx1509b_disable_input_buffer(pin, true);    // Desativa input buffer
    sx1509b_set_pull(pin, false, false);       // Desativa pull-up/pull-down
    sx1509b_set_open_drain(pin, true);         // Ativa open-drain
    sx1509b_set_pin_direction(pin, true);      // Configura como saída

    // 2. Habilita o modo LED driver para este pino
    sx1509b_led_enable(pin, true);

    // 3. Configura o clock LED (usa clock interno de 2MHz com divisor 1)
    sx1509b_config_clock(SX1509B_CLK_2MHZ, SX1509B_OSCIO_OUT, 0);

    // 4. Configura os registros específicos do LED
    uint8_t reg_ton, reg_ion, reg_off, reg_trise, reg_tfall;
    
    // Determina os registradores com base no pino
    switch(pin) {
        case 0:
            reg_ton = REG_TON_0; reg_ion = REG_ION_0; reg_off = REG_OFF_0;
            reg_trise = 0; reg_tfall = 0; // Pinos 0-3 não têm TRise/TFall
            break;
        case 1:
            reg_ton = REG_TON_1; reg_ion = REG_ION_1; reg_off = REG_OFF_1;
            reg_trise = 0; reg_tfall = 0;
            break;
        case 2:
            reg_ton = REG_TON_2; reg_ion = REG_ION_2; reg_off = REG_OFF_2;
            reg_trise = 0; reg_tfall = 0;
            break;
        case 3:
            reg_ton = REG_TON_3; reg_ion = REG_ION_3; reg_off = REG_OFF_3;
            reg_trise = 0; reg_tfall = 0;
            break;
        case 4:
            reg_ton = REG_TON_4; reg_ion = REG_ION_4; reg_off = REG_OFF_4;
            reg_trise = REG_TRISE_4; reg_tfall = REG_TFALL_4;
            break;
        case 5:
            reg_ton = REG_TON_5; reg_ion = REG_ION_5; reg_off = REG_OFF_5;
            reg_trise = REG_TRISE_5; reg_tfall = REG_TFALL_5;
            break;
        case 6:
            reg_ton = REG_TON_6; reg_ion = REG_ION_6; reg_off = REG_OFF_6;
            reg_trise = REG_TRISE_6; reg_tfall = REG_TFALL_6;
            break;
        case 7:
            reg_ton = REG_TON_7; reg_ion = REG_ION_7; reg_off = REG_OFF_7;
            reg_trise = REG_TRISE_7; reg_tfall = REG_TFALL_7;
            break;
        case 8:
            reg_ton = REG_TON_8; reg_ion = REG_ION_8; reg_off = REG_OFF_8;
            reg_trise = 0; reg_tfall = 0;
            break;
        case 9:
            reg_ton = REG_TON_9; reg_ion = REG_ION_9; reg_off = REG_OFF_9;
            reg_trise = 0; reg_tfall = 0;
            break;
        case 10:
            reg_ton = REG_TON_10; reg_ion = REG_ION_10; reg_off = REG_OFF_10;
            reg_trise = 0; reg_tfall = 0;
            break;
        case 11:
            reg_ton = REG_TON_11; reg_ion = REG_ION_11; reg_off = REG_OFF_11;
            reg_trise = 0; reg_tfall = 0;
            break;
        case 12:
            reg_ton = REG_TON_12; reg_ion = REG_ION_12; reg_off = REG_OFF_12;
            reg_trise = REG_TRISE_12; reg_tfall = REG_TFALL_12;
            break;
        case 13:
            reg_ton = REG_TON_13; reg_ion = REG_ION_13; reg_off = REG_OFF_13;
            reg_trise = REG_TRISE_13; reg_tfall = REG_TFALL_13;
            break;
        case 14:
            reg_ton = REG_TON_14; reg_ion = REG_ION_14; reg_off = REG_OFF_14;
            reg_trise = REG_TRISE_14; reg_tfall = REG_TFALL_14;
            break;
        case 15:
            reg_ton = REG_TON_15; reg_ion = REG_ION_15; reg_off = REG_OFF_15;
            reg_trise = REG_TRISE_15; reg_tfall = REG_TFALL_15;
            break;
        default:
            return; // Nunca deve acontecer devido à verificação inicial
    }

    // Configura os registros do LED para modo estático
    sx1509b_write_reg(reg_ton, 0);      // TOn = 0 (modo estático)
    sx1509b_write_reg(reg_ion, intensity); // IOn = intensidade desejada
    sx1509b_write_reg(reg_off, 0);      // OFF = 0
    
    // Configura TRise e TFall apenas para pinos que suportam fading
    if (reg_trise != 0) {
        sx1509b_write_reg(reg_trise, 0); // TRise = 0
        sx1509b_write_reg(reg_tfall, 0); // TFall = 0
    }

    // 5. Configura o pino como LOW no registrador de dados
    sx1509b_write_pin(pin, false);
}


/**
 * @brief Configura um pino no modo LED piscante (blinking)
 * 
 * @param pin Número do pino (0-15)
 * @param intensity Intensidade do LED quando ligado (0-255)
 * @param on_time_ms Tempo em milissegundos que o LED fica ligado
 * @param off_time_ms Tempo em milissegundos que o LED fica desligado
 * @param fade_in_ms Tempo de fade in em ms (0 para desativar, apenas pinos 4-7 e 12-15)
 * @param fade_out_ms Tempo de fade out em ms (0 para desativar, apenas pinos 4-7 e 12-15)
 * 
 * @note Esta função configura todos os parâmetros necessários para o modo blinking:
 *       - Desativa o input buffer
 *       - Desativa pull-up/pull-down
 *       - Ativa open-drain
 *       - Configura direção como saída
 *       - Habilita o modo LED driver
 *       - Configura o clock LED
 *       - Define registros TOn, IOn, OFF, TRise e TFall
 *       - Seta RegData como 0 para iniciar a operação
 */
void sx1509b_led_blinking(uint8_t pin, uint8_t intensity, uint16_t on_time_ms, 
                         uint16_t off_time_ms, uint16_t fade_in_ms, uint16_t fade_out_ms)
{
    // Verificação de segurança do pino
    if (pin > 15) return;

    // 1. Configurações básicas do pino
    sx1509b_disable_input_buffer(pin, true);    // Desativa input buffer
    sx1509b_set_pull(pin, false, false);       // Desativa pull-up/pull-down
    sx1509b_set_open_drain(pin, true);         // Ativa open-drain
    sx1509b_set_pin_direction(pin, true);      // Configura como saída

    // 2. Habilita o modo LED driver para este pino
    sx1509b_led_enable(pin, true);

    // 3. Configura o clock LED (usa clock interno de 2MHz com divisor 1)
    sx1509b_config_clock(SX1509B_CLK_2MHZ, SX1509B_OSCIO_OUT, 0);

    // 4. Configura os registros específicos do LED
    uint8_t reg_ton, reg_ion, reg_off, reg_trise, reg_tfall;
    
    // Determina os registradores com base no pino
    switch(pin) {
        case 0:
            reg_ton = REG_TON_0; reg_ion = REG_ION_0; reg_off = REG_OFF_0;
            reg_trise = 0; reg_tfall = 0; // Pinos 0-3 não têm TRise/TFall
            break;
        case 1:
            reg_ton = REG_TON_1; reg_ion = REG_ION_1; reg_off = REG_OFF_1;
            reg_trise = 0; reg_tfall = 0;
            break;
        case 2:
            reg_ton = REG_TON_2; reg_ion = REG_ION_2; reg_off = REG_OFF_2;
            reg_trise = 0; reg_tfall = 0;
            break;
        case 3:
            reg_ton = REG_TON_3; reg_ion = REG_ION_3; reg_off = REG_OFF_3;
            reg_trise = 0; reg_tfall = 0;
            break;
        case 4:
            reg_ton = REG_TON_4; reg_ion = REG_ION_4; reg_off = REG_OFF_4;
            reg_trise = REG_TRISE_4; reg_tfall = REG_TFALL_4;
            break;
        case 5:
            reg_ton = REG_TON_5; reg_ion = REG_ION_5; reg_off = REG_OFF_5;
            reg_trise = REG_TRISE_5; reg_tfall = REG_TFALL_5;
            break;
        case 6:
            reg_ton = REG_TON_6; reg_ion = REG_ION_6; reg_off = REG_OFF_6;
            reg_trise = REG_TRISE_6; reg_tfall = REG_TFALL_6;
            break;
        case 7:
            reg_ton = REG_TON_7; reg_ion = REG_ION_7; reg_off = REG_OFF_7;
            reg_trise = REG_TRISE_7; reg_tfall = REG_TFALL_7;
            break;
        case 8:
            reg_ton = REG_TON_8; reg_ion = REG_ION_8; reg_off = REG_OFF_8;
            reg_trise = 0; reg_tfall = 0;
            break;
        case 9:
            reg_ton = REG_TON_9; reg_ion = REG_ION_9; reg_off = REG_OFF_9;
            reg_trise = 0; reg_tfall = 0;
            break;
        case 10:
            reg_ton = REG_TON_10; reg_ion = REG_ION_10; reg_off = REG_OFF_10;
            reg_trise = 0; reg_tfall = 0;
            break;
        case 11:
            reg_ton = REG_TON_11; reg_ion = REG_ION_11; reg_off = REG_OFF_11;
            reg_trise = 0; reg_tfall = 0;
            break;
        case 12:
            reg_ton = REG_TON_12; reg_ion = REG_ION_12; reg_off = REG_OFF_12;
            reg_trise = REG_TRISE_12; reg_tfall = REG_TFALL_12;
            break;
        case 13:
            reg_ton = REG_TON_13; reg_ion = REG_ION_13; reg_off = REG_OFF_13;
            reg_trise = REG_TRISE_13; reg_tfall = REG_TFALL_13;
            break;
        case 14:
            reg_ton = REG_TON_14; reg_ion = REG_ION_14; reg_off = REG_OFF_14;
            reg_trise = REG_TRISE_14; reg_tfall = REG_TFALL_14;
            break;
        case 15:
            reg_ton = REG_TON_15; reg_ion = REG_ION_15; reg_off = REG_OFF_15;
            reg_trise = REG_TRISE_15; reg_tfall = REG_TFALL_15;
            break;
        default:
            return; // Nunca deve acontecer devido à verificação inicial
    }

    // Configura intensidade quando ligado
    sx1509b_write_reg(reg_ion, intensity);
    
    // Configura tempo ligado e intensidade desligado
    uint8_t off_value = 0; // Intensidade desligado (0)
    uint8_t ton_value = calculate_ton_value(on_time_ms);
    uint8_t toff_value = calculate_toff_value(off_time_ms);
    
    // Para blinking, precisamos combinar toff_value (bits 7:3) e off_value (bits 2:0)
    uint8_t off_reg_value = (toff_value << 3) | (off_value & 0x07);
    sx1509b_write_reg(reg_off, off_reg_value);
    
    // Configura tempo ligado
    sx1509b_write_reg(reg_ton, ton_value);
    
    // Configura fade in/out apenas para pinos que suportam
    if (reg_trise != 0) {
        uint8_t trise_value = (fade_in_ms > 0) ? sx1509b_calculate_reg_trise(pin, fade_in_ms) : 0;
        uint8_t tfall_value = (fade_out_ms > 0) ? sx1509b_calculate_reg_trise(pin, fade_out_ms) : 0;
        
        sx1509b_write_reg(reg_trise, trise_value);
        sx1509b_write_reg(reg_tfall, tfall_value);
    }

    // 5. Inicia a operação configurando o pino como LOW no registrador de dados
    sx1509b_write_pin(pin, false);
}

/**
 * @brief Configura um pino no modo LED com fade (breathing)
 * 
 * @param pin Número do pino (4-7 ou 12-15 - apenas pinos que suportam fading)
 * @param min_intensity Intensidade mínima do LED (0-255)
 * @param max_intensity Intensidade máxima do LED (0-255)
 * @param fade_in_time_ms Tempo de fade in em milissegundos
 * @param fade_out_time_ms Tempo de fade out em milissegundos
 * @param on_time_ms Tempo em milissegundos que o LED fica na intensidade máxima (0 para contínuo)
 * @param off_time_ms Tempo em milissegundos que o LED fica na intensidade mínima (0 para contínuo)
 * 
 * @note Esta função só funciona com pinos que suportam fading (4-7 e 12-15)
 * @note Configura todos os parâmetros necessários para o modo breathing
 */
void sx1509b_led_fade(uint8_t pin, uint8_t min_intensity, uint8_t max_intensity,
                     uint16_t fade_in_time_ms, uint16_t fade_out_time_ms,
                     uint16_t on_time_ms, uint16_t off_time_ms)
{
    // Verifica se é um pino que suporta fading (4-7 ou 12-15)
    if (!((pin >= 4 && pin <= 7) || (pin >= 12 && pin <= 15))) {
        return;
    }

    // Garante que as intensidades estão dentro do range válido
    min_intensity = (min_intensity > 255) ? 255 : min_intensity;
    max_intensity = (max_intensity > 255) ? 255 : max_intensity;

    // 1. Configurações básicas do pino
    sx1509b_disable_input_buffer(pin, true);    // Desativa input buffer
    sx1509b_set_pull(pin, false, false);       // Desativa pull-up/pull-down
    sx1509b_set_open_drain(pin, true);         // Ativa open-drain
    sx1509b_set_pin_direction(pin, true);      // Configura como saída

    // 2. Habilita o modo LED driver para este pino
    sx1509b_led_enable(pin, true);

    // 3. Configura o clock LED (usa clock interno de 2MHz com divisor 1)
    sx1509b_config_clock(SX1509B_CLK_2MHZ, SX1509B_OSCIO_OUT, 0);

    // 4. Determina os registradores para este pino
    uint8_t reg_ton, reg_ion, reg_off, reg_trise, reg_tfall;
    
    switch(pin) {
        case 4:
            reg_ton = REG_TON_4; reg_ion = REG_ION_4; reg_off = REG_OFF_4;
            reg_trise = REG_TRISE_4; reg_tfall = REG_TFALL_4;
            break;
        case 5:
            reg_ton = REG_TON_5; reg_ion = REG_ION_5; reg_off = REG_OFF_5;
            reg_trise = REG_TRISE_5; reg_tfall = REG_TFALL_5;
            break;
        case 6:
            reg_ton = REG_TON_6; reg_ion = REG_ION_6; reg_off = REG_OFF_6;
            reg_trise = REG_TRISE_6; reg_tfall = REG_TFALL_6;
            break;
        case 7:
            reg_ton = REG_TON_7; reg_ion = REG_ION_7; reg_off = REG_OFF_7;
            reg_trise = REG_TRISE_7; reg_tfall = REG_TFALL_7;
            break;
        case 12:
            reg_ton = REG_TON_12; reg_ion = REG_ION_12; reg_off = REG_OFF_12;
            reg_trise = REG_TRISE_12; reg_tfall = REG_TFALL_12;
            break;
        case 13:
            reg_ton = REG_TON_13; reg_ion = REG_ION_13; reg_off = REG_OFF_13;
            reg_trise = REG_TRISE_13; reg_tfall = REG_TFALL_13;
            break;
        case 14:
            reg_ton = REG_TON_14; reg_ion = REG_ION_14; reg_off = REG_OFF_14;
            reg_trise = REG_TRISE_14; reg_tfall = REG_TFALL_14;
            break;
        case 15:
            reg_ton = REG_TON_15; reg_ion = REG_ION_15; reg_off = REG_OFF_15;
            reg_trise = REG_TRISE_15; reg_tfall = REG_TFALL_15;
            break;
        default:
            return; // Nunca deve acontecer devido à verificação inicial
    }

    // 5. Configura as intensidades
    sx1509b_write_reg(reg_ion, max_intensity); // Intensidade máxima
    
    // Configura intensidade mínima (usando os 3 bits menos significativos de RegOff)
    // A intensidade mínima é calculada como 4 x RegOff[2:0]
    uint8_t off_intensity = min_intensity / 4;
    if (off_intensity > 0x07) off_intensity = 0x07; // Limita a 3 bits

    // 6. Configura os tempos
    uint8_t ton_value = (on_time_ms > 0) ? calculate_ton_value(on_time_ms) : 0;
    uint8_t toff_value = (off_time_ms > 0) ? calculate_toff_value(off_time_ms) : 0;
    
    // Combina toff_value (bits 7:3) e off_intensity (bits 2:0)
    uint8_t off_reg_value = (toff_value << 3) | (off_intensity & 0x07);
    sx1509b_write_reg(reg_off, off_reg_value);
    
    // Configura tempo ligado (se for 0, modo contínuo)
    sx1509b_write_reg(reg_ton, ton_value);
    
    // 7. Configura os tempos de fade
    uint8_t trise_value = (fade_in_time_ms > 0) ? sx1509b_calculate_reg_trise(pin, fade_in_time_ms) : 0;
    uint8_t tfall_value = (fade_out_time_ms > 0) ? sx1509b_calculate_reg_trise(pin, fade_out_time_ms) : 0;
    
    sx1509b_write_reg(reg_trise, trise_value);
    sx1509b_write_reg(reg_tfall, tfall_value);

    // 8. Inicia a operação configurando o pino como LOW no registrador de dados
    sx1509b_write_pin(pin, false);
}