#ifndef SX1509B_H
#define SX1509B_H

#include <stdint.h>
#include <stdbool.h>
#include "register_sx1509b.h"

/**
 * @file sx1509b.h
 * @brief Biblioteca para controle do expansor de GPIO e controlador LED SX1509B
 * 
 * Esta biblioteca fornece funções para configurar e controlar os pinos do SX1509B,
 * incluindo funções básicas de GPIO e recursos avançados para controle de LEDs.
 */

// =============================================
// Seção: Tipos enumerados e definições
// =============================================

/**
 * @enum sx1509b_clk_source_t
 * @brief Define as fontes de clock disponíveis para o SX1509B
 */
typedef enum {
    SX1509B_CLK_OFF = 0,     ///< Desliga o clock completamente
    SX1509B_CLK_EXT = 1,     ///< Utiliza clock externo (OSCIN)
    SX1509B_CLK_2MHZ = 2     ///< Utiliza oscilador interno de 2MHz
} sx1509b_clk_source_t;

/**
 * @enum sx1509b_oscio_mode_t
 * @brief Define os modos de operação do pino OSCIO
 */
typedef enum {
    SX1509B_OSCIO_IN = 0,    ///< Configura OSCIO como entrada (OSCIN)
    SX1509B_OSCIO_OUT = 1    ///< Configura OSCIO como saída (OSCOUT)
} sx1509b_oscio_mode_t;

/**
 * @enum sx1509b_nreset_mode_t
 * @brief Modos de operação do pino NRESET
 */
typedef enum {
    SX1509B_NRESET_FULL_RESET = 0,  ///< NRESET equivale a Power-On Reset (POR)
    SX1509B_NRESET_COUNTERS_ONLY = 1 ///< NRESET reseta apenas contadores PWM/Blink/Fade
} sx1509b_nreset_mode_t;

/**
 * @enum sx1509b_autoinc_mode_t
 * @brief Modos de auto-incremento de endereço
 */
typedef enum {
    SX1509B_AUTOINC_ENABLED = 0,  ///< Habilita auto-incremento em leituras/escritas consecutivas
    SX1509B_AUTOINC_DISABLED = 1   ///< Mantém endereço fixo em leituras/escritas consecutivas
} sx1509b_autoinc_mode_t;

/**
 * @enum sx1509b_intclr_mode_t
 * @brief Modos de limpeza de interrupção
 */
typedef enum {
    SX1509B_INTCLR_AUTO = 0,  ///< Limpa NINT automaticamente ao ler RegData
    SX1509B_INTCLR_MANUAL = 1  ///< Requer limpeza manual de NINT
} sx1509b_intclr_mode_t;

// =============================================
// Seção: Configuração de GPIO básica
// =============================================

/**
 * @brief Configura a direção de um pino (entrada ou saída)
 * @param pin Número do pino a ser configurado (0-15)
 * @param output true para configurar como saída, false para entrada
 */
void sx1509b_set_pin_direction(uint8_t pin, bool output);

/**
 * @brief Escreve um valor lógico em um pino configurado como saída
 * @param pin Número do pino (0-15)
 * @param value Valor a ser escrito (true = alto, false = baixo)
 */
void sx1509b_write_pin(uint8_t pin, bool value);

/**
 * @brief Lê o valor lógico de um pino configurado como entrada
 * @param pin Número do pino (0-15)
 * @return true se o pino está em nível alto, false se está em nível baixo
 */
bool sx1509b_read_pin(uint8_t pin);

/**
 * @brief Configura os resistores de pull-up/pull-down de um pino
 * @param pin Número do pino (0-15)
 * @param pull_up true para habilitar pull-up
 * @param pull_down true para habilitar pull-down
 * @note Não é possível habilitar pull-up e pull-down simultaneamente
 */
void sx1509b_set_pull(uint8_t pin, bool pull_up, bool pull_down);

/**
 * @brief Configura um pino para modo open-drain
 * @param pin Número do pino (0-15)
 * @param open_drain true para habilitar modo open-drain, false para push-pull
 */
void sx1509b_set_open_drain(uint8_t pin, bool open_drain);

/**
 * @brief Habilita/desabilita o buffer de entrada de um pino
 * @param pin Número do pino (0-15)
 * @param disable true para desabilitar o buffer, false para habilitar
 */
void sx1509b_disable_input_buffer(uint8_t pin, bool disable);

// =============================================
// Seção: Level Shifter (Conversor de Nível)
// =============================================

/**
 * @brief Configura um level shifter entre dois pinos
 * @param pin_from Pino de origem (0-15)
 * @param pin_to Pino de destino (0-15)
 */
void sx1509b_set_level_shifter(uint8_t pin_from, uint8_t pin_to);

/**
 * @brief Desabilita o level shifter de um pino
 * @param pin Número do pino (0-15)
 */
void sx1509b_disable_level_shifter(uint8_t pin);

// =============================================
// Seção: Configuração de Clock
// =============================================

/**
 * @brief Configura o clock do SX1509B
 * @param source Fonte do clock (ver sx1509b_clk_source_t)
 * @param oscio_mode Modo do pino OSCIO (ver sx1509b_oscio_mode_t)
 * @param output_div Divisor de frequência para saída (0x0=GND, 0xF=VCC, outros=fOSC/(2^(N-1)))
 */
void sx1509b_config_clock(sx1509b_clk_source_t source, sx1509b_oscio_mode_t oscio_mode, uint8_t output_div);

/**
 * @brief Obtém a frequência atual do clock
 * @return Frequência atual em Hz
 */
uint32_t get_current_clkx_freq();

// =============================================
// Seção: Controle de LEDs
// =============================================

/**
 * @brief Ajusta a frequência base para os efeitos de LED
 * @param frequency_hz Frequência em Hertz
 */
bool sx1509b_set_led_frequency(uint8_t frequency_hz);


/**
 * @brief Configura um LED em modo estático (intensidade constante)
 * @param pin Número do pino (0-15)
 * @param intensity Intensidade do LED (0-255)
 */
void sx1509b_led_static(uint8_t pin, uint8_t intensity);

/**
 * @brief Configura um LED em modo blink (piscar)
 * @param pin Número do pino (0-15)
 * @param on_time_ms Tempo ligado em milissegundos
 * @param off_time_ms Tempo desligado em milissegundos
 * @param on_intensity Intensidade quando ligado (0-255)
 * @param off_intensity Intensidade quando desligado (0-255)
 */
void sx1509b_led_blinking(uint8_t pin, uint8_t intensity, uint16_t on_time_ms, uint16_t off_time_ms, uint16_t fade_in_ms, uint16_t fade_out_ms);

/**
 * @brief Configura um LED em modo breathing (fade in/out)
 * @param pin Número do pino (0-15)
 * @param rise_time_ms Tempo de fade in em milissegundos
 * @param fall_time_ms Tempo de fade out em milissegundos
 * @param max_intensity Intensidade máxima (0-255)
 * @param min_intensity Intensidade mínima (0-255)
 */
void sx1509b_led_breathing(uint8_t pin, uint16_t rise_time_ms, uint16_t fall_time_ms, uint8_t max_intensity, uint8_t min_intensity);

/**
 * @brief Habilita ou desabilita o driver de LED de um pino
 * @param pin Número do pino (0-15)
 * @param enable true para habilitar, false para desabilitar
 */
void sx1509b_led_enable(uint8_t pin, bool enable);

/**
 * @brief Configura o modo de fading (linear ou logarítmico) para os bancos de LED
 * @param bank_a_log true para logarítmico no banco A, false para linear
 * @param bank_b_log true para logarítmico no banco B, false para linear
 */
void sx1509b_led_set_fade_mode(bool bank_a_log, bool bank_b_log);

void sx1509b_led_fade(uint8_t pin, uint8_t min_intensity, uint8_t max_intensity,uint16_t fade_in_time_ms, uint16_t fade_out_time_ms,uint16_t on_time_ms, uint16_t off_time_ms);

// =============================================
// Seção: Configurações Miscelâneas
// =============================================

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
                             sx1509b_intclr_mode_t intclr_mode);

#endif // SX1509B_H
