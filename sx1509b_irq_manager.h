#ifndef SX1509B_IRQ_MANAGER_H
#define SX1509B_IRQ_MANAGER_H




#include "pico/stdlib.h"
#include "register_sx1509b.h"
#include "hardware/gpio.h"

#ifndef SX1509B_INTERRUPT_PIN
#define SX1509B_INTERRUPT_PIN 11  // Valor padrão
#endif

#define MAX_SX1509B_PINS 16

typedef enum 
{
    NONE    = 0b00,     // Nenhuma borda  
    RISING  = 0b01,    // Subida  
    FALLING = 0b10,    // Descida  
    BOTH    = 0b11     // Subida e descida  
}EdgeTrigger;


typedef void (*sx1509b_callback_t)(void);

// Inicialização
void sx1509b_irq_init(void);

// Gerenciamento de callbacks
bool sx1509b_register_callback(uint8_t pin, sx1509b_callback_t callback);
bool sx1509b_unregister_callback(uint8_t pin);

// Configuração de pinos
bool sx1509b_configure_pin(uint8_t pin, uint8_t sense_config);

// Função para processar interrupção (chamar manualmente se não usar IRQ)
void sx1509b_process_interrupt();

#endif // SX1509B_IRQ_MANAGER_H