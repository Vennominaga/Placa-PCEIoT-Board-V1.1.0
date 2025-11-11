#include "sx1509b_irq_manager.h"
#include <stdio.h>



sx1509b_callback_t callbacks[MAX_SX1509B_PINS];


static void handle_interrupt() 
{
    uint8_t int_source_a = sx1509b_read_reg(REG_INTERRUPT_SOURCE_A);
    uint8_t int_source_b = sx1509b_read_reg(REG_INTERRUPT_SOURCE_B);
    
    for (uint8_t pin = 0; pin < 8; pin++) 
    {
        if (int_source_a & (1 << pin) && callbacks[pin]) 
        {
           callbacks[pin]();
        }
        if (int_source_b & (1 << pin) &&callbacks[pin + 8]) 
        {
            callbacks[pin + 8]();
        }
    }
    
    

    //Limpeza Manual
    if(sx1509b_read_reg(REG_MISC) && 1)
    {
        sx1509b_write_reg(REG_INTERRUPT_SOURCE_A,0xFF);
        sx1509b_write_reg(REG_INTERRUPT_SOURCE_B,0xFF);
    }

    //Limpeza Automatica
    else
    {
        sx1509b_read_reg(REG_DATA_A);
        sx1509b_read_reg(REG_DATA_B);
    }


   
}

static void gpio_irq_handler(uint gpio, uint32_t events) 
{
    if (gpio == SX1509B_INTERRUPT_PIN) 
    {
        sx1509b_process_interrupt();
    }
}

void sx1509b_irq_init()
{
      uint8_t nint_pin = SX1509B_INTERRUPT_PIN;
    
    // Inicializa callbacks
    for (int i = 0; i < MAX_SX1509B_PINS; i++) 
    {
        callbacks[i] = NULL;
    }
    
    // Configura pino de interrupção
    gpio_init(nint_pin);
    gpio_set_dir(nint_pin, GPIO_IN);
    gpio_pull_up(nint_pin);
    gpio_set_irq_enabled_with_callback(nint_pin,GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);
    
    // Habilita interrupções em todos os pinos
    sx1509b_write_reg(REG_INTERRUPT_MASK_A, 0x00);
    sx1509b_write_reg(REG_INTERRUPT_MASK_B, 0x00);
}

bool sx1509b_register_callback(uint8_t pin, sx1509b_callback_t callback) 
{
    if (pin >= MAX_SX1509B_PINS || callback == NULL) 
    {
        return false;
    }


    if(pin < 8)
    {
        callbacks[pin] = callback;

        uint8_t reg = sx1509b_read_reg(REG_INTERRUPT_MASK_A);
        uint8_t mask = 1<< pin;
        
        uint8_t current = reg & ~mask;
        sx1509b_write_reg(REG_INTERRUPT_MASK_A,current);
    }
    else
    {
        callbacks[pin] = callback;

        uint8_t reg = sx1509b_read_reg(REG_INTERRUPT_MASK_B);
        pin-=8;
        uint8_t mask = 1<< pin;

         uint8_t current = reg & ~mask;
        sx1509b_write_reg(REG_INTERRUPT_MASK_B,current);

    }
    
    
    return true;
}

bool sx1509b_unregister_callback(uint8_t pin) 
{
    if (pin >= MAX_SX1509B_PINS) 
    {
        return false;
    }
    
    callbacks[pin] = NULL;
    return true;
}

bool sx1509b_configure_pin(uint8_t pin, uint8_t sense_config) 
{
    if (pin >= MAX_SX1509B_PINS) 
    {
        return false;
    }
    
    uint8_t reg;
    uint8_t bit_pos;
    
    if (pin < 4) 
    {
        reg = REG_SENSE_LOW_A;
        bit_pos = pin * 2;
    } 
    else if (pin < 8) 
    {
        reg = REG_SENSE_HIGH_A;
        bit_pos = (pin - 4) * 2;
    } 
    else if (pin < 12) 
    {
        reg = REG_SENSE_LOW_B;
        bit_pos = (pin - 8) * 2;
    } 
    else 
    {
        reg = REG_SENSE_HIGH_B;
        bit_pos = (pin - 12) * 2;
    }
    
    uint8_t current = sx1509b_read_reg(reg);
    current &= ~(0x03 << bit_pos);  // Limpa bits
    current |= (sense_config & 0x03) << bit_pos;  // Seta novos bits
    sx1509b_write_reg(reg, current);
    
    return true;
}

void sx1509b_process_interrupt() 
{
    handle_interrupt();
}