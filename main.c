#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "opt4001.h"
#include "icm42670p.h"
#include "Register_ICM42670P.h"
#include "register_sx1509b.h"
#include "sx1509b.h"
#include "sx1509b_irq_manager.h"


// --- Definições de Hardware para o Sensor de luminosidade---
#define I2C_APP_BAUDRATE 400 * 1000 // 100 kHz
#define I2C_PORT_USED i2c0
#define SDA_PIN_USED 4
#define SCL_PIN_USED 5
#define SENSOR_I2C_ADDRESS 0x45
#define SENSOR_VARIANT OPT4001_VARIANT_SOT5X3

// Estruturas de dados para os sensores (simulados)
typedef struct {
    float x, y, z;
} GyroData;

typedef struct {
    float x, y, z;
} AccelData;

typedef struct {
    float lux;
} LuxData;

// Handles para filas e semáforos
QueueHandle_t QueueGyro, QueueAccel, QueueLux;
SemaphoreHandle_t SemReadGyro, SemReadAccel, SemReadLux, SemEnableUpdate;

// Protótipos das tasks
void vTaskReadGyro(void *pvParameters);
void vTaskReadAccel(void *pvParameters);
void vTaskReadLux(void *pvParameters);
void vTaskUpdateDisplay(void *pvParameters);

// Variaveis globais (Motion Sensor)
icm42670p sensor;

int main() {
    stdio_init_all();
    printf("Iniciando sistema com FreeRTOS (15 leituras + 10s pausa)...\n");

    // --- Inicializacao da biblioteca OPT4001 ---
    opt4001_init(I2C_PORT_USED, SDA_PIN_USED, SCL_PIN_USED, SENSOR_I2C_ADDRESS, SENSOR_VARIANT, I2C_APP_BAUDRATE);

    // Configura o sensor OPT4001 para operacao continua 
    opt4001_set_operating_mode(OPT4001_MODE_CONTINUOUS);
    opt4001_set_range(OPT4001_RANGE_AUTO);
    opt4001_set_conversion_time(OPT4001_CONV_TIME_400MS);

    // Inicializando Biblioteca ICM42670P
    // SPI initialisation. This example will use SPI at 1MHz.
    spi_init(SPI_PORT, 1000*1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    
    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    // For more examples of SPI use see https://github.com/raspberrypi/pico-examples/tree/master/spi


    sensor.spi_instance = SPI_PORT;
    sensor.cs_pin = PIN_CS;
    sensor.rate_accel = ICM42670_SCALE_ACCEL_2G;
    sensor.f_accel = ICM42670_FREQ_ACCEL_12_5HZ;
    sensor.rate_gyro = ICM42670_SCALE_GYRO_250DPS;
    sensor.f_gyro = ICM42670_FREQ_GYRO_12_5HZ;
    sensor.mode = ICM42670_MODE_6_AXIS_LN;

    icm42670p_init(&sensor);

    // Inicialização Expansor
    sx1509b_config_misc_bits(SX1509B_NRESET_FULL_RESET,SX1509B_AUTOINC_ENABLED,SX1509B_INTCLR_AUTO);
    sx1509b_set_led_frequency(3);
    sx1509b_config_clock(SX1509B_CLK_2MHZ,SX1509B_OSCIO_OUT,0);
    sx1509b_led_static(15,100);

    sx1509b_irq_init();

    void Update_Display(void)
    {
        xSemaphoreGive(SemReadGyro);
    }

    sx1509b_configure_pin(0,RISING);
    sx1509b_register_callback(0,Update_Display);
    
    // Cria as filas
    QueueGyro = xQueueCreate(1, sizeof(GyroData));
    QueueAccel = xQueueCreate(1, sizeof(AccelData));
    QueueLux = xQueueCreate(1, sizeof(LuxData));

    // Cria os semáforos binários
    SemReadGyro = xSemaphoreCreateBinary();
    SemReadAccel = xSemaphoreCreateBinary();
    SemReadLux = xSemaphoreCreateBinary();
    SemEnableUpdate = xSemaphoreCreateBinary();

    // Verifica se todos os recursos foram criados
    if (QueueGyro == NULL || QueueAccel == NULL || QueueLux == NULL ||
        SemReadGyro == NULL || SemReadAccel == NULL || 
        SemReadLux == NULL || SemEnableUpdate == NULL) {
        printf("Erro ao criar filas ou semáforos\n");
        while(1);
    }

    // Cria as tasks
    xTaskCreate(vTaskUpdateDisplay, "UpdateDisplay", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(vTaskReadGyro, "ReadGyro", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(vTaskReadAccel, "ReadAccel", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
    xTaskCreate(vTaskReadLux, "ReadLux", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

    // Inicia o escalonador
    vTaskStartScheduler();

    // Nunca deveria chegar aqui
    while(1);
    return 0;
}

// Task para atualizar o display
void vTaskUpdateDisplay(void *pvParameters) {
    GyroData gyroData;
    AccelData accelData;
    LuxData luxData;
    const TickType_t xDelay = pdMS_TO_TICKS(1000); // 1 segundo entre leituras
    const TickType_t xLongDelay = pdMS_TO_TICKS(10000); // 10 segundos de pausa
    int readingCount = 0;
    const int maxReadings = 15;
    
    while(1) {
        // Realiza 15 leituras seguidas
        if(readingCount < maxReadings) {
            // Libera a cadeia de tasks de sensores
            
            
            // Espera até que todas as leituras estejam completas
            if (xSemaphoreTake(SemEnableUpdate, portMAX_DELAY) == pdTRUE) {
                // Coleta os dados das filas
                if (xQueueReceive(QueueGyro, &gyroData, 0) == pdPASS &&
                    xQueueReceive(QueueAccel, &accelData, 0) == pdPASS &&
                    xQueueReceive(QueueLux, &luxData, 0) == pdPASS) {
                    
                    // Mostra os dados via USART (serial)
                    printf("=== Leitura %d/%d ===\n", readingCount+1, maxReadings);
                    printf("Giroscópio: X=%.2f, Y=%.2f, Z=%.2f\n", 
                           gyroData.x, gyroData.y, gyroData.z);
                    printf("Acelerômetro: X=%.2f, Y=%.2f, Z=%.2f\n", 
                           accelData.x, accelData.y, accelData.z);
                    printf("Luminosidade: %.2f lux\n", luxData.lux);
                    printf("=====================\n");
                    
                    readingCount++;
                } else {
                    printf("Erro ao ler dados das filas\n");
                }
            }
            
            // Aguarda 1 segundo antes da próxima leitura
            vTaskDelay(xDelay);
        } else {
            // Após 15 leituras, espera 10 segundos
            printf("Pausa de 10 segundos...\n");
            vTaskDelay(xLongDelay);
            readingCount = 0; // Reinicia o contador
            printf("Reiniciando ciclo de leituras...\n");
        }
    }
}

// Task para ler o giroscópio (mantida igual)
void vTaskReadGyro(void *pvParameters) {
    
    GyroData gyroData;
    
    while(1) {

        icm42670p_get_gyro(&sensor);

        if (xSemaphoreTake(SemReadGyro, portMAX_DELAY) == pdTRUE) {
            gyroData.x = sensor.gyro_x;
            gyroData.y = sensor.gyro_y;
            gyroData.z = sensor.gyro_z;
            
            xQueueOverwrite(QueueGyro, &gyroData);
            xSemaphoreGive(SemReadAccel);
        }
    }
}

// Task para ler o acelerômetro (mantida igual)
void vTaskReadAccel(void *pvParameters) {

    AccelData accelData;
    
    while(1) {
        
        icm42670p_get_accel(&sensor);

        if (xSemaphoreTake(SemReadAccel, portMAX_DELAY) == pdTRUE) {
            accelData.x = sensor.accel_x;
            accelData.y = sensor.accel_y;
            accelData.z = sensor.accel_z;
            
            xQueueOverwrite(QueueAccel, &accelData);
            xSemaphoreGive(SemReadLux);
        }
    }
}

// --- Task para ler o sensor de luminosidade (MODIFICADA) ---
void vTaskReadLux(void *pvParameters) {
    LuxData luxData;
    opt4001_data_t sensorData;
    
    while(1) {
        // Espera a permissao para ler o sensor
        if (xSemaphoreTake(SemReadLux, portMAX_DELAY) == pdTRUE) {
            // Espera a conversao do sensor ser completada (sincronizacao)
            if (opt4001_wait_for_conversion_complete(1000)) {
                // Le os dados do sensor
                if (opt4001_get_data(&sensorData)) {
                    luxData.lux = sensorData.lux;
                } else {
                    printf("ERRO: Falha na leitura do sensor de luminosidade.\n");
                    luxData.lux = -1.0f; // Sinaliza um erro
                }
            } else {
                printf("AVISO: Tempo limite excedido na espera por conversao do sensor de luz.\n");
                luxData.lux = -1.0f; // Sinaliza um erro
            }

            xQueueOverwrite(QueueLux, &luxData);
            xSemaphoreGive(SemEnableUpdate);
        }
    }
}
