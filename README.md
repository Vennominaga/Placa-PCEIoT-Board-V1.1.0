# Sistema de Leitura de Sensores com FreeRTOS

## Descrição Geral

Este projeto implementa um sistema embarcado baseado no Raspberry Pi Pico que realiza leituras simultâneas de múltiplos sensores usando o sistema operacional em tempo real FreeRTOS. O sistema coleta dados de giroscópio, acelerômetro e sensor de luminosidade, processando-os de forma sincronizada e exibindo os resultados via interface serial.

## Características Principais

O sistema possui leitura simultânea de múltiplos sensores usando tasks FreeRTOS, sincronização entre tasks através de semáforos e filas, ciclo de operação controlado com 15 leituras consecutivas seguidas de pausa de 10 segundos, interface com expansor de I/O para controle de LEDs e interrupções, e comunicação serial para exibição dos dados coletados.

## Sensores e Periféricos Integrados

O sensor de luminosidade OPT4001 utiliza interface I2C no endereço 0x45, configurado em modo contínuo com range automático e tempo de conversão de 400ms. O sensor de movimento ICM42670P utiliza interface SPI com funcionalidades de giroscópio e acelerômetro, configurado para 2G para acelerômetro, 250DPS para giroscópio e frequência de 12.5Hz. O expansor de I/O SX1509B é responsável pelo controle de LEDs, gerenciamento de interrupções e configuração de clock em 2MHz.

## Estrutura do Software

As tasks FreeRTOS incluem vTaskReadGyro para leitura do giroscópio que aguarda semáforo SemReadGyro para iniciar leitura, escreve dados na QueueGyro e libera semáforo SemReadAccel para próxima task. A vTaskReadAccel realiza leitura do acelerômetro aguardando semáforo SemReadAccel, escreve dados na QueueAccel e libera semáforo SemReadLux. A vTaskReadLux faz leitura do sensor de luminosidade aguardando semáforo SemReadLux, escreve dados na QueueLux e libera semáforo SemEnableUpdate. A vTaskUpdateDisplay atua como task coordenadora que gerencia o ciclo de leituras, controla a sequência de 15 leituras com 10s de pausa, coleta dados de todas as filas e exibe via serial, e inicia a cadeia de leituras liberando o primeiro semáforo.

Os mecanismos de sincronização utilizam quatro semáforos binários para coordenar a execução sequencial das tasks, três filas para armazenar dados dos sensores entre tasks, e callback de interrupção através do expansor SX1509B.

## Configuração de Hardware

Para os pinos I2C do sensor OPT4001 são utilizados SDA no GPIO 4, SCL no GPIO 5 com velocidade de 400 kHz. Os pinos SPI para o sensor ICM42670P utilizam MISO, MOSI, SCK e CS definidos externamente com velocidade de 1 MHz.

## Fluxo de Operação

A inicialização configura todos os sensores e cria recursos FreeRTOS. No ciclo de leituras, a task de display libera SemReadGyro, o giroscópio lê e escreve na fila liberando SemReadAccel, o acelerômetro lê e escreve na fila liberando SemReadLux, o sensor de luz lê e escreve na fila liberando SemEnableUpdate, e o display coleta dados de todas as filas e exibe. O controle temporal realiza 15 leituras com intervalo de 1 segundo seguido por 10 segundos de pausa, repetindo o ciclo.

## Compilação e Execução

Os pré-requisitos incluem SDK do Raspberry Pi Pico, FreeRTOS configurado para RP2040 e bibliotecas dos sensores (opt4001, icm42670p, sx1509b). A estrutura de arquivos do projeto contém main.c, opt4001.h/c, icm42670p.h/c, register_icm42670p.h, sx1509b.h/c, register_sx1509b.h e sx1509b_irq_manager.h/c. Para compilação utilize os comandos: mkdir build && cd build, cmake .., e make.

## Aplicações

As aplicações incluem sistemas de monitoramento ambiental, dispositivos de coleta de dados móveis, plataformas de prototipagem com múltiplos sensores e sistemas educacionais para ensino de RTOS.

## Notas de Desenvolvimento

As definições de pinos para SPI (PIN_MISO, PIN_MOSI, etc.) devem ser definidas no projeto. Podem ser necessários ajustes de prioridade de tasks para otimização de performance. 
