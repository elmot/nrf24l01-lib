# nrf24l01-stm32F303
Demo project for NRF24L01 + STM32 NUCLEO-F303RE board.

Based on https://github.com/LonelyWolf/stm32/tree/master/nrf24l01

Improvements
--
 
 - Rewritten to be hardware-independent
 - STM32 HAL support added
 - Run on ST NUCLEO-F303RE board
 
 Key files
 ---
 
 - [nrf24l01/nrf24.h](nrf24l01/nrf24.h) - main header file
 - [nrf24l01/nrf24.c](nrf24l01/nrf24.c) - nrf module support code
 - [demo-stm32f303-receiver/Core/Inc/support.h](demo-stm32f303-receiver/Core/Inc/support.h) - all hardware-dependent code (nucleo-f303re)
 - [demo-stm32f303-receiver/Core/Src/radio_demo.c](demo-stm32f303-receiver/Core/Src/radio_demo.c) - demo code(receiver)
 
 [NUCLEO-303RE](demo-stm32f303-receiver/demo-stm32f303-receiver.pdf) Pinout
--
| MCU Pin | Function | NRF Pin |
|-----|----------|--------|
|PC0|GPIO_Output|CE|
|PC1|GPIO_Output|CSN|
|PC2|GPIO_EXTI2|IRQ|
|PC10|	SPI3_SCK	|SCK|
|PC11|	SPI3_MISO	|MISO|
|PC12|	SPI3_MOSI	|MOSI|

[NUCLEO32-432KC](demo-stm32l432-transmitter/demo-stm32l432-transmitter.pdf) Pinout
--
| MCU Pin | Function | NRF Pin |
|-----|----------|--------|
|PA3|GPIO_Output|CE|
|PA4|GPIO_Output|CSN|
|PB0|GPIO_EXTI2|IRQ|
|PA5|	SPI3_SCK	|SCK|
|PA6|	SPI3_MISO	|MISO|
|PA7|	SPI3_MOSI	|MOSI|
