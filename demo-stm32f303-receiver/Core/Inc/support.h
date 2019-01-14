//
// Created by ilia.motornyi on 13-Dec-18.
//

#ifndef __SUPPORT_H
#define __SUPPORT_H

#include "main.h"


#ifdef USE_HAL_DRIVER
extern SPI_HandleTypeDef hspi3;


static inline void nRF24_CE_L() {
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
}

static inline void nRF24_CE_H() {
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
}

static inline void nRF24_CSN_L() {
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
}

static inline void nRF24_CSN_H() {
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}


static inline uint8_t nRF24_LL_RW(uint8_t data) {
    // Wait until TX buffer is empty
    uint8_t result;
    if(HAL_SPI_TransmitReceive(&hspi3,&data,&result,1,2000)!=HAL_OK) {
        Error_Handler();
    };
    return result;
}


static inline void Delay_ms(uint32_t ms) { HAL_Delay(ms); }

#elif USE_FULL_LL_DRIVER
#define NRF_SPI SPI3

static inline void nRF24_CE_L() {
    LL_GPIO_ResetOutputPin(NRF_CE_GPIO_Port, NRF_CE_Pin);
}

static inline void nRF24_CE_H() {
    LL_GPIO_SetOutputPin(NRF_CE_GPIO_Port, NRF_CE_Pin);
}

static inline void nRF24_CSN_L() {
    LL_GPIO_ResetOutputPin(NRF_CSN_GPIO_Port, NRF_CSN_Pin);
}

static inline void nRF24_CSN_H() {
    LL_GPIO_SetOutputPin(NRF_CSN_GPIO_Port, NRF_CSN_Pin);
}


static inline uint8_t nRF24_LL_RW(uint8_t data) {
    LL_SPI_SetRxFIFOThreshold(NRF_SPI,LL_SPI_RX_FIFO_TH_QUARTER);
    LL_SPI_Enable(NRF_SPI);
    // Wait until TX buffer is empty
    while (LL_SPI_IsActiveFlag_BSY(NRF_SPI));
    while (!LL_SPI_IsActiveFlag_TXE(NRF_SPI));
    LL_SPI_TransmitData8(NRF_SPI, data);
    while (!LL_SPI_IsActiveFlag_RXNE(NRF_SPI));
    return LL_SPI_ReceiveData8(NRF_SPI);
}


static inline void Delay_ms(uint32_t ms) { LL_mDelay(ms); }

#else // nor USE_FULL_LL_DRIVER, nor USE_HAL_DRIVER

#error LL or HAL support must be enabled

#endif // USE_FULL_LL_DRIVER

#endif //__SUPPORT_H
