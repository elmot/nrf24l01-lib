//
// Created by ilia.motornyi on 13-Dec-18.
//

#ifndef __SUPPORT_H
#define __SUPPORT_H

#include "main.h"

extern SPI_HandleTypeDef hspi1;


static inline void nRF24_CE_L() {
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_RESET);
}

static inline void nRF24_CE_H() {
    HAL_GPIO_WritePin(CE_GPIO_Port, CE_Pin, GPIO_PIN_SET);
}

static inline void nRF24_CSN_L() {
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_RESET);
}

static inline void nRF24_CSN_H() {
    HAL_GPIO_WritePin(CSN_GPIO_Port, CSN_Pin, GPIO_PIN_SET);
}


static inline uint8_t nRF24_LL_RW(uint8_t data) {
    // Wait until TX buffer is empty
    uint8_t result;
    if(HAL_SPI_TransmitReceive(&hspi1,&data,&result,1,2000)!=HAL_OK) {
        Error_Handler();
    };
    return result;
}


static inline void Delay_ms(uint32_t ms) { HAL_Delay(ms); }

#endif //__SUPPORT_H
