/* ########################################################################################
##### THIS FILE GETS MODIFIED OR OVERWRITTEN BY THE preprocess.py SCRIPT BEFORE BUILD #####
######################################################################################## */
#ifndef __INIT_H
#define __INIT_H

#ifdef __cplusplus
extern "C" {
#endif


#include "stm32g0xx_hal.h"

#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define IRR0_Pin GPIO_PIN_4
#define IRR0_GPIO_Port GPIOB
#define IRR0_EXTI_IRQn EXTI4_15_IRQn

extern I2C_HandleTypeDef hi2c1;
#define ZST_HAL_I2C1_PRESENT
extern SPI_HandleTypeDef hspi1;
#define ZST_HAL_SPI1_PRESENT

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_SPI1_Init(void);
void MX_I2C1_Init(void);
void Error_Handler(void);

#ifdef HAL_TIM_MODULE_ENABLED
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
#endif


#ifdef __cplusplus
}
#endif

#endif
