/**
  ******************************************************************************
  * File Name          : gpio.h
  * Description        : This file contains all the functions prototypes for 
  *                      the gpio  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ws2812_H__
#define __ws2812_H__
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */
#define TOTAL_RGB 74
extern volatile uint16_t neopixel_pin;
/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define gpio_low() GPIOA->BRR = neopixel_pin
#define gpio_high() GPIOA->BSRR = neopixel_pin
#define gpio_low_b() GPIOB->BRR = neopixel_pin
#define gpio_high_b() GPIOB->BSRR = neopixel_pin
#define gpio_low_f() GPIOF->BRR = neopixel_pin
#define gpio_high_f() GPIOF->BSRR = neopixel_pin
#define delay_150ns() __asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP")
#define delay_300ns() delay_150ns(); __asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP")
#define delay_600ns() delay_300ns(); delay_300ns()
#define delay_900ns() delay_600ns(); delay_300ns()

#define out_bit_low() \
  gpio_high(); \
  delay_300ns(); \
  gpio_low(); \
  delay_600ns();

#define out_bit_high() \
  gpio_high(); \
  delay_900ns(); \
  gpio_low(); \
  delay_150ns();

#define out_bit_low_b() \
  gpio_high_b(); \
  delay_300ns(); \
  gpio_low_b(); \
  delay_600ns();

#define out_bit_high_b() \
  gpio_high_b(); \
  delay_900ns(); \
  gpio_low_b(); \
  delay_150ns();

#define out_bit_low_f() \
  gpio_high_f(); \
  delay_300ns(); \
  gpio_low_f(); \
  delay_600ns();

#define out_bit_high_f() \
  gpio_high_f(); \
  delay_900ns(); \
  gpio_low_f(); \
  delay_150ns();

/* USER CODE END Private defines */
void restart(void);
void GPIO_init(void);
void sk6812_init(uint16_t num);
void neopixel_clear_color(void);
void neopixel_set_color(uint16_t num, uint32_t color, uint8_t scale, uint8_t ch);
void neopixel_set_all_color(uint32_t *color);
void neopixel_show(uint16_t index, uint8_t ch);
void neopixel_show_portb(uint16_t index, uint8_t ch);
void neopixel_show_portf(uint16_t index, uint8_t ch);
void color_set_single(uint32_t color);
void color_set_single_portb(uint32_t color);
void color_set_single_portf(uint32_t color);
void neopixel_set_single_color(uint8_t num, uint8_t color, uint8_t shiftnum);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
