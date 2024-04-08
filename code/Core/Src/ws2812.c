/**
  ******************************************************************************
  * File Name          : encoder.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "ws2812.h"
#include <stdlib.h>
#include <string.h>

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
volatile uint16_t neopixel_pin;
uint8_t led_num;
uint32_t color_buf[6][TOTAL_RGB] = {0};
void color_set_single(uint32_t color);
// uint8_t rled[TOTAL_RGB] = {0};
// uint8_t gled[TOTAL_RGB] = {0};
// uint8_t bled[TOTAL_RGB] = {0};

void restart(void) {
  for (uint8_t i = 0; i < 113; i++) {
    delay_600ns();
  }
}

void GPIO_init(void) {
	__HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void sk6812_init(uint16_t num) {
  // color_buf = (uint32_t *)calloc(num, sizeof(uint32_t));
  led_num = num;
}

void neopixel_clear_color(void) {
  memset(color_buf, 0, sizeof(uint32_t)*TOTAL_RGB);
}

static float min(float a, float b, float c)
{
  float m;
  m = a < b ? a : b;
  return (m < c ? m : c);
}

static float max(float a, float b, float c)
{
  float m;
  m = a > b ? a : b;
  return (m > c ? m : c);
}

void rgb2hsv(uint8_t g, uint8_t r, uint8_t b, float *h, float *s, float *v)
{
  float red, green ,blue;
  float cmax, cmin, delta;

  red =   (float)r / 255;
  green = (float)g / 255;
  blue =  (float)b / 255;

  cmax = max(red, green, blue);
  cmin = min(red, green, blue);
  delta = cmax - cmin;

  /* H */
  if(delta == 0)
  {
    *h = 0;
  }
  else
  {
    if(cmax == red)
    {
      if(green >= blue)
      {
        *h = 60 * ((green - blue) / delta);
      }
      else
      {
        *h = 60 * ((green - blue) / delta) + 360;
      }
    }
    else if(cmax == green)
    {
      *h = 60 * ((blue - red) / delta + 2);
    }
    else if(cmax == blue)
    {
      *h = 60 * ((red - green) / delta + 4);
    }
  }

  /* S */
  if(cmax == 0)
  {
    *s = 0;
  }
  else
  {
    *s = delta / cmax;
  }

  /* V */
  *v = cmax;
}

void neopixel_set_color(uint16_t num, uint32_t color, uint8_t scale, uint8_t ch) {
  uint8_t rled, gled, bled;

  if (scale == 0) {
    rled = 0;
    gled = 0;
    bled = 0;
  } else if (scale == 255) {
    rled = (color >> 24) & 0xff;
    gled = (color >> 16) & 0xff;
    bled = (color >> 8)  & 0xff;
  } else {
    rled = ((color >> 24) & 0xff) / (255/scale);
    gled = ((color >> 16) & 0xff) / (255/scale);
    bled = ((color >> 8)  & 0xff) / (255/scale);
  }

	color_buf[ch][num] = gled << 16 | rled << 8 | bled;
}

//void neopixel_set_single_color(uint8_t num, uint8_t color, uint8_t shiftnum) {
//	if(shiftnum == 0)
//		rled[num] = color;
//	else if(shiftnum == 1)
//		gled[num] = color;
//	else if(shiftnum == 2)
//		bled[num] = color;
//	color_buf[num] = gled[num] << 16 | rled[num] << 8 | bled[num];
//}

//void neopixel_set_all_color(uint32_t *color) {
//  for (uint8_t i = 0; i < led_num; i++) {
//		uint8_t rled = (color[i] >> 24) & 0xff;
//		uint8_t gled = (color[i] >> 16) & 0xff;
//		uint8_t bled = (color[i] >> 8)  & 0xff;
//		color_buf[i] = gled << 16 | rled << 8 | bled;
//  }
//}

void neopixel_show(uint16_t index, uint8_t ch) {
  __disable_irq();
  color_set_single(color_buf[ch][index]);
  __enable_irq();
  // restart();
}

void neopixel_show_portb(uint16_t index, uint8_t ch) {
  __disable_irq();
  color_set_single_portb(color_buf[ch][index]);
  __enable_irq();
  // restart();
}

void neopixel_show_portf(uint16_t index, uint8_t ch) {
  __disable_irq();
  color_set_single_portf(color_buf[ch][index]);
  __enable_irq();
  // restart();
}

void color_set_single(uint32_t color) {
  for (uint8_t i = 0; i < 24; i++) {
    if (color & (1 << (23 - i))) {
      out_bit_high();
    }
    else {
      out_bit_low();
    }
  }
}

void color_set_single_portb(uint32_t color) {
  for (uint8_t i = 0; i < 24; i++) {
    if (color & (1 << (23 - i))) {
      out_bit_high_b();
    }
    else {
      out_bit_low_b();
    }
  }
}

void color_set_single_portf(uint32_t color) {
  for (uint8_t i = 0; i < 24; i++) {
    if (color & (1 << (23 - i))) {
      out_bit_high_f();
    }
    else {
      out_bit_low_f();
    }
  }
}
/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
