/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "flash.h"
#include "ws2812.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FIRMWARE_VERSION 2
#define I2C_ADDRESS 0x61
#define CH_NUMBER 6
#define ANGLE_TO_PULSE(angle) ((uint16_t)(500 + (angle * 2000 / 180)))//Angle convert to pulse
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t fm_version = FIRMWARE_VERSION;
uint8_t i2c_address[1] = {0};
volatile uint32_t i2c_stop_timeout_delay = 0;

#define APPLICATION_ADDRESS     ((uint32_t)0x08001000)
#define BOOTLOADER_VER_ADDR ((uint32_t)0x08001000 - 1)
uint32_t bootloader_version;

volatile uint16_t rgb_led_nums[CH_NUMBER] = {0};
uint8_t rgb_led_brightness[CH_NUMBER] = {255, 255, 255, 255, 255, 255};
uint8_t rgb_one_led_color[CH_NUMBER][5];
uint8_t rgb_multi_led_color[CH_NUMBER][7];

uint8_t pwm_duty[CH_NUMBER][2] = {0};
volatile uint16_t pwm_pulse[CH_NUMBER][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
uint8_t pwm_disable[CH_NUMBER][2] = {{1, 1}, {1, 1}, {1, 1}, {1, 1}, {1, 1}, {1, 1}};
uint8_t servo_angle[CH_NUMBER][2] = {0};
volatile uint16_t servo_pulse[CH_NUMBER][2] = {{0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}};
uint8_t servo_disable[CH_NUMBER][2] = {{1, 1}, {1, 1}, {1, 1}, {1, 1}, {1, 1}, {1, 1}};
static char Switch[CH_NUMBER][2] = {0};

uint8_t gpio_digi_read_state = 0;
uint8_t gpio_digi_write_state[CH_NUMBER][2] = {0};
// 0 -> portA; 1 -> portB; 2 -> portC
uint8_t gpio_ch_to_port[CH_NUMBER][2] = {{0, 0}, {0, 0}, {0, 1}, {0, 2}, {0, 2}, {0, 0}};
uint16_t gpio_pinmap[CH_NUMBER][2] = {{GPIO_PIN_0, OUT0_Pin}, {IN1_Pin, OUT1_Pin}, {IN2_Pin, OUT2_Pin}, 
                             {IN3_Pin, OUT3_Pin}, {IN4_Pin, OUT4_Pin}, {IN5_Pin, OUT5_Pin}};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void IAP_Set()
{
	uint8_t i;
 
	uint32_t *pVecTab=(uint32_t *)(0x20000000);
	//????????SRAM???
	for(i = 0; i < 48; i++)
	{
		*(pVecTab++) = *(__IO uint32_t*)(APPLICATION_ADDRESS + (i<<2));
	}
  /* Enable the SYSCFG peripheral clock*/
#if 1 //STM32
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  //??? SRAM ??? 0x00000000
  __HAL_SYSCFG_REMAPMEMORY_SRAM();
#else //AMP32
    RCM_EnableAPB2PeriphClock(RCM_APB2_PERIPH_SYSCFG);
    /* Remap SRAM at 0x00000000 */
    SYSCFG->CFG1_B.MMSEL = SYSCFG_MemoryRemap_SRAM;
#endif
}

void i2c_address_write_to_flash(void) 
{   
  writeMessageToFlash(i2c_address , 1);   
}

void i2c_address_read_from_flash(void) 
{   
  if (!(readPackedMessageFromFlash(i2c_address, 1))) {
    i2c_address[0] = I2C_ADDRESS;
    i2c_address_write_to_flash();
  }
}

void switchInputAndOutput(uint8_t port, uint16_t pin, uint32_t type, uint32_t pull_type, uint32_t freq_type)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (type == GPIO_MODE_ANALOG)
      __HAL_RCC_ADC1_CLK_ENABLE();    
    /*Configure GPIO pins : PAPin PAPin PAPin PAPin
                            PAPin PAPin PAPin PAPin */
    GPIO_InitStruct.Pin = pin;
    GPIO_InitStruct.Mode = type;
    GPIO_InitStruct.Pull = pull_type;
    GPIO_InitStruct.Speed = freq_type;
    if (port == 0)
      HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    else if (port == 1)
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    else if (port == 2)
      HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);      

    if (type == GPIO_MODE_ANALOG) {
      hadc.Instance = ADC1;
      hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
      hadc.Init.Resolution = ADC_RESOLUTION_12B;
      hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
      hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
      hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
      hadc.Init.LowPowerAutoWait = DISABLE;
      hadc.Init.LowPowerAutoPowerOff = DISABLE;
      hadc.Init.ContinuousConvMode = DISABLE;
      hadc.Init.DiscontinuousConvMode = DISABLE;
      hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
      hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
      hadc.Init.DMAContinuousRequests = DISABLE;
      hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
      HAL_ADC_Init(&hadc);      
    }    
}

void gpio_write_level(uint8_t port, uint16_t pin, GPIO_PinState PinState)
{
    if (port == 0)
      HAL_GPIO_WritePin(GPIOA, pin, PinState);
    else if (port == 1)
      HAL_GPIO_WritePin(GPIOB, pin, PinState);
    else if (port == 2)
      HAL_GPIO_WritePin(GPIOF, pin, PinState);         
}

void gpio_write_level_low(uint8_t port, uint16_t pin)
{
    if (port == 0)
      GPIOA->BRR = pin;
    else if (port == 1)
      GPIOB->BRR = pin;
    else if (port == 2)
      GPIOF->BRR = pin;
}

void gpio_write_level_high(uint8_t port, uint16_t pin)
{
    if (port == 0)
      GPIOA->BSRR = pin;
    else if (port == 1)
      GPIOB->BSRR = pin;
    else if (port == 2)
      GPIOF->BSRR = pin;
}

uint8_t gpio_read_level(uint8_t port, uint16_t pin)
{
    if (port == 0)
      return HAL_GPIO_ReadPin(GPIOA, pin);
    else if (port == 1)
      return HAL_GPIO_ReadPin(GPIOB, pin);
    else if (port == 2)
      return HAL_GPIO_ReadPin(GPIOF, pin);
    return 202;
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void readADCState(uint32_t channel, uint8_t bit_mode)
{
    uint32_t AD_Value = 0;
    uint32_t Value[22] = {0};
    uint8_t tmp_buff[16] = {0};
    uint16_t tmp_buff_len = 0;
    ADC_ChannelConfTypeDef sConfig = {0};
    hadc.Instance->CHSELR = 0;
    uint32_t max = 0;
    uint32_t min = 0;

    sConfig.Channel = channel;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);

    HAL_ADCEx_Calibration_Start(&hadc);

    for(int n=0;n<22;n++)
    {
        HAL_ADC_Start(&hadc);
        HAL_ADC_PollForConversion(&hadc, 10);
        if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc), HAL_ADC_STATE_REG_EOC))
        {
        	Value[n] = HAL_ADC_GetValue(&hadc);
          AD_Value += Value[n];
        }
    }
    max=Value[0];
    min=Value[0];

    for(uint8_t n=0;n<22;n++)//取最大�?��?�最小�??
    {
        max=(Value[n]<max)?max:Value[n];    
        min=(min<Value[n])?min:Value[n];
    }     

    AD_Value = (AD_Value - max - min) / 20;
    
    if (bit_mode == 12) {
      tmp_buff[0] = (uint8_t)AD_Value;
      tmp_buff[1] = (uint8_t)(AD_Value>>8);
      tmp_buff_len = 2;
    } else if (bit_mode == 8) {
      tmp_buff[0] = map(AD_Value,0,4095,0,255);
      tmp_buff_len = 1;
    }
    i2c1_set_send_data(tmp_buff, tmp_buff_len);
}

void SetPWMPulse(uint8_t ch, uint8_t pos, uint16_t pulse)
{
  pwm_pulse[ch][pos] = pulse*10;  
}

void set_single_rgb_led(uint16_t index, uint8_t ch)
{
  if (index < rgb_led_nums[ch]) {
    neopixel_set_color(index, (rgb_one_led_color[ch][2] << 24) | (rgb_one_led_color[ch][3] << 16) | (rgb_one_led_color[ch][4] << 8), rgb_led_brightness[ch], ch);
    for (int i = 0; i < index+1; i++) {
      switch (gpio_ch_to_port[ch][1])
      {
      case 0:
        neopixel_show(i, ch);
        break;
      case 1:
        neopixel_show_portb(i, ch);
        break;
      case 2:
        neopixel_show_portf(i, ch);
        break;
      default:
        break;
      }
    }
    restart();    
  }
}

void set_multi_rgb_led(uint16_t index, uint16_t end, uint8_t ch)
{
  if (index < rgb_led_nums[ch]) {
    if (end < rgb_led_nums[ch]) {
      for (int i = index; i < end+index; i++) {
        neopixel_set_color(i, (rgb_multi_led_color[ch][4] << 24) | (rgb_multi_led_color[ch][5] << 16) | (rgb_multi_led_color[ch][6] << 8), rgb_led_brightness[ch], ch);
      }
      for (int i = 0; i < end+index; i++) {
        switch (gpio_ch_to_port[ch][1])
        {
        case 0:
          neopixel_show(i, ch);
          break;
        case 1:
          neopixel_show_portb(i, ch);
          break;
        case 2:
          neopixel_show_portf(i, ch);
          break;
        default:
          break;
        }
      }              
      restart();              
    } else {
      for (int i = index; i < rgb_led_nums[ch]; i++) {
        neopixel_set_color(i, (rgb_multi_led_color[ch][4] << 24) | (rgb_multi_led_color[ch][5] << 16) | (rgb_multi_led_color[ch][6] << 8), rgb_led_brightness[ch], ch);
      }
      for (int i = 0; i < rgb_led_nums[ch]; i++) {
        switch (gpio_ch_to_port[ch][1])
        {
        case 0:
          neopixel_show(i, ch);
          break;
        case 1:
          neopixel_show_portb(i, ch);
          break;
        case 2:
          neopixel_show_portf(i, ch);
          break;
        default:
          break;
        }
      }      
      restart();      
    }
  }
}

void Slave_Complete_Callback(uint8_t *rx_data, uint16_t len) 
{
  uint8_t cmd = rx_data[0] & 0x0f;
  uint8_t ch = rx_data[0] & 0xf0;
  uint8_t buf[16];
  uint16_t index;
  uint16_t end;

  switch (ch)
  {
  case 0x40:
    ch = 0;
    break;
  case 0x50:
    ch = 1;
    break;    
  case 0x60:
    ch = 2;
    break;
  case 0x70:
    ch = 3;
    break;
  case 0x80:
    ch = 4;
    break;
  case 0xA0:
    ch = 5;
    break;              
  default:
    break;
  }  

  if (len > 1) {
    if (ch <= 5) {
      if (cmd <= 1) {
        pwm_disable[ch][cmd] = 1;
        servo_disable[ch][cmd] = 1;
        pwm_pulse[ch][cmd] = 0;
        servo_pulse[ch][cmd] = 0;
        switchInputAndOutput(gpio_ch_to_port[ch][cmd], gpio_pinmap[ch][cmd], GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
        gpio_digi_write_state[ch][cmd] = rx_data[1];
        switch (gpio_digi_write_state[ch][cmd] & 0x01)
        {
        case 0:
          gpio_write_level(gpio_ch_to_port[ch][cmd], gpio_pinmap[ch][cmd], GPIO_PIN_RESET);
          break;
        case 1:
          gpio_write_level(gpio_ch_to_port[ch][cmd], gpio_pinmap[ch][cmd], GPIO_PIN_SET);
          break;      
        default:
          break;
        }
      } 
      else if (cmd >= 2 && cmd <= 3) {
        servo_disable[ch][cmd-2] = 1;
        servo_pulse[ch][cmd-2] = 0;        
        pwm_disable[ch][cmd-2] = 0;
        switchInputAndOutput(gpio_ch_to_port[ch][cmd-2], gpio_pinmap[ch][cmd-2], GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
        pwm_duty[ch][cmd-2] = rx_data[1]; 
        if (pwm_duty[ch][cmd-2] == 0)
          gpio_write_level_low(gpio_ch_to_port[ch][cmd-2], gpio_pinmap[ch][cmd-2]);
        SetPWMPulse(ch, cmd-2, pwm_duty[ch][cmd-2]);
      }     
      else if (cmd >= 0x0C && cmd <= 0x0D) {
        pwm_disable[ch][cmd-0x0C] = 1;
        pwm_pulse[ch][cmd-0x0C] = 0;
        servo_disable[ch][cmd-0x0C] = 0;
        switchInputAndOutput(gpio_ch_to_port[ch][cmd-0x0C], gpio_pinmap[ch][cmd-0x0C], GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
        if (rx_data[1] > 180)
          return;
        servo_angle[ch][cmd-0x0C] = rx_data[1]; 
        servo_pulse[ch][cmd-0x0C] = ANGLE_TO_PULSE(rx_data[1]);
      }     
      else if (cmd >= 0x0E && cmd <= 0x0F) {
        if (len == 3) {
          pwm_disable[ch][cmd-0x0E] = 1;
          pwm_pulse[ch][cmd-0x0E] = 0;
          servo_disable[ch][cmd-0x0E] = 0;
          switchInputAndOutput(gpio_ch_to_port[ch][cmd-0x0E], gpio_pinmap[ch][cmd-0x0E], GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH);
          uint16_t p = rx_data[1] | (rx_data[2] << 8);
          if (p < 500 || p > 2500)
            return;
          servo_pulse[ch][cmd-0x0E] = p;
        }
      }     
      else if (cmd == 8) {
        if (len == 3) {
          uint16_t n = rx_data[1] | (rx_data[2] << 8);
          if (n <= TOTAL_RGB)
            rgb_led_nums[ch] = n;
          else
            rgb_led_nums[ch] = TOTAL_RGB;
        }
      }      
      else if (cmd == 9) {
        if (len == 6) {
          pwm_disable[ch][1] = 1;
          servo_disable[ch][1] = 1;
          pwm_pulse[ch][1] = 0;
          servo_pulse[ch][1] = 0;          
          switchInputAndOutput(gpio_ch_to_port[ch][1], gpio_pinmap[ch][1], GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM);
          for (int i = 0; i < 5; i++) {
            rgb_one_led_color[ch][i] = rx_data[1+i];
          }
          index = rgb_one_led_color[ch][0] | (rgb_one_led_color[ch][1] << 8);
          neopixel_pin = gpio_pinmap[ch][1];
          set_single_rgb_led(index, ch);
        }
      }
      else if (cmd == 0x0A) {
        if (len == 8) {
          pwm_disable[ch][1] = 1;
          servo_disable[ch][1] = 1;
          pwm_pulse[ch][1] = 0;
          servo_pulse[ch][1] = 0;           
          switchInputAndOutput(gpio_ch_to_port[ch][1], gpio_pinmap[ch][1], GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_MEDIUM);
          for (int i = 0; i < 7; i++) {
            rgb_multi_led_color[ch][i] = rx_data[1+i];
          }
          index = rgb_multi_led_color[ch][0] | (rgb_multi_led_color[ch][1] << 8);
          end = rgb_multi_led_color[ch][2] | (rgb_multi_led_color[ch][3] << 8);
          neopixel_pin = gpio_pinmap[ch][1];
          set_multi_rgb_led(index, end, ch);
        }
      }    
      else if (cmd == 0x0B) {
        rgb_led_brightness[ch] = rx_data[1];
      } 
    } else {
      if (rx_data[0] == 0xff) {
        if (len == 2) {
          if (rx_data[1] < 128) {
            i2c_address[0] = rx_data[1];
            i2c_address_write_to_flash();
            user_i2c_init();
          }
        }       
      }
      else if (rx_data[0] == 0xFD)
      {
        if (rx_data[1] == 1) {
          LL_I2C_DeInit(I2C1);
          LL_I2C_DisableAutoEndMode(I2C1);
          LL_I2C_Disable(I2C1);
          LL_I2C_DisableIT_ADDR(I2C1);
          HAL_ADC_DeInit(&hadc);
          HAL_TIM_Base_Stop_IT(&htim16);
          HAL_TIM_Base_Stop_IT(&htim17);
          HAL_TIM_Base_Stop(&htim16);
          HAL_TIM_Base_Stop(&htim17);
          HAL_TIM_Base_DeInit(&htim16); 
          HAL_TIM_Base_DeInit(&htim17);  
          NVIC_SystemReset();                 
        }
      }       
    }   
  } else if (len == 1) {
    if (ch <= 5) {
      if (cmd <= 1) {
        i2c1_set_send_data((uint8_t *)&gpio_digi_write_state[ch][cmd], 1);
      }
      else if (cmd >= 2 && cmd <= 3) {
        i2c1_set_send_data((uint8_t *)&pwm_duty[ch][cmd-2], 1);
      }    
      else if (cmd >= 0x0C && cmd <= 0x0D) {
        i2c1_set_send_data((uint8_t *)&servo_angle[ch][cmd-0x0C], 1);
      }     
      else if (cmd >= 0x0E && cmd <= 0x0F) {
        buf[0] = servo_pulse[ch][cmd-0x0E] &0xff;
        buf[1] = (servo_pulse[ch][cmd-0x0E] >> 8) & 0xff;
        i2c1_set_send_data(buf, 2);
      }     
      else if (cmd >= 4 && cmd <= 5) {
        pwm_disable[ch][cmd-4] = 1;
        servo_disable[ch][cmd-4] = 1;
        pwm_pulse[ch][cmd-4] = 0;
        servo_pulse[ch][cmd-4] = 0;         
        switchInputAndOutput(gpio_ch_to_port[ch][cmd-4], gpio_pinmap[ch][cmd-4], GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
        gpio_digi_read_state = gpio_read_level(gpio_ch_to_port[ch][cmd-4], gpio_pinmap[ch][cmd-4]);
        i2c1_set_send_data((uint8_t *)&gpio_digi_read_state, 1);
      } 
      else if (cmd == 6) {
        pwm_disable[ch][0] = 1;
        servo_disable[ch][0] = 1;
        pwm_pulse[ch][0] = 0;
        servo_pulse[ch][0] = 0;
        switchInputAndOutput(gpio_ch_to_port[ch][cmd-6], gpio_pinmap[ch][cmd-6], GPIO_MODE_ANALOG, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW);
        readADCState(ch, 12);
      }
      else if (cmd == 8) {
        buf[0] = rgb_led_nums[ch] & 0xff;
        buf[1] = (rgb_led_nums[ch] >> 8) & 0xff;
        i2c1_set_send_data(buf, 2);
      }   
      else if (cmd == 9) {
        for (int i = 0; i < 5; i++) {
          buf[i] = rgb_one_led_color[ch][i];
        }
        i2c1_set_send_data(buf, 5);
      }
      else if (cmd == 0x0A) {
        for (int i = 0; i < 7; i++) {
          buf[i] = rgb_multi_led_color[ch][i];
        }      
        i2c1_set_send_data(buf, 7);
      }     
      else if (cmd == 0x0B) {
        i2c1_set_send_data((uint8_t *)&rgb_led_brightness[ch], 1);
      } 
    } else {
      if (rx_data[0] == 0xfe)
      {
        i2c1_set_send_data((uint8_t *)&fm_version, 1);
      }    
      else if (rx_data[0] == 0xff)
      {
        i2c1_set_send_data(i2c_address, 1);
      }
      else if (rx_data[0] == 0xFC)
      {
        bootloader_version = *(uint8_t*)BOOTLOADER_VER_ADDR;
        i2c1_set_send_data((uint8_t *)&bootloader_version, 1);
      }         
    }
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  IAP_Set();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  // MX_I2C1_Init();
  MX_ADC_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
  i2c_address_read_from_flash();
  user_i2c_init();
  sk6812_init(TOTAL_RGB);
  for (int i = 0; i < CH_NUMBER; i++) {
    rgb_led_nums[i] = TOTAL_RGB;
  }
  i2c1_it_enable();
  HAL_TIM_Base_Start_IT(&htim16);
  HAL_TIM_Base_Start_IT(&htim17);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(TIM16->CNT >= 2550)	
    {
      TIM16->CNT = 0;  
    } 
    for(int i=0; i<CH_NUMBER; i++)
    {
      for(int j=0; j<2; j++) {
        if (pwm_disable[i][j])
          continue;
        if(TIM16->CNT <= pwm_pulse[i][j] && Switch[i][j]==0)
        {
          gpio_write_level_high(gpio_ch_to_port[i][j], gpio_pinmap[i][j]);
          Switch[i][j]=1;
        }
        else if(TIM16->CNT > pwm_pulse[i][j] && Switch[i][j]!=0 && TIM16->CNT <= 2550)
        {
          gpio_write_level_low(gpio_ch_to_port[i][j], gpio_pinmap[i][j]);
          Switch[i][j]=0;
        }
      }
    }
    if(TIM17->CNT >= 20000)	
    {
      TIM17->CNT = 0;  
    } 
    for(int i=0; i<CH_NUMBER; i++)
    {
      for(int j=0; j<2; j++) {
        if (servo_disable[i][j])
          continue;
        if(TIM17->CNT <= servo_pulse[i][j] && Switch[i][j]==0)
        {
          gpio_write_level_high(gpio_ch_to_port[i][j], gpio_pinmap[i][j]);
          Switch[i][j]=1;
        }
        else if(TIM17->CNT > servo_pulse[i][j] && Switch[i][j]!=0)
        {
          gpio_write_level_low(gpio_ch_to_port[i][j], gpio_pinmap[i][j]);
          Switch[i][j]=0;
        }
      }
    }  

    i2c_timeout_counter = 0;
    if (i2c_stop_timeout_flag) {
      if (i2c_stop_timeout_delay < HAL_GetTick()) {
        i2c_stop_timeout_counter++;
        i2c_stop_timeout_delay = HAL_GetTick() + 10;
      }
    }
    if (i2c_stop_timeout_counter > 50) {
      LL_I2C_DeInit(I2C1);
      LL_I2C_DisableAutoEndMode(I2C1);
      LL_I2C_Disable(I2C1);
      LL_I2C_DisableIT_ADDR(I2C1);     
      user_i2c_init();    
      i2c1_it_enable();
      HAL_Delay(500);
    }       
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while(LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1)
  {
  }
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI14_Enable();

   /* Wait till HSI14 is ready */
  while(LL_RCC_HSI14_IsReady() != 1)
  {

  }
  LL_RCC_HSI14_SetCalibTrimming(16);
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI_DIV_2, LL_RCC_PLL_MUL_12);
  LL_RCC_PLL_Enable();

   /* Wait till PLL is ready */
  while(LL_RCC_PLL_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
  {

  }
  LL_SetSystemCoreClock(48000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_HSI14_EnableADCControl();
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
