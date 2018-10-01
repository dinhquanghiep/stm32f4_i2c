/**
  ******************************************************************************
  * @file     main.c
  * @author   hiepdq
  * @version  xxx
  * @date     26.09.2018
  * @brief    This file provides main firmware functions for MCU 
  *           Try to using all peripheral and standard coding style    
 ===============================================================================      
                       ##### How to use this driver #####
 ===============================================================================
  
  ******************************************************************************
  * @attention
  *
  ******************************************************************************  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stm32f4xx.h>
#include <stm32f4xx_rcc.h>
#include <stm32f4xx_gpio.h>
#include <stm32f4xx_dma.h>
#include <stm32f4xx_usart.h>
#include <stm32f4xx_i2c.h>
#include <stm32f4xx_tim.h>
#include <misc.h>
#include <string.h>
#if defined (__GNUC__)
#include <malloc.h>
#elif defined (__ICCARM__)

#endif

/* Private macro -------------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define LED_GREEN   GPIO_Pin_12
#define LED_ORANGE  GPIO_Pin_13
#define LED_RED     GPIO_Pin_14
#define LED_BLUE    GPIO_Pin_15
#define USER_BUTTON GPIO_Pin_0

//I2C Slave Address  
const uint8_t DS1307_ADDRESS = 0x68;

//DS1307 Register Addresses
const uint8_t DS1307_REG_TIMEDATE   = 0x00;
const uint8_t DS1307_REG_STATUS     = 0x00;
const uint8_t DS1307_REG_CONTROL    = 0x07;
const uint8_t DS1307_REG_RAMSTART   = 0x08;
const uint8_t DS1307_REG_RAMEND     = 0x3f;
// const uint8_t DS1307_REG_RAMSIZE = DS1307_REG_RAMEND - DS1307_REG_RAMSTART;

//DS1307 Register Data Size if not just 1
const uint8_t DS1307_REG_TIMEDATE_SIZE = 7;

// DS1307 Control Register Bits
const uint8_t DS1307_RS0   = 0;
const uint8_t DS1307_RS1   = 1;
const uint8_t DS1307_SQWE  = 4;
const uint8_t DS1307_OUT   = 7;

// DS1307 Status Register Bits
const uint8_t DS1307_CH       = 7;

enum DS1307SquareWaveOut
{
    DS1307SquareWaveOut_1Hz  =  0b00010000,
    DS1307SquareWaveOut_4kHz =  0b00010001,
    DS1307SquareWaveOut_8kHz =  0b00010010,
    DS1307SquareWaveOut_32kHz = 0b00010011,
    DS1307SquareWaveOut_High =  0b10000000,
    DS1307SquareWaveOut_Low =   0b00000000,
};
/* Private typedef -----------------------------------------------------------*/
/* Public variables ----------------------------------------------------------*/
volatile uint32_t time_ms = 0;
volatile uint32_t timeout = 0;
/* Private variables ---------------------------------------------------------*/
static uint8_t buff_recv[100];
static uint8_t count = 0;
static uint8_t buff_send[] = "Dinh Quang Hiep\nDinh Quang Hiep\nDinh Quang Hiep\n";
static uint8_t buffer_count = 0;
uint8_t rtc_data[] = {0x22, /* Second */
                      0x22, /* Minute */
                      0x06, /* Hour */
                      0x01, /* Day */
                      0x22, /* Date */
                      0x10, /* Month */
                      0x18};/* Year */
uint8_t rtc_data_recv[7];
static volatile uint8_t second;
static volatile uint8_t minute;
static volatile uint8_t hour;
static volatile uint8_t day;
static volatile uint8_t date;
static volatile uint8_t month;
static volatile uint8_t year;
/* Private function prototypes -----------------------------------------------*/
static void rcc_config(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
uint32_t millis(void);
static void gpio_config(void);
static void dma_config(void);
static void usart_config(void);
static void timer_config(void);
static void i2c_config(void);
static void nvic_config(void);
/* Public functions ----------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/** @brief  Config the clocks for system
  * @param  None
  * 
  * @retval None
  */
static void rcc_config(void) {
  /* Switch systemclock to HSI */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_HSI);
  // RCC_HSEConfig(RCC_HSE_ON);
  // while (RCC_WaitForHSEStartUp() == ERROR) {
  //   /* Waitng for HSE config  */
  // }
  /* PLL must be disabled before config */
  RCC_PLLCmd(DISABLE);
  RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);
  RCC_PLLCmd(ENABLE);
  while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET) {
    /* Waitng for PLL enable  */
  }
  /* Switch the systemclock to PLLCLK */
  RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
  /* Ensure the systemclock is switched to PLL then disable HSI */
  if (RCC_GetSYSCLKSource() == 0x08) {
    RCC_HSICmd(DISABLE);
  }
  RCC_ClockSecuritySystemCmd(DISABLE);
  RCC_HCLKConfig(RCC_SYSCLK_Div1);
  RCC_PCLK1Config(RCC_HCLK_Div4);
  RCC_PCLK2Config(RCC_HCLK_Div2);

  SystemCoreClockUpdate();
  SysTick_Config(168000); 
}

/** @brief  Delay in ms
  * @param  the time to delay(unit: ms)
  * @note   error very small, base others interrupt
  * @retval None
  */
void delay_ms(uint32_t ms) {
  if (!ms) return;

  uint32_t curr_time_ms = time_ms;
  uint32_t curr_tick = SysTick->VAL;
  while (ms) {
    if (curr_time_ms != time_ms) {
      ms--;
      curr_time_ms = time_ms;
    }
  }
  while ((SysTick->VAL > curr_tick) && (curr_time_ms == time_ms)) {
    /* Waiting for delay, do nothing */
  }
}

/** @brief  Delay in us
  * @param  the time to delay(unit: us)
  * @note   
  * @retval None
  */
void delay_us(uint32_t us) {
  if (us > 1000) {
    uint32_t ms_count = us / 1000;
    us %= 1000;
    delay_ms(ms_count);
  }
  uint32_t curr_systick_value = SysTick->VAL;
  uint32_t curr_time_ms = time_ms;
  uint32_t total_tick = us * 168;
  if (curr_systick_value < total_tick) {
    while (curr_time_ms == time_ms) {
      /* waiting here */
    }
    while (curr_time_ms + 1 == time_ms) {
      if (SysTick->VAL <= (SysTick->LOAD + curr_systick_value - total_tick)) break;
    }
  } else {
    while (curr_time_ms == time_ms) {
      if (SysTick->VAL <= (curr_systick_value - total_tick)) break;
    }
  }
}

/** @brief  return the time of system in ms
  * @param  none
  * 
  * @retval None
  */
uint32_t millis(void) {
  return time_ms;
}

/** @brief  Config the clocks for system
  * @param  None
  * 
  * @retval None
  */
static void gpio_config(void) {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

  GPIO_DeInit(GPIOA);
  GPIO_DeInit(GPIOD);

  GPIO_InitTypeDef GPIO_InitStruct;
  /* For User button */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStruct.GPIO_Pin = USER_BUTTON;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* For User LED */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Pin = LED_BLUE | LED_GREEN | LED_RED | LED_ORANGE;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_WriteBit(GPIOD, LED_BLUE, Bit_SET);

  /* For USART2 */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

  /* For USART3 PD8 - TX, PB11 - RX */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
  GPIO_Init(GPIOD, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource3, GPIO_AF_USART3);
  
  /* For I2C3 PA8 - SCL, PC9 - SDA */
  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C3);

  GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
  GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C3);
}

/** @brief  Config the clocks for system
  * @param  None
  * 
  * @retval None
  */
static void dma_config(void) {
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  /* For USART2 Tx */
  DMA_DeInit(DMA1_Stream6);
  DMA_InitTypeDef DMA_InitStruct;
  DMA_StructInit(&DMA_InitStruct);
  DMA_InitStruct.DMA_Channel = DMA_Channel_4;
  DMA_InitStruct.DMA_PeripheralBaseAddr = USART2_BASE + 0x04;
  DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)buff_send;
  DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStruct.DMA_BufferSize = strlen((char *)buff_send);
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
  DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOStatus_1QuarterFull;
  DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream6, &DMA_InitStruct);

  /* For USART2 Rx */
  DMA_InitStruct.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStruct.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStruct.DMA_BufferSize = 100;
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)buff_recv;
  DMA_Init(DMA1_Stream5, &DMA_InitStruct);
  // DMA_ITConfig(DMA1_Stream6, DMA_IT_TC, ENABLE);

  DMA_Cmd(DMA1_Stream6, ENABLE);
  DMA_Cmd(DMA1_Stream5, ENABLE);
  
  /* For I2C3 Tx */
  DMA_InitStruct.DMA_Channel = DMA_Channel_3;
  DMA_InitStruct.DMA_PeripheralBaseAddr = I2C3_BASE + 0x10;
  DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)rtc_data;
  DMA_InitStruct.DMA_DIR = DMA_DIR_MemoryToPeripheral;
  DMA_InitStruct.DMA_BufferSize = sizeof(rtc_data);
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStruct.DMA_Priority = DMA_Priority_Low;
  DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOStatus_1QuarterFull;
  DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream4, &DMA_InitStruct);
  DMA_ITConfig(DMA1_Stream4, DMA_IT_TC, ENABLE);

  /* For I2C3 Rx */
  DMA_InitStruct.DMA_Channel = DMA_Channel_3;
  DMA_InitStruct.DMA_PeripheralBaseAddr = I2C3_BASE + 0x10;
  DMA_InitStruct.DMA_Memory0BaseAddr = (uint32_t)rtc_data_recv;
  DMA_InitStruct.DMA_DIR = DMA_DIR_PeripheralToMemory;
  DMA_InitStruct.DMA_BufferSize = sizeof(rtc_data_recv);
  DMA_InitStruct.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStruct.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStruct.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStruct.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStruct.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStruct.DMA_Priority = DMA_Priority_High;
  DMA_InitStruct.DMA_FIFOMode = DMA_FIFOMode_Disable;
  DMA_InitStruct.DMA_FIFOThreshold = DMA_FIFOStatus_1QuarterFull;
  DMA_InitStruct.DMA_MemoryBurst = DMA_MemoryBurst_Single;
  DMA_InitStruct.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream2, &DMA_InitStruct);
  DMA_ITConfig(DMA1_Stream2, DMA_IT_TC, ENABLE);
}

/** @brief  Config the UASRT2
  * @param  None
  * 
  * @retval None
  */
static void usart_config(void) {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
  USART_InitTypeDef USART_InitStruct;
  USART_StructInit(&USART_InitStruct);
  USART_InitStruct.USART_BaudRate = 9600;
  USART_InitStruct.USART_WordLength = USART_WordLength_8b;
  USART_InitStruct.USART_StopBits = USART_StopBits_1;
  USART_InitStruct.USART_Parity = USART_Parity_No;
  USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_OverSampling8Cmd(USART2, DISABLE);
  USART_Init(USART2, &USART_InitStruct);

  // USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

  USART_Cmd(USART2, ENABLE);
  USART_DMACmd(USART2, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
  
  // RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  // USART_Init(USART3, &USART_InitStruct);
  // USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
}

/** @brief  Config the Timer for receive time from DS1307
  * @param  None
  * 
  * @retval None
  */
static void timer_config(void) {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  /** input clock: 42Mhz
    * period: 1s
    * prescale: 420000
    * timer period: 1000 
   */
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
  TIM_TimeBaseStructInit(&TIM_TimeBaseInitStruct);
  TIM_TimeBaseInitStruct.TIM_Prescaler = 419999;
  TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInitStruct.TIM_Period = 999;
  TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStruct);
  
  TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM2, ENABLE);

}
/** @brief  Config the I2C
  * @param  None
  * 
  * @retval None
  */
static void i2c_config(void) {
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
  I2C_InitTypeDef I2C_InitStruct;
  I2C_StructInit(&I2C_InitStruct);
  I2C_InitStruct.I2C_ClockSpeed = 100000;
  I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
  I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
  I2C_InitStruct.I2C_OwnAddress1 = 0b0101010;
  I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
  I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
  I2C_Init(I2C3, &I2C_InitStruct);
  I2C_Cmd(I2C3, ENABLE);

  I2C_DMALastTransferCmd(I2C3, ENABLE);
  I2C_DMACmd(I2C3, ENABLE);

}

/** @brief  Config the NVIC
  * @param  None
  * 
  * @retval None
  */
static void nvic_config(void) {
  NVIC_InitTypeDef NVIC_InitStruct;
  NVIC_InitStruct.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream6_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream2_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 2;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = DMA1_Stream4_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

  NVIC_InitStruct.NVIC_IRQChannel = TIM2_IRQn;
  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 4;
  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStruct);

}
/* Main source ---------------------------------------------------------------*/
int main(void) {
  rcc_config();
  gpio_config();
  dma_config();
  usart_config();
  i2c_config();
  nvic_config();
  uint8_t chuoi[] = "Chao mung cac ban den voi phan quan ly su dung command line\n"
"    + nhap vao ki tu, ket thuc bang dau cham\n";
/* Toc do baud 115200 truye chuoi tren mat 12ms */
  while (1) {
    if (I2C_GetFlagStatus(I2C3, I2C_FLAG_BUSY) == RESET) {
      I2C_GenerateSTART(I2C3, ENABLE);
      timeout = 1000; /* Timeout = 1000ms */
      while ((I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT) == ERROR) && timeout) {
        /* Wating for SB bit */
      }
      if (!timeout) {
        if (USART_GetFlagStatus(USART2, USART_FLAG_TC) != RESET) {
          for (uint16_t len = 0; len < strlen("Loi I2C, khong the ket noi"); len++) {
            while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
              /* Wait until Transmistion complete */
            }
            USART_SendData(USART2, len["Loi I2C, khong the ket noi"]);
          }
        }
      }
      /* For remove EV5 */
      I2C_Send7bitAddress(I2C3, 0b11010000, I2C_Direction_Transmitter);
      timeout = 1000; /* Timeout = 1000ms */
      while ((I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR) && timeout) {
        /* Waiting for ADDR */
      }
      /* Clear ADDR bit */
      I2C_SendData(I2C3, DS1307_REG_TIMEDATE);
      timeout = 1000;
      while ((I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR) && timeout) {
        /* Wait until byte transmitted */
      }
      /* use DMA */
      DMA_ClearFlag(DMA1_Stream4, DMA_FLAG_TCIF4);
      DMA_Cmd(DMA1_Stream4, ENABLE);
    }
  timer_config();

    while (1) {
      #if 0
      delay_ms(100);
      if (USART_GetFlagStatus(USART2, USART_FLAG_TC) != RESET) {
        for (uint16_t len = 0; len < strlen((char *)chuoi); len++) {
          while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
            /* Wait until Transmistion complete */
          }
          USART_SendData(USART2, chuoi[len]);
        }
      }
      GPIO_ToggleBits(GPIOD, LED_BLUE);
      delay_ms(100);
      // delay_us(500000);
      if (USART_GetFlagStatus(USART2, USART_FLAG_TC) != RESET) {
        DMA1_Stream6->M0AR = (uint32_t)buff_send;
        DMA1_Stream6->NDTR = strlen((char *)buff_send);
        DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);
        DMA_Cmd(DMA1_Stream6, ENABLE);
      }
      delay_ms(100);
      if (USART_GetFlagStatus(USART2, USART_FLAG_TC) != RESET) {
        DMA1_Stream6->M0AR = (uint32_t)"Toi ten la dinh quang hiep";
        DMA1_Stream6->NDTR = (uint32_t)strlen("Toi ten la dinh quang hiep");
        DMA_ClearFlag(DMA1_Stream6, DMA_FLAG_TCIF6);
        DMA_Cmd(DMA1_Stream6, ENABLE);
      }
      #endif
    }
  }
  return 0;
}
/**
  * @brief  This function handles EXTI0_IRQHandler interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void) {
  if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {
    uint8_t tmp = (uint8_t)USART_ReceiveData(USART2);
    if (tmp != '\n') {
      buff_recv[buffer_count++] = tmp;
    } else {
      const uint8_t chuoi_tmp[] = "\nBan da nhap vao chuoi: ";
      for (uint8_t tmp = 0; tmp < strlen((char *)chuoi_tmp); tmp++) {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
          /* waiting for transfer */
        }
        USART_SendData(USART2, chuoi_tmp[tmp]);
      }
      for (uint8_t tmp = 0; tmp < buffer_count; tmp++) {
        while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET) {
          /* waiting for transfer */
        }
        USART_SendData(USART2, buff_recv[tmp]);
      }
      buffer_count = 0;
    }
  }
}

void DMA1_Stream2_IRQHandler(void) {
  I2C_GenerateSTOP(I2C3, ENABLE);
  DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
}

void DMA1_Stream4_IRQHandler(void) {
  I2C_GenerateSTOP(I2C3, ENABLE);
  DMA_ClearITPendingBit(DMA1_Stream4, DMA_IT_TCIF4);
}

void TIM2_IRQHandler(void) {
  if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) {
    TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
    if (I2C_GetFlagStatus(I2C3, I2C_FLAG_BUSY) == RESET) {
      I2C_GenerateSTART(I2C3, ENABLE);
      while (I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT) == ERROR) {
        /* Wating for SB bit */
      }
      /* For remove EV5 */
      I2C_Send7bitAddress(I2C3, 0b11010000, I2C_Direction_Transmitter);
      while (I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR) {
        /* Waiting for ADDR */
      }
      /* Clear ADDR bit */
      I2C_SendData(I2C3, DS1307_REG_TIMEDATE);
      /* Restart sau khi ghi dia chi muon doc */
      I2C_GenerateSTART(I2C3, ENABLE);
      while (I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_MODE_SELECT) == ERROR) {
        /* Wating for SB bit */
      }
      /* For remove EV5 */
      I2C_Send7bitAddress(I2C3, 0b11010000, I2C_Direction_Receiver);
      while (I2C_CheckEvent(I2C3, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) == ERROR) {
        /* Waiting for ADDR */
      }
      /* Clear ADDR bit */
      /* using DMA to receive data */
      DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
      DMA_Cmd(DMA1_Stream2, ENABLE);
    }
  }
}