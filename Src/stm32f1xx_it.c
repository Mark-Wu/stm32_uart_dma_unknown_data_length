/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "main.h"
#include "stm32f1xx_it.h"

#define ARRAY_LEN(x)            (sizeof(x) / sizeof((x)[0]))



extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern UART_HandleTypeDef huart2;



extern uint8_t usart_rx_dma_buffer[DMA_RX_BUFFER_SIZE];
extern uint8_t usart_tx_dma_buffer[LOCAL_CACHE_SIZE];
extern uint8_t local_cache[LOCAL_CACHE_SIZE];
extern uint8_t cache_flag;
extern uint8_t frame_flag;
extern uint16_t frame_size;

static uint8_t idle_flag = 0;

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
 
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
 
  HAL_IncTick();
  
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel6 global interrupt.
  */
void DMA1_Channel6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

/**
  * @brief This function handles DMA1 channel7 global interrupt.
  */
void DMA1_Channel7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

void UsartProcessData(const void* data, size_t len,uint8_t ex_flag) {
  static size_t cache_pos = 0;
  
  if((ex_flag != 0) && (idle_flag == 1)){
    frame_size = cache_pos;
    frame_flag = 1;
    cache_pos = 0;
    idle_flag = 0;
    return;
  }
  
  memcpy(local_cache + cache_pos,data,len);
  cache_pos += len;
  if(idle_flag == 1){
    memcpy(usart_tx_dma_buffer,local_cache,cache_pos);  
    frame_size = cache_pos;
    frame_flag = 1;
    cache_pos = 0;
    idle_flag = 0;
  } 
}

void UsartRxCheck(void) {
  static size_t old_pos = 0;
  size_t pos;

  /* Calculate current position in buffer */
  //pos = ARRAY_LEN(usart_rx_dma_buffer) - hdma_usart2_rx.Instance->CNDTR;
  pos = DMA_RX_BUFFER_SIZE - hdma_usart2_rx.Instance->CNDTR;
  if (pos != old_pos) {                       /* Check change in received data */
    if (pos > old_pos) {                    /* Current position is over previous one */
        /* We are in "linear" mode */
        /* Process data directly by subtracting "pointers" */
        UsartProcessData(&usart_rx_dma_buffer[old_pos], pos - old_pos,0);
    } else {
        /* We are in "overflow" mode */
        /* First process data to the end of buffer */
        UsartProcessData(&usart_rx_dma_buffer[old_pos], ARRAY_LEN(usart_rx_dma_buffer) - old_pos,0);
        /* Continue from beginning of buffer */
        UsartProcessData(&usart_rx_dma_buffer[0], pos,0);
    }
  }else{
    /* for very special situation: pos == old_pos == &usart_rx_dma_buffer[0]  */
    UsartProcessData(&usart_rx_dma_buffer[0],old_pos,1);
  }
    
  old_pos = pos;                              /* Save current position as old */

  /* Check and manually update if we reached end of buffer */
  if (old_pos == ARRAY_LEN(usart_rx_dma_buffer)) {
    old_pos = 0;
  }
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  if (huart2.Instance->SR & UART_FLAG_IDLE)
  {
    volatile uint32_t tmp;     /* Must be volatile to prevent optimizations */
    tmp = huart2.Instance->SR; /* Read status register for clear idle */
    tmp = huart2.Instance->DR; /* Read data register for clear idle */ 
    idle_flag = 1;
    UsartRxCheck();
    idle_flag = 0;       
  }
  HAL_UART_IRQHandler(&huart2);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* complete rx */
  UsartRxCheck();
  return;
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  /* half rx */
  UsartRxCheck();
  return;
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
