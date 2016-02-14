/**
  ******************************************************************************
  * @file    ADC/ADC_RegularConversion_DMA/Inc/main.h 
  * @author  MCD Application Team
  * @version V1.2.1
  * @date    13-March-2015
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

#define __DMAx_TxRx_CLK_ENABLE            __DMA2_CLK_ENABLE
#define SD_DMAx_Tx_CHANNEL                DMA_CHANNEL_4
#define SD_DMAx_Rx_CHANNEL                DMA_CHANNEL_4
#define SD_DMAx_Tx_STREAM                 DMA2_Stream6
#define SD_DMAx_Rx_STREAM                 DMA2_Stream3
#define SD_DMAx_Tx_IRQn                   DMA2_Stream6_IRQn
#define SD_DMAx_Rx_IRQn                   DMA2_Stream3_IRQn
#define SD_DMAx_Tx_IRQHandler             DMA2_Stream6_IRQHandler
#define SD_DMAx_Rx_IRQHandler             DMA2_Stream3_IRQHandler


/* User can use this section to tailor ADCx instance used and associated 
   resources */
/* Definition for ADCx clock resources */
#define ADCx                            ADC1
#define ADCx_CLK_ENABLE()               __HAL_RCC_ADC1_CLK_ENABLE()

#define ADCy                            ADC2
#define ADCy_CLK_ENABLE()               __HAL_RCC_ADC2_CLK_ENABLE()


#define ADCxy_DMA_CLK_ENABLE()           __HAL_RCC_DMA2_CLK_ENABLE()
#define ADCx_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()
#define ADCy_CHANNEL_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOC_CLK_ENABLE()
     
#define ADCx_FORCE_RESET()              __HAL_RCC_ADC_FORCE_RESET()
#define ADCx_RELEASE_RESET()            __HAL_RCC_ADC_RELEASE_RESET()

/* Definition for ADCx Channel Pin */
#define ADCx_CHANNEL_PIN                GPIO_PIN_0
#define ADCx_CHANNEL_GPIO_PORT          GPIOC

#define ADCy_CHANNEL_PIN                GPIO_PIN_1
#define ADCy_CHANNEL_GPIO_PORT          GPIOC

/* Definition for ADCx's Channel */
#define ADCx_CHANNEL                    ADC_CHANNEL_10
#define ADCy_CHANNEL                    ADC_CHANNEL_11

/* Definition for ADCx's DMA */
#define ADCx_DMA_CHANNEL                DMA_CHANNEL_0
#define ADCxy_DMA_STREAM                DMA2_Stream0

/* Definition for ADCx's NVIC */
#define ADCxy_DMA_IRQn                  DMA2_Stream0_IRQn
#define ADCxy_DMA_IRQHandler            DMA2_Stream0_IRQHandler


#define TIMx                            TIM3

#define TIMx_CH1                        TIM_CHANNEL_1
#define TIMx_CH2                        TIM_CHANNEL_2

#define TIMx_CLK_ENABLE()               __HAL_RCC_TIM3_CLK_ENABLE()

#define TIMx_CH1_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()
#define TIMx_CH2_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOC_CLK_ENABLE()

#define TIMx_CH1_PIN                    GPIO_PIN_6
#define TIMx_CH1_GPIO_PORT              GPIOC

#define TIMx_CH2_PIN                    GPIO_PIN_7
#define TIMx_CH2_GPIO_PORT              GPIOC

#define TIMy                            TIM1

#define TIMy_ENCODER_CHANNEL            TIM_CHANNEL_1
#define TIMy_GPIO_CHANNEL               TIM_CHANNEL_2

#define TIMy_CLK_ENABLE()               __HAL_RCC_TIM1_CLK_ENABLE()
#define TIMy_DMA_CLK_ENABLE()           __HAL_RCC_DMA2_CLK_ENABLE()

#define TIMy_DMA_CHANNEL                DMA_CHANNEL_6

#define TIMy_ENCODER_DMA_STREAM         DMA2_Stream1
#define TIMy_GPIO_DMA_STREAM            DMA2_Stream2

/* Definition for ADCx's NVIC */
#define TIMy_ENCODER_DMA_IRQn           DMA2_Stream1_IRQn
#define TIMy_ENCODER_DMA_IRQHandler     DMA2_Stream1_IRQHandler

#define TIMy_GPIO_DMA_IRQn              DMA2_Stream2_IRQn
#define TIMy_GPIO_DMA_IRQHandler        DMA2_Stream2_IRQHandler




#ifdef __cplusplus
extern "C" {
#endif

extern ADC_HandleTypeDef AdcHandle1;
extern ADC_HandleTypeDef AdcHandle2;

extern TIM_HandleTypeDef TimHandle1;
extern TIM_HandleTypeDef TimHandle2;

#ifdef __cplusplus
};
#endif

/* Exported macro ------------------------------------------------------------*/


/* Exported functions ------------------------------------------------------- */


#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
