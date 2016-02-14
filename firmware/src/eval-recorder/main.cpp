/*
 * recroder.cpp
 *
 *  Created on: 03.01.2016
 *      Author: andi
 */
#include "stm32f4xx_hal.h"
#include "stm32h405_sd.h"
#include <string.h>

static void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_OscInitTypeDef RCC_OscInitStruct;

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;             /* 1 Mhz Reference Clock for PLL */
    RCC_OscInitStruct.PLL.PLLN = 48 * 2 * 2;    /* PLL shall output a 192 MHz Clock */
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4; /* System clock should be 48 MHz as well */
    RCC_OscInitStruct.PLL.PLLQ = 4;             /* Provide an 48 Mhz Reference Clock for USB */
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK |
                                   RCC_CLOCKTYPE_HCLK |
                                   RCC_CLOCKTYPE_PCLK1 |
                                   RCC_CLOCKTYPE_PCLK2);

    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1; /* AHB is  running with 48 MHz */
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;   /* APB1 is running wth 24 MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;   /* APB2 is running with 12 MHz */
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
}

ADC_HandleTypeDef AdcHandle1;
ADC_HandleTypeDef AdcHandle2;

TIM_HandleTypeDef TimHandle1;
TIM_HandleTypeDef TimHandle2;




uint16_t ausSampleBuffer[127 * 2][2]   = {0};
uint16_t ausEncoderBuffer[254 * 2] = {0};
uint8_t  abGpioBuffer[508 * 2]    = {0};

struct element
{
  uint32_t id;

  uint8_t  data[508];
};

static struct element   adc_buffer_pool[32];
static struct element   enc_buffer_pool[16];
static struct element   gpio_buffer_pool[8];

volatile unsigned int adcreq = 0, adccnf = 0;
volatile unsigned int encreq = 0, enccnf = 0;
volatile unsigned int gpioreq = 0, gpiocnf = 0;

unsigned int adcseq = 0;
unsigned int encseq = 0;
unsigned int gpioseq = 0;

uint64_t     writeoffset = 0;

static void Error_Handler(void);
static void ADC_Config(void);
static void TIM_Config(void);


static void Enc_DMACaptureHalfCplt(DMA_HandleTypeDef *hdma);
static void Enc_DMACaptureCplt(DMA_HandleTypeDef *hdma);

static void Gpio_DMACaptureHalfCplt(DMA_HandleTypeDef *hdma);
static void Gpio_DMACaptureCplt(DMA_HandleTypeDef *hdma);

static void DMAError(DMA_HandleTypeDef *hdma);

HAL_SD_CardInfoTypedef cardinfo;

int main()
{
  uint_fast8_t Result;


  HAL_Init();

  SystemClock_Config();

  ADC_Config();

  TIM_Config();

  if(MSD_OK != BSP_SD_Init())
  {
    Error_Handler();
  }

  BSP_SD_GetCardInfo(&cardinfo);

  /* erase whole card to avoid stalling when recording */
  if(MSD_OK != BSP_SD_Erase(0, cardinfo.CardCapacity-1))
  {
    Error_Handler();
  }

  /*##-2- Enable ADC2 ########################################################*/
  if(HAL_ADC_Start(&AdcHandle2) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  /*##-3- Start ADC1 and ADC2 multimode conversion process and enable DMA ####*/
  if(HAL_ADCEx_MultiModeStart_DMA(&AdcHandle1, (uint32_t*)&ausSampleBuffer,
      sizeof(ausSampleBuffer) / sizeof(ausSampleBuffer[0])) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }


  /* Start Timer 2 DMA to record timer triggers (encoder) */
  TimHandle2.hdma[TIM_DMA_ID_CC1]->XferCpltCallback     = Enc_DMACaptureCplt;
  TimHandle2.hdma[TIM_DMA_ID_CC1]->XferHalfCpltCallback = Enc_DMACaptureHalfCplt;
  TimHandle2.hdma[TIM_DMA_ID_CC1]->XferErrorCallback    = DMAError ;

  /* Enable the DMA Stream */
  HAL_DMA_Start_IT(TimHandle2.hdma[TIM_DMA_ID_CC1], (uint32_t)&TimHandle1.Instance->CNT,
      (uint32_t)(ausEncoderBuffer), sizeof(ausEncoderBuffer) / sizeof(ausEncoderBuffer[0]));

  /* Enable the TIM Capture/Compare 1 DMA request */
  __HAL_TIM_ENABLE_DMA(&TimHandle2, TIM_DMA_CC1);

  /* Enable the Output Compare channel */
  TIM_CCxChannelCmd(TimHandle2.Instance, TIMy_ENCODER_CHANNEL, TIM_CCx_ENABLE);

  /* Start Timer 2 DMA to record timer triggers (gpio) */
  TimHandle2.hdma[TIM_DMA_ID_CC2]->XferCpltCallback     = Gpio_DMACaptureCplt;
  TimHandle2.hdma[TIM_DMA_ID_CC2]->XferHalfCpltCallback = Gpio_DMACaptureHalfCplt;
  TimHandle2.hdma[TIM_DMA_ID_CC2]->XferErrorCallback    = DMAError ;

  /* Enable the DMA Stream */
  HAL_DMA_Start_IT(TimHandle2.hdma[TIM_DMA_ID_CC2], (uint32_t)&(TIMx_CH1_GPIO_PORT->IDR),
      (uint32_t)(abGpioBuffer), sizeof(abGpioBuffer) / sizeof(abGpioBuffer[0]));

  /* Enable the TIM Capture/Compare 2 DMA request */
  __HAL_TIM_ENABLE_DMA(&TimHandle2, TIM_DMA_CC2);

  /* Enable the Output Compare channel */
  TIM_CCxChannelCmd(TimHandle2.Instance, TIMy_GPIO_CHANNEL, TIM_CCx_ENABLE);

  HAL_TIM_Base_Start(&TimHandle2);

  /* start the encoder timer */
  if(HAL_TIM_Encoder_Start(&TimHandle1, TIMx_CH1 | TIMx_CH2) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }




  while(1)
  {
    if (writeoffset >= cardinfo.CardCapacity)
    {
      /* full */
    }
    else if(adcreq != adccnf)
    {
      const unsigned int num_buffers = sizeof(adc_buffer_pool) / sizeof(adc_buffer_pool[0]);
      unsigned int i = adccnf % num_buffers;

      if(MSD_OK != BSP_SD_WriteBlocks((uint32_t*) &(adc_buffer_pool[i]), writeoffset, 512,1))
      {
        Error_Handler();
      }

      writeoffset += 512;
      adccnf      += 1;
    }
    else if(gpioreq != gpiocnf)
    {
      const unsigned int num_buffers = sizeof(gpio_buffer_pool) / sizeof(gpio_buffer_pool[0]);
      unsigned int i = gpiocnf % num_buffers;

      if(MSD_OK != BSP_SD_WriteBlocks((uint32_t*) &(gpio_buffer_pool[i]), writeoffset, 512,1))
      {
        Error_Handler();
      }

      writeoffset += 512;
      gpiocnf      += 1;
    }
    else if(encreq != enccnf)
    {
      const unsigned int num_buffers = sizeof(enc_buffer_pool) / sizeof(enc_buffer_pool[0]);
      unsigned int i = enccnf % num_buffers;

      if(MSD_OK != BSP_SD_WriteBlocks((uint32_t*) &(enc_buffer_pool[i]), writeoffset, 512,1))
      {
        Error_Handler();
      }

      writeoffset += 512;
      enccnf      += 1;
    }
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED3 on */
  while(1)
  {
  }
}

/**
  * @brief  ADC configuration
  * @note   This function Configure the ADC peripheral
            1) Enable peripheral clocks
            2) Configure ADC Channel 12 pin as analog input
            3) DMA2_Stream0 channel2 configuration
            4) Configure ADC1 Channel 12
            5) Configure ADC2 Channel 12
  * @param  None
  * @retval None
  */
static void ADC_Config(void)
{
  ADC_ChannelConfTypeDef sConfig;
  ADC_MultiModeTypeDef   mode;

  /*##-1- Configure the ADC2 peripheral ######################################*/
  AdcHandle2.Instance          = ADCy;

  AdcHandle2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV8; /* 1.5 MHz */
  AdcHandle2.Init.Resolution     = ADC_RESOLUTION_12B;
  AdcHandle2.Init.ScanConvMode   = ENABLE;
  AdcHandle2.Init.ContinuousConvMode = ENABLE;
  AdcHandle2.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle2.Init.NbrOfDiscConversion = 0;
  AdcHandle2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  AdcHandle2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  AdcHandle2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  AdcHandle2.Init.NbrOfConversion = 1;
  AdcHandle2.Init.DMAContinuousRequests = ENABLE;
  AdcHandle2.Init.EOCSelection = ENABLE;

  if(HAL_ADC_Init(&AdcHandle2) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-2- Configure ADC2 regular channel #####################################*/
  sConfig.Channel = ADCy_CHANNEL;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES; /* 1.5MHz / (144 + 12) -> 9615 sps */
  sConfig.Offset = 0;

  if(HAL_ADC_ConfigChannel(&AdcHandle2, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }

  /*##-3- Configure the ADC1 peripheral ######################################*/
  AdcHandle1.Instance          = ADCx;

  AdcHandle1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV8;
  AdcHandle1.Init.Resolution = ADC_RESOLUTION_12B;
  AdcHandle1.Init.ScanConvMode = DISABLE;
  AdcHandle1.Init.ContinuousConvMode = ENABLE;
  AdcHandle1.Init.DiscontinuousConvMode = DISABLE;
  AdcHandle1.Init.NbrOfDiscConversion = 0;
  AdcHandle1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  AdcHandle1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
  AdcHandle1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  AdcHandle1.Init.NbrOfConversion = 1;
  AdcHandle1.Init.DMAContinuousRequests = ENABLE;
  AdcHandle1.Init.EOCSelection = ENABLE;

  if(HAL_ADC_Init(&AdcHandle1) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }

  /*##-4- Configure ADC1 regular channel #####################################*/
  sConfig.Channel = ADCx_CHANNEL;

  if(HAL_ADC_ConfigChannel(&AdcHandle1, &sConfig) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }

  /*##-5- Configure Multimode ################################################*/
  mode.Mode             = ADC_DUALMODE_REGSIMULT;
  mode.DMAAccessMode    = ADC_DMAACCESSMODE_2;
  mode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_6CYCLES;
  if(HAL_ADCEx_MultiModeConfigChannel(&AdcHandle1, &mode) != HAL_OK)
  {
    /* Channel Configuration Error */
    Error_Handler();
  }
}

static void
FillAdcBuffer(uint16_t *pData)
{
  const unsigned int num_buffers = sizeof(adc_buffer_pool) / sizeof(adc_buffer_pool[0]);
  unsigned int free_buffers = num_buffers - (adcreq - adccnf);

  if (free_buffers > 0)
  {
    unsigned int i = adcreq % num_buffers;

    memcpy(adc_buffer_pool[i].data, pData, sizeof(adc_buffer_pool[i].data));
    adc_buffer_pool[i].id = 0x10000000 | (adcseq & 0x0FFFFFFF);

    adcreq += 1;
  }
  else
  {
    Error_Handler();
  }

  adcseq += 1;
}

static void
FillEncBuffer(uint16_t *pData)
{
  const unsigned int num_buffers = sizeof(enc_buffer_pool) / sizeof(enc_buffer_pool[0]);
  unsigned int free_buffers = num_buffers - (encreq - enccnf);

  if (free_buffers > 0)
  {
    unsigned int i = encreq % num_buffers;

    memcpy(enc_buffer_pool[i].data, pData, sizeof(enc_buffer_pool[i].data));
    enc_buffer_pool[i].id = 0x20000000 | (encseq & 0x0FFFFFFF);

    encreq += 1;
  }
  else
  {
    Error_Handler();
  }

  encseq += 1;
}

static void
FillGpioBuffer(uint8_t *pData)
{
  const unsigned int num_buffers = sizeof(gpio_buffer_pool) / sizeof(gpio_buffer_pool[0]);
  unsigned int free_buffers = num_buffers - (gpioreq - gpiocnf);

  if (free_buffers > 0)
  {
    unsigned int i = gpioreq % num_buffers;

    memcpy(gpio_buffer_pool[i].data, pData, sizeof(gpio_buffer_pool[i].data));
    gpio_buffer_pool[i].id = 0x30000000 | (gpioseq & 0x0FFFFFFF);

    gpioreq += 1;
  }
  else
  {
    Error_Handler();
  }

  gpioseq += 1;
}

/**
  * @brief  Regular conversion complete callback in non blocking mode
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  FillAdcBuffer(&ausSampleBuffer[254 / 2][0]);
}

/**
  * @brief  Regular conversion half DMA transfer callback in non blocking mode
  * @param  hadc: pointer to a ADC_HandleTypeDef structure that contains
  *         the configuration information for the specified ADC.
  * @retval None
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
  FillAdcBuffer(&ausSampleBuffer[0][0]);
}

void TIM_Config(void)
{
  TIM_Encoder_InitTypeDef sEncConfig;
  TIM_OC_InitTypeDef      sOCChannelConfig;

  /* TIM2/3 ia APB1 Peripheral. APB1 is at 24 MHz with Divider 2
   * TIM2/3 PCLK = 48 MHz
   */

  TimHandle1.Instance               = TIMx;
  TimHandle1.Init.Prescaler         = 0;
  TimHandle1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle1.Init.Period            = 0xFFFF;
  TimHandle1.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  TimHandle1.Init.RepetitionCounter = 0;

  sEncConfig.EncoderMode = TIM_ENCODERMODE_TI12;

  sEncConfig.IC1Polarity  = TIM_ICPOLARITY_BOTHEDGE;
  sEncConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sEncConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sEncConfig.IC1Filter    = 0x8; /* -> (48 MHz / 8 / 6); */

  sEncConfig.IC2Polarity  = TIM_ICPOLARITY_BOTHEDGE;
  sEncConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sEncConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sEncConfig.IC2Filter    = 0x8; /* -> (48 MHz / 8 / 6); */

  HAL_TIM_Encoder_Init(&TimHandle1, &sEncConfig);

  TimHandle2.Instance               = TIMy;
  TimHandle2.Init.Prescaler         = 48-1; /* 1 MHz base clock */
  TimHandle2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle2.Init.Period            = 50-1; /* 20 kHz Sample Rate */
  TimHandle2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  TimHandle2.Init.RepetitionCounter = 0;

  HAL_TIM_OC_Init(&TimHandle2);

  sOCChannelConfig.OCMode = TIM_OCMODE_TIMING;
  sOCChannelConfig.Pulse  = 0x0000;
  sOCChannelConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  sOCChannelConfig.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sOCChannelConfig.OCFastMode  = TIM_OCFAST_DISABLE;
  sOCChannelConfig.OCIdleState = TIM_OCIDLESTATE_RESET;
  sOCChannelConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  HAL_TIM_OC_ConfigChannel(&TimHandle2, &sOCChannelConfig, TIMy_ENCODER_CHANNEL);
  HAL_TIM_OC_ConfigChannel(&TimHandle2, &sOCChannelConfig, TIMy_GPIO_CHANNEL);
}


void Enc_DMACaptureHalfCplt(DMA_HandleTypeDef *hdma)
{
  FillEncBuffer(&ausEncoderBuffer[0]);
}
void Enc_DMACaptureCplt(DMA_HandleTypeDef *hdma)
{
  FillEncBuffer(&ausEncoderBuffer[254]);
}

void Gpio_DMACaptureHalfCplt(DMA_HandleTypeDef *hdma)
{
  FillGpioBuffer(&abGpioBuffer[0]);
}

void Gpio_DMACaptureCplt(DMA_HandleTypeDef *hdma)
{
  FillGpioBuffer(&abGpioBuffer[508]);
}

void DMAError(DMA_HandleTypeDef *hdma)
{
  Error_Handler();
}

