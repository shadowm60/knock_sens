/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/

/* Include core modules */
#include "stm32f4xx.h"
/* Include my libraries here */
#include "defines.h"
#include "tm_stm32f4_delay.h"
#include "tm_stm32f4_adc.h"
#include "tm_stm32f4_disco.h"
#include "tm_stm32f4_dac_signal.h"

#include <stdio.h>

/* Include arm_math.h mathematic functions */
#include "arm_math.h"

arm_cfft_radix4_instance_f32 S;	/* ARM CFFT module */

#define BUFFERSIZE 				1024 // 200KHz x2 HT/TC at 1KHz
#define SAMPLES					512 			/* 256 real party and 256 imaginary parts */
#define FFT_SIZE				SAMPLES / 2		/* FFT size is always the same size as we have samples, so 256 in our case */
#define SAMPLE_RATE             44000 /* 44Khz */ /* 200000 200Khz*/
/* Maximal frequency you can analyze is always for one resolution less than half of sampling frequency -> 45450 / 2 – 177.5 = ~22547 */

__IO uint16_t ADCConvertedValues[BUFFERSIZE];
float32_t Input0[SAMPLES];
float32_t Input1[SAMPLES];
/* we sample with 44Khz -> 22.542 Khz */
/* values from 0 -> 127 are the same as 128 -> 256 */
/* 22.542 Khz / 127 -> 0.177 Khz/window */
float32_t Output0[FFT_SIZE];
float32_t Output1[FFT_SIZE];

uint8_t firstHalf = 0;
uint8_t secondHalf = 0;

#define PIF 3.14159f
/* more in mm */
/* for M111.960 bore: 89.9 mm */
/* >>> 6.38 Khz */
/* http://megasquirtavr.sourceforge.net/manualhtml/Detailed.Sensor.Knock.html */
#define BORE	(89.9) //mm
#define BAND(bore) (900 / (PIF * (bore) / 2))

void RCC_Configuration(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
}

void GPIO_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* ADC Channel 11 -> PC1 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void ADC_Configuration(void)
{
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	/* ADC Common Init */
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE; // 1 Channel
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // Conversions Triggered
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channel 11 configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_15Cycles); // PC1

	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
}


static void DMA_Configuration(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADCConvertedValues[0];
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = BUFFERSIZE; // Count of 16-bit words
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	DMA_Init(DMA2_Stream0, &DMA_InitStructure);

	/* Enable DMA Stream Half / Transfer Complete interrupt */
	DMA_ITConfig(DMA2_Stream0, DMA_IT_TC | DMA_IT_HT, ENABLE);

	/* DMA2_Stream0 enable */
	DMA_Cmd(DMA2_Stream0, ENABLE);
}

void TIM2_Configuration(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	/* Time base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Period = (84000000 / SAMPLE_RATE) - 1; // 200 KHz, from 84 MHz TIM2CLK (ie APB1 = HCLK/4, TIM2CLK = HCLK/2)
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* TIM2 TRGO selection */
	TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update); // ADC_ExternalTrigConv_T2_TRGO

	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}

void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the DMA Stream IRQ Channel */
	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void DMA2_Stream0_IRQHandler(void) // Called at 1 KHz for 200 KHz sample rate, LED Toggles at 500 Hz
{
	uint16_t i;
	/* Test on DMA Stream Half Transfer interrupt */
	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0))
	{
		/* Clear DMA Stream Half Transfer interrupt pending bit */
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);

		/* Turn LED3 off: Half Transfer */
		TM_DISCO_LedToggle(LED_GREEN);

		// Add code here to process first half of buffer (ping)
		if (firstHalf == 0)
		{
			for (i = 0; i < SAMPLES; i += 2) {
				/* Real part, must be between -1 and 1 */
				Input0[(uint16_t)i] = (float32_t)((float32_t)ADCConvertedValues[i] - (float32_t)2048.0) / (float32_t)2048.0;
				/* Imaginary part */
				Input0[(uint16_t)(i + 1)] = 0;
			}
			firstHalf = 1;
		}
	}

	/* Test on DMA Stream Transfer Complete interrupt */
	if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
	{
		/* Clear DMA Stream Transfer Complete interrupt pending bit */
		DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
		/* Turn LED3 on: End of Transfer */
		TM_DISCO_LedToggle(LED_ORANGE);
		// Add code here to process second half of buffer (pong)
		if (secondHalf == 0)
		{
			for (i = 0; i < SAMPLES; i += 2) {
				/* Real part, must be between -1 and 1 */
				Input1[(uint16_t)i] = (float32_t)((float32_t)ADCConvertedValues[SAMPLES+i] - (float32_t)2048.0) / (float32_t)2048.0;
				/* Imaginary part */
				Input1[(uint16_t)(i + 1)] = 0;
			}
			secondHalf = 1;
		}
	}
}

void doFFT(uint8_t buff)
{
	static arm_cfft_radix4_instance_f32 S;	/* ARM CFFT module */
	static float32_t maxValue;				/* Max FFT value is stored here */
	static uint32_t maxIndex;				/* Index in Output array where max value is */

	/* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
	arm_cfft_radix4_init_f32(&S, FFT_SIZE, 0, 1);

	TM_DISCO_LedToggle(LED_BLUE);

	if (buff == 0)
	{
		/* Process the data through the CFFT/CIFFT module */
		arm_cfft_radix4_f32(&S, Input0);

		/* Process the data through the Complex Magniture Module for calculating the magnitude at each bin */
		arm_cmplx_mag_f32(Input0, Output0, FFT_SIZE);

		/* Calculates maxValue and returns corresponding value */
		arm_max_f32(Output0, FFT_SIZE, &maxValue, &maxIndex);
	}else{
		/* Process the data through the CFFT/CIFFT module */
		arm_cfft_radix4_f32(&S, Input1);

		/* Process the data through the Complex Magniture Module for calculating the magnitude at each bin */
		arm_cmplx_mag_f32(Input0, Output0, FFT_SIZE);

		/* Calculates maxValue and returns corresponding value */
		arm_max_f32(Output0, FFT_SIZE, &maxValue, &maxIndex);
	}


}
int main(void)
{

	/* Initialize system */
	SystemInit();

	/* Delay init */
	TM_DELAY_Init();

	/* Initialize LED's on board */
	TM_DISCO_LedInit();

	RCC_Configuration();
	GPIO_Configuration();
	NVIC_Configuration();
	TIM2_Configuration();
	DMA_Configuration();
	ADC_Configuration();


	ADC_SoftwareStartConv(ADC1);

	while (1) {



		if (firstHalf)
		{
			doFFT(0);
			firstHalf = 0;
		}
		if (secondHalf)
		{
			doFFT(1);
			secondHalf = 0;
		}


	}
}
