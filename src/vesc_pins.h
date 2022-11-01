
#pragma once

#include <Arduino.h>

#define H1 PA8
#define H2 PA9
#define H3 PA10

#define L1 PB13
#define L2 PB14
#define L3 PB15

#define BR_SO2 PB0
#define BR_SO1 PB1
#define DC_CAL PB12

#define EN_GATE PB5

#define UART_BAUD_RATE 115200

void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState		 = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState	 = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource	 = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM		 = 8;
	RCC_OscInitStruct.PLL.PLLN		 = 336;
	RCC_OscInitStruct.PLL.PLLP		 = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ		 = 7;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource	 = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider	 = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}
}

#define IOUTA PC0
#define IOUTB PC1
#define IOUTC PC2

#define HAS_LowCurrentSense 1