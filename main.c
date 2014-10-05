/**
 ******************************************************************************
 * @file    main.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    31-October-2011
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4x7_eth.h"
#include "netconf.h"
#include "main.h"
#include "httpd.h"
#include "stm32f4xx_tim.h"		// Modulos Timers
#include "serial_debug.h"
//#include "stm32f4_discovery.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTEMTICK_PERIOD_MS  10
/*--------------- LCD Messages ---------------*/
#define MESSAGE1   "     STM32F4x7      "
#define MESSAGE2   "  STM32F-4 Series   "
#define MESSAGE3   "   Webserver Demo   "
#define MESSAGE4   "                    "
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/__IO uint32_t LocalTime =
		0; /* this variable is used to create a time reference incremented by 10ms */
uint32_t timingdelay;

/* Private function prototypes -----------------------------------------------*/
void LCD_LED_Init(void);
void rgb_config(void);
void bsp_led_init();
void bsp_sw_init();
uint16_t const leds[] = { GPIO_Pin_0, GPIO_Pin_1, GPIO_Pin_2, GPIO_Pin_3,
		GPIO_Pin_6, GPIO_Pin_7, GPIO_Pin_10, GPIO_Pin_11 };
uint16_t const switches[] = { GPIO_Pin_2, GPIO_Pin_4, GPIO_Pin_5, GPIO_Pin_6 };

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
RCC_ClocksTypeDef RCC_Clocks;
int main(void) {
	/*!< At this stage the microcontroller clock setting is already configured to
	 168 MHz, this is done through SystemInit() function which is called from
	 startup file (startup_stm32f4xx.s) before to branch to application main.
	 To reconfigure the default setting of SystemInit() function, refer to
	 system_stm32f4xx.c file
	 */
	char pase = 0;
	char pase1 = 0;
	uint16_t duty = 0;
	char estado = 'r';
	char led = 0;
	int i;

	RCC_GetClocksFreq(&RCC_Clocks);

#ifdef SERIAL_DEBUG
	DebugComPort_Init();
	printf("STM32DISCOVERY is booting...\r\n");
#endif

	/*Initialize LCD and Leds */
	LCD_LED_Init();

	/* configure ethernet (GPIOs, clocks, MAC, DMA) */
	ETH_BSP_Config();

	/* Initilaize the LwIP stack */
	LwIP_Init();

	/* Http webserver Init */
	httpd_init();

	ADC_Configuration();
	rgb_config();
	bsp_led_init();
	bsp_sw_init();

	/* Infinite loop */
	while (1) {
		/* check if any packet received */
		if (ETH_CheckFrameReceived()) {
			/* process received ethernet packet */
			LwIP_Pkt_Handle();
		}
		/* handle periodic timers for LwIP */
		LwIP_Periodic_Handle(LocalTime);

		if (LocalTime % 20 == 0) {
			if (!pase1) {
				pase1 = 1;

				if (LocalTime % 100 == 0) {

					// escalera Led
					GPIO_ToggleBits(GPIOD, leds[led]);
					led++;
					if (led > 7)
						led = 0;

					//Switchs
					for (i=0; i < 4; i++) {
						if (GPIO_ReadInputDataBit(GPIOE, switches[i]) == 0) {
							printf("Switch %d apretado\n", i);
						}
					}
				}

				// RGB secuencia
				switch (estado) {
				case 'r':
					duty++;
					if (duty >= 100)
						estado = 'R';
					TIM3->CCR3 = 1000 - duty * 10;
					break;
				case 'R':
					duty--;
					if (duty <= 0)
						estado = 'g';
					TIM3->CCR3 = 1000 - duty * 10;
					break;
				case 'g':
					duty++;
					if (duty >= 100)
						estado = 'G';
					TIM3->CCR4 = 1000 - duty * 10;
					break;
				case 'G':
					duty--;
					if (duty <= 0)
						estado = 'b';
					TIM3->CCR4 = 1000 - duty * 10;
					break;
				case 'b':
					duty++;
					if (duty >= 100)
						estado = 'B';
					TIM3->CCR1 = 1000 - duty * 10;
					break;
				case 'B':
					duty--;
					if (duty <= 0)
						estado = 'r';
					TIM3->CCR1 = 1000 - duty * 10;
					break;
				}

			}
		} else {
			pase1 = 0;
		}
		if (LocalTime % 2000 == 0) {
			if (!pase) {
				pase = 1;
				printf("ADC: %u\n", ADC_GetConversionValue(ADC3));
			}
		} else {
			pase = 0;
		}
	}
	return 0;
}

/**
 * @brief  Inserts a delay time.
 * @param  nCount: number of 10ms periods to wait for.
 * @retval None
 */
void Delay(uint32_t nCount) {
	/* Capture the current local time */
	timingdelay = LocalTime + nCount;

	/* wait until the desired delay finish */
	while (timingdelay > LocalTime) {
	}
}

/**
 * @brief  Updates the system local time
 * @param  None
 * @retval None
 */
void Time_Update(void) {
	LocalTime += SYSTEMTICK_PERIOD_MS;
}

/**
 * @brief  Initializes the STM324xG-EVAL's LCD and LEDs resources.
 * @param  None
 * @retval None
 */
void LCD_LED_Init(void) {

	/* Initialize STM324xG-EVAL's LEDs */
	STM_EVAL_LEDInit(LED5);
	STM_EVAL_LEDInit(LED6);
	STM_EVAL_LEDInit(LED3);
	STM_EVAL_LEDInit(LED4);

}

#ifdef  USE_FULL_ASSERT

/**
 * @brief  Reports the name of the source file and the source line number
 *   where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{}
}
#endif

/**
 * @brief  Configures the ADC.
 * @param  None
 * @retval None
 */
void ADC_Configuration(void) {
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Enable ADC3 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);

	/* Configure ADC Channel 12 as analog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* ADC Common Init */
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC3 Configuration ------------------------------------------------------*/
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC3, &ADC_InitStructure);

	/* ADC3 Regular Channel Config */
	ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, ADC_SampleTime_56Cycles);

	/* Enable ADC3 */
	ADC_Cmd(ADC3, ENABLE);

	/* ADC3 regular Software Start Conv */
	ADC_SoftwareStartConv(ADC3);
}

void rgb_config(void) {
	TIM_TimeBaseInitTypeDef TIM_config;
	GPIO_InitTypeDef GPIO_config;
	TIM_OCInitTypeDef TIM_OC_config;

	/* Habilito el clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* Configuro leds como Segunda Funcion */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	GPIO_config.GPIO_Mode = GPIO_Mode_AF;
	GPIO_config.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4;
	GPIO_config.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_config.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_config.GPIO_OType = GPIO_OType_PP;

	GPIO_Init(GPIOB, &GPIO_config);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource0, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);

	uint16_t PrescalerValue = 0;
	/* Compute the prescaler value */
	PrescalerValue =

	TIM_config.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_config.TIM_ClockDivision = 0;
	TIM_config.TIM_Period = 1000;
	TIM_config.TIM_Prescaler = (uint16_t)((SystemCoreClock / 2) / 1000000) - 1;
	TIM_TimeBaseInit(TIM3, &TIM_config);

	TIM_OC_config.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OC_config.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC_config.TIM_Pulse = 1000;
	TIM_OC_config.TIM_OCPolarity = TIM_OCPolarity_High;

	// CH1 del pwm
	TIM_OC3Init(TIM3, &TIM_OC_config);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	//CH2 del pwm
	TIM_OC_config.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC_config.TIM_Pulse = 1000;
	TIM_OC4Init(TIM3, &TIM_OC_config);
	TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

	//CH3 del pwm
	TIM_OC_config.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OC_config.TIM_Pulse = 1000;
	TIM_OC1Init(TIM3, &TIM_OC_config);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);

	TIM_ARRPreloadConfig(TIM3, ENABLE);

	TIM_Cmd(TIM3, ENABLE);

}

void bsp_led_init() {
	GPIO_InitTypeDef GPIO_InitStruct;

	// Arranco el clock del periferico
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3
			| GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_10 | GPIO_Pin_11;

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // (Push/Pull)
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStruct);
}

void bsp_sw_init() {
	GPIO_InitTypeDef GPIO_InitStruct;

	// Arranco el clock del periferico
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5
			| GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStruct);

}
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
