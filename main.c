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
#include "serial_debug.h"
//#include "stm32f4_discovery.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SYSTEMTICK_PERIOD_MS  10
/*--------------- LCD Messages ---------------*/
#define MESSAGE1   "     STM32F4x7      "
#define MESSAGE2   "  STM32F-4 Series   "
#define MESSAGE3   "   WebServer Demo   "
#define MESSAGE4   "                    "
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint32_t LocalTime = 0; /* this variable is used to create a time reference incremented by 10ms */
uint32_t timingdelay;

/* Private function prototypes -----------------------------------------------*/



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
	char led = 0;
	int i;

	uint8_t valor_web=0;

	RCC_GetClocksFreq(&RCC_Clocks);

#ifdef SERIAL_DEBUG
	DebugComPort_Init();
	printf("STM32DISCOVERY is booting...\r\n");
#endif



	/* configure ethernet (GPIOs, clocks, MAC, DMA) */
	ETH_BSP_Config();

	/* Initilaize the LwIP stack */
	LwIP_Init();

	/* Http webserver Init */
	httpd_init();

	/*Se inicializa la placa de expansion*/
	STM_EVAL_EXP_INIT();

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
					STM_EVAL_EXP_LED_TOGGLE(led);
					led++;
					if (led > 7)
						led = 0;
				}
			}
		} else {
			pase1 = 0;
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


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/


