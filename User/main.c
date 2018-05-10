#include "stm32f10x.h"
#include "bsp_usart.h"
#include "bsp_adc.h"
#include "./led/bsp_led.h"
#include "./pid/pid.h"
#include "./basetime/bsp_basetime.h"
#include "./generaltim/bsp_generaltim.h"


volatile uint32_t time = 0; // ms ???? 

// 软件延时
void Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
} 

/**
  * @brief  主函数
  * @param  无
  * @retval 无
  */
int main(void)
{	
	LED_GPIO_Config();
	USART_Config();
	ADCx_Init();
	BASIC_TIM_Init();
	GENERAL_TIM_Init();
	
	while (1)
	{
		//GPIO_SetBits(LED1_GPIO_PORT, LED1_GPIO_PIN);
		//GPIO_ResetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);
		
		if(time==100)
		{
			time = 0;	
			temperature_sampling();		
		}
		
	}
}
/*********************************************END OF FILE**********************/

