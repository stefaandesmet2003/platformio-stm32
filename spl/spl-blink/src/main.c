#ifdef STM32L1
	#include <stm32l1xx_gpio.h>
	#include <stm32l1xx_rcc.h>
	#define LEDPORT (GPIOB)
	#define LEDPIN (GPIO_Pin_7)
	#define ENABLE_GPIO_CLOCK (RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE))
#elif STM32F3
	#include <stm32f30x_gpio.h>
	#include <stm32f30x_rcc.h>
	#define LEDPORT (GPIOE)
	#define LEDPIN (GPIO_Pin_8)
	#define ENABLE_GPIO_CLOCK (RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOE, ENABLE))
#elif STM32F4
	#include <stm32f4xx_gpio.h>
	#include <stm32f4xx_rcc.h>
	#define LEDPORT (GPIOD)
	#define LEDPIN (GPIO_Pin_12)
	#define ENABLE_GPIO_CLOCK (RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE))
#elif STM32F1
	// sds added for STM32F103 bluepill
	#include <stm32f10x_gpio.h>
	#include <stm32f10x_rcc.h>
	#define LEDPORT (GPIOC)
	#define LEDPIN (GPIO_Pin_13)
	#define ENABLE_GPIO_CLOCK (RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE))
#endif

/* timing is not guaranteed :) */
void simple_delay(uint32_t us)
{
	/* simple delay loop */
	while (us--) {
		asm volatile ("nop");
	}
}

/* system entry point */
int main(void)
{
	/* gpio init struct */
	GPIO_InitTypeDef gpio;
	/* reset rcc */
	RCC_DeInit();
	/* enable clock GPIO */
	ENABLE_GPIO_CLOCK;
	/* use LED pin */
	gpio.GPIO_Pin = LEDPIN;
	/* mode: output */
	#ifdef STM32F1
		/* output type: push-pull */
		gpio.GPIO_Mode = GPIO_Mode_Out_PP; // sds : SPL v 3.5.0
		gpio.GPIO_Speed = GPIO_Speed_50MHz;
	#else
		gpio.GPIO_Mode = GPIO_Mode_OUT;
		/* output type: push-pull */
		gpio.GPIO_OType = GPIO_OType_PP;
	#endif
	/* apply configuration */
	GPIO_Init(LEDPORT, &gpio);
	/* main program loop */
	for (;;) {
		/* set led on */
		GPIO_SetBits(LEDPORT, LEDPIN);
		/* delay */
		simple_delay(100000);
		/* clear led */
		GPIO_ResetBits(LEDPORT, LEDPIN);
		/* delay */
		simple_delay(100000);
	}

	/* never reached */
	return 0;
}