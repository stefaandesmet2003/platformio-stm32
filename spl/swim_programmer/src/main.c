/**
*****************************************************************************
**
**  File        : main.c
**
**  Abstract    : main function.
**
**  Functions   : main
**
**  Environment : Atollic TrueSTUDIO/STM32
**                STMicroelectronics STM32F10x Standard Peripherals Library
**
**  Distribution: The file is distributed �as is,� without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
**
*****************************************************************************
*/

/* Includes */
#include <stddef.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"
#include "swim.h"

	// sds added for STM32F103 bluepill
	#include <stm32f10x_gpio.h>
	#include <stm32f10x_rcc.h>
	#define LEDPORT (GPIOC)
	#define LEDPIN (GPIO_Pin_13)
	#define ENABLE_GPIO_CLOCK (RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE))

/* timing is not guaranteed :) */
void simple_delay(uint32_t us)
{
	/* simple delay loop */
	while (us--) {
		asm volatile ("nop");
	}
}

void show_blinks (uint32_t blinks) {
	if (blinks) {
		for (uint8_t i=0; i<blinks; i++) {
			/* set led on */
			GPIO_ResetBits(LEDPORT, LEDPIN);
			/* delay */
			simple_delay(1000000);
			/* clear led */
			GPIO_SetBits(LEDPORT, LEDPIN);
			/* delay */
			simple_delay(1000000);
		}
		simple_delay(5000000);
	}
}

void setup_blinkie() {
	/* gpio init struct */
	GPIO_InitTypeDef gpio;
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
	GPIO_ResetBits(LEDPORT, LEDPIN);
}

int main(void)
{
	uint8_t data[4];
	uint32_t ret1, ret2, ret3, ret4;
	uint32_t retval,blinks;
	blinks = 0;

	setup_blinkie();

	// komt uit dwt_init(), anders werkt dwt niet in release mode
	// in debug doet blijkbaar de debugger de DWT init 

 /* Enable use of DWT */
  if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  }

  /* Unlock */
  //dwt_access(true); // niet nodig op stm32f103

  /* Reset the clock cycle counter value */
  DWT->CYCCNT = 0;

  /* Enable  clock cycle counter */
  DWT->CTRL |=  DWT_CTRL_CYCCNTENA_Msk;

  /* 3 NO OPERATION instructions */
  __asm volatile(" nop      \n\t"
                 " nop      \n\t"
                 " nop      \n\t");

  /* Check if clock cycle counter has started */
	if (!DWT->CYCCNT) {
		show_blinks(16);
	}

	retval = swim_init();
	if (retval != SWIM_ERR_OK) {
		blinks = retval;
		for (;;) {
			show_blinks(blinks);
		}
	}
	
	data[0] = SWIM_FLASH_PUKR_KEY1;
	retval = swim_write_on_the_fly(1, SWIM_FLASH_PUKR, data);
	if (retval != SWIM_ERR_OK)
		blinks = 12;

	simple_delay(20000);

	data[0] = SWIM_FLASH_PUKR_KEY2;
	retval = swim_write_on_the_fly(1, SWIM_FLASH_PUKR, data);
	if (retval != SWIM_ERR_OK)
		blinks = 13;

	simple_delay(20000);

	data[0] = 0xFF;
	retval = swim_write_on_the_fly(1, SWIM_PROGRAM + 0x200, data);
	if (retval != SWIM_ERR_OK)
		blinks = 14;

	/*
	if((ret1 = swim_init()) == SWIM_ERR_OK)	//init
	{
		data[0] = SWIM_FLASH_PUKR_KEY1;
		if((ret2 = swim_write_on_the_fly(1, SWIM_FLASH_PUKR, data)) == SWIM_ERR_OK)	//write pukr1
		{
			data[0] = SWIM_FLASH_PUKR_KEY2;
			if((ret3 = swim_write_on_the_fly(1, SWIM_FLASH_PUKR, data)) == SWIM_ERR_OK) //write pukr2
			{
				data[0] = 0xFF;
				ret4 = swim_write_on_the_fly(1, SWIM_PROGRAM + 0x200, data);	//write data (one byte to the right address)
			}
		}
	}
	else
	{
		ret1 = ret1;
	}
	*/
	/* dit doet een system reset op STM8
	 * omdat SWIM_CSR.RST=0, blijft swim actief na de reset, en dus CPU blijft gestalled !
	 * op de logic analyzer zie je dat STM8 RST laag trekt (STM32 zet die pin nochtans hoog!), 135us na het SWIM command, en dit voor ongeveer 600us
	 */
	/*
	retval = swim_system_reset();
	if (retval != SWIM_ERR_OK)
		blinks = 15;	
	simple_delay(20000);
	*/

	// dit werkt!
	// na swim-entry is CPU stalled, en blijft tot je dit doet, ofwel een reset
	/*
	data[0] = 0;
	retval = swim_write_on_the_fly(1, DM_CSR2, data); // reset stall bit -> cpu terug running!
	if (retval != SWIM_ERR_OK)
		blinks = 16;
	*/

	// dit zou het device een reset moeten geven, zonder swim, dus een normale opstart
	// maar CPU blijft gestalled, dus is niet wat je wil
	/*
	RESET_ASSERT();
	simple_delay(20000);
	RESET_DEASSERT();
	*/
	// dit werkt ook.
	data[0] = 0xA4; // SWIM_CSR.RST=1, als we dan een reset doen, wordt SWIM ook gereset, en niet meer geactiveerd dus
	swim_write_on_the_fly(1, SWIM_CSR, data);
	swim_system_reset();


	// while(1);
	// vervangen door een blinkie
	for (;;) {
		if (blinks) {
			for (uint8_t i=0; i<blinks; i++) {
				/* set led on */
				GPIO_ResetBits(LEDPORT, LEDPIN);
				/* delay */
				simple_delay(1000000);
				/* clear led */
				GPIO_SetBits(LEDPORT, LEDPIN);
				/* delay */
				simple_delay(1000000);
			}
			simple_delay(5000000);
		}
		else {
			/* set led on */
			GPIO_SetBits(LEDPORT, LEDPIN);
			/* delay */
			simple_delay(500000);
			/* clear led */
			GPIO_ResetBits(LEDPORT, LEDPIN);
			/* delay */
			simple_delay(500000);
		}
	}

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
  {
  }
}
#endif

/*
 * Minimal __assert_func used by the assert() macro
 * */
void __assert_func(const char *file, int line, const char *func, const char *failedexpr)
{
  while(1)
  {}
}

/*
 * Minimal __assert() uses __assert__func()
 * */
void __assert(const char *file, int line, const char *failedexpr)
{
   __assert_func (file, line, NULL, failedexpr);
}

#ifdef USE_SEE
#ifndef USE_DEFAULT_TIMEOUT_CALLBACK
/**
  * @brief  Basic management of the timeout situation.
  * @param  None.
  * @retval sEE_FAIL.
  */
uint32_t sEE_TIMEOUT_UserCallback(void)
{
  /* Return with error code */
  return sEE_FAIL;
}
#endif
#endif /* USE_SEE */

