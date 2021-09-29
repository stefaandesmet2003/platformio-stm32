/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Copyright (C) 2012 Karl Palsson <karlp@tweak.net.au>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/scb.h>

// blinkie on PC13 (builtin led on bluepill)
// trigger a reboot in dfu mode by setting PA10 to ground

static void gpio_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOC);
	gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

	rcc_periph_clock_enable(RCC_GPIOA);
	gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO10);
	gpio_set(GPIOA,GPIO10);

}


// dit is hoe een app een reboot moet doen in DFU mode, zonder dat de BOOT1 pin moet gezet worden
// de GPIO_CRL wordt gebruikt als een cookie
// deze functie wordt in de bootloader niet gebruikt natuurlijk
static void platform_request_boot(void)
{
  uint32_t crl = GPIOA_CRL;
  /* Assert bootloader marker.
   * Enable Pull on GPIOA1. We don't rely on the external pin
   * really pulled, but only on the value of the CNF register
   * changed from the reset value
   */
  crl &= 0xffffff0f;
  crl |= 0x80;
  GPIOA_CRL = crl;
  SCB_VTOR = 0;
  scb_reset_core();
}


int main(void)
{
	uint32_t i;

  // clock setup
	// met de volgende line commented werken we op de default HSI, is voldoende voor een blinkie
  //rcc_clock_setup_in_hse_8mhz_out_72mhz();

	gpio_setup();
	/* Blink the LED on the board. */
	while (1) {
		/* Using API function gpio_toggle(): */
		gpio_toggle(GPIOC, GPIO13);	/* LED on/off */
		for (i = 0; i < 10000000; i++) {	/* Wait a bit. */
			__asm__("nop");
		}
		if (!gpio_get(GPIOA,GPIO10)) {
			platform_request_boot();
		}
	}

	return 0;
}
