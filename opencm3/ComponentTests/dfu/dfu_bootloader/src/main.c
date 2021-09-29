/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2013 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>

#include "usbdfu.h"

uint32_t app_address = 0x08002000;

static void jump_app_if_valid(void)
{
  /* Boot the application if it's valid */
  if((*(volatile uint32_t*)app_address & 0x2FFE0000) == 0x20000000) {
    /* Set vector table base address */
    SCB_VTOR = app_address & 0x1FFFFF; /* Max 2 MByte Flash*/
    /* Initialise master stack pointer */
    asm volatile ("msr msp, %0"::"g"
        (*(volatile uint32_t*)app_address));
    /* Jump to application */
    (*(void(**)())(app_address + 4))();
  }
}

int main(void)
{
  /* Check the force bootloader pin*/
  bool normal_boot = 0;

  // clock setup
  rcc_clock_setup_in_hse_8mhz_out_72mhz();
  rcc_periph_clock_enable(RCC_GPIOA); // using A1 as cookie
  rcc_periph_clock_enable(RCC_GPIOB); // B2 = BOOT1
  rcc_periph_clock_enable(RCC_GPIOC); // C13 = led

  gpio_set_mode(GPIOB, GPIO_MODE_INPUT,
          GPIO_CNF_INPUT_FLOAT, GPIO2);
  normal_boot = !(gpio_get(GPIOB, GPIO2));

  gpio_set(GPIOC, GPIO13); /* LED on Blupill is active low!*/
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
          GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

  /* Disconnect USB after reset:
   * Pull USB_DP low. Device will reconnect automatically
   * when USB is set up later, as Pull-Up is hard wired*/
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
          GPIO_CNF_OUTPUT_OPENDRAIN, GPIO12);
  gpio_clear(GPIOA, GPIO12);
  rcc_periph_reset_pulse(RST_USB);
  rcc_periph_clock_enable(RCC_USB);

  if(((GPIOA_CRL & 0x40) == 0x40) && normal_boot)
    jump_app_if_valid();

  dfu_protect(false);


  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
  systick_set_reload(900000);

  systick_interrupt_enable();
  systick_counter_enable();

  dfu_init(&st_usbfs_v1_usb_driver);

  dfu_main();
}

void dfu_event(void)
{
}

void dfu_detach(void)
{
  /* Disconnect USB cable by resetting USB Device
     and pulling USB_DP low*/
  rcc_periph_reset_pulse(RST_USB);
  rcc_periph_clock_enable(RCC_USB);
  rcc_periph_clock_enable(RCC_GPIOA);
  gpio_clear(GPIOA, GPIO12);
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
    GPIO_CNF_OUTPUT_OPENDRAIN, GPIO12);
  scb_reset_system();
}

void sys_tick_handler(void)
{
  gpio_toggle(GPIOC, GPIO13);
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
