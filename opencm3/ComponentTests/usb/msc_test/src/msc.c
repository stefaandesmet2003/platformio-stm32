/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2013 Weston Schmidt <weston_schmidt@alumni.purdue.edu>
 * Copyright (C) 2013 Pavol Rusnak <stick@gk2.sk>
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

// plug in en toont een 524kB volume met een 2kB file ramdisk.dat
/* zo toont het device zich : 
[15988.607657] usb 1-1: new full-speed USB device number 23 using xhci_hcd
[15988.757406] usb 1-1: New USB device found, idVendor=0483, idProduct=5741, bcdDevice= 2.00
[15988.757428] usb 1-1: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[15988.757438] usb 1-1: Product: MSC Demo
[15988.757445] usb 1-1: Manufacturer: Black Sphere Technologies
[15988.757453] usb 1-1: SerialNumber: DEMO
[15988.762090] usb-storage 1-1:1.0: USB Mass Storage device detected
[15988.763570] scsi host2: usb-storage 1-1:1.0
[15989.780341] scsi 2:0:0:0: Direct-Access     VendorID ProductID        0.00 PQ: 0 ANSI: 4
[15989.781477] sd 2:0:0:0: Attached scsi generic sg2 type 0
[15989.781996] sd 2:0:0:0: [sdb] 1024 512-byte logical blocks: (524 kB/512 KiB)
[15989.782222] sd 2:0:0:0: [sdb] Write Protect is off
[15989.782236] sd 2:0:0:0: [sdb] Mode Sense: 03 00 00 ff
[15989.782462] sd 2:0:0:0: [sdb] No Caching mode page found
[15989.782483] sd 2:0:0:0: [sdb] Assuming drive cache: write through
[15989.806844]  sdb:
[15989.833188] sd 2:0:0:0: [sdb] Attached SCSI removable disk
*/

#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

// voor de systick & de inthandler
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

// uart printf
#include <libopencm3/stm32/usart.h>
#include <stdio.h>

// msc example
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/msc.h>
#include "ramdisk.h"


/*****************************************************************************/
/* millis                                                                    */
/*****************************************************************************/
// iets simpels om millis time keeping te hebben
volatile uint32_t _millis = 0;

// sysclk 72MHz
static void systick_setup(void) {
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
  // 9MHz -> 9000000 counts per second
	/* 9000000/9000 = 1000 overflows per second - every 1ms one interrupt */
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(8999);

	systick_interrupt_enable();

	/* Start counting. */
	systick_counter_enable();
}

static uint32_t millis(void) {
	return _millis;
}

void sys_tick_handler(void)
{
	_millis++;
}

// busyloop
static void delay(uint32_t ms) {
  uint32_t delayMillis = millis();
  while (millis()-delayMillis < ms);
}

/*****************************************************************************/
/* uart printf                                                               */
/*****************************************************************************/
static void usart_setup(void)
{
  /* Enable clocks for USART1. */
  rcc_periph_clock_enable(RCC_USART1);

  /* Setup GPIO pin GPIO_USART1_TX */
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

  /* Setup GPIO pin GPIO_USART1_RX  */
  // is eigenlijk de GPIO reset waarde, dus overbodig hier
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

  /* Setup USART1 parameters. */
  usart_set_baudrate(USART1, 115200);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX_RX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

  /* Finally enable the USART. */
  usart_enable(USART1);
}

int _write(int fd, char *ptr, int len) {
  int i = 0;

  /*
   * Write "len" of char from "ptr" to file id "fd"
   * Return number of char written.
   *
   * Only work for STDOUT, STDIN, and STDERR
   */
  if (fd > 2) {
    return -1;
  }
  while (*ptr && (i < len)) {
    usart_send_blocking(USART1, *ptr);
    if (*ptr == '\n') {
      usart_send_blocking(USART1, '\r');
    }
    i++;
    ptr++;
  }
  return i;
}
/*****************************************************************************/
/* usb descriptors                                                           */
/*****************************************************************************/

static const struct usb_device_descriptor dev_descr = {
  .bLength = USB_DT_DEVICE_SIZE,
  .bDescriptorType = USB_DT_DEVICE,
  .bcdUSB = 0x0110,
  .bDeviceClass = 0,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0,
  .bMaxPacketSize0 = 64,
  .idVendor = 0x0483,
  .idProduct = 0x5741,
  .bcdDevice = 0x0200,
  .iManufacturer = 1,
  .iProduct = 2,
  .iSerialNumber = 3,
  .bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor msc_endp[] = {{
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  .bEndpointAddress = 0x01,
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,
  .wMaxPacketSize = 64,
  .bInterval = 0,
}, {
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  .bEndpointAddress = 0x82,
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,
  .wMaxPacketSize = 64,
  .bInterval = 0,
}};

static const struct usb_interface_descriptor msc_iface[] = {{
  .bLength = USB_DT_INTERFACE_SIZE,
  .bDescriptorType = USB_DT_INTERFACE,
  .bInterfaceNumber = 0,
  .bAlternateSetting = 0,
  .bNumEndpoints = 2,
  .bInterfaceClass = USB_CLASS_MSC,
  .bInterfaceSubClass = USB_MSC_SUBCLASS_SCSI,
  .bInterfaceProtocol = USB_MSC_PROTOCOL_BBB,
  .iInterface = 0,
  .endpoint = msc_endp,
  .extra = NULL,
  .extralen = 0
}};

static const struct usb_interface ifaces[] = {{
  .num_altsetting = 1,
  .altsetting = msc_iface,
}};

static const struct usb_config_descriptor config_descr = {
  .bLength = USB_DT_CONFIGURATION_SIZE,
  .bDescriptorType = USB_DT_CONFIGURATION,
  .wTotalLength = 0,
  .bNumInterfaces = 1,
  .bConfigurationValue = 1,
  .iConfiguration = 0,
  .bmAttributes = 0x80,
  .bMaxPower = 0x32,
  .interface = ifaces,
};

static const char *usb_strings[] = {
  "Black Sphere Technologies",
  "MSC Demo",
  "DEMO",
};

static usbd_device *msc_dev;
/* Buffer to be used for control requests. */
static uint8_t usbd_control_buffer[128];

int main(void)
{
  uint32_t blinkMillis;
  // sds : blijkbaar loopt platformio achter met opencm3, want deze func bestaat niet in package-opencm3
  // na manual upgrade van framework-opencm3 werkt dit ok (git clone van opencm3 in packages/framework-opencm3 gedaan, en .piopm + package.json gecopieerd)
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
  // en in latest opencm3 is rcc_clock_setup_in_hse_8mhz_out_72mhz al deprecated ..
  // voor outdated opencm3 package
  //rcc_clock_setup_in_hse_8mhz_out_72mhz();

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOC);

  systick_setup();
  usart_setup();
  printf("\nUSB Mass Storage Example.\n");
  printf("rcc_ahb_frequency = %lu\n",rcc_ahb_frequency);
  printf("rcc_apb1_frequency = %lu\n",rcc_apb1_frequency);
  printf("rcc_apb2_frequency = %lu\n",rcc_apb2_frequency);


  /* Setup GPIOC Pin 13 for the LED -> sds bluepill */
  gpio_set(GPIOC, GPIO13);
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

  /* Disconnect USB after reset:
	 * Pull USB_DP low. Device will reconnect automatically
	 * when USB is set up later, as Pull-Up is hard wired*/
	gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
				  GPIO_CNF_OUTPUT_OPENDRAIN, GPIO12);
	gpio_clear(GPIOA, GPIO12);

  for (unsigned i = 0; i < 800000; i++) {
    __asm__("nop");
  }  

  // rcc_periph_clock_enable(RCC_USB); // -> dat gebeurt al in de driver init
  // usbd takes over the A11/A12 pins from here
	msc_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev_descr, &config_descr, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));

  ramdisk_init();
  usb_msc_init(msc_dev, 0x82, 64, 0x01, 64, "VendorID", "ProductID",
    "0.00", ramdisk_blocks(), ramdisk_read, ramdisk_write);

  for (;;) {
    usbd_poll(msc_dev);

    // beetje delay, maar niet te veel anders lukt enumeration niet
    // jongens, pallsen doe eens iets man
    for (unsigned i = 0; i < 500; i++) {
      __asm__("nop");
    }  
    // dit werkt niet, mijn issue https://github.com/libopencm3/libopencm3/issues/1362
    // delay(1);

    // blinkie
    if ((millis() - blinkMillis) > 500){
      blinkMillis = millis();
      gpio_toggle(GPIOC, GPIO13); // toggle led
    }
  }
}
