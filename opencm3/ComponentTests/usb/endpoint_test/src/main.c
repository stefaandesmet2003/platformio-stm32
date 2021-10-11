/* 
 * simpele test om te zien hoe ISTR en endpoint registers werken
 * met het python script kan je packet per packet sturen
 * en dan met de debugger kijken wat er gebeurt
 * geleerd dat als er gelijktijdig een pending interrupt is op 2 endpoints,
 * dan krijgt het endpoint met het laagste nummer (EP_ID in ISTR, dus zonder de DIR bit!) 
 * de prioriteit. De interrupt van het andere endpoint gaat niet verloren
 * want CTR_RX /CTR_TX blijft gezet. 
 * Als het ene endpoint is afgehandeld, wordt ISTR geupdate, en opnieuw CTR=1,
 * maar nu met het andere EP_ID
 * er gaan dus geen ints verloren, ze kunnen enkel in een onverwachte volgorde
 * aangeboden worden.
 * 
 */
#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
// systick & inthandler
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
// uart printf
#include <libopencm3/stm32/usart.h>
#include <stdio.h>
#include <libopencm3/usb/usbd.h>

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

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

static const struct usb_device_descriptor dev = {
  .bLength = USB_DT_DEVICE_SIZE,
  .bDescriptorType = USB_DT_DEVICE,
  .bcdUSB = 0x0200,
  .bDeviceClass = 0,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0,
  .bMaxPacketSize0 = 64,
  .idVendor = 0xcafe, // whatever
  .idProduct = 0xcafe, // whatever
  .bcdDevice = 0x0100,
  .iManufacturer = 1,
  .iProduct = 2,
  .iSerialNumber = 3,
  .bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor data_endp[] = {{
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  //.bEndpointAddress = 0x81,
  .bEndpointAddress = 0x82, // ook descriptors aanpassen voor OUT=1,IN=0x82
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,
  .wMaxPacketSize = 64,
  .bInterval = 0,
}, {
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  //.bEndpointAddress = 0x02,
  .bEndpointAddress = 0x01, // ook descriptors aanpassen voor OUT=1,IN=0x82
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,
  .wMaxPacketSize = 64,
  .bInterval = 0,
}, {
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  .bEndpointAddress = 0x83,
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,
  .wMaxPacketSize = 64,
  .bInterval = 0,
}};

static const struct usb_interface_descriptor data_iface[] = {{
  .bLength = USB_DT_INTERFACE_SIZE,
  .bDescriptorType = USB_DT_INTERFACE,
  .bInterfaceNumber = 0,
  .bAlternateSetting = 0,
  .bNumEndpoints = 3,
  .bInterfaceClass = 255, // vendor specific class
  .bInterfaceSubClass = 255, // vendor specific subclass
  .bInterfaceProtocol = 255, // vendor specific protocol
  .iInterface = 4,

  .endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {
  {
  .num_altsetting = 1,
  .altsetting = data_iface,
  }
};

static const struct usb_config_descriptor config = {
  .bLength = USB_DT_CONFIGURATION_SIZE,
  .bDescriptorType = USB_DT_CONFIGURATION,
  .wTotalLength = 0, // filled in by usb_standard.c
  .bNumInterfaces = 1,
  .bConfigurationValue = 1,
  .iConfiguration = 0,
  .bmAttributes = 0x80,
  .bMaxPower = 0x64,

  .interface = ifaces,
};

static const char *usb_strings[] = {
  "SDS BUG FACTORY",
  "bugproduct",
  "bugserial",
  "buginterface",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

/* endpoint buffer */
uint8_t ep_buffer[64];

/* */
uint8_t tx_counter; // counts the number of times the TX callback is called
uint8_t rx_counter; // counts the number of times the TX callback is called
uint32_t delay_value; // sometimes we will delay usbd_poll and see what happens

// got IN transaction
static void ep_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
  (void)ep;
  (void)usbd_dev;
  tx_counter++;
  printf ("tx #%d, rx %d\n",tx_counter,rx_counter);
} // ep_tx_cb

// got OUT transaction
static void ep_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
  (void)ep;
  (void)usbd_dev;

  rx_counter++;
  //int len = usbd_ep_read_packet(usbd_dev, 0x02, ep_buffer, 64);
  // ep addresses aanpassen voor OUT=1,IN=0x82
  int len = usbd_ep_read_packet(usbd_dev, 0x01, ep_buffer, 64);
  ep_buffer[0] = tx_counter;
  // ep addresses aanpassen voor OUT=1,IN=0x82
  //usbd_ep_write_packet(usbd_dev, 0x81, ep_buffer, 1);
  usbd_ep_write_packet(usbd_dev, 0x82, ep_buffer, 1);
  /*
  if (rx_counter % 2) {
    delay_value = 100;
  }
  else {
    delay_value = 0;
  }
  */
  printf ("rx #%d, %d bytes, tx_cnt:%d, delay:%ld\n",rx_counter,len,tx_counter,delay_value);

} // ep_rx_cb

static void stlink_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
  (void)wValue;
  (void)usbd_dev;

/*
  // met IN endpoint = highest priority
  usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, ep_tx_cb);
  usbd_ep_setup(usbd_dev, 0x02, USB_ENDPOINT_ATTR_BULK, 64, ep_rx_cb);
*/
  // met OUT endpoint = highest priority, zoals in msctest
  usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, ep_tx_cb);
  usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, ep_rx_cb);
}

int main(void)
{
  usbd_device *usbd_dev;
  uint32_t blinkMillis;

  // sds : blijkbaar loopt platformio achter met opencm3, want deze func bestaat nog niet in package/framework-libopencm3
  // na manual upgrade van framework-opencm3 werkt dit ok (git clone van opencm3 in packages/framework-opencm3 gedaan, en .piopm + package.json gecopieerd)
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
  // en in latest opencm3 is rcc_clock_setup_in_hse_8mhz_out_72mhz al deprecated ..
  // voor outdated opencm3 package
  // rcc_clock_setup_in_hse_8mhz_out_72mhz();

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOC);

  systick_setup();
  usart_setup();
  printf("Endpoint Test\n");
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
  usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
  usbd_register_set_config_callback(usbd_dev, stlink_set_config);

  while (1) {
    usbd_poll(usbd_dev);
    delay(delay_value);

    // blinkie
    if ((millis() - blinkMillis) > 500){
      blinkMillis = millis();
      gpio_toggle(GPIOC, GPIO13); // toggle led
    }
  }

} // main
