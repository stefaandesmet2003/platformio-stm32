/* 
 *
 * test of printf redirect to usb
 * can we print when usb is not connected?
 *   NO, as long as endpoints are not configured, usbd_ep_write_packet to 0x82 will result in a hard fault

 *   after the device is configured, data can be printed but may be lost if the TX endpoint is not read by the host (active serial monitor)
 * if the device is connected, but no serial monitor open:
 *   the first usbd_ep_write_packet will prepare a packet to be sent
 *   the packet will not be sent until a serial monitor on the host is reading the endpoint
 *   subsequent calls to usbd_ep_write_packet will return 0 
 *   because TX_STAT is invalid until the endpoint has had the chance to transmit the previous packet.
 
 * on usb disconnect, the device remains configured
     usbd_ep_write_packet will return 0 (again because endpoint is not read by host)
 * 
 * on reconnect host will issue a usb reset, this resets all previous endpoint configuration
 * 
 * note : the platformio serial monitor sends "\r\n" on opening
 * -> the CDC driver must process the rx data, otherwise the rx_callback keeps coming back because the endpoint is not read.
 * -> a dummy endpoint read is implemented here
 * led indication : 
 *  slow blink : usb not configured (before first usb connection)
 *  normal blink : usb configured but no active host reading from the TX endpoint
 *  fast blink : active host reading from TX endpoint (serial monitor opened)
 */ 

#include <stdlib.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

// voor de systick & de inthandler
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#ifdef UART_PRINTF
// uart printf
#include <libopencm3/stm32/usart.h>
#endif
#include <stdio.h>

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#define SLOW_BLINK    1000 // no usb connection
#define NORMAL_BLINK  500  // configured usb device
#define FAST_BLINK    250  // device connected to an active monitor, reading from our TX endpoint
static usbd_device *usbSerial; // the usbSerial
static bool isConfigured; // need this to avoid _write to unconfigured endpoint
static uint32_t blinkPeriodMillis = SLOW_BLINK;

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

#ifdef UART_PRINTF
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
#endif

// de uart versie verving \n door \r, hier niet, want send is niet char-based, en geen zin om te parsen.
int _write(int fd, char *ptr, int len) {
  uint16_t retval;

  /*
   * Write "len" of char from "ptr" to file id "fd"
   * Return number of char written.
   *
   * Only work for STDOUT, STDIN, and STDERR
   */
  if (fd > 2) {
    return -1;
  }
  if (!isConfigured)
    return 0; // or -1?

  // make sure endpoints are configured before calling usbd_ep_write_packet
  // otherwise we end up in a hard fault access 0
  retval = usbd_ep_write_packet(usbSerial, 0x82, ptr, len);
  usbd_poll(usbSerial);
  if (retval == len) {
    blinkPeriodMillis = FAST_BLINK; // indicate successful TX
    return len;
  }

  blinkPeriodMillis = NORMAL_BLINK; // nothing transmitted (endpoint is not read by host)
  return -1;
}

/*****************************************************************************/
/* usb descriptors                                                           */
/*****************************************************************************/

static const struct usb_device_descriptor dev = {
  .bLength = USB_DT_DEVICE_SIZE,
  .bDescriptorType = USB_DT_DEVICE,
  .bcdUSB = 0x0200,
  .bDeviceClass = USB_CLASS_CDC,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0,
  .bMaxPacketSize0 = 64,
  .idVendor = 0x0483,
  .idProduct = 0x5740,
  .bcdDevice = 0x0200,
  .iManufacturer = 1,
  .iProduct = 2,
  .iSerialNumber = 3,
  .bNumConfigurations = 1,
};

/*
 * This notification endpoint isn't implemented. According to CDC spec its
 * optional, but its absence causes a NULL pointer dereference in Linux
 * cdc_acm driver.
 */
static const struct usb_endpoint_descriptor comm_endp[] = {{
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  .bEndpointAddress = 0x83,
  .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
  .wMaxPacketSize = 16,
  .bInterval = 255,
}};

static const struct usb_endpoint_descriptor data_endp[] = {{
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  .bEndpointAddress = 0x01,
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,
  .wMaxPacketSize = 64,
  .bInterval = 1,
}, {
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  .bEndpointAddress = 0x82,
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,
  .wMaxPacketSize = 64,
  .bInterval = 1,
}};

static const struct {
  struct usb_cdc_header_descriptor header;
  struct usb_cdc_call_management_descriptor call_mgmt;
  struct usb_cdc_acm_descriptor acm;
  struct usb_cdc_union_descriptor cdc_union;
} __attribute__((packed)) cdcacm_functional_descriptors = {
  .header = {
    .bFunctionLength = sizeof(struct usb_cdc_header_descriptor),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_HEADER,
    .bcdCDC = 0x0110,
  },
  .call_mgmt = {
    .bFunctionLength =
      sizeof(struct usb_cdc_call_management_descriptor),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_CALL_MANAGEMENT,
    .bmCapabilities = 0,
    .bDataInterface = 1,
  },
  .acm = {
    .bFunctionLength = sizeof(struct usb_cdc_acm_descriptor),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_ACM,
    .bmCapabilities = 0,
  },
  .cdc_union = {
    .bFunctionLength = sizeof(struct usb_cdc_union_descriptor),
    .bDescriptorType = CS_INTERFACE,
    .bDescriptorSubtype = USB_CDC_TYPE_UNION,
    .bControlInterface = 0,
    .bSubordinateInterface0 = 1,
   },
};

static const struct usb_interface_descriptor comm_iface[] = {{
  .bLength = USB_DT_INTERFACE_SIZE,
  .bDescriptorType = USB_DT_INTERFACE,
  .bInterfaceNumber = 0,
  .bAlternateSetting = 0,
  .bNumEndpoints = 1,
  .bInterfaceClass = USB_CLASS_CDC,
  .bInterfaceSubClass = USB_CDC_SUBCLASS_ACM,
  .bInterfaceProtocol = USB_CDC_PROTOCOL_AT,
  .iInterface = 0,

  .endpoint = comm_endp,

  .extra = &cdcacm_functional_descriptors,
  .extralen = sizeof(cdcacm_functional_descriptors),
}};

static const struct usb_interface_descriptor data_iface[] = {{
  .bLength = USB_DT_INTERFACE_SIZE,
  .bDescriptorType = USB_DT_INTERFACE,
  .bInterfaceNumber = 1,
  .bAlternateSetting = 0,
  .bNumEndpoints = 2,
  .bInterfaceClass = USB_CLASS_DATA,
  .bInterfaceSubClass = 0,
  .bInterfaceProtocol = 0,
  .iInterface = 0,

  .endpoint = data_endp,
}};

static const struct usb_interface ifaces[] = {{
  .num_altsetting = 1,
  .altsetting = comm_iface,
}, {
  .num_altsetting = 1,
  .altsetting = data_iface,
}};

static const struct usb_config_descriptor config = {
  .bLength = USB_DT_CONFIGURATION_SIZE,
  .bDescriptorType = USB_DT_CONFIGURATION,
  .wTotalLength = 0,
  .bNumInterfaces = 2,
  .bConfigurationValue = 1,
  .iConfiguration = 0,
  .bmAttributes = 0x80,
  .bMaxPower = 0x32,

  .interface = ifaces,
};

static const char *usb_strings[] = {
  "Black Sphere Technologies",
  "CDC-ACM Demo",
  "DEMO",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf,
    uint16_t *len, void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
  (void)complete;
  (void)buf;
  (void)usbd_dev;

  switch (req->bRequest) {
  case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
    /*
     * This Linux cdc_acm driver requires this to be implemented
     * even though it's optional in the CDC spec, and we don't
     * advertise it in the ACM functional descriptor.
     */
    char local_buf[10];
    struct usb_cdc_notification *notif = (void *)local_buf;

    /* We echo signals back to host as notification. */
    notif->bmRequestType = 0xA1;
    notif->bNotification = USB_CDC_NOTIFY_SERIAL_STATE;
    notif->wValue = 0;
    notif->wIndex = 0;
    notif->wLength = 2;
    local_buf[8] = req->wValue & 3;
    local_buf[9] = 0;
    // usbd_ep_write_packet(0x83, buf, 10);
    return USBD_REQ_HANDLED;
    }
  case USB_CDC_REQ_SET_LINE_CODING:
    if (*len < sizeof(struct usb_cdc_line_coding))
      return USBD_REQ_NOTSUPP;
    return USBD_REQ_HANDLED;
  }
  return USBD_REQ_NOTSUPP;
}

static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
  (void)ep;

  // niet gebruikt, maar we moeten wel ep lezen, anders blijft de callback komen
  char buf[64]; // ep-size=64, dus zoveel data kunnen we ineens krijgen
  int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

  // om iets te doen met len
  if (len)
    gpio_toggle(GPIOC, GPIO13);
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
  (void)wValue;
  (void)usbd_dev;

  usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);
  usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);
  usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

  usbd_register_control_callback(
        usbd_dev,
        USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
        cdcacm_control_request);

  isConfigured = true; // from now usbd_ep_write_packet can be done without ending up in a hardfault handler
  blinkPeriodMillis = NORMAL_BLINK;
  
}

static void usb_reset_cb (void) {
  // usb peripheral will auto-reset its endpoints, so we have to wait another config before using _write!
  isConfigured = false;
  blinkPeriodMillis = SLOW_BLINK;
}

int main(void)
{
  uint32_t blinkMillis;
  uint32_t printMillis;

  // sds : blijkbaar loopt platformio achter met opencm3, want deze func bestaat niet in package-opencm3
  // na manual upgrade van framework-opencm3 werkt dit ok (git clone van opencm3 in packages/framework-opencm3 gedaan, en .piopm + package.json gecopieerd)
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
  // en in latest opencm3 is rcc_clock_setup_in_hse_8mhz_out_72mhz al deprecated ..
  // voor outdated opencm3 package
  //rcc_clock_setup_in_hse_8mhz_out_72mhz();

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOC);

  systick_setup();
  #ifdef UART_PRINTF
    usart_setup();
    printf("\nUSB printf Example.\n");
    printf("rcc_ahb_frequency = %lu\n",rcc_ahb_frequency);
    printf("rcc_apb1_frequency = %lu\n",rcc_apb1_frequency);
    printf("rcc_apb2_frequency = %lu\n",rcc_apb2_frequency);  
  #endif

  /* Setup GPIOC Pin 13 for the LED -> sds bluepill */
  gpio_set(GPIOC, GPIO13);
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
          GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

  /* Disconnect USB after reset:
   * Pull USB_DP low. Device will reconnect automatically
   * when USB is set up later, as Pull-Up is hard wired*/
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
          GPIO_CNF_OUTPUT_OPENDRAIN, GPIO12);
  gpio_clear(GPIOA, GPIO12);

  for (unsigned i = 0; i < 800000; i++) {
    __asm__("nop");
  }  

  usbSerial = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
  usbd_register_set_config_callback(usbSerial, cdcacm_set_config);
  usbd_register_reset_callback(usbSerial, usb_reset_cb);


  while (1) {
    usbd_poll(usbSerial);

    // blinkie
    if ((millis() - blinkMillis) > blinkPeriodMillis) {
      blinkMillis = millis();
      gpio_toggle(GPIOC, GPIO13); // toggle led
    }

    // printie
    if ((millis() - printMillis) > 3000) {
      printMillis = millis();
      printf("%ld:alive!\n",millis());
    }
  }
}
