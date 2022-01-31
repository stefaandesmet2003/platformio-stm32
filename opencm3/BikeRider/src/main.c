/* 
 * simulation of a ANTUSB2 Stick connected to an indoor trainer
 * 
 */
#include <stdlib.h>
#include <string.h> // memcpy
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
// systick & inthandler
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
// uart printf
#include <libopencm3/stm32/usart.h>
#include <stdio.h>
#include <libopencm3/usb/usbd.h>

#include <math.h>

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

/* endpoint buffers */
uint8_t rx_buffer[64];
uint8_t tx_buffer[64];
uint8_t rx_len, tx_len;

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

void printHexData(uint8_t *data, int len)
{
  for (int i=0;i<len;i++) {
    printf(" %0x -",data[i]);
  }
  printf("\n");
} // printHexData

/*****************************************************************************/
/* usb descriptors                                                           */
/*****************************************************************************/

static const struct usb_device_descriptor usb_dev = {
  .bLength = USB_DT_DEVICE_SIZE,
  .bDescriptorType = USB_DT_DEVICE,
  .bcdUSB = 0x0200,
  .bDeviceClass = 0,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0,
  .bMaxPacketSize0 = 32, // copied from lsbusb, but ep have 64 ??
  .idVendor = 0x0FCF, // USB ANT dongle
  .idProduct = 0x1008, //  USB ANT dongle
  .bcdDevice = 0x0100,
  .iManufacturer = 1,
  .iProduct = 2,
  .iSerialNumber = 3,
  .bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor data_endp[] = {{
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  .bEndpointAddress = 0x81, // EP 1 IN
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,
  .wMaxPacketSize = 64,
  .bInterval = 1, // wat is dit ?? volgens lsusb is dit 1, hier stond 0
}, {
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  .bEndpointAddress = 0x01, // EP 1 OUT
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,
  .wMaxPacketSize = 64,
  .bInterval = 1,
}};

static const struct usb_interface_descriptor data_iface[] = {{
  .bLength = USB_DT_INTERFACE_SIZE,
  .bDescriptorType = USB_DT_INTERFACE,
  .bInterfaceNumber = 0,
  .bAlternateSetting = 0,
  .bNumEndpoints = 2,
  .bInterfaceClass = 255, // vendor specific class
  .bInterfaceSubClass = 0, // copied from lsusb
  .bInterfaceProtocol = 0, // copied from lsusb
  .iInterface = 2,

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
  .iConfiguration = 2,
  .bmAttributes = 0x80,
  .bMaxPower = 0x64,

  .interface = ifaces,
};

static const char *usb_strings[] = {
  "Dynastream Innovations",
  "ANT USBStick2",
  "123"
};

/*****************************************************************************/
/* bike model                                                                */
/*****************************************************************************/

#define FEC_STATE_READY   2
#define FEC_STATE_STARTED 3
#define FEC_STATE_PAUSED  4

typedef struct {
  uint8_t id; // set resistance, set target power
  uint8_t seq_num;
} fec_command_t;

/* vermoedelijk is instant_cadence een output van het rider model
 * namelijk rider duwt een torque, en dit veroorzaakt een delta_rpm
 * en wordt de crank_next_event_millis berekend (wanneer 360° verlopen, geëxtrapoleerd)
 */ 

typedef struct {
  float wheel_diameter;  // m
  uint32_t elapsed_time; // ms
  uint32_t wheel_revs; // elapsed distance = wheel_revs * wheel_diameter
  uint32_t wheel_last_event_millis;
  uint32_t wheel_next_event_millis; // based on instant_speed
  uint32_t crank_revs;
  uint32_t crank_last_event_millis;
  uint32_t crank_next_event_millis; // based on instant_cadence
  uint8_t state; // 2 = READY, 3 = STARTED, 4 = PAUSED (FE-C model)
  uint16_t instant_power;
  uint16_t resistance; // cmd_page 48, range 0..200
  int16_t slope; // cmd_page 51 (that's what RGT & Zwift seem to use), range -20000..+20000 = -200%..+200%
  fec_command_t command;
  // TODO : moet hier ook gear ratio?
  // TODO : sum_power ? nodig voor fec.p25
} bike_t;

bike_t theBike;

void init_bike () {
  theBike.wheel_diameter = 0.7;
  theBike.state = FEC_STATE_READY;
  theBike.resistance = 33; // zoals direto
  theBike.slope = 0;
  theBike.command.seq_num = 255; // hoort zo volgens FE-C
} // init_bike

void update_bike () {
  // loop functie
  theBike.elapsed_time = millis(); // ofwel rekening houden met tijd in FEC_STATE_STARTED
  // TODO : rest!
} // update_bike

/*****************************************************************************/
/* the heart                                                                 */
/*****************************************************************************/
// a simulation of a heart beating at 120bpm
typedef struct {
  uint8_t   bpm;
  uint32_t  last_beat_millis;
  uint8_t   beat_count;
} heart_t;

heart_t theHeart;

void init_heart () {
  theHeart.bpm = 120;
}

void update_heart() {
  if ((millis() - theHeart.last_beat_millis) > 500) { // 500ms = fixed 120bpm for now
    theHeart.last_beat_millis = millis();
    theHeart.beat_count += 1;
  }
} // update_heart

/*****************************************************************************/
/* ANT interface                                                             */
/*****************************************************************************/
#define MAX_CHANNELS          8
#define SYNC                  0xA4
#define CMD_REQUEST_MESSAGE   0x4D
#define BROADCAST_DATA        0x4E
#define ACKNOWLEDGED_DATA     0x4F

#define CMD_SET_CHANNEL_ID    0x51
#define CMD_UNASSIGN_CHANNEL  0x41
#define CMD_ASSIGN_CHANNEL    0x42
#define CMD_SET_MSG_PERIOD    0x43
#define CMD_SEARCH_TIMEOUT    0x44
#define CMD_SET_CHANNEL_FREQ  0x45
#define CMD_SET_NETWORK_KEY   0x46
#define CMD_SET_TX_POWER      0x47
#define CMD_RESET_SYSTEM      0x4A
#define CMD_OPEN_CHANNEL      0x4B
#define CMD_CLOSE_CHANNEL     0x4C
#define CMD_LP_SEARCH_TIMEOUT 0x63
#define CMD_ENABLE_EXT_RX_MSG 0x66
#define CMD_LIB_CONFIG        0x6E // used by zwift

#define CHANNEL_STATE_CLOSED  0
#define CHANNEL_STATE_OPEN    1

#define MAX_DEVICES             4 // FEC (17),power (11),speed&cadence (121),hrm (??)
#define DEV_TYPE_FEC            17
#define DEV_TYPE_BICYCLE_POWER  11
#define DEV_TYPE_SPEED_CADENCE  121
#define DEV_TYPE_HEARTRATE      120

typedef uint8_t (*handle_ack_data_func)(uint8_t *);

typedef struct {
  uint8_t               dev_type;
  uint8_t               page_idx; // loop counter over the pages to transmit,
  uint8_t               channel_id; // TODO : ofwel ptr naar channel_t
  handle_ack_data_func  ack_data_func;
  uint8_t               ack_page; // TODO improve, to let the msgloop know that it has to insert this page
  uint8_t               ack_page_transmits;
  uint8_t               ack_page_desc_bytes[2];
} device_t;

typedef struct {
  uint8_t   type;
  uint8_t   network;
  uint8_t   state;
  uint16_t  dev_num;
  uint8_t   dev_type;
  uint8_t   trans_type;
  uint16_t  period; // in ms, om de hoeveel ms een tx
  uint32_t  last_tx_millis;
  device_t *device;
} channel_t;

bool channel_ext_rx = false;
channel_t channels[MAX_CHANNELS];



uint8_t fec_handle_ack_data (uint8_t *ack_data);
uint8_t hrm_handle_ack_data (uint8_t *ack_data);

// devices start to transmit on channel 0, simplified implementation
// corresponds with opening channel 0 with wildcard * = what RGT does
device_t devices[4] = {{DEV_TYPE_FEC,0,0, fec_handle_ack_data,0,0,{0,0}},
                       {DEV_TYPE_BICYCLE_POWER,0,0,NULL,0,0,{0,0}},
                       {DEV_TYPE_SPEED_CADENCE,0,0,NULL,0,0,{0,0}},
                       {DEV_TYPE_HEARTRATE,0,0,hrm_handle_ack_data,0,0,{0,0}}};

device_t* devices_set_channel (uint8_t dev_type, uint8_t channel_id) {
  for (uint8_t i=0;i<MAX_DEVICES;i++) {
    if (devices[i].dev_type == dev_type) {
      devices[i].channel_id = channel_id;
      return &devices[i];
    }
  }
  return NULL;
} // devices_set_channel

// find on which channel the device is configured to transmit
uint8_t devices_get_channel (uint8_t dev_type) {
  for (uint8_t i=0;i<MAX_DEVICES;i++) {
    if (devices[i].dev_type == dev_type) {
      return (devices[i].channel_id);
    }
  }
  return 255;
} // devices_get_channel

uint8_t calc_checksum (uint8_t *buf, uint8_t len) {
  uint8_t cs = 0;
  for (uint8_t i=0; i<len; i++) {
    cs = cs ^ buf[i];
  }
  return cs;
} // calc_checksum

uint8_t set_reply_ok (uint8_t msg_id, uint8_t channel_id) {
  tx_buffer[0] = 0xa4;
  tx_buffer[1] = 0x03;
  tx_buffer[2] = 0x40;
  tx_buffer[3] = channel_id;
  tx_buffer[4] = msg_id;
  tx_buffer[5] = 0; // response code NO_ERROR
  tx_buffer[6] = calc_checksum (tx_buffer,6);
  tx_len = 7;
  return 1;
} // set_reply_ok

uint8_t set_event_tx_completed (uint8_t channel_id) {
  tx_buffer[0] = 0xa4;
  tx_buffer[1] = 0x03;
  tx_buffer[2] = 0x40;
  tx_buffer[3] = channel_id;
  tx_buffer[4] = 1; // according to protocol, 1 = RF EVENT
  tx_buffer[5] = 5; // EVENT_TRANSFER_TX_COMPLETED
  tx_buffer[6] = calc_checksum (tx_buffer,6);
  tx_len = 7;
  return 1;
} // set_event_tx_completed

uint8_t set_event_channel_closed (uint8_t channel_id) {
  tx_buffer[0] = 0xa4;
  tx_buffer[1] = 0x03;
  tx_buffer[2] = 0x40;
  tx_buffer[3] = channel_id;
  tx_buffer[4] = 1; // according to protocol, 1 = RF EVENT
  tx_buffer[5] = 7; // EVENT_CHANNEL_CLOSED
  tx_buffer[6] = calc_checksum (tx_buffer,6);
  tx_len = 7;
  return 1;
} // set_event_channel_closed


// retval 0 : unsupported, 1 = OK response data prepared
uint8_t handle_request_message (uint8_t *req_data) {
  uint8_t retval = 0;
  uint8_t channel_id, req_id, *data;
  channel_id = req_data[0];
  req_id = req_data[1];
  data = &req_data[2];
  printf("request(0x4D) for 0x%x\n",req_id);
  switch (req_id) {
    case 0x54 : // capabilities
    {
      uint8_t cap[] = {0xa4,6,0x54,8,3,0,186,54,0}; // size 9
      memcpy (tx_buffer, cap, 9);
      tx_buffer[9] = calc_checksum (tx_buffer,9);
      tx_len = 10;
      retval = 1;
      break;
    }
    case 0x61 : // serial number (for zwift)
    {
      uint8_t serialno[] = {0xa4,4,0x61,221,144,37,18}; // size 7
      memcpy (tx_buffer, serialno, 7);
      tx_buffer[7] = calc_checksum (tx_buffer,7);
      tx_len = 8;
      retval = 1;
      break;
    }
    case 0xC6 : // used by zwift
    {
      // undocumented. sending same response as anself dongle, 
      // but code works without response as well
      uint8_t xx[] = {0xa4,5,0xc6,0xa0,0xa1,0xa2,0xa3,0xa4}; // size 8
      memcpy (tx_buffer, xx, 8);
      tx_buffer[8] = calc_checksum (tx_buffer,8);
      tx_len = 9;
      retval = 1;
      break;
    }

  }
  return retval;
} // handle_request_message

// usb host sends ack_data (ANT msgId=0x4F)
// a real ant dongle will send these data during the ack time of the specific channel
// and issue a TX_COMPLETED event to the host when the RF transmission was successful
// here we handle the request immediately and fake the OK response
// ack_data = 9 bytes following the 0x4F msgId, [0] = channel_id, [1..8] = a control data page
// retval 0 : unsupported, 1 = OK response data prepared
uint8_t fec_handle_ack_data (uint8_t *ack_data) {
  uint8_t retval = 0;
  uint8_t channel_id, page_id;
  device_t *dev = &devices[0]; // TODO dirty! (hardcoded ref to the FE-C device)

  channel_id = ack_data[0];
  page_id = ack_data[1];
  printf("fec ack page %d received on channel %d\n",page_id,channel_id);
  switch (page_id) {
    case 48 : // basic resistance
      theBike.resistance = ack_data[8];
      theBike.command.id = page_id; // store the last successful command
      theBike.command.seq_num += 1;
      retval = set_event_tx_completed (channel_id);
      printf("basic resistance %d\n",theBike.resistance);
      break;
    case 49 : // target power
      retval = set_event_tx_completed (channel_id);
      printf("target power %d\n",4*((ack_data[7] << 8) + ack_data[6]));
      break;
    case 51 : // track resistance
      theBike.slope = (ack_data[7] << 8) + ack_data[6] - 0x4E20; // little endian
      printf("track resistance, theBike.slope = %d (x0.01%%)\n",theBike.slope);
      theBike.command.id = page_id; // store the last successful command
      theBike.command.seq_num += 1;
      retval = set_event_tx_completed (channel_id);
      break;
    case 55 :
      // TODO : wordt dit gebruikt door RGT?
      retval = set_event_tx_completed (channel_id);
      break;
    case 70 : // request data page
      retval = set_event_tx_completed (channel_id);
      if (ack_data[8] == 1) { // request data page, other command types not supported here
        dev->ack_page = ack_data[7];
        dev->ack_page_transmits = ack_data[6]; // don't care about 0x80 special case
        dev->ack_page_desc_bytes[0] = ack_data[4];
        dev->ack_page_desc_bytes[1] = ack_data[5];
        printf("request data page %d, rtx = %d\n",dev->ack_page,dev->ack_page_transmits);
      }

      // hier moeten we nog de msgloop wijzigen
      // zodat de gevraagde page ertussen wordt gebroadcast! 
      // get_fec_page evt aanvullen met nodige pages (54,55,71)
      break;
    default : 
      // don't support other commands for now
      // TODO : RGT gebruikt command 51 (0x33) track resistance
      // moeten we set_event_tx_completed of niet?
      // page 54 is niet opgevraagd, dus RGT weet niet of we simulation ondersteunen Y/N
      break;
  }
  return retval;
} // fec_handle_ack_data

uint8_t hrm_handle_ack_data (uint8_t *ack_data) {
  uint8_t retval = 0;
  uint8_t channel_id, page_id;
  device_t *dev = &devices[3]; // TODO dirty! (hardcoded ref to the HRM device)
  channel_id = ack_data[0];
  page_id = ack_data[1];

  printf("hrm ack page %d received on channel %d\n",page_id,channel_id);
  switch (page_id) {
    case 70 : // request data page
      retval = set_event_tx_completed (channel_id);
      if (ack_data[8] == 1) { // request data page, other command types not supported here
        dev->ack_page = ack_data[7];
        dev->ack_page_transmits = ack_data[6]; // don't care about 0x80 special case
        dev->ack_page_desc_bytes[0] = ack_data[4];
        dev->ack_page_desc_bytes[1] = ack_data[5];
        printf("request data page %d, rtx = %d\n",dev->ack_page,dev->ack_page_transmits);
      }
      break;
    default : 
      break;
  }
  return retval;
} // hrm_handle_ack_data

// 0 = no reply, 1 = reply data prepared in tx_buffer
// hack : zwift waits for the CHANNEL_CLOSED notification after the CMD_CLOSE_CHANNEL command
// retval = 2 : also send a channel closed notification
uint8_t handle_ant_request (void) {
  uint8_t msg_id, msg_len, checksum;
  uint8_t channel_id;

  uint8_t retval = 0;

  if (rx_buffer[0] != SYNC){
    printf("sync error in rx_buffer, got %c\n",rx_buffer[0]);
    return 0; // of error code?
  }
  msg_len = rx_buffer[1];
  msg_id = rx_buffer[2];
  channel_id = rx_buffer[3];

  // het volledige packet moet msg_len + 4 bytes zijn (sync, msglen, msgid, checksum)
  if (rx_len < (msg_len + 4)) {
    // not enough bytes -> can't handle for now
    printf("msg length error in rx_buffer, msg_len = %d + 4, got %d\n", msg_len, rx_len);
    return 0; // of error code?
  }
  else if (rx_len > (msg_len + 4)) {
    // too much bytes in usb packet -> restrict to using msg_len + 4 bytes
    // seems to happen!
    rx_len = msg_len + 4;
  }
  checksum = calc_checksum (rx_buffer,msg_len + 3);
  if (checksum != rx_buffer[rx_len-1]) {
    printf("checksum error in rx_buffer, got %x, expected %x\n", rx_buffer[rx_len-1],checksum);
    return 0;
  }
  switch (msg_id) {
    case CMD_RESET_SYSTEM :
      // TODO : initialize ant state
      // we sturen geen reply, maar een startup message
      tx_buffer[0] = 0xa4;
      tx_buffer[1] = 0x01;
      tx_buffer[2] = 0x6f;
      tx_buffer[3] = 0x00;
      tx_buffer[4] = 0xca; // dat kan eventueel in de send routine?
      tx_len = 5; // dat kan eventueel in de send routine?
      retval = 1;
      break;

    case CMD_SET_NETWORK_KEY :
    case CMD_SET_TX_POWER :
    case CMD_SEARCH_TIMEOUT :
    case CMD_LP_SEARCH_TIMEOUT :
    case CMD_SET_CHANNEL_FREQ :
      // TODO : anything else for these commands?
      retval = set_reply_ok (msg_id, channel_id);
      break;

    case CMD_ENABLE_EXT_RX_MSG : // dit is voor alle channels tegelijk blijkbaar
      channel_ext_rx = rx_buffer[4];
      retval = set_reply_ok (msg_id, channel_id);
      break;
    
    case CMD_LIB_CONFIG : // used by zwift
      channel_ext_rx = (rx_buffer[4] != 0);
      retval = set_reply_ok (msg_id, channel_id);
      break;

    case CMD_UNASSIGN_CHANNEL :
      channels[channel_id].type = 0;
      channels[channel_id].network = 0;
      channels[channel_id].trans_type = 0;
      channels[channel_id].device = NULL;
      retval = set_reply_ok (msg_id, channel_id);
      break;

    case CMD_ASSIGN_CHANNEL :
      channels[channel_id].type = rx_buffer[4];
      channels[channel_id].network = rx_buffer[5];
      channels[channel_id].trans_type = rx_buffer[6];
      retval = set_reply_ok (msg_id, channel_id);
      break;

    case CMD_OPEN_CHANNEL :
      channels[channel_id].state = CHANNEL_STATE_OPEN;
      // TODO set sensible defaults if not done by the caller
      // for now only need msg period
      if (channels[channel_id].period == 0)
        channels[channel_id].period = 250; // set default 250ms
      retval = set_reply_ok (msg_id, channel_id);
      printf("open channel %d with period %u ms for dev_type %d\n",channel_id,channels[channel_id].period, channels[channel_id].dev_type);
      break;

    case CMD_CLOSE_CHANNEL :
      channels[channel_id].state = CHANNEL_STATE_CLOSED;
      retval = set_reply_ok (msg_id, channel_id);
      printf("close channel %d\n",channel_id);
      // hack for zwift
      retval = 2;
      break;

    case CMD_SET_CHANNEL_ID :
      channels[channel_id].dev_num = (rx_buffer[5] << 8) + rx_buffer[4]; // little endian
      channels[channel_id].dev_type = rx_buffer[6];
      channels[channel_id].trans_type = rx_buffer[7];
      channels[channel_id].device = devices_set_channel (channels[channel_id].dev_type,channel_id);
      retval = set_reply_ok (msg_id, channel_id);
      break;
    case CMD_SET_MSG_PERIOD :
    {
      // eg. period = 8192 corresponds to 250ms msg period : (8192/32768*1000)
      uint32_t period = (rx_buffer[5] << 8) + rx_buffer[4]; // little endian
      period = 32768 / period;
      channels[channel_id].period = 1000 / period; // in ms
      printf("set_msg_period on channel %d = %d\n",channel_id,channels[channel_id].period);
      retval = set_reply_ok (msg_id, channel_id);
      break;
    }
    case CMD_REQUEST_MESSAGE :
      // these are requests for the ANT dongle itself
      retval = handle_request_message(&rx_buffer[3]);
      break;
    
    case ACKNOWLEDGED_DATA :
      // these are requests for the ANT devices, but we simulate their behavior
      if ((channels[channel_id].device) && (channels[channel_id].device->ack_data_func)) {
        retval = channels[channel_id].device->ack_data_func(&rx_buffer[3]);
      }
      break;
  }

  return retval;

} // handle_ant_request

// 1 : data prepared in buffer, 0 : else (unsupported page requested)
// fill page with data from the bike model
// in : buffer for complete msg, ie. buffer[0] = SYNC
// out : caller to complete channel_id (buffer[3]) & checksum
uint8_t get_fec_page (uint8_t page_num, uint8_t *buffer) {
  uint8_t retval = 1;

  switch (page_num) {
    case 16 : // general FE data
      // [16, 25, 38, 0, 0, 0, 255, 36, 128, 174, 25, 17, 5]
    {
      uint8_t p_data[] = {SYNC,9,BROADCAST_DATA,0,16,25};
      uint32_t distance_travelled = roundf(theBike.wheel_diameter * theBike.wheel_revs); // er gebeurt hier nog een impliciete trunc door de cast van float naar uint32!
      memcpy (buffer, p_data, 6);
      buffer[6] = (theBike.elapsed_time / 250) & 0xFF; // elapsed time (250ms), 64s rollover
      buffer[7] = distance_travelled & 0xFF; // travelled distance, 256m rollover
      buffer[8] = 0; // TODO : speed LSB
      buffer[9] = 0; // TODO : speed MSB
      buffer[10] = 255; // heartrate not supported
      buffer[11] = 0b0100 + (theBike.state << 4); // 0b0100 = fixed capabilities bit field
      break;
    }
    case 17 : // general settings page
      // 17-255-255-219-255-127-33-32
    {
      uint8_t p_data[] = {SYNC,9,BROADCAST_DATA,0,17,255,255,0,255,127};
      uint32_t cycle_length = lroundf(theBike.wheel_diameter * 100 * 3.1415); // wheel diameter in cm
      memcpy (buffer, p_data, 10);
      buffer[7] = cycle_length & 0xFF; // wheel diameter in cm, 2.54m rollover
      buffer[10] = theBike.resistance; // resistance level 0..200 in 0.5%
      buffer[11] = theBike.state << 4; // capabilities bit field = 0 (<> page 16)
      break;
    }
    case 25 : // trainer data
    {
      uint8_t p_data[] = {SYNC,9,BROADCAST_DATA,0,25};
      memcpy (buffer, p_data, 5);
      buffer[5] = 0; // update event count : TODO!! we moeten in theBike bijhouden dat er nieuwe data zijn!
      buffer[6] = 47; // TODO instant cadence
      buffer[7] = 0; // TODO sum_power LSB
      buffer[8] = 0; // TODO sum_power MSB
      buffer[9] = 180; // TODO instant power LSB
      buffer[10] = 0; // TODO instant power MSB + trainer status (0, not simulated)
      buffer[11] = theBike.state << 4; // flags bit field = 0 in resistance mode
      break;
    }
    case 80 : // manufacturer data
    {
      uint8_t p_data[] = {SYNC,9,BROADCAST_DATA,0,80,255,255,4,86,0,23,0};
      memcpy (buffer, p_data, 12);
      break;
    }
    case 81 : // product info
    {
      uint8_t prod_data[] = {SYNC,9,BROADCAST_DATA,0,81,255,255,52,174,25,0,0};
      memcpy (buffer, prod_data, 12);
      break;
    }
    default : 
      retval = 0; // unsupported page
      break;
  }
  if (retval && channel_ext_rx) {
    // add the extended data
    uint8_t ext_data[] = {0x80,0xAE,0x19,DEV_TYPE_FEC,0x5};
    memcpy (buffer+12,ext_data, 5);
    buffer[1] = 14; // msg_len 9+5 bytes
  }

  if (retval == 0) {
    printf ("fec page %d not supported\n",page_num);
  }

  return retval;
} // get_fec_page


// 1 : data prepared in buffer, 0 : else (unsupported page requested)
// fill page with data from the bike model
// in : buffer for complete msg, ie. buffer[0] = SYNC
// out : caller to complete channel_id (buffer[3]) & checksum
uint8_t get_hrm_page (uint8_t page_num, uint8_t *buffer) {
  uint8_t retval = 1;

  if ((page_num != 0) && (page_num != 2) && (page_num != 3) && (page_num != 7)) {
    printf("hrm page %d not supported\n",page_num);
    return 0; // not supported
  }

  uint8_t p_data[] = {SYNC,9,BROADCAST_DATA,0 /* channel_id */, page_num /* toggle_bit to be added by caller */,255,255,255};
  uint32_t hb_event_time;
  hb_event_time = theHeart.last_beat_millis % 64000;
  hb_event_time = hb_event_time * 128 / 125;
  // these data are present in all HRM pages
  memcpy (buffer, p_data, 8);
  buffer[8] = hb_event_time & 0xFF;
  buffer[9] = (hb_event_time >> 8) & 0xFF;
  buffer[10] = theHeart.beat_count;
  buffer[11] = theHeart.bpm;

  switch (page_num) {
    case 0 : // main page heart data
      // [0/80, 255, 255, 255, hbeventLSB, hbeventMSB, hbCount, computedHR]
      // default data as above
      break;
    case 2 : // mnf data, dummy
      buffer[5] = 123; // polar
      buffer[6] = 0x28; // serialno LSB
      buffer[7] = 0x11; // serialno MSB
      break;
    case 3 : // product data, dummy
      buffer[5] = 2; // hw revision
      buffer[6] = 10; // sw version
      buffer[7] = 10; // model id
      break;
    case 7 : // battery status, fake 4,01V battery
      buffer[5] = 90; // battery level %
      buffer[6] = 3; // fractional 3/256 = 0.01V
      buffer[7] = 0x24; // battery status ok, 4V coarse voltage
      break;
    default : 
      retval = 0; // unsupported page
      break;
  }
  if (retval && channel_ext_rx) {
    // add the extended data
    uint8_t ext_data[] = {0x80,0x28,0x11,DEV_TYPE_HEARTRATE,0x5};
    memcpy (buffer+12,ext_data, 5);
    buffer[1] = 14; // msg_len 9+5 bytes
  }

  return retval;
} // get_hrm_page


/*****************************************************************************/
/* usb interface                                                             */
/*****************************************************************************/

// got IN transaction
// TODO CHECK : komt deze cb NA een geslaagde IN transactie?
// maw. moeten we response data al eerder via usbd_ep_write_packet hebben geschreven of kan dat hier nog?
static void ep_tx_cb(usbd_device *usbd_dev, uint8_t ep)
{
  (void)ep;
  (void)usbd_dev;
  // printf ("IN on ep %d\n",ep);
  // TODO : broadcast data hier doen, of in een timer in main?
} // ep_tx_cb

// got OUT transaction
static void ep_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
  uint8_t retval;

  rx_len = usbd_ep_read_packet(usbd_dev, 0x01, rx_buffer, 64);
  printf ("rx %d bytes on ep %d :",rx_len,ep);
  printHexData(rx_buffer,rx_len);
  retval = handle_ant_request (); // we gebruiken de global rx_buffer en rx_len
  if (retval) {
    printf("reply:");
    printHexData(tx_buffer,tx_len);
    usbd_ep_write_packet(usbd_dev, 0x81, tx_buffer, tx_len);
  }
  // hack for zwift -> also send a EVENT_CHANNEL_CLOSED event
  if (retval == 2) {
    uint8_t channel_id = tx_buffer[3];
    printf("for zwift:send channel_closed event on channel %d:",channel_id);
    set_event_channel_closed(channel_id);
    printHexData(tx_buffer,tx_len);
    usbd_ep_write_packet(usbd_dev, 0x81, tx_buffer, tx_len);
  }
  
} // ep_rx_cb

static void antstick_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
  (void)wValue;
  (void)usbd_dev;
  // ant dongle, vreemd dat hetzelfde ep=1 zowel IN als OUT
  usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, ep_tx_cb);
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
  printf("The Ultimate BikeRider\n");
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
  usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &usb_dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
  usbd_register_set_config_callback(usbd_dev, antstick_set_config);

  // init application
  // ant heeft geen init nodig (voorlopig?)
  init_bike();
  init_heart();

  while (1) {
    usbd_poll(usbd_dev);

    // TODO : update bike simulation & rider simulation
    update_bike();
    update_heart(); // simulation of a heart beating at 120bpm

    // tx data
    // todo for-loop over channels[]
    bool a_ch_open = false;
    for (uint8_t ch=0;ch<MAX_CHANNELS;ch++) {
      if (channels[ch].state == CHANNEL_STATE_OPEN) {
        a_ch_open = true;
        // if (channels[ch].dev_type == 0) : TODO : nog nodig??
        // FE-C data
        if (devices_get_channel(DEV_TYPE_FEC) == ch) {
          // FE-C loop : 16-16-25-17-16-16-17-25 (8x) - 80-80 -> 66 msgs
          //             16-16-25-17-16-16-17-25 (8x) - 81-81 -> 66 msgs
          // pattern (c) from fig 8-1, but without page 18
          device_t *dev = &devices[0]; // TODO cleanup hardcoded pointer! 0 is het FE-C device
          if ((millis() - channels[ch].last_tx_millis) > channels[ch].period) {
            uint8_t page_num;
            uint8_t msg_len;
            uint8_t reply_ok = false;
            // first check for outstanding requested data pages
            if (dev->ack_page_transmits != 0) {
              page_num = dev->ack_page;
              reply_ok = get_fec_page(page_num, tx_buffer);
              if (reply_ok)
                dev->ack_page_transmits-= 1;
              else
                dev->ack_page_transmits= 0; // a non-supported page is requested -> cancel request
            }
            else { // continue with normal page loop -> transmit page_idx
              uint8_t page_idx = dev->page_idx;
              uint8_t page_nums[] = {16,16,25,17,16,16,17,25};
              if (page_idx < 64) {
                page_idx = page_idx % 8;
                page_num = page_nums[page_idx];
              }
              else if (page_idx < 66) {
                page_num = 80;
              }
              else if (page_idx < 130) {
                page_idx = (page_idx - 66) % 8;
                page_num = page_nums[page_idx];
              }
              else if (page_idx < 132) {
                page_num = 81;
              }
              reply_ok = get_fec_page(page_num, tx_buffer);
              dev->page_idx += 1;
              if (dev->page_idx >= 132) dev->page_idx = 0;
            }

            tx_buffer[3] = ch; // fill correct channel id
            msg_len = tx_buffer[1] + 3; // 9+3 or 14+3 depending on extended messages on/off
            tx_buffer[msg_len] = calc_checksum(tx_buffer,msg_len); // add checksum to message

            // transmit
            if (reply_ok) {
              usbd_ep_write_packet(usbd_dev, 0x81, tx_buffer, 18); // 0x81
              gpio_toggle(GPIOC, GPIO13); // toggle led
              printf ("fec page %d :",page_num);
              printHexData(tx_buffer,msg_len+1);
              channels[ch].last_tx_millis = millis();
            }
          }
        }
        
        // TODO : last_tx_millis werkt niet ok als dev_type == 0, en er verschillende dev_types tegelijk moeten tx
        // transmit bicycle power
        if (devices_get_channel(DEV_TYPE_BICYCLE_POWER) == ch) {
          // TODO
        }
        
        // transmit bicycle speed & cadence
        if (devices_get_channel(DEV_TYPE_SPEED_CADENCE) == ch) {
          // TODO
        }
        // transmit heart rate
        if (devices_get_channel(DEV_TYPE_HEARTRATE) == ch) {
          // HRM loop : 2-2-2-2 - 0..0 (64x) - 3-3-3-3 - 0..0 (64x) -> 2x68=136 msgs
          device_t *dev = &devices[3]; // TODO cleanup hardcoded pointer! 3 is het HRM device
          if ((millis() - channels[ch].last_tx_millis) > channels[ch].period) {
            uint8_t page_num;
            uint8_t msg_len;
            uint8_t reply_ok = false;
            // first check for outstanding requested data pages
            if (dev->ack_page_transmits != 0) {
              page_num = dev->ack_page;
              reply_ok = get_hrm_page(page_num, tx_buffer);
              if (reply_ok)
                dev->ack_page_transmits-= 1;
              else
                dev->ack_page_transmits = 0; // a non-supported page is requested -> cancel request
              // don't care about toggle bit here
            }
            else { // continue with normal page loop -> transmit page_idx
              uint8_t page_idx = dev->page_idx;
              if (page_idx < 4) {
                page_num = 2;
              }
              else if (page_idx < 68) {
                page_num = 0;
              }
              else if (page_idx < 72) {
                page_num = 3;
              }
              else if (page_idx < 136) {
                page_num = 0;
              }

              uint8_t toggle_bit;
              reply_ok = get_hrm_page(page_num, tx_buffer);
              toggle_bit = (page_idx >> 2) & 0x1; // toggles every 4 pages
              toggle_bit = toggle_bit << 7;
              tx_buffer[4] += toggle_bit;

              dev->page_idx += 1;
              if (dev->page_idx >= 136) dev->page_idx = 0;
            }

            tx_buffer[3] = ch; // fill correct channel id
            msg_len = tx_buffer[1] + 3; // 9+3 or 14+3 depending on extended messages on/off
            tx_buffer[msg_len] = calc_checksum(tx_buffer,msg_len); // add checksum to message

            // transmit
            if (reply_ok) {
              usbd_ep_write_packet(usbd_dev, 0x81, tx_buffer, 18); // 0x81
              gpio_toggle(GPIOC, GPIO13); // toggle led
              printf ("hrm page %d :",page_num);
              printHexData(tx_buffer,msg_len+1);
              channels[ch].last_tx_millis = millis();
            }
          }

        }
      }
    } // for all channels

    // a blinkie if nothing else is going on
    if (!a_ch_open) {
      // blinkie
      if ((millis() - blinkMillis) > 500){
        blinkMillis = millis();
        gpio_toggle(GPIOC, GPIO13); // toggle led
      }
    }

  } // while (1)

} // main
