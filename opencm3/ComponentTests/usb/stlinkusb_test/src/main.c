/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2010 Gareth McMullin <gareth@blacksphere.co.nz>
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

#include <stdlib.h>
#include <string.h> // sds, voor memset blijkbaar
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>

#define max(a,b) \
   ({ __typeof__ (a) _a = (a); \
       __typeof__ (b) _b = (b); \
     _a > _b ? _a : _b; })

static const struct usb_device_descriptor dev = {
  .bLength = USB_DT_DEVICE_SIZE,
  .bDescriptorType = USB_DT_DEVICE,
  .bcdUSB = 0x0200,
  .bDeviceClass = 0,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0,
  .bMaxPacketSize0 = 64,
  .idVendor = 0x0483, // STMicroelectronics
  .idProduct = 0x3748, // ST-LINK/V2
  .bcdDevice = 0x0100,
  .iManufacturer = 1,
  .iProduct = 2,
  .iSerialNumber = 3,
  .bNumConfigurations = 1,
};

static const struct usb_endpoint_descriptor data_endp[] = {{
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  .bEndpointAddress = 0x81,
  .bmAttributes = USB_ENDPOINT_ATTR_BULK,
  .wMaxPacketSize = 64,
  .bInterval = 0,
}, {
  .bLength = USB_DT_ENDPOINT_SIZE,
  .bDescriptorType = USB_DT_ENDPOINT,
  .bEndpointAddress = 0x02,
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
  "STMicroelectronics",
  "STM32 STLink",
  "PÿnPfVW59",
  "ST Link",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

#define STLINK_GET_VERSION             0xF1
#define STLINK_DEBUG_COMMAND           0xF2
#define STLINK_DFU_COMMAND             0xF3
#define STLINK_SWIM_COMMAND            0xF4
#define STLINK_GET_CURRENT_MODE        0xF5
#define STLINK_GET_TARGET_VOLTAGE      0xF7

#define STLINK_DEV_DFU_MODE            0x00
#define STLINK_DEV_MASS_MODE           0x01
#define STLINK_DEV_DEBUG_MODE          0x02
#define STLINK_DEV_SWIM_MODE           0x03
#define STLINK_DEV_BOOTLOADER_MODE     0x04
#define STLINK_DEV_UNKNOWN_MODE        -1

#define STLINK_DFU_EXIT                0x07

// swim commands
#define STLINK_SWIM_ENTER                  0x00
#define STLINK_SWIM_EXIT                   0x01
#define STLINK_SWIM_READ_CAP               0x02
#define STLINK_SWIM_SPEED                  0x03
#define STLINK_SWIM_ENTER_SEQ              0x04
#define STLINK_SWIM_GEN_RST                0x05
#define STLINK_SWIM_RESET                  0x06
#define STLINK_SWIM_ASSERT_RESET           0x07
#define STLINK_SWIM_DEASSERT_RESET         0x08
#define STLINK_SWIM_READSTATUS             0x09
#define STLINK_SWIM_WRITEMEM               0x0a
#define STLINK_SWIM_READMEM                0x0b
#define STLINK_SWIM_READBUF                0x0c

// swim error codes
#define STLINK_SWIM_OK          0x00
#define STLINK_SWIM_BUSY        0x01
#define STLINK_SWIM_NO_RESPONSE 0x04 // Target did not respond. SWIM not active?
#define STLINK_SWIM_BAD_STATE   0x05 // ??

// hier moeten we het stlink protocol afhandelen
#define STLINK_STATE_CMD    0
#define STLINK_STATE_READ   1
#define STLINK_STATE_WRITE  2
// nog meer nodig voor state : read_ptr, write_ptr etc

static uint8_t stlink_state = STLINK_STATE_CMD;
static uint8_t stlink_mode = STLINK_DEV_DFU_MODE;
static uint8_t swim_speed = 0;
static uint32_t memAddress = 0;
static uint16_t totalBytes = 0;
static uint16_t cntBytes = 0;
static uint8_t swimcsr_fake = 0x2;

uint8_t buf[64];

static void stlink_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
  (void)ep;
  (void)usbd_dev;


#define STLINK_SWIM_ENTER                  0x00
#define STLINK_SWIM_EXIT                   0x01
#define STLINK_SWIM_READ_CAP               0x02
#define STLINK_SWIM_SPEED                  0x03
#define STLINK_SWIM_ENTER_SEQ              0x04
#define STLINK_SWIM_GEN_RST                0x05
#define STLINK_SWIM_RESET                  0x06
#define STLINK_SWIM_ASSERT_RESET           0x07
#define STLINK_SWIM_DEASSERT_RESET         0x08
#define STLINK_SWIM_READSTATUS             0x09
#define STLINK_SWIM_WRITEMEM               0x0a
#define STLINK_SWIM_READMEM                0x0b
#define STLINK_SWIM_READBUF                0x0c
#define STLINK_SWIM_READ_BUFFERSIZE        0x0d


  int len = usbd_ep_read_packet(usbd_dev, 0x02, buf, 64);
  switch (stlink_state) {
    case STLINK_STATE_CMD : // get a new command
      // TODO check if we get exactly 16 bytes
      switch (buf[0]) {
        case STLINK_GET_VERSION : 
          // TODO : wat moeten we hier terugsturen?
          memset(buf,0,64);
          usbd_ep_write_packet(usbd_dev, 0x81, buf, 6);
          break;
        case STLINK_GET_CURRENT_MODE : 
          memset(buf,0,64);
          buf[0] = STLINK_DEV_DFU_MODE;
          buf[1] = 0; // TODO : wat moet hier komen??
          usbd_ep_write_packet(usbd_dev, 0x81, buf, 2);
          break;
        case STLINK_SWIM_COMMAND :
          switch (buf[1]) { // swim subcommand
            case STLINK_SWIM_ENTER : 
              stlink_mode = STLINK_DEV_SWIM_MODE;
              gpio_set(GPIOC, GPIO13); // TODO REMOVE led off as indication for test
              break;
            case STLINK_SWIM_EXIT : 
              // voorlopig ignore
              break;
            case STLINK_SWIM_READ_CAP : 
              // voorlopig ignore, moet 8 bytes terugsturen
              memset(buf,0,64);
              usbd_ep_write_packet(usbd_dev, 0x81, buf, 8);
              break;
            case STLINK_SWIM_SPEED : 
              swim_speed = buf[2];
              // voorlopig ignore
              break;
            case STLINK_SWIM_ENTER_SEQ : 
              // TODO generate entry sequence
              break;
            case STLINK_SWIM_GEN_RST : 
              // TODO generate 16us pulse on RST
              break;
            case STLINK_SWIM_RESET : 
              // TODO generate swim reset (128swim clocks low on RST)
              break;
            case STLINK_SWIM_ASSERT_RESET : 
              // TODO RST low (no response)
              break;
            case STLINK_SWIM_DEASSERT_RESET : 
              // TODO RST high (no response)
              break;
            case STLINK_SWIM_READSTATUS : 
              // TODO! (for now we fake)
              memset(buf,0,64);
              buf[0] = STLINK_SWIM_OK;
              buf[1] = totalBytes & 0xFF;
              buf[2] = (totalBytes >> 8) & 0xFF;
              usbd_ep_write_packet(usbd_dev, 0x81, buf, 4);
              break;
            case STLINK_SWIM_WRITEMEM :
              // niet zeker of dit juist is
              //totalBytes = *((uint16_t*) &buf[2]);
              //memAddress = *((uint32_t*) &buf[4]);
              totalBytes = ((uint16_t)buf[2] << 8) + buf[3];
              memAddress = (uint32_t)buf[7]+ ((uint32_t)buf[6]<<8);
              // onderscheid maken afh van totalBytes, of alle bytes hier zitten, 
              // dan wel of er nog een write volgt -> stlink_state = STLINK_STATE_WRITE
              // TODO : val in buf[8..15]
              // fake swim_csr TODO REMOVE
              if ((memAddress == 0x7f80) && (totalBytes == 1))
                swimcsr_fake = 0x2 | buf[8];

              if (totalBytes > 8){ 
                stlink_state = STLINK_STATE_WRITE;
                cntBytes = 8; // enfin, we gaan er toch van uit dat len==16 hier, en dus al 8 bytes gekregen om te schrijven
              }
              break;
            case STLINK_SWIM_READMEM : 
              // niet zeker of dit juist is
              //totalBytes = *((uint16_t*) &buf[2]);
              //memAddress = *((uint32_t*) &buf[4]);
              totalBytes = ((uint16_t)buf[2] << 8) + buf[3];
              memAddress = (uint32_t)buf[7]+ ((uint32_t)buf[6]<<8);
              cntBytes = 0;
              break;
            case STLINK_SWIM_READBUF :
              // TODO : wat moet READBUF terugsturen als de data nog niet beschikbaar zijn?
              // fake for now, we sturen max (totalBytes,64) terug, de rest volgt buiten callback, maar 64 bytes per IN packet
              memset(buf,0xFF,64);
              // fake swim_csr
              if ((memAddress == 0x7f80) && (totalBytes == 1))
                buf[0] = swimcsr_fake;

              // max macro lijkt fout, geeft hier 0 ??
              //usbd_ep_write_packet(usbd_dev, 0x81, buf, max (totalBytes,(uint32_t)64));
              if (totalBytes <= 64)
                usbd_ep_write_packet(usbd_dev, 0x81, buf, totalBytes);
              else {
                usbd_ep_write_packet(usbd_dev, 0x81, buf, 64);
              }

              if (totalBytes > 64) {
                stlink_state = STLINK_STATE_READ;
                cntBytes = 64;
                // we hebben nog niet alle data gereturnd, 
                // de rest sturen we vanuit de main loop
              }
              break;
            case STLINK_SWIM_READ_BUFFERSIZE : 
              // TODO, for now fake a buffer of 0x1800 like stlink
              memset(buf,0,64);
              buf[0] = 0x00;
              buf[1] = 0x18;
              usbd_ep_write_packet(usbd_dev, 0x81, buf, 2);
              break;
          }
          break;
        case STLINK_DFU_COMMAND : 
        default : 
          // ignore
          gpio_clear(GPIOC, GPIO13); // TODO REMOVE led on as indication for test
          break;
      }
      break;
    case STLINK_STATE_READ : 
      // return bytes for a read
      // niet hier, dat moet in main loop, we moeten data klaar zetten voor volgende IN packet
      // van zodra ep beschikbaar
      break;
    case STLINK_STATE_WRITE : 
      // we already received STLINK_SWIM_WRITEMEM, but expect more bytes to write
      cntBytes += len;
      // TODO echt schrijven
      if (cntBytes >= totalBytes) {
        // transactie afgelopen
        stlink_state = STLINK_STATE_CMD;
      }
      break;
  }
} // stlink_data_rx_cb

static void stlink_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
  (void)wValue;
  (void)usbd_dev;

  usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_BULK, 64, NULL);
  usbd_ep_setup(usbd_dev, 0x02, USB_ENDPOINT_ATTR_BULK, 64, stlink_data_rx_cb);
  usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_BULK, 64, NULL); // TODO, geen idee hoe dit werkt bij STLINK
  /*
  usbd_register_control_callback(
        usbd_dev,
        USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
        cdcacm_control_request);
  */
}

int main(void)
{
  int i;

  usbd_device *usbd_dev;

  // sds : blijkbaar loopt platformio achter met opencm3, want deze func bestaat niet in package-opencm3
  // en in latest opencm3 is rcc_clock_setup_in_hse_8mhz_out_72mhz al deprecated ..
  //rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
  rcc_clock_setup_in_hse_8mhz_out_72mhz();

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOC);

  /* Setup GPIOC Pin 13 for the LED -> sds bluepill */
  gpio_set(GPIOC, GPIO13);
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ,
          GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

  usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
  usbd_register_set_config_callback(usbd_dev, stlink_set_config);

  for (i = 0; i < 0x800000; i++)
    __asm__("nop");
  gpio_clear(GPIOC, GPIO13);

  while (1) {
    usbd_poll(usbd_dev);

    if (stlink_state == STLINK_STATE_READ) {
      uint16_t numBytes;

      // fake read data
      numBytes = totalBytes - cntBytes; // the number of bytes left to do
      if (numBytes > 64) numBytes = 64; // limit to 64 bytes 
      numBytes = usbd_ep_write_packet(usbd_dev, 0x81, buf, numBytes);
      // numBytes = actual number sent to host, will be 0 if ep is busy
      cntBytes += numBytes;
      if (cntBytes >= totalBytes)
        stlink_state = STLINK_STATE_CMD; // read is finished
    }
  }
}
