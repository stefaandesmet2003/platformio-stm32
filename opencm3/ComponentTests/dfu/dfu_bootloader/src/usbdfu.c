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
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dfu.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/scb.h>

#include "usbdfu.h"

/* Commands sent with wBlockNum == 0 as per ST implementation. */
#define CMD_SETADDR	0x21
#define CMD_ERASE	0x41

#define DFU_IFACE_STRING  "@Internal Flash   /0x08000000/8*001Ka,000*001Kg"
#define DFU_IFACE_STRING_OFFSET 38
#define DFU_IFACE_PAGESIZE 1

usbd_device *usbdev;
/* We need a special large control buffer for this device: */
uint8_t usbd_control_buffer[1024];

static uint32_t max_address;

static enum dfu_state usbdfu_state = STATE_DFU_IDLE;

static char *get_dev_unique_id(char *serial_no);

static struct {
	uint8_t buf[sizeof(usbd_control_buffer)];
	uint16_t len;
	uint32_t addr;
	uint16_t blocknum;
} prog;
static uint8_t current_error;

const struct usb_device_descriptor dev_desc = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x1D50,
	.idProduct = 0x6017,
	.bcdDevice = 0x0100,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

const struct usb_dfu_descriptor dfu_function = {
	.bLength = sizeof(struct usb_dfu_descriptor),
	.bDescriptorType = DFU_FUNCTIONAL,
	.bmAttributes = USB_DFU_CAN_DOWNLOAD | USB_DFU_CAN_UPLOAD | USB_DFU_WILL_DETACH,
	.wDetachTimeout = 255,
	.wTransferSize = 1024,
	.bcdDFUVersion = 0x011A,
};

const struct usb_interface_descriptor iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 0,
	.bInterfaceClass = 0xFE, /* Device Firmware Upgrade */
	.bInterfaceSubClass = 1,
	.bInterfaceProtocol = 2,

	/* The ST Microelectronics DfuSe application needs this string.
	 * The format isn't documented... */
	.iInterface = 4,

	.extra = &dfu_function,
	.extralen = sizeof(dfu_function),
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,

	.interface = ifaces,
};

static char serial_no[DFU_SERIAL_LENGTH];
static char if_string[] = DFU_IFACE_STRING;

static const char *usb_strings[] = {
	"Black Sphere Technologies",
	BOARD_IDENT_DFU,
	serial_no,
	/* This string is used by ST Microelectronics' DfuSe utility */
	if_string,
};

static uint32_t get_le32(const void *vp)
{
	const uint8_t *p = vp;
	return ((uint32_t)p[3] << 24) + ((uint32_t)p[2] << 16) + (p[1] << 8) + p[0];
}

static uint32_t dfu_poll_timeout(uint8_t cmd, uint32_t addr, uint16_t blocknum)
{
	(void)cmd;
	(void)addr;
	(void)blocknum;
	return 100;
}

// flash programming helpers

#define FLASH_OBP_RDP 0x1FFFF800
#define FLASH_OBP_WRP10 0x1FFFF808

#define FLASH_OBP_RDP_KEY 0x5aa5

#if defined (STM32_CAN)
#	define FLASHBLOCKSIZE 2048
#else
#	define FLASHBLOCKSIZE 1024
#endif

static uint32_t last_erased_page = 0xffffffff;

static void dfu_check_and_do_sector_erase(uint32_t sector)
{
	sector &= (~(FLASHBLOCKSIZE - 1));
	if (sector != last_erased_page) {
		flash_erase_page(sector);
		last_erased_page = sector;
	}
}

static void dfu_flash_program_buffer(uint32_t baseaddr, void *buf, int len)
{
	for(int i = 0; i < len; i += 2)
		flash_program_half_word(baseaddr + i,
				*(uint16_t*)(buf+i));

	/* Call the platform specific dfu event callback. */
	dfu_event();
}


// the usb dfu class implementation
static uint8_t usbdfu_getstatus(uint32_t *bwPollTimeout)
{
	switch(usbdfu_state) {
	case STATE_DFU_DNLOAD_SYNC:
		usbdfu_state = STATE_DFU_DNBUSY;
		*bwPollTimeout = dfu_poll_timeout(prog.buf[0],
					get_le32(prog.buf + 1),
					prog.blocknum);
		return DFU_STATUS_OK;

	case STATE_DFU_MANIFEST_SYNC:
		/* Device will reset when read is complete */
		usbdfu_state = STATE_DFU_MANIFEST;
		return DFU_STATUS_OK;
	case STATE_DFU_ERROR:
		return current_error;
	default:
		return DFU_STATUS_OK;
	}
}

static void
usbdfu_getstatus_complete(usbd_device *dev, struct usb_setup_data *req)
{
	(void)req;
	(void)dev;

	switch(usbdfu_state) {
	case STATE_DFU_DNBUSY:

		flash_unlock();
		if(prog.blocknum == 0) {
			uint32_t addr = get_le32(prog.buf + 1);
			switch(prog.buf[0]) {
			case CMD_ERASE:
				if ((addr <  app_address) || (addr >= max_address)) {
					usbdfu_state = 	STATE_DFU_ERROR;
					flash_lock();
					return;
				}
				dfu_check_and_do_sector_erase(addr);
			}
		} else {
			uint32_t baseaddr = prog.addr +
				((prog.blocknum - 2) *
					dfu_function.wTransferSize);
			dfu_flash_program_buffer(baseaddr, prog.buf, prog.len);
		}
		flash_lock();

		/* We jump straight to dfuDNLOAD-IDLE,
		 * skipping dfuDNLOAD-SYNC
		 */
		usbdfu_state = STATE_DFU_DNLOAD_IDLE;
		return;

	case STATE_DFU_MANIFEST:
		dfu_detach();
		return; /* Will never return */
	default:
		return;
	}
}

static enum usbd_request_return_codes usbdfu_control_request(usbd_device *dev,
		struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
		void (**complete)(usbd_device *dev, struct usb_setup_data *req))
{
	(void)dev;

	if((req->bmRequestType & 0x7F) != 0x21)
		return USBD_REQ_NOTSUPP; /* Only accept class request */

	switch(req->bRequest) {
	case DFU_DNLOAD:
		if((len == NULL) || (*len == 0)) {
			usbdfu_state = STATE_DFU_MANIFEST_SYNC;
			return USBD_REQ_HANDLED;
		} else {
			/* Copy download data for use on GET_STATUS */
			prog.blocknum = req->wValue;
			prog.len = *len;
			memcpy(prog.buf, *buf, *len);
			if ((req->wValue == 0) && (prog.buf[0] == CMD_SETADDR)) {
				uint32_t addr = get_le32(prog.buf + 1);
				if ((addr < app_address) || (addr >= max_address)) {
					current_error = DFU_STATUS_ERR_TARGET;
					usbdfu_state = STATE_DFU_ERROR;
					return USBD_REQ_HANDLED;
				} else
					prog.addr = addr;
			}
			usbdfu_state = STATE_DFU_DNLOAD_SYNC;
			return USBD_REQ_HANDLED;
		}
	case DFU_CLRSTATUS:
		/* Clear error and return to dfuIDLE */
		if(usbdfu_state == STATE_DFU_ERROR)
			usbdfu_state = STATE_DFU_IDLE;
		return USBD_REQ_HANDLED;
	case DFU_ABORT:
		/* Abort returns to dfuIDLE state */
		usbdfu_state = STATE_DFU_IDLE;
		return USBD_REQ_HANDLED;
	case DFU_UPLOAD:
		if ((usbdfu_state == STATE_DFU_IDLE) ||
			(usbdfu_state == STATE_DFU_DNLOAD_IDLE) ||
			(usbdfu_state == STATE_DFU_UPLOAD_IDLE)) {
			prog.blocknum = req->wValue;
			usbdfu_state = STATE_DFU_UPLOAD_IDLE;
			if(prog.blocknum > 1) {
				uint32_t baseaddr = prog.addr +
					((prog.blocknum - 2) *
					 dfu_function.wTransferSize);
				memcpy(*buf, (void*)baseaddr, *len);
			}
			return USBD_REQ_HANDLED;
		} else {
			usbd_ep_stall_set(dev, 0, 1);
			return USBD_REQ_NOTSUPP;
		}
	case DFU_GETSTATUS: {
		uint32_t bwPollTimeout = 0; /* 24-bit integer in DFU class spec */

		(*buf)[0] = usbdfu_getstatus(&bwPollTimeout);
		(*buf)[1] = bwPollTimeout & 0xFF;
		(*buf)[2] = (bwPollTimeout >> 8) & 0xFF;
		(*buf)[3] = (bwPollTimeout >> 16) & 0xFF;
		(*buf)[4] = usbdfu_state;
		(*buf)[5] = 0;	/* iString not used here */
		*len = 6;

		*complete = usbdfu_getstatus_complete;

		return USBD_REQ_HANDLED;
		}
	case DFU_GETSTATE:
		/* Return state with no state transision */
		*buf[0] = usbdfu_state;
		*len = 1;
		return USBD_REQ_HANDLED;
	}

	return USBD_REQ_NOTSUPP;
}

void dfu_init(const usbd_driver *driver)
{
	get_dev_unique_id(serial_no);

	usbdev = usbd_init(driver, &dev_desc, &config,
			   usb_strings, 4,
			   usbd_control_buffer, sizeof(usbd_control_buffer));

	usbd_register_control_callback(usbdev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				usbdfu_control_request);
}

void dfu_main(void)
{
	while (1)
		usbd_poll(usbdev);
}

static void set_dfu_iface_string(uint32_t size)
{
	uint32_t res;
	char *p = if_string + DFU_IFACE_STRING_OFFSET;
	size /= DFU_IFACE_PAGESIZE;
	/* We do not want the whole printf library in the bootloader.
	 * Fill the size digits by hand.
	 */
	res = size / 100;
	if (res > 9) {
		*p++ = '9';
		*p++ = '9';
		*p++ = '9';
		return;
	} else {
		*p++ = res + '0';
		size -= res * 100;
	}
	res = size / 10;
	*p++ = res + '0';
	size -= res * 10;
	*p++ = size + '0';
}

// BMP had nog andere lengths, deze is goed genoeg
static char *serial_no_read(char *s)
{
	int i;
	volatile uint32_t *unique_id_p = (volatile uint32_t *)DESIG_UNIQUE_ID_BASE;
	uint32_t unique_id = *unique_id_p +
			*(unique_id_p + 1) +
			*(unique_id_p + 2);
	/* Fetch serial number from chip's unique ID */
	for(i = 0; i < 8; i++) {
		s[7-i] = ((unique_id >> (4*i)) & 0xF) + '0';
	}
	for(i = 0; i < 8; i++)
		if(s[i] > '9')
			s[i] += 'A' - '9' - 1;
	s[DFU_SERIAL_LENGTH - 1] = 0;
	return s;
}

static char *get_dev_unique_id(char *s)
{
	uint32_t fuse_flash_size;

	/* Calculated the upper flash limit from the exported data
	   in theparameter block*/
	fuse_flash_size = desig_get_flash_size();
	if (fuse_flash_size == 0x40) /* Handle F103x8 as F103xB! */
		fuse_flash_size = 0x80;
	set_dfu_iface_string(fuse_flash_size - 8);
	max_address = FLASH_BASE + (fuse_flash_size << 10);
	return serial_no_read(s);
}





void dfu_protect(bool enable)
{
    if (enable) {
#ifdef DFU_SELF_PROTECT
	if ((FLASH_WRPR & 0x03) != 0x00) {
		flash_unlock();
		FLASH_CR = 0;
		flash_erase_option_bytes();
		flash_program_option_bytes(FLASH_OBP_RDP, FLASH_OBP_RDP_KEY);
		/* CL Device: Protect 2 bits with (2 * 2k pages each)*/
		/* MD Device: Protect 2 bits with (4 * 1k pages each)*/
		flash_program_option_bytes(FLASH_OBP_WRP10, 0x03FC);
	}
#endif
    }
    /* There is no way we can update the bootloader with a programm running
	 * on the same device when the bootloader pages are write
	 * protected or the device is read protected!
	 *
	 * Erasing option bytes to remove write protection will make the
	 * device read protected. Read protection means that the first pages
	 * get write protected again (PM0075, 2.4.1 Read protection.)
	 *
	 * Removing read protection after option erase results in device mass
	 * erase, crashing the update (PM0075, 2.4.2, Unprotection, Case 1).
     */
#if 0
    else if ((mode == UPD_MODE) && ((FLASH_WRPR & 0x03) != 0x03)) {
		flash_unlock();
		FLASH_CR = 0;
		flash_erase_option_bytes();
		flash_program_option_bytes(FLASH_OBP_RDP, FLASH_OBP_RDP_KEY);
    }
#endif
}



