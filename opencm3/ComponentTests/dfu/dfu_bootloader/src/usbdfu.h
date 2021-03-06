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

#ifndef __USBDFU_H
#define __USBDFU_H

#include <libopencm3/usb/usbd.h>

#define DFU_SERIAL_LENGTH 9
#define BOARD_IDENT_DFU "BMP-DFU sds version"
extern uint32_t app_address;

void dfu_init(const usbd_driver *driver);
void dfu_main(void);
void dfu_protect(bool enable);

void dfu_event(void);
void dfu_detach(void);

#endif /* __USBDFU_H */
