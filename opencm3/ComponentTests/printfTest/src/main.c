/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2015 Chuck McManis <cmcmanis@mcmanis.com>
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
 *
 * werkt in blocking mode, ie. zonder UART interrupts
 * dus niet combineerbaar met usb polling bv.
 * de blinking delay is niet zichtbaar, beetje lullige demo
 * er is ook iets mis met de get_buffered_line, want soms wordt geldige input als <0 gelezen
 * of teveel backspace geeft foute chars op de output etc.
 */

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
// voor de systick & de inthandler
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

// iets simpels om millis time keeping te hebben
uint32_t _millis = 0;

// we draaien hier voor de fun op de HSI @ 8MHz, en niet @72MHz
// dus defaults efkes aanpassen!
static void systick_setup(void) {
	//systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
  // 8MHz -> 8000000 counts per second
  systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);

	/* 8000000/8000 = 1000 overflows per second - every 1ms one interrupt */
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload(7999);

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

/*
 * To implement the STDIO functions you need to create
 * the _read and _write functions and hook them to the
 * USART you are using. This example also has a buffered
 * read function for basic line editing.
 */
int _write(int fd, char *ptr, int len);
int _read(int fd, char *ptr, int len);
void get_buffered_line(void);

/*
 * This is a pretty classic ring buffer for characters
 */
#define BUFLEN 127

static uint16_t start_ndx;
static uint16_t end_ndx;
static char buf[BUFLEN+1];
#define buf_len ((end_ndx - start_ndx) % BUFLEN)
static inline int inc_ndx(int n) { return ((n + 1) % BUFLEN); }
static inline int dec_ndx(int n) { return (((n + BUFLEN) - 1) % BUFLEN); }


static void clock_setup(void)
{
  /* Enable GPIOA clock for USART1 */
  rcc_periph_clock_enable(RCC_GPIOA);

  /* Enable GPIOC clock for builtin_led PC13 */
  rcc_periph_clock_enable(RCC_GPIOC);

  /* Enable clocks for USART1. */
  rcc_periph_clock_enable(RCC_USART1);
}

static void usart_setup(void)
{
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

static void gpio_setup(void)
{
  /* Setup GPIO pin PC13 for LED. */
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
}



int main(void)
{
  clock_setup();
  systick_setup();
  gpio_setup();
  usart_setup();
  printf("\nStandard I/O Example.\n");
  printf("rcc_ahb_frequency = %lu\n",rcc_ahb_frequency);
  printf("rcc_apb1_frequency = %lu\n",rcc_apb1_frequency);
  printf("rcc_apb2_frequency = %lu\n",rcc_apb2_frequency);

  /* Blink the LED (PC13) on the board with every transmitted byte. */
  // aangezien we hier een blocking implementatie van printf/fgets gebruiken
  // kunnen we niet knipperen terwijl we user input afwachten
  // dan maar efkes knipperen erna
  while (1) {
    char local_buf[32];
    uint8_t blinkCount = 10;
    uint32_t blinkMillis;
    int delay = 0;

    if ((millis() - blinkMillis) > delay) {
      blinkMillis = millis();
      gpio_toggle(GPIOC, GPIO13);	/* LED on/off */
    }
    
    do {
      printf("Enter the delay constant for blink : ");
      fflush(stdout);
      fgets(local_buf, 32, stdin);
      delay = atoi(local_buf);
      if (delay <= 0) {
        printf("Error: expected a delay > 0\n");
      }
    } while (delay <= 0);

    printf("Blinking with a delay of %d\n", delay);
    blinkMillis = millis();
    while (blinkCount) {
      if ((millis() - blinkMillis) > delay) {
        blinkMillis = millis();
        gpio_toggle(GPIOC, GPIO13);	/* LED on/off */
        blinkCount--;
      }
    }
  }
  return 0;
}

/* back up the cursor one space */
static inline void back_up(void)
{
  end_ndx = dec_ndx(end_ndx);
  usart_send_blocking(USART1, '\010');
  usart_send_blocking(USART1, ' ');
  usart_send_blocking(USART1, '\010');
}

/*
 * A buffered line editing function.
 */
void
get_buffered_line(void) {
  char	c;

  if (start_ndx != end_ndx) {
    return;
  }
  while (1) {
    c = usart_recv_blocking(USART1);
    if (c == '\r') {
      buf[end_ndx] = '\n';
      end_ndx = inc_ndx(end_ndx);
      buf[end_ndx] = '\0';
      usart_send_blocking(USART1, '\r');
      usart_send_blocking(USART1, '\n');
      return;
    }
    /* ^H or DEL erase a character */
    if ((c == '\010') || (c == '\177')) {
      if (buf_len == 0) {
        usart_send_blocking(USART1, '\a');
      } else {
        back_up();
      }
    /* ^W erases a word */
    } else if (c == 0x17) {
      while ((buf_len > 0) &&
          (!(isspace((int) buf[end_ndx])))) {
        back_up();
      }
    /* ^U erases the line */
    } else if (c == 0x15) {
      while (buf_len > 0) {
        back_up();
      }
    /* Non-editing character so insert it */
    } else {
      if (buf_len == (BUFLEN - 1)) {
        usart_send_blocking(USART1, '\a');
      } else {
        buf[end_ndx] = c;
        end_ndx = inc_ndx(end_ndx);
        usart_send_blocking(USART1, c);
      }
    }
  }
}


/*
 * Called by libc stdio fwrite functions
 */
int
_write(int fd, char *ptr, int len)
{
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

/*
 * Called by the libc stdio fread fucntions
 *
 * Implements a buffered read with line editing.
 */
int
_read(int fd, char *ptr, int len)
{
  int	my_len;

  if (fd > 2) {
    return -1;
  }

  get_buffered_line();
  my_len = 0;
  while ((buf_len > 0) && (len > 0)) {
    *ptr++ = buf[start_ndx];
    start_ndx = inc_ndx(start_ndx);
    my_len++;
    len--;
  }
  return my_len; /* return the length we got */
}
