
 /*
 * simpele test van external interrupts, pin A0 met pullup
 * togglet de PC13 led bij elke negative edge op A0
 * als A0 niet gedebounced is, krijg je wel massaal veel ints en toggles!
 * printf:
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
// systick & inthandler
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
// uart printf
#include <libopencm3/stm32/usart.h>
#include <stdio.h>
// exti test
#include <libopencm3/stm32/exti.h>
// nvic hebben we al

volatile uint32_t isrCount;
volatile bool isrFlag;

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

// test : A0 -> EXTI0 -> NVIC IRQ opzetten
static void exti_init (void) {

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_AFIO); // voor exti_select_source -> AFIO_EXTICR1
  gpio_set(GPIOA, GPIO0); // activate pull-up
  gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO0);
  exti_set_trigger (EXTI0, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI0);
  exti_reset_request(EXTI0); // reset pending bit just in case
  exti_select_source(EXTI0, GPIOA); // map A0 to EXTI0
  // enable the exti through nvic
  nvic_set_priority(NVIC_EXTI0_IRQ, 255); // set lowest priority
  nvic_clear_pending_irq(NVIC_EXTI0_IRQ); // just in case
  nvic_enable_irq(NVIC_EXTI0_IRQ);
}

void exti0_isr (void)
{
  gpio_toggle(GPIOC,GPIO13);
  /* Clear the interrupt before leaving. */
  exti_reset_request (EXTI0);
  isrCount++;
  isrFlag = true;
}


int main(void)
{
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
  exti_init(); // setup the EXTI test

  printf("EXTI Test\n");
  printf("rcc_ahb_frequency = %lu\n",rcc_ahb_frequency);
  printf("rcc_apb1_frequency = %lu\n",rcc_apb1_frequency);
  printf("rcc_apb2_frequency = %lu\n",rcc_apb2_frequency);

  /* Setup GPIOC Pin 13 for the LED -> sds bluepill */
  gpio_set(GPIOC, GPIO13);
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

  while(1) {
    if (isrFlag) {
      printf("isrCount=%ld\n",isrCount);
      isrFlag = false;
    }

  }

  return 0;
}
