#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>

#include <stdio.h> //printf test
#include "SWIM.h"
#include <timing_stm32.h>
#include "stm8_internal.h"

// vgl BMP
static void platform_init (void)
{
  // clock setup
  rcc_clock_setup_in_hse_8mhz_out_72mhz();

  /* Enable peripherals */
  rcc_periph_clock_enable(RCC_AFIO);
  rcc_periph_clock_enable(RCC_CRC);
  // gpio clocks nodig?
  // swim doet z'n eigen clocks? blijkbaar nog niet! TODO
  // versaloon doet dat in stm32_gpio_init
  rcc_periph_clock_enable(RCC_GPIOB);  
}

static void usart_setup(void)
{
  /* Setup GPIO pin GPIO_USART1_TX. */
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
          GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

  /* Setup UART parameters. */
  usart_set_baudrate(USART1, 115200);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

  /* Finally enable the USART. */
  usart_enable(USART1);
} //usart_setup

static void show_error(void) {
  while (1) {
    platform_delay(200);
    gpio_toggle(GPIOC,GPIO13);
  }
}

static void loop(void)
{
  platform_delay(1000);
  gpio_toggle(GPIOC,GPIO13);
}

int main(void)
{
  uint8_t dataBuffer[10]; // for test
  int retval;

  platform_init();
  platform_timing_init();

  // voor de blinkie
  rcc_periph_clock_enable(RCC_GPIOC);
  gpio_set_mode(GPIOC,GPIO_MODE_OUTPUT_2_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
  gpio_clear(GPIOC,GPIO13);

  /*
  // uart clocks nodig? 
  rcc_periph_clock_enable(RCC_USART1);
  // TODO no printf voor testing
  usart_setup();
  */
  swim_init();

  swim_deassertReset();
  platform_delay(20);
  swim_assertReset();
  platform_delay(20);
  swim_doEntrySequence();

  platform_delay(10);
  swim_srst();
  platform_delay(10);
  uint32_t val32 = STM8_SWIM_CSR_SAFT_MASK | STM8_SWIM_CSR_SWIM_DM | STM8_SWIM_CSR_RST;
  uint8_t val8 = STM8_SWIM_CSR_SAFT_MASK | STM8_SWIM_CSR_SWIM_DM | STM8_SWIM_CSR_RST;
  (void) val32;
  swim_wotf(STM8_REG_SWIM_CSR,1,&val8);
  platform_delay(10);
  swim_deassertReset();
  platform_delay(10);
  // enable double speed if supported
  // voorlopig niet ondersteund (enkel 8MHz SWIM in)

  swim_commsReset(); // sync pulse on SWIM, now after HSI calibration

  // volgens versaloon is dit een chipId, maar vind dit niet terug in de STM8S103 datasheet ...
  // leest consistent [71,71,71,71,71,71], geen idee
  // swim_rotf(0x0067F0, 6, dataBuffer);

  // test high speed swim
  // enable high speed mode if available
  retval = swim_rotf(STM8_REG_SWIM_CSR, 1, dataBuffer);
  if (retval) show_error();

  val8 = STM8_SWIM_CSR_SAFT_MASK | STM8_SWIM_CSR_SWIM_DM | STM8_SWIM_CSR_RST | STM8_SWIM_CSR_PRI;
  if (dataBuffer[0] & STM8_SWIM_CSR_HSIT) {
    val8 |= STM8_SWIM_CSR_HS;
    swim_wotf(STM8_REG_SWIM_CSR, 1, &val8);
    platform_delay(10);
    swim_setHighSpeed(true);
  }
  else {
    swim_wotf(STM8_REG_SWIM_CSR, 1, &val8);
    platform_delay(10);
  }

  // srst
  swim_srst(); // STM8_SWIM_CSR_RST is gezet, dus hier geeft STM8 zichzelf een reset (+- 600us gemeten)
  swim_exit();

  while (1) {
    loop();
  } 

  return 0;
}
