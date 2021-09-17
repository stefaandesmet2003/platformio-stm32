/**************************************************************************
 *  Copyright (C) 2008 - 2010 by Simon Qian                               *
 *  SimonQian@SimonQian.com                                               *
 *                                                                        *
 *  Project:    Versaloon                                                 *
 *  File:       SWIM.c                                                    *
 *  Author:     SimonQian                                                 *
 *  Versaion:   See changelog                                             *
 *  Purpose:    SWIM interface implementation file                        *
 *  License:    See license                                               *
 *------------------------------------------------------------------------*
 *  Change Log:                                                           *
 *      YYYY-MM-DD:     What(by Who)                                      *
 *      2008-11-07:     created(by SimonQian)                             *
 **************************************************************************/

#include "general.h"
#include "timing_stm32.h"
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

#include <libopencm3/stm32/rcc.h> // peripheral clocks activeren
#include <libopencm3/stm32/dma.h>

#include "hw_cfg_BluePill.h"
#include "SWIM.h"

#define SWIM_0_CNT_SLOW   20
#define SWIM_1_CNT_SLOW   2
#define SWIM_0_CNT_FAST   8
#define SWIM_1_CNT_FAST   2

static uint8_t stm8_target_mhz = 8;
static bool SWIM_highSpeed = false;
// TODO : code onafhankelijk maken van deze target_mhz
// dit is de SWIM clock, die is 8MHz of 16MHz, voorlopig enkel 8MHz
// versaloon had code om de target SWIM clock op 16MHz te brengen via CLK_SWIMCCR
// maar dat is niet ondersteund op STLINK protocol, en niet op alle STM8 devices

static uint8_t SWIM_Inited = 0;
static uint16_t SWIM_PULSE_0;
static uint16_t SWIM_PULSE_1;
static uint16_t SWIM_PULSE_Threshold;
// max length is 1(header)+8(data)+1(parity)+1(ack from target)+1(empty)
static uint16_t SWIM_DMA_IN_Buffer[12];
static uint16_t SWIM_DMA_OUT_Buffer[12];
static uint16_t SWIM_clock_div = 0;

#define SWIM_MAX_DLY					0xFFFFF
#define SWIM_MAX_RESEND_CNT				0

#define SWIM_CMD_BITLEN					3
#define SWIM_CMD_SRST					0x00
#define SWIM_CMD_ROTF					0x01
#define SWIM_CMD_WOTF					0x02

#define SWIM_SYNC_CYCLES				128

// nog wa brol uit versaloon
#define SET_U32_LSBFIRST(p, v)		\
	do{\
		*((uint8_t *)(p) + 0) = (((uint32_t)(v)) >> 0) & 0xFF;\
		*((uint8_t *)(p) + 1) = (((uint32_t)(v)) >> 8) & 0xFF;\
		*((uint8_t *)(p) + 2) = (((uint32_t)(v)) >> 16) & 0xFF;\
		*((uint8_t *)(p) + 3) = (((uint32_t)(v)) >> 24) & 0xFF;\
	} while (0)

#define SET_LE_U32(p, v)			SET_U32_LSBFIRST(p, v)

// not available in BMP, so we define it here
// this code assumes systick is running, but that's the case in BMP
inline void delayMicroseconds(uint32_t) __attribute__((always_inline, unused));
inline void delayMicroseconds(uint32_t us)
{
  volatile uint32_t currentTicks = STK_CVR;
  /* Number of ticks per millisecond */
  const uint32_t tickPerMs = STK_RVR + 1;
  /* Number of ticks to count */
  const uint32_t nbTicks = ((us - ((us > 0) ? 1 : 0)) * tickPerMs) / 1000;
  /* Number of elapsed ticks */
  uint32_t elapsedTicks = 0;
  volatile uint32_t oldTicks = currentTicks;
  do {
    currentTicks = STK_CVR;
    elapsedTicks += (oldTicks < currentTicks) ? tickPerMs + oldTicks - currentTicks :
                    oldTicks - currentTicks;
    oldTicks = currentTicks;
  } while (nbTicks > elapsedTicks);
}


static void SWIM_SET(void)
{
  gpio_set_mode(SYNCSWPWM_GPIO_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, SYNCSWPWM_GPIO_PIN);
  gpio_set(SYNCSWPWM_GPIO_PORT, SYNCSWPWM_GPIO_PIN); // pullup activeren
}
static void SWIM_CLR(void)
{
  gpio_clear(SYNCSWPWM_GPIO_PORT, SYNCSWPWM_GPIO_PIN);
  gpio_set_mode(SYNCSWPWM_GPIO_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, SYNCSWPWM_GPIO_PIN);
}

// stlink=CCR1/CCR2, versaloon=CCR2/CCR1
// stlink=TIM_Channel_1, versaloon=TIM_Channel_2
// stlink=TIM_TS_TI2FP2, versaloon=TIM_TS_TI1FP1
static void SYNCSWPWM_IN_TIMER_INIT(void) {
  rcc_periph_clock_enable(RCC_DMA1);
  rcc_periph_clock_enable(RCC_TIM4);
  // original code doet hier DMA_DeInit, dat is niet enkel disable..
  dma_disable_channel(DMA1, SYNCSWPWM_IN_TIMER_RISE_DMA);
  DMA1_CCR(SYNCSWPWM_IN_TIMER_RISE_DMA) = 0;
  DMA1_IFCR |= DMA_IFCR_CIF(SYNCSWPWM_IN_TIMER_RISE_DMA); // clear interrupts

  dma_set_peripheral_address(DMA1, SYNCSWPWM_IN_TIMER_RISE_DMA,(uint32_t)&TIM_CCR1(SYNCSWPWM_IN_TIMER));
  dma_set_read_from_peripheral(DMA1, SYNCSWPWM_IN_TIMER_RISE_DMA);
  dma_set_number_of_data(DMA1, SYNCSWPWM_IN_TIMER_RISE_DMA,0); // filled in later
  dma_disable_peripheral_increment_mode(DMA1, SYNCSWPWM_IN_TIMER_RISE_DMA);
  dma_enable_memory_increment_mode(DMA1, SYNCSWPWM_IN_TIMER_RISE_DMA);
  dma_set_peripheral_size(DMA1, SYNCSWPWM_IN_TIMER_RISE_DMA,DMA_CCR_PSIZE_16BIT); // check halfword
  dma_set_memory_size(DMA1, SYNCSWPWM_IN_TIMER_RISE_DMA,DMA_CCR_MSIZE_16BIT);
  dma_set_priority(DMA1, SYNCSWPWM_IN_TIMER_RISE_DMA,DMA_CCR_PL_HIGH);
  dma_enable_channel(DMA1, SYNCSWPWM_IN_TIMER_RISE_DMA);

  // timer input capture (pwm mode)
  timer_ic_disable(SYNCSWPWM_IN_TIMER,TIM_IC1);
  timer_ic_set_polarity(SYNCSWPWM_IN_TIMER,TIM_IC1,TIM_IC_RISING);
  timer_ic_set_filter(SYNCSWPWM_IN_TIMER,TIM_IC1,TIM_IC_OFF);
  timer_ic_set_input(SYNCSWPWM_IN_TIMER,TIM_IC1,TIM_IC_IN_TI2); // IC1->TI2
  timer_ic_set_prescaler(SYNCSWPWM_IN_TIMER,TIM_IC1,TIM_IC_PSC_OFF);
  timer_ic_enable(SYNCSWPWM_IN_TIMER,TIM_IC1);

  timer_ic_disable(SYNCSWPWM_IN_TIMER,TIM_IC2);
  timer_ic_set_polarity(SYNCSWPWM_IN_TIMER,TIM_IC2,TIM_IC_FALLING);
  timer_ic_set_filter(SYNCSWPWM_IN_TIMER,TIM_IC2,TIM_IC_OFF);
  timer_ic_set_input(SYNCSWPWM_IN_TIMER,TIM_IC2,TIM_IC_IN_TI2); // IC2->TI2
  timer_ic_set_prescaler(SYNCSWPWM_IN_TIMER,TIM_IC2,TIM_IC_PSC_OFF);
  timer_ic_enable(SYNCSWPWM_IN_TIMER,TIM_IC2);

  timer_slave_set_trigger(SYNCSWPWM_IN_TIMER,TIM_SMCR_TS_TI2FP2);
  timer_slave_set_mode(SYNCSWPWM_IN_TIMER,TIM_SMCR_SMS_RM); // slave mode reset
  timer_enable_irq(SYNCSWPWM_IN_TIMER, TIM_DIER_CC2DE | TIM_DIER_CC1DE);
  timer_set_prescaler(SYNCSWPWM_IN_TIMER,0);
  timer_generate_event(SYNCSWPWM_IN_TIMER,TIM_EGR_UG); // forceer een update
  timer_enable_counter(SYNCSWPWM_IN_TIMER);
  // TIM_SelectMasterSlaveMode(SYNCSWPWM_IN_TIMER, TIM_MasterSlaveMode_Enable);
  // geen idee waarom dit in versaloon stond, daar is geen func voor in opencm3
  TIM_SMCR(SYNCSWPWM_IN_TIMER) |= TIM_SMCR_MSM; 
}
static void SYNCSWPWM_IN_TIMER_FINI(void) 
{
  timer_disable_counter(SYNCSWPWM_IN_TIMER);
  rcc_periph_clock_disable(RCC_TIM4); // hm, als SYNCSWPWM_IN_TIMER != TIM4 werkt dit wel nie
  dma_disable_channel(DMA1, SYNCSWPWM_IN_TIMER_RISE_DMA);
  rcc_periph_clock_disable(RCC_DMA1);
}

static void SYNCSWPWM_IN_TIMER_RISE_DMA_INIT (uint16_t len, uint32_t address)
{
  DMA1_CCR(SYNCSWPWM_IN_TIMER_RISE_DMA) &= ~1;
  DMA1_CNDTR(SYNCSWPWM_IN_TIMER_RISE_DMA) = len;
  DMA1_CMAR(SYNCSWPWM_IN_TIMER_RISE_DMA) = address;
  DMA1_CCR(SYNCSWPWM_IN_TIMER_RISE_DMA) |= 1;
}

#define SYNCSWPWM_IN_TIMER_RISE_DMA_READY()		(DMA_ISR(DMA1) & DMA_ISR_TCIF1)
#define SYNCSWPWM_IN_TIMER_RISE_DMA_RESET()		(DMA_IFCR(DMA1) = DMA_IFCR_CTCIF1)
#define SYNCSWPWM_IN_TIMER_RISE_DMA_WAIT(dly)	do{\
                          while(!SYNCSWPWM_IN_TIMER_RISE_DMA_READY() && --dly);\
                          SYNCSWPWM_IN_TIMER_RISE_DMA_RESET();\
                        }while(0)


static void SYNCSWPWM_OUT_TIMER_INIT(uint8_t dmaChannel, uint8_t polarity)
{
  rcc_periph_clock_enable(RCC_DMA1);
  rcc_periph_clock_enable(RCC_TIM2);
  // DMA_DeInit in versaloon code
  dma_disable_channel(DMA1, dmaChannel);
  DMA1_CCR(dmaChannel) = 0;
  DMA1_IFCR |= DMA_IFCR_CIF(dmaChannel); // clear interrupts, == dma_clear_interrupt_flags(DMA1, dmaChannel, DMA_IFCR_CIF_BIT)
  dma_set_peripheral_address(DMA1, dmaChannel,(uint32_t)&TIM_CCR4(SYNCSWPWM_OUT_TIMER));
  dma_set_read_from_memory(DMA1, dmaChannel);
  dma_set_number_of_data(DMA1, dmaChannel,0); // filled in later
  dma_disable_peripheral_increment_mode(DMA1, dmaChannel);
  dma_enable_memory_increment_mode(DMA1, dmaChannel);
  dma_set_peripheral_size(DMA1, dmaChannel,DMA_CCR_PSIZE_16BIT); // check halfword
  dma_set_memory_size(DMA1, dmaChannel,DMA_CCR_MSIZE_16BIT);
  dma_set_priority(DMA1, dmaChannel,DMA_CCR_PL_MEDIUM);
  dma_enable_channel(DMA1, dmaChannel);

  // time base
  timer_set_mode(SYNCSWPWM_OUT_TIMER,TIM_CR1_CKD_CK_INT,TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
  timer_set_prescaler(SYNCSWPWM_OUT_TIMER,0);
  timer_set_period(SYNCSWPWM_OUT_TIMER,0);
  timer_set_repetition_counter(SYNCSWPWM_OUT_TIMER,0);

  // output compare OC4
  timer_set_oc_mode(SYNCSWPWM_OUT_TIMER,TIM_OC4,TIM_OCM_PWM1);
  timer_enable_oc_output(SYNCSWPWM_OUT_TIMER,TIM_OC4);
  timer_set_oc_value(SYNCSWPWM_OUT_TIMER,TIM_OC4,0); // TIM_Pulse in orig code
  if (polarity)
    timer_set_oc_polarity_high(SYNCSWPWM_OUT_TIMER,TIM_OC4);
  else
    timer_set_oc_polarity_low(SYNCSWPWM_OUT_TIMER,TIM_OC4);
  timer_enable_oc_preload(SYNCSWPWM_OUT_TIMER,TIM_OC4);
  timer_enable_preload(SYNCSWPWM_OUT_TIMER);

  timer_enable_irq(SYNCSWPWM_OUT_TIMER, TIM_DIER_UDE); // enkel deze wordt gebruikt voor SWIM, compare niet
  timer_enable_counter(SYNCSWPWM_OUT_TIMER);
  // original code deed ook dit, maar dat is enkel voor TIM1 ??
  // asserts doen niets omdat FULL_ASSERT niet defined is
  //TIM_CtrlPWMOutputs(SYNCSWPWM_OUT_TIMER, ENABLE);

}

static void SYNCSWPWM_OUT_TIMER_FINI(uint8_t dmaChannel)
{
  timer_disable_counter(SYNCSWPWM_OUT_TIMER);
  rcc_periph_clock_disable(RCC_TIM2); // hm, als SYNCSWPWM_OUT_TIMER != TIM2 werkt dit wel nie
  dma_disable_channel(DMA1, dmaChannel);
  rcc_periph_clock_disable(RCC_DMA1);
}

static void SYNCSWPWM_OUT_TIMER_SetCycle (uint16_t cycle)
{
  TIM_ARR(SYNCSWPWM_OUT_TIMER) = cycle;
  TIM_EGR(SYNCSWPWM_OUT_TIMER) = TIM_EGR_UG;
}

#define SYNCSWPWM_OUT_TIMER_GetCycle()	TIM_ARR(SYNCSWPWM_OUT_TIMER)
#define SYNCSWPWM_OUT_TIMER_GetRate()	TIM_CCR4(SYNCSWPWM_OUT_TIMER)
#define SYNCSWPWM_OUT_TIMER_SetRate(r)	TIM_CCR4(SYNCSWPWM_OUT_TIMER) = (r)

static void SYNCSWPWM_OUT_TIMER_DMA_INIT (uint8_t dmaChannel, uint16_t len, uint32_t address)
{
  TIM_EGR(SYNCSWPWM_OUT_TIMER) = TIM_EGR_UG;
  DMA1_CCR(dmaChannel) &= ~1;
  DMA1_CNDTR(dmaChannel) = len;
  DMA1_CMAR(dmaChannel) = address;
  DMA1_CCR(dmaChannel) |= 1;
}

#define SYNCSWPWM_OUT_TIMER_DMA_UPDATE_READY()	(DMA1_ISR & DMA_ISR_TCIF2)
#define SYNCSWPWM_OUT_TIMER_DMA_UPDATE_RESET()	(DMA1_IFCR = DMA_IFCR_CTCIF2)
#define SYNCSWPWM_OUT_TIMER_DMA_UPDATE_WAIT()	do{\
                          while(!SYNCSWPWM_OUT_TIMER_DMA_UPDATE_READY());\
                          SYNCSWPWM_OUT_TIMER_DMA_UPDATE_RESET();\
                        }while(0)

/* gpio_primary_remap kunnen we hier niet gebruiken, want die overschrijft altijd de SWJ_CFG bits,
 * maar die worden bij BMP gezet in platform_init
 * deze bits zijn write-only, dus je weet eigenlijk niet wat die gaan teruglezen
 * maar BMP doet het toch ook in platform_init
 * we gaan dat hier ook doen, en enkel TIM2_REMAP veranderen
 * BMP (swlink) gebruikt TIM2_REMAP_PARTIAL_REMAP1, dus nog uit te zoeken hoe dit met BMP te integreren
 * de config hier is toch voor stlink pinning
 */
static void SYNCSWPWM_PORT_INIT(void)
{
  uint16_t afio_mapr = AFIO_MAPR;
  afio_mapr &= ~AFIO_MAPR_TIM2_REMAP_FULL_REMAP; // gebruik als mask
  afio_mapr |=  AFIO_MAPR_TIM2_REMAP_PARTIAL_REMAP2; 
  AFIO_MAPR = afio_mapr;
}

// todo : ofwel no_remap, ofwel de AFIO_MAPR_TIM2_REMAP_PARTIAL_REMAP1 voor bmp terugzetten?
// voorlopig doen we no remap
static void SYNCSWPWM_PORT_FINI(void)
{
  uint16_t afio_mapr = AFIO_MAPR;
  afio_mapr &= ~AFIO_MAPR_TIM2_REMAP_FULL_REMAP; // gebruik als mask
  afio_mapr |=  AFIO_MAPR_TIM2_REMAP_NO_REMAP; // is overbodig, zijn enkel nullen
  AFIO_MAPR = afio_mapr;
}
#define SYNCSWPWM_PORT_OD_INIT()		do{\
                      SYNCSWPWM_PORT_INIT();\
                      gpio_set(SYNCSW_OUT_PORT, SYNCSW_OUT_PIN);\
                      gpio_set_mode(SYNCSW_OUT_PORT,GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_ALTFN_OPENDRAIN,SYNCSW_OUT_PIN);\
                    }while(0)

#define SYNCSWPWM_PORT_ODPP_FINI()		do{\
                      gpio_set_mode(SYNCSW_OUT_PORT,GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, SYNCSW_OUT_PIN); \
											SYNCSWPWM_PORT_FINI();\
										}while(0)

static void SWIM_SetClockParam(uint8_t mHz, uint8_t cnt0, uint8_t cnt1)
{
  uint16_t clock_div;
  
  if (SWIM_clock_div)
  {
    clock_div = SWIM_clock_div;
  }
  else
  {
    clock_div = SYNCSWPWM_OUT_TIMER_MHZ / mHz;
    if ((SYNCSWPWM_OUT_TIMER_MHZ % mHz) >= (mHz / 2))
    {
      clock_div++;
    }
    clock_div *= SWIM_SYNC_CYCLES;
  }

  SWIM_PULSE_0 = cnt0 * clock_div / SWIM_SYNC_CYCLES;
  if ((cnt0 * clock_div % SWIM_SYNC_CYCLES) >= SWIM_SYNC_CYCLES / 2)
  {
    SWIM_PULSE_0++;
  }
  SWIM_PULSE_1 = cnt1 * clock_div / SWIM_SYNC_CYCLES;
  if ((cnt1 * clock_div % SWIM_SYNC_CYCLES) >= SWIM_SYNC_CYCLES / 2)
  {
    SWIM_PULSE_1++;
  }
  SWIM_PULSE_Threshold = SWIM_PULSE_0 + SWIM_PULSE_1;

  // 1.125 times
  SYNCSWPWM_OUT_TIMER_SetCycle(SWIM_PULSE_Threshold + (SWIM_PULSE_Threshold >> 3));

  SWIM_PULSE_Threshold >>= 1;
}

static uint8_t SWIM_HW_Out(uint8_t cmd, uint8_t bitlen, uint16_t retry_cnt)
{
  int8_t i, p;
  uint32_t dly;
  uint16_t *ptr = &SWIM_DMA_OUT_Buffer[0];

retry:

  SYNCSWPWM_IN_TIMER_RISE_DMA_INIT(bitlen + 3, (uint32_t) SWIM_DMA_IN_Buffer);

  *ptr++ = SWIM_PULSE_0;

  p = 0;
  for (i = bitlen - 1; i >= 0; i--)
  {
    if ((cmd >> i) & 1)
    {
      *ptr++ = SWIM_PULSE_1;
      p++;
    }
    else
    {
      *ptr++ = SWIM_PULSE_0;
    }
  }
  // parity bit
  if (p & 1)
  {
    *ptr++ = SWIM_PULSE_1;
  }
  else
  {
    *ptr++ = SWIM_PULSE_0;
  }
  // wait for last waveform -- parity bit
  *ptr++ = 0;
  SYNCSWPWM_OUT_TIMER_DMA_INIT(SYNCSWPWM_OUT_TIMER_DMA_UPDATE, bitlen + 3, (uint32_t) SWIM_DMA_OUT_Buffer);
  SYNCSWPWM_OUT_TIMER_DMA_UPDATE_WAIT();

  dly = SWIM_MAX_DLY;
  SYNCSWPWM_IN_TIMER_RISE_DMA_WAIT(dly);
  SYNCSWPWM_IN_TIMER_RISE_DMA_INIT(10, (uint32_t) (SWIM_DMA_IN_Buffer + 1)); // sds: anticipeer een read (0-8bits-parity)
  // sds :die +1 is niet duidelijk waarom, maar SWIM_HW_In checkt het start bit ook op SWIM_DMA_IN_Buffer[1], dus coherent

  if (!dly)
  {
    // timeout
    return 1;
  }
  else if (SWIM_DMA_IN_Buffer[bitlen + 2] > SWIM_PULSE_Threshold)
  {
    // nack
    if (retry_cnt)
    {
      retry_cnt--;
      goto retry;
    }
    else
    {
      return 1;
    }
  }
  else
  {
    return 0;
  }
}

static uint8_t SWIM_HW_In(uint8_t* data, uint8_t bitlen)
{
  uint32_t dly;

  (void) bitlen; // sds, altijd 8

  dly = SWIM_MAX_DLY;
  SYNCSWPWM_IN_TIMER_RISE_DMA_WAIT(dly); // sds : de SYNCSWPWM_IN_TIMER_RISE_DMA_INIT is al eerder gebeurd!
  *data = 0;
  if (dly && (SWIM_DMA_IN_Buffer[1] < SWIM_PULSE_Threshold)) // sds : eerste bit moet 0 zijn, slave start
  {
    for (dly = 0; dly < 8; dly++)
    {
      if (SWIM_DMA_IN_Buffer[2 + dly] < SWIM_PULSE_Threshold)
      {
        *data |= 1 << (7 - dly);
      }
    }
    // sds : parity wordt hier niet gecheckt ??
    SYNCSWPWM_IN_TIMER_RISE_DMA_INIT(11, (uint32_t) SWIM_DMA_IN_Buffer); // sds: anticipeer een next read (ACK-0-8bits-parity)

    // sds : altijd ack dus
    SWIM_DMA_OUT_Buffer[0] = SWIM_PULSE_1;
    SWIM_DMA_OUT_Buffer[1] = 0;
    SYNCSWPWM_OUT_TIMER_DMA_INIT(SYNCSWPWM_OUT_TIMER_DMA_UPDATE, 2, (uint32_t) SWIM_DMA_OUT_Buffer);
    SYNCSWPWM_OUT_TIMER_DMA_UPDATE_WAIT();
    return 0;
  }
  else
  {
    return 1;
  }
}

static int SWIM_sync(uint8_t mHz)
{
  uint32_t dly;
  uint16_t clock_div;
  uint16_t arr_save;

  SWIM_highSpeed = false; // after a sync pulse the target returns to low speed SWIM, so do we
  
  clock_div = SYNCSWPWM_OUT_TIMER_MHZ / mHz;
  if ((SYNCSWPWM_OUT_TIMER_MHZ % mHz) > (mHz / 2))
  {
    clock_div++;
  }
  
  SYNCSWPWM_IN_TIMER_RISE_DMA_INIT(2, (uint32_t) SWIM_DMA_IN_Buffer);
  
  arr_save = SYNCSWPWM_OUT_TIMER_GetCycle();
  SYNCSWPWM_OUT_TIMER_SetCycle(SWIM_SYNC_CYCLES * clock_div + 1);

  SWIM_DMA_OUT_Buffer[0] = SWIM_SYNC_CYCLES * clock_div;
  SWIM_DMA_OUT_Buffer[1] = 0;
  SYNCSWPWM_OUT_TIMER_DMA_INIT(SYNCSWPWM_OUT_TIMER_DMA_UPDATE, 2, (uint32_t) SWIM_DMA_OUT_Buffer);
  SYNCSWPWM_OUT_TIMER_DMA_UPDATE_WAIT();

  dly = SWIM_MAX_DLY;
  SYNCSWPWM_IN_TIMER_RISE_DMA_WAIT(dly);
  SYNCSWPWM_OUT_TIMER_SetCycle(arr_save);

  if (!dly)
  {
    return VSFERR_FAIL;
  }
  else
  {
    SWIM_clock_div = SWIM_DMA_IN_Buffer[1];
    // TODO moeten we hier geen range check doen op SWIM_DMA_IN_Buffer[1] ?
    SWIM_SetClockParam(stm8_target_mhz, SWIM_0_CNT_SLOW, SWIM_1_CNT_SLOW);
    return VSFERR_NONE;
  }
}

/*****************************************************************************/
/* PUBLIC INTERFACE - STLINK STYLE                                           */
/*****************************************************************************/

// enkel basic settings op de timers
// de sync gebeurt bij de entry sequence
int swim_init(void) // F4-00
{
  SWIM_clock_div = 0;
  SYNCSWPWM_IN_TIMER_INIT();

  if (!SWIM_Inited)
  {
    SWIM_Inited = 1;
    SYNCSWPWM_OUT_TIMER_INIT(SYNCSWPWM_OUT_TIMER_DMA_UPDATE, 0);
    SYNCSWPWM_PORT_OD_INIT();
  }
  // start met een default setting voor SWIM_clock_div
  // TODO : is dit nodig ? hebben we niet nodig voor de entry sequence, en SWIM_sync bepaalt de juiste waarde
  SWIM_SetClockParam(stm8_target_mhz, SWIM_0_CNT_SLOW, SWIM_1_CNT_SLOW);
  return VSFERR_NONE;
}

// TODO : is dit volledig? zijn alle resources gereleased, voor bv een switch naar SWP/JTAG?
int swim_exit(void) // F4-01
{
  SYNCSWPWM_PORT_ODPP_FINI();
  SYNCSWPWM_OUT_TIMER_FINI(SYNCSWPWM_OUT_TIMER_DMA_UPDATE);
  SYNCSWPWM_IN_TIMER_FINI();
  SWIM_Inited = 0;
  return VSFERR_NONE;
}

// this updates only our clock parameters,
// caller has to prepare target for high speed swim in advance:  check SWIM_CSR.HSIT & set SWIM_CSR.HS
// note : calling this during a rotf/wotf will corrupt the transfer
int swim_setHighSpeed(bool highSpeed) // F4-03
{
  SWIM_highSpeed = highSpeed;
  if (SWIM_highSpeed)
    SWIM_SetClockParam(stm8_target_mhz, SWIM_0_CNT_FAST,SWIM_1_CNT_FAST);
  else
    SWIM_SetClockParam(stm8_target_mhz, SWIM_0_CNT_SLOW,SWIM_1_CNT_SLOW);
  return VSFERR_NONE;
}

int swim_doEntrySequence(void) // F4-04
{
  uint32_t dly;

  SYNCSWPWM_IN_TIMER_RISE_DMA_INIT(10, (uint32_t) SWIM_DMA_IN_Buffer);

  SWIM_CLR();
  delayMicroseconds(1000);

  for (uint8_t i = 0; i < 4; i++)
  {
    SWIM_SET();
    delayMicroseconds(500);
    SWIM_CLR();
    delayMicroseconds(500);
  }
  for (uint8_t i = 0; i < 4; i++)
  {
    SWIM_SET();
    delayMicroseconds(250);
    SWIM_CLR();
    delayMicroseconds(250);
  }
  SWIM_SET();

  dly = SWIM_MAX_DLY;
  SYNCSWPWM_IN_TIMER_RISE_DMA_WAIT(dly);
  if (!dly)
  {
    return VSFERR_FAIL;
  }
  return VSFERR_NONE;
} // swim_doEntrySequence

int swim_srst(void) // F4-05
{
  if (SWIM_HW_Out(SWIM_CMD_SRST, SWIM_CMD_BITLEN, SWIM_MAX_RESEND_CNT)) {
    return VSFERR_FAIL;
  }
  else {
    return VSFERR_NONE;
  }
}

// SWIM low for 128 SWIM clocks
// SWIM_sync measures the target reply, 
// and updates SWIM_clock_div, SWIM_PULSE_0, SWIM_PULSE_1 & SWIM_PULSE_Threshold
// for future communications
// TODO : target goes back to low speed SWIM, so we have to adapt
int swim_commsReset(void) // F4-06
{
  return SWIM_sync(stm8_target_mhz);
}

// set SWIM_RESET low
// todo : de naam van die pinnen is zo verwarrend!!
int swim_assertReset(void) // F4-07
{
  SW_CLR();
  SW_SETOUTPUT();
  return VSFERR_NONE;
}

// release SWIM_RESET line (input+pullup)
// target can pull line low if needed 
int swim_deassertReset(void) // F4-08
{
  SW_SETINPUT_PU();
  return VSFERR_NONE;
}

int swim_readStatus(uint8_t *status) // F4-09
{
  (void) status;
  // TODO, async behavior
  return VSFERR_NONE;
}

// TODO make this async
int swim_wotf(uint32_t addr, uint16_t len, uint8_t *data) // F4 - 0A
{
  uint16_t processed_len;
  uint8_t cur_len, i;
  uint32_t cur_addr, addr_tmp;

  if ((0 == len) || ((uint8_t*)0 == data))
  {
    return VSFERR_FAIL;
  }

  processed_len = 0;
  cur_addr = addr;
  while (processed_len < len)
  {
    if ((len - processed_len) > 255)
    {
      cur_len = 255;
    }
    else
    {
      cur_len = len - processed_len;
    }

    SET_LE_U32(&addr_tmp, cur_addr);

    if(SWIM_HW_Out(SWIM_CMD_WOTF, SWIM_CMD_BITLEN, SWIM_MAX_RESEND_CNT))
    {
      return VSFERR_FAIL;
    }
    if (SWIM_HW_Out(cur_len, 8, 0))
    {
      return VSFERR_FAIL;
    }
    if (SWIM_HW_Out((addr_tmp >> 16) & 0xFF, 8, 0))
    {
      return VSFERR_FAIL;
    }
    if (SWIM_HW_Out((addr_tmp >> 8) & 0xFF, 8, 0))
    {
      return VSFERR_FAIL;
    }
    if (SWIM_HW_Out((addr_tmp >> 0) & 0xFF, 8, 0))
    {
      return VSFERR_FAIL;
    }
    for (i = 0; i < cur_len; i++)
    {
      if (SWIM_HW_Out(data[processed_len + i], 8, SWIM_MAX_RESEND_CNT))
      {
        return VSFERR_FAIL;
      }
    }

    cur_addr += cur_len;
    processed_len += cur_len;
  }

  return VSFERR_NONE;
}  

// TODO make this async
int swim_rotf(uint32_t addr, uint16_t len, uint8_t *data) // F4 - 0B
{
  uint16_t processed_len;
  uint8_t cur_len, i;
  uint32_t cur_addr, addr_tmp;

  if ((0 == len) || ((uint8_t*)0 == data))
  {
    return VSFERR_FAIL;
  }

  processed_len = 0;
  cur_addr = addr;
  while (processed_len < len)
  {
    if ((len - processed_len) > 255)
    {
      cur_len = 255;
    }
    else
    {
      cur_len = len - processed_len;
    }

    SET_LE_U32(&addr_tmp, cur_addr);

    if(SWIM_HW_Out(SWIM_CMD_ROTF, SWIM_CMD_BITLEN, SWIM_MAX_RESEND_CNT))
    {
      return VSFERR_FAIL;
    }
    if (SWIM_HW_Out(cur_len, 8, 0))
    {
      return VSFERR_FAIL;
    }
    if (SWIM_HW_Out((addr_tmp >> 16) & 0xFF, 8, 0))
    {
      return VSFERR_FAIL;
    }
    if (SWIM_HW_Out((addr_tmp >> 8) & 0xFF, 8, 0))
    {
      return VSFERR_FAIL;
    }
    if (SWIM_HW_Out((addr_tmp >> 0) & 0xFF, 8, 0))
    {
      return VSFERR_FAIL;
    }
    for (i = 0; i < cur_len; i++)
    {
      if (SWIM_HW_In(&data[processed_len + i], 8))
      {
        return VSFERR_FAIL;
      }
    }

    cur_addr += cur_len;
    processed_len += cur_len;
  }

  return VSFERR_NONE;
} // swim_rotf

int swim_readBuffer(uint8_t *data) // F4-0C
{
  (void) data;
  // TODO, async behavior
  return VSFERR_NONE;
}

int swim_getBufferSize(uint16_t *bufferSize) // F4-0D
{
  *bufferSize = 4192; // STLINK heeft 6144 = 0x2400
  return VSFERR_NONE;
}


