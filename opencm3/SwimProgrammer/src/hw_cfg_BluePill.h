/**************************************************************************
 *  Copyright (C) 2008 - 2010 by Simon Qian                               *
 *  SimonQian@SimonQian.com                                               *
 *                                                                        *
 *  Project:    Versaloon                                                 *
 *  File:       hw_cfg_MiniRelease1.h                                     *
 *  Author:     SimonQian                                                 *
 *  Versaion:   See changelog                                             *
 *  Purpose:    hardware configuration file for Mini Version Release1     *
 *  License:    See license                                               *
 *------------------------------------------------------------------------*
 *  Change Log:                                                           *
 *      YYYY-MM-DD:     What(by Who)                                      *
 *      2008-11-07:     created(by SimonQian)                             *
 *      2008-11-22:     rewrite GPIO_Dir(by SimonQian)                    *
 **************************************************************************/

#ifndef HSE_VALUE
#define HSE_VALUE						((uint32_t)8000000)
#endif
#define OSC_HZ							HSE_VALUE

#define _SYS_FREQUENCY					72		// in MHz
#define _SYS_FLASH_VECTOR_TABLE_SHIFT	FLASH_LOAD_OFFSET // From board_defs.mk


/****************************** SW ******************************/
//sds : 
// SW_PORT/SW_PIN = PB6 = SWIM_RST
// SYNCSW_IN_PORT/SYNCSW_IN_PIN = PB7 = SWIM_IN, via TIM4 input capture
// SYNCSW_OUT_PORT/SYNCSW_OUT_PIN = PB11 = SWIM (out), via TIM2.ch4 output compare

#define SW_PORT							GPIOB
#define SW_PIN							GPIO6 // stlink=GPIO_PIN_6, versaloon=GPIO_PIN_11, gebruikt als SRST
#define SW_RST_PORT					GPIOB
#define SW_RST_PIN          GPIO10
#define SYNCSW_IN_PORT      GPIOB
#define SYNCSW_IN_PIN       GPIO7 // stlink=GPIO_PIN_7, versaloon=GPIO_PIN_6
#define SYNCSW_OUT_PORT     GPIOB
#define SYNCSW_OUT_PIN      GPIO11 // stlink=GPIO_PIN_11, versaloon=GPIO_PIN_4

#define SW_PULL_INIT()
#define SW_DIR_INIT()
#define SW_SETINPUT()         gpio_set_mode(SW_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, SW_PIN)
#define SW_SETINPUT_PU()      do {\
                                gpio_set_mode(SW_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, SW_PIN); \
                                gpio_set(SW_PORT, SW_PIN);\
                              }while(0)
#define SW_SETINPUT_PD()		  do {\
                                gpio_set_mode(SW_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, SW_PIN); \
                                gpio_clear(SW_PORT, SW_PIN);\
                              }while(0)
#define SW_SETOUTPUT()			  gpio_set_mode(SW_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, SW_PIN)
#define SW_SETOUTPUT_OD()		  gpio_set_mode(SW_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, SW_PIN)
#define SW_SET()						  gpio_set(SW_PORT, SW_PIN)
#define SW_CLR()						  gpio_clear(SW_PORT, SW_PIN)
#define SW_GET()						  gpio_get(SW_PORT, SW_PIN)

#define SW_RST_PULL_INIT()
#define SW_RST_DIR_INIT()
#define SW_RST_SETINPUT()         gpio_set_mode(SW_RST_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, SW_RST_PIN)
#define SW_RST_SETINPUT_PU()      do {\
                                    gpio_set_mode(SW_RST_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, SW_RST_PIN); \
                                    gpio_set(SW_RST_PORT, SW_RST_PIN);\
                                  }while(0)
#define SW_RST_SETINPUT_PD()      do {\
                                    gpio_set_mode(SW_RST_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, SW_RST_PIN);\
                                    gpio_clear(SW_RST_PORT, SW_RST_PIN);\
                                  }while(0)
#define SW_RST_SETOUTPUT()        gpio_set_mode(SW_RST_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, SW_RST_PIN)
#define SW_RST_SETOUTPUT_OD()     gpio_set_mode(SW_RST_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_OPENDRAIN, SW_RST_PIN)
#define SW_RST_SET()              gpio_set(SW_RST_PORT, SW_RST_PIN)
#define SW_RST_CLR()              gpio_clear(SW_RST_PORT, SW_RST_PIN)
#define SW_RST_GET()              gpio_get(SW_RST_PORT, SW_RST_PIN)

// SYNCSW in PWM mode
#define SYNCSWPWM_GPIO_PORT				    GPIOB
#define SYNCSWPWM_GPIO_PIN				    GPIO7 // stlink=GPIO_PIN_7, versaloon=GPIO_PIN_6

#define SYNCSWPWM_GPIO_SETINPUT()     gpio_set_mode(SYNCSWPWM_GPIO_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, SYNCSWPWM_GPIO_PIN)
#define SYNCSWPWM_GPIO_SETINPUT_PU()  do {\
                                        gpio_set_mode(SYNCSWPWM_GPIO_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, SYNCSWPWM_GPIO_PIN);\
                                        gpio_set(SYNCSWPWM_GPIO_PORT, SYNCSWPWM_GPIO_PIN);\
                                      }while(0)
#define SYNCSWPWM_GPIO_SETINPUT_PD()  do {\
                                        gpio_set_mode(SYNCSWPWM_GPIO_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, SYNCSWPWM_GPIO_PIN); \
                                        gpio_clear(SYNCSWPWM_GPIO_PORT, SYNCSWPWM_GPIO_PIN);\
                                      }while(0)
#define SYNCSWPWM_GPIO_SETOUTPUT()    gpio_set_mode(SYNCSWPWM_GPIO_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, SYNCSWPWM_GPIO_PIN)
#define SYNCSWPWM_GPIO_SET()          gpio_set(SYNCSWPWM_GPIO_PORT, SYNCSWPWM_GPIO_PIN)
#define SYNCSWPWM_GPIO_CLR()          gpio_clear(SYNCSWPWM_GPIO_PORT, SYNCSWPWM_GPIO_PIN)
#define SYNCSWPWM_GPIO_GET()          gpio_get(SYNCSWPWM_GPIO_PORT, SYNCSWPWM_GPIO_PIN)

#define SYNCSWPWM_OUT_TIMER_MHZ			    _SYS_FREQUENCY
#define SYNCSWPWM_OUT_TIMER				      TIM2 // stlink=TIM2, versaloon=TIM3
#define SYNCSWPWM_OUT_TIMER_DMA_UPDATE	DMA_CHANNEL2 // stlink=DMA1_Channel2, versaloon=DMA1_Channel3
#define SYNCSWPWM_OUT_TIMER_DMA_COMPARE	DMA_CHANNEL6 // not used for swim don't care
#define SYNCSWPWM_IN_TIMER				      TIM4
#define SYNCSWPWM_IN_TIMER_RISE_DMA		  DMA_CHANNEL1 // stlink=DMA1_Channel1, versaloon=DMA1_Channel4
#define SYNCSWPWM_IN_TIMER_FALL_DMA		  DMA_CHANNEL4 // stlink=DMA1_Channel4, versaloon=DMA1_Channel1


/***************************** STM8_SWIM ******************************/
#define SWIM_GET()						SYNCSWPWM_GPIO_GET()



/****************************** Reset ******************************/
#define RST_SET()						SW_SET()
#define RST_CLR()						SW_CLR()
#define RST_GET()						SW_GET()

#define RST_SETOUTPUT()					SW_SETOUTPUT()
#define RST_SETINPUT()					SW_SETINPUT_PU()


/****************************** USB *****************************/
/*
#define USB_PULL_PORT					2
#define USB_PULL_PIN					13

#define USB_Pull_Init()					do{\
                      core_interfaces.gpio.init(USB_PULL_PORT);\
                      core_interfaces.gpio.clear(USB_PULL_PORT, 1 << USB_PULL_PIN);\
                      core_interfaces.gpio.config_pin(USB_PULL_PORT, USB_PULL_PIN, GPIO_OUTPP);\
                    } while (0)
#define USB_Connect()					core_interfaces.gpio.set(USB_PULL_PORT, 1 << USB_PULL_PIN)
#define USB_Disconnect()				core_interfaces.gpio.clear(USB_PULL_PORT, 1 << USB_PULL_PIN)
*/