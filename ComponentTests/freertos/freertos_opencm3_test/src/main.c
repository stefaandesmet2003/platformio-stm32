/*
 * aangepast met opencm3
 * LED3+LED4 -> PC13 beide
 * pushbutton op A0 (zelfde als discovery)
 * de printf bij start van de task neemt ongeveer 42x4 bytes stack depth!!
 * zonder printf is de high watermark = 36 (dus 70-38=32words stack gebruikt)
 * met printf is de high watermark = 28 (dus 70-28=42words stack gebruikt)
 * (dus toch maar 40 bytes meer door printf)
 * als tasks crashen door printf is het doorgaans omwille van de stack overflow
 * dus 70 words stack voor die tasks is wel voldoende (opgelet stack size bij taskCreate is in 32-bit words!)
 * free heap func returnt aantal bytes free heap, niet 32-bit words!
 * 
 * wat doet het example : je moet A0 low brengen, dan gaat led aan
 * na 5s gaat de led uit, tenzij je eerder opnieuw A0 low brengt (timer reset)

/*
This simple demo project runs on the STM32 Discovery board, which is
populated with an STM32F100RB Cortex-M3 microcontroller.  The discovery board
makes an ideal low cost evaluation platform, but the 8K of RAM provided on the
STM32F100RB does not allow the simple application to demonstrate all of all the
FreeRTOS kernel features.  Therefore, this simple demo only actively
demonstrates task, queue, timer and interrupt functionality.  In addition, the
demo is configured to include malloc failure, idle and stack overflow hook
functions.

The idle hook function:
The idle hook function queries the amount of FreeRTOS heap space that is
remaining (see vApplicationIdleHook() defined in this file).  The demo
application is configured to use 7K of the available 8K of RAM as the FreeRTOS
heap.  Memory is only allocated from this heap during initialisation, and this
demo only actually uses 1.6K bytes of the configured 7K available - leaving 5.4K
bytes of heap space unallocated.

The main() Function:
main() creates one software timer, one queue, and two tasks.  It then starts the
scheduler.

The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function in this
file.  prvQueueSendTask() sits in a loop that causes it to repeatedly block for
200 milliseconds, before sending the value 100 to the queue that was created
within main().  Once the value is sent, the task loops back around to block for
another 200 milliseconds.

The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function
in this file.  prvQueueReceiveTask() sits in a loop where it repeatedly blocks
on attempts to read data from the queue that was created within main().  When
data is received, the task checks the value of the data, and if the value equals
the expected 100, toggles the green LED.  The 'block time' parameter passed to
the queue receive function specifies that the task should be held in the Blocked
state indefinitely to wait for data to be available on the queue.  The queue
receive task will only leave the Blocked state when the queue send task writes
to the queue.  As the queue send task writes to the queue every 200
milliseconds, the queue receive task leaves the Blocked state every 200
milliseconds, and therefore toggles the green LED every 200 milliseconds.

The LED Software Timer and the Button Interrupt:
The user button B1 is configured to generate an interrupt each time it is
pressed.  The interrupt service routine switches the red LED on, and resets the
LED software timer.  The LED timer has a 5000 millisecond (5 second) period, and
uses a callback function that is defined to just turn the red LED off.
Therefore, pressing the user button will turn the red LED on, and the LED will
remain on until a full five seconds pass without the button being pressed.
*/


/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
// systick laten we nu aan freeRTOS over
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h> // set priority grouping, stond zo in het example ook

#include <stdio.h>


/*****************************************************************************/
/* freeRTOS stuff                                                            */
/*****************************************************************************/

/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY    ( tskIDLE_PRIORITY + 2 )
#define  mainQUEUE_SEND_TASK_PRIORITY    ( tskIDLE_PRIORITY + 1 )

/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the portTICK_PERIOD_MS constant. */
#define mainQUEUE_SEND_FREQUENCY_MS      ( 200 / portTICK_PERIOD_MS )

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH          ( 1 )

/*-----------------------------------------------------------*/

/*
 * Setup the NVIC, LED outputs, and button inputs.
 */
static void prvSetupHardware( void );

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask( void *pvParameters );

/*
 * The LED timer callback function.  This does nothing but switch the red LED
 * off.
 */
static void vLEDTimerCallback( TimerHandle_t xTimer );

/*-----------------------------------------------------------*/

/* The queue used by both tasks. */
static QueueHandle_t xQueue = NULL;

/* The LED software timer.  This uses vLEDTimerCallback() as its callback
 * function.
 */
static TimerHandle_t xLEDTimer = NULL;

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

/*-----------------------------------------------------------*/
TaskHandle_t hRxTask,hTxTask;

int main(void)
{
  /* Configure the NVIC, LED outputs and button inputs. */
  prvSetupHardware();

  usart_setup();  
  printf("\nfreeRTOS Example.\n");
  printf("rcc_ahb_frequency = %lu\n",rcc_ahb_frequency);
  printf("rcc_apb1_frequency = %lu\n",rcc_apb1_frequency);
  printf("rcc_apb2_frequency = %lu\n",rcc_apb2_frequency);  

  /* Create the queue. */
  xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( unsigned long ) );

  if( xQueue != NULL )
  {
    /* Start the two tasks as described in the comments at the top of this
    file. */
    //xTaskCreate( prvQueueReceiveTask, "Rx", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY, NULL );
    //xTaskCreate( prvQueueSendTask, "TX", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, NULL );
    xTaskCreate( prvQueueReceiveTask, "Rx", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_RECEIVE_TASK_PRIORITY, &hRxTask );
    xTaskCreate( prvQueueSendTask, "TX", configMINIMAL_STACK_SIZE, NULL, mainQUEUE_SEND_TASK_PRIORITY, &hTxTask );

    /* Create the software timer that is responsible for turning off the LED
    if the button is not pushed within 5000ms, as described at the top of
    this file. */
    xLEDTimer = xTimerCreate(   "LEDTimer",         /* A text name, purely to help debugging. */
                  ( 5000 / portTICK_PERIOD_MS ),/* The timer period, in this case 5000ms (5s). */
                  pdFALSE,          /* This is a one-shot timer, so xAutoReload is set to pdFALSE. */
                  ( void * ) 0,        /* The ID is not used, so can be set to anything. */
                  vLEDTimerCallback      /* The callback function that switches the LED off. */
                );
    
    xTimerStart(xLEDTimer, 0); // dat mag blijkbaar al vóór vTaskStartScheduler

    /* Start the tasks and timer running. */
    vTaskStartScheduler();
  }

  /* If all is well, the scheduler will now be running, and the following line
  will never be reached.  If the following line does execute, then there was
  insufficient FreeRTOS heap memory available for the idle and/or timer tasks
  to be created.  See the memory management section on the FreeRTOS web site
  for more details. */
  for( ;; );
}
/*-----------------------------------------------------------*/

static void vLEDTimerCallback( TimerHandle_t xTimer )
{
  /* The timer has expired - so no button pushes have occurred in the last
  five seconds - turn the LED off.  NOTE - accessing the LED port should use
  a critical section because it is accessed from multiple tasks, and the
  button interrupt - in this trivial case, for simplicity, the critical
  section is omitted. */
  gpio_set(GPIOC, GPIO13);
}
/*-----------------------------------------------------------*/

/* The ISR executed when the user button is pushed. */

void exti0_isr (void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  /* The button was pushed, so ensure the LED is on before resetting the
  LED timer.  The LED timer will turn the LED off if the button is not
  pushed within 5000ms. */
  gpio_clear(GPIOC,GPIO13);

  /* This interrupt safe FreeRTOS function can be called from this interrupt
  because the interrupt priority is below the
  configMAX_SYSCALL_INTERRUPT_PRIORITY setting in FreeRTOSConfig.h. */
  xTimerResetFromISR( xLEDTimer, &xHigherPriorityTaskWoken );

  /* Clear the interrupt before leaving. */
  exti_reset_request (EXTI0);

  /* If calling xTimerResetFromISR() caused a task (in this case the timer
  service/daemon task) to unblock, and the unblocked task has a priority
  higher than or equal to the task that was interrupted, then
  xHigherPriorityTaskWoken will now be set to pdTRUE, and calling
  portEND_SWITCHING_ISR() will ensure the unblocked task runs next. */
  portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}
/*-----------------------------------------------------------*/

static void prvQueueSendTask( void *pvParameters )
{
  TickType_t xNextWakeTime;
  const unsigned long ulValueToSend = 100UL;

  //printf("prvQueueSendTask started!\n");

  /* Initialise xNextWakeTime - this only needs to be done once. */
  xNextWakeTime = xTaskGetTickCount();

  for( ;; )
  {
    /* Place this task in the blocked state until it is time to run again.
    The block time is specified in ticks, the constant used converts ticks
    to ms.  While in the Blocked state this task will not consume any CPU
    time. */
    vTaskDelayUntil( &xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS );

    /* Send to the queue - causing the queue receive task to unblock and
    toggle an LED.  0 is used as the block time so the sending operation
    will not block - it shouldn't need to block as the queue should always
    be empty at this point in the code. */
    xQueueSend( xQueue, &ulValueToSend, 0 );
  }
}
/*-----------------------------------------------------------*/

static void prvQueueReceiveTask( void *pvParameters )
{
  unsigned long ulReceivedValue;

  printf("prvQueueReceiveTask started!\n");

  for( ;; )
  {
    /* Wait until something arrives in the queue - this task will block
    indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
    FreeRTOSConfig.h. */
    xQueueReceive( xQueue, &ulReceivedValue, portMAX_DELAY );

    /*  To get here something must have been received from the queue, but
    is it the expected value?  If it is, toggle the green LED. */
    if( ulReceivedValue == 100UL )
    {
      /* NOTE - accessing the LED port should use a critical section
      because it is accessed from multiple tasks, and the button interrupt
      - in this trivial case, for simplicity, the critical section is
      omitted. */
      //gpio_toggle(GPIOC, GPIO13);
      printf("!\n");
    }
  }
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
  /* Ensure that all 4 interrupt priority bits are used as the pre-emption
  priority. */
  scb_set_priority_grouping (SCB_AIRCR_PRIGROUP_GROUP16_NOSUB);

  // sds : blijkbaar loopt platformio achter met opencm3, want deze func bestaat niet in package-opencm3
  // na manual upgrade van framework-opencm3 werkt dit ok (git clone van opencm3 in packages/framework-opencm3 gedaan, en .piopm + package.json gecopieerd)
  rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);
  // en in latest opencm3 is rcc_clock_setup_in_hse_8mhz_out_72mhz al deprecated ..
  // voor outdated opencm3 package
  //rcc_clock_setup_in_hse_8mhz_out_72mhz();

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOC);

  /* Setup GPIOC Pin 13 for the LED -> sds bluepill */
  gpio_set(GPIOC, GPIO13);
  gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);

  // pushbutton + exti init
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
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
  /* Called if a call to pvPortMalloc() fails because there is insufficient
  free memory available in the FreeRTOS heap.  pvPortMalloc() is called
  internally by FreeRTOS API functions that create tasks, queues, software
  timers, and semaphores.  The size of the FreeRTOS heap is set by the
  configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
  for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
  ( void ) pcTaskName;
  ( void ) pxTask;

  /* Run time stack overflow checking is performed if
  configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
  function is called if a stack overflow is detected. */
  while(1) {
    gpio_toggle(GPIOC,GPIO13);
    for (uint32_t i=0;i<10000;i++)
    __asm volatile (
        " nop					\n"
        );    
  }
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
  volatile size_t xFreeStackSpace;
  volatile TickType_t xTickCount;
  volatile uint16_t wmRxTask, wmTxTask; // volatile zodat we kunnen inspecteren met debugger, die worden niet weggeoptimized
  
  
  xTickCount = xTaskGetTickCount();
  if (xTickCount > 100) {
    // blabla
  }

  wmRxTask = uxTaskGetStackHighWaterMark2 (hRxTask);
  wmTxTask = uxTaskGetStackHighWaterMark2 (hTxTask);

  /* This function is called on each cycle of the idle task.  In this case it
  does nothing useful, other than report the amout of FreeRTOS heap that
  remains unallocated. */
  // dit is in bytes, niet in 32-bit words!
  xFreeStackSpace = xPortGetFreeHeapSize();

  if( xFreeStackSpace > 100 )
  {
    /* By now, the kernel has allocated everything it is going to, so
    if there is a lot of heap remaining unallocated then
    the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
    reduced accordingly. */
  }
}
