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
 * note : met format strings neemt printf nog meer stack
 * 
 */

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
The queue send task is implemented by the prvSendTask() function in this
file.  prvSendTask() sits in a loop that causes it to repeatedly block for
200 milliseconds, before sending the value 100 to the queue that was created
within main().  Once the value is sent, the task loops back around to block for
another 200 milliseconds.

The Queue Receive Task:
The queue receive task is implemented by the prvReceiveTask() function
in this file.  prvReceiveTask() sits in a loop where it repeatedly blocks
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
#include "message_buffer.h"

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
// systick laten we nu aan freeRTOS over
#include <libopencm3/stm32/exti.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h> // set priority grouping, stond zo in het example ook

#include <libopencm3/stm32/timer.h> // voor de runtime stats timer

#include <stdio.h>
#include <string.h> // for strlen

/*****************************************************************************/
/* freeRTOS stuff                                                            */
/*****************************************************************************/

/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY    ( tskIDLE_PRIORITY + 2 )
#define  mainQUEUE_SEND_TASK_PRIORITY    ( tskIDLE_PRIORITY + 1 )

/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the portTICK_PERIOD_MS constant. */
//sds #define mainQUEUE_SEND_FREQUENCY_MS      ( 200 / portTICK_PERIOD_MS )
// vind dit duidelijker:
#define mainQUEUE_SEND_FREQUENCY_MS      pdMS_TO_TICKS(200)

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH          ( 1 )

static TaskHandle_t hRxTask,hTxTask;
static QueueHandle_t xQueue = NULL; /* The queue used by both tasks. */
static TimerHandle_t xLEDTimer = NULL; /* The LED software timer.  This uses vLEDTimerCallback() as its callback function. */
static TimerHandle_t xTaskStatsTimer = NULL; // print task stats every 5 seconds
static MessageBufferHandle_t xTaskStatsMessageBuffer = NULL;

static void prvReceiveTask( void *pvParameters );
static void prvSendTask( void *pvParameters );
static void vLEDTimerCallback( TimerHandle_t xTimer );

// deferred interrupt experiment
static void fPendedExtInterrupt (void * pvParameter1, uint32_t ulParameter2);

// compiler warning missing prototypes
void vApplicationIdleHook( void );
void vApplicationMallocFailedHook( void );

/*****************************************************************************/
/* uart printf                                                               */
/*****************************************************************************/
int _write(int fd, char *ptr, int len);
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

/*****************************************************************************/
/* assert with printf                                                        */
/*****************************************************************************/
// configASSERT is defined in FreeRTOSConfig.h and is now routed here
// add an info line before going in the endless loop
// bv als timer niet created is in isr_exti0 : "assert failed in lib/FreeRTOS-platformio/FreeRTOS-Kernel/timers.c:395"
// flash image wordt hierdoor wel 2k groter
void vAssertCalled (const char *pcFile, uint32_t ulLine)
{ 
  printf("assert failed in %s:%ld\n",pcFile, ulLine);
  fflush(stdout); 
  taskDISABLE_INTERRUPTS(); 
  for( ;; );
}	

/*****************************************************************************/
/* fast running counter for freeRTOS runtime stats                           */
/*****************************************************************************/
#if ( configGENERATE_RUN_TIME_STATS == 1 )

volatile unsigned long ulHighFrequencyTimerTicks; // us counter
static void runtime_stats_timer_setup(void)
{
	/* set up a microsecond free running timer */
	rcc_periph_clock_enable(RCC_TIM4);
	/* microsecond counter */
	timer_set_prescaler(TIM4, rcc_apb1_frequency / 500000 - 1); // PPRE=2->timers run at 2x APB1 -> 1MHz input clock naar tim6
	//timer_set_period(TIM4, 1); // auto-reload 1, dus tim4_isr elke us.
  timer_set_period(TIM4, 10); // auto-reload 1, dus tim4_isr elke 10us.
  timer_enable_irq(TIM4, TIM_DIER_UIE);
  // nvic_set_priority(NVIC_TIM4_IRQ, 20); // set a priority higher than freeRTOS, or keep the default 0
  nvic_clear_pending_irq(NVIC_TIM4_IRQ); // just in case
  nvic_enable_irq(NVIC_TIM4_IRQ);
  timer_enable_counter(TIM4);
}

void tim4_isr(void) {
  ulHighFrequencyTimerTicks++;
  timer_clear_flag(TIM4,TIM_SR_UIF);
}
#endif 

/*****************************************************************************/
/* EXTI ISR                                                                  */
/*****************************************************************************/

/* The ISR executed when the user button is pushed. */
void exti0_isr (void)
{
  portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

  /* The button was pushed, so ensure the LED is on before resetting the
  LED timer.  The LED timer will turn the LED off if the button is not
  pushed within 5000ms. */
  gpio_clear(GPIOC,GPIO13);

  /* Clear the interrupt before leaving. */
  exti_reset_request (EXTI0);

  // sds, add if for cases without led timer
  if (xLEDTimer) {
    /* This interrupt safe FreeRTOS function can be called from this interrupt
    because the interrupt priority is below the
    configMAX_SYSCALL_INTERRUPT_PRIORITY setting in FreeRTOSConfig.h. */

    xTimerResetFromISR( xLEDTimer, &xHigherPriorityTaskWoken );
  }
  // experiment with deferred interrupt handling
  // will print a message that we've seen an external interrupt
  xTimerPendFunctionCallFromISR (fPendedExtInterrupt,0,5,&xHigherPriorityTaskWoken);

  /* If previous FromISR API calls caused a task (in this case the timer
  service/daemon task) to unblock, and the unblocked task has a priority
  higher than or equal to the task that was interrupted, then
  xHigherPriorityTaskWoken will now be set to pdTRUE, and calling
  portEND_SWITCHING_ISR() will ensure the unblocked task runs next. */
  // portEND_SWITCHING_ISR is hetzelfde als portYIELD_FROM_ISR (zie portmacro.h)
  // forceer een context switch indien nodig
  // portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}

/*****************************************************************************/
/* freeRTOS                                                                  */
/*****************************************************************************/
// deferred interrupt handler, triggered from external interrupt, will be called from tmr task
static void fPendedExtInterrupt (void * pvParameter1, uint32_t ulParameter2)
{
  printf("got external interrupt, and ISR sent parameters (%ld,%ld)\n",(uint32_t)pvParameter1, ulParameter2);
}

static void vLEDTimerCallback( TimerHandle_t xTimer )
{
  (void) xTimer;
  /* The timer has expired - so no button pushes have occurred in the last
  five seconds - turn the LED off.  NOTE - accessing the LED port should use
  a critical section because it is accessed from multiple tasks, and the
  button interrupt - in this trivial case, for simplicity, the critical
  section is omitted. */
  gpio_set(GPIOC, GPIO13);
  printf("timer timeout\n"); // opgelet met printf op TMR task, die heeft ook minimal stack
}

static void vStatsTimerCallback( TimerHandle_t xTimer )
{
  size_t xFreeStackSpace;
  TickType_t xTickCount;
  char cWriteBuffer[150];
  //uint16_t wmRxTask, wmTxTask;

  (void) xTimer;

  xTickCount = xTaskGetTickCount();
  printf("current time (ticks): %ld\n",xTickCount);
  // dit is in bytes, niet in 32-bit words!
  xFreeStackSpace = xPortGetFreeHeapSize();
  printf("free heap: %d bytes\n",xFreeStackSpace);

#if ( ( configUSE_TRACE_FACILITY == 1 ) && ( configUSE_STATS_FORMATTING_FUNCTIONS > 0 ) )
  // convenience stats function
  vTaskList(cWriteBuffer);
  // a message buffer test (testCaseId == 4 has a message buffer)
  // if we have message buffer, send the stats to the Rx task to print
  // otherwise print them here in the timer task
  // TODO CHECK does this provoke task switches if RX task has higher priority than timer task??
  if (xTaskStatsMessageBuffer) {
    printf("about to MB %ld bytes:\n",strlen(cWriteBuffer)+1);
    xMessageBufferSend(xTaskStatsMessageBuffer, "MB:\n", 5, 0);
    xMessageBufferSend(xTaskStatsMessageBuffer, cWriteBuffer, strlen(cWriteBuffer)+1, 0);
  }
  else {
    printf("Name          State   Prio  Free-stack Num\n");
    printf(cWriteBuffer);
  }
#endif

#if ( configGENERATE_RUN_TIME_STATS == 1 )
  // convenience stats function
  vTaskGetRunTimeStats(cWriteBuffer);
  if (xTaskStatsMessageBuffer) {
    printf("about to MB %ld bytes:\n",strlen(cWriteBuffer)+1);
    xMessageBufferSend(xTaskStatsMessageBuffer, "MB:\n", 5, 0);
    xMessageBufferSend(xTaskStatsMessageBuffer, cWriteBuffer, strlen(cWriteBuffer)+1, 0);
  }
  else {
    printf("Name          Counts    percent(%%)\n");
    printf(cWriteBuffer);
  }
#endif

  //wmRxTask = uxTaskGetStackHighWaterMark2 (hRxTask);
  //wmTxTask = uxTaskGetStackHighWaterMark2 (hTxTask);
}


static void prvSendTask( void *pvParameters )
{
  TickType_t xNextWakeTime;
  const unsigned long ulValueToSend = 100UL;

  uint32_t testCaseId = (uint32_t) pvParameters;

  printf("prvSendTask with testCaseId %ld started!\n",testCaseId);

  /* Initialise xNextWakeTime - this only needs to be done once. */
  xNextWakeTime = xTaskGetTickCount();

  for( ;; )
  {
    switch (testCaseId) {
      case 1 :
        /* Place this task in the blocked state until it is time to run again.
        The block time is specified in ticks, the constant used converts ticks
        to ms.  While in the Blocked state this task will not consume any CPU
        time. */
        // sds : interessante functie voor een constante execution frequency <-> xTaskDelay
        // returnval indicates if task is delayed, not used here
        xTaskDelayUntil( &xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS );

        /* Send to the queue - causing the queue receive task to unblock and
        toggle an LED.  0 is used as the block time so the sending operation
        will not block - it shouldn't need to block as the queue should always
        be empty at this point in the code. */
        xQueueSend( xQueue, &ulValueToSend, 0 );
        break;
      case 2 :
        vTaskSuspend(NULL); // suspend ourself, not used in this testcase
        break;
      case 3 :
      case 4 :
        xTaskDelayUntil( &xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS );
        xTaskNotifyIndexed (hRxTask, 1, ulValueToSend, eSetValueWithOverwrite);
        break;
      default : 
        break;
    }

  }
} // prvSendTask

static void prvReceiveTask( void *pvParameters )
{
  unsigned long ulReceivedValue;
  uint32_t testCaseId = (uint32_t) pvParameters;

  printf("prvReceiveTask with testCaseId %ld started!\n",testCaseId);

  for( ;; )
  {
    switch (testCaseId) {
      case 1 :
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
          gpio_toggle(GPIOC, GPIO13);
        }
        break;
      case 2 :
        vTaskSuspend(NULL); // suspend ourself, not used in this testcase
        break;
      case 3 :
        xTaskNotifyWaitIndexed (1, 0, 0, &ulReceivedValue, portMAX_DELAY);
        if( ulReceivedValue == 100UL ) {
          gpio_toggle(GPIOC, GPIO13);
        }
        break;
      case 4 : 
      {
        // we willen task notifications & messages kunnen ontvangen
        // dus kunnen we niet forever blocken op 1 van de 2
        // eerst was prio(RXtask) > prio(TXtask); met xTicksToWait = 0 blockt de RX task nooit, 
        // en krijgt de lower priority TX task geen kans (starvation)
        // ook taskYIELD() doet dan niets aangezien de RX task altijd ready is, en de scheduler dus telkens opnieuw RX laat verderdoen.
        // door de prio(RXtask) = prio(TXtask), werkt yield wel, en krijgt TX ook schedule time
        // idle task blijft volledig gestarved
        // timer task kan wel steeds runnen (ook met prio(RXtask) > prio(TXtask)), 
        // omdat die een higher priority heeft en dan werkt de preemption (scheduler komt tussen na een systick)
        // alternatief is xTicksToWait=1 te gebruiken, dan is RX regelmatig geblockt en krijgen ook lower-prio tasks een kans

        BaseType_t isNotify;
        size_t xReceivedBytes;
        char rxBuffer[150];
        isNotify = xTaskNotifyWaitIndexed (1, 0, 0, &ulReceivedValue, 0);
        if ((isNotify) && (ulReceivedValue == 100UL)) { // received a notification
          gpio_toggle(GPIOC, GPIO13);
        }
        xReceivedBytes = xMessageBufferReceive(xTaskStatsMessageBuffer, rxBuffer, 150, 0);
        if (xReceivedBytes) {
          printf("Rx %ld bytes:\n",xReceivedBytes);
          printf(rxBuffer);
        }
        taskYIELD(); // heeft geen zin, tenzij RX & TX dezelfde prioriteit hebben
        break;
      }
      default : 
        break;
    }
  }
} // prvReceiveTask


void vApplicationMallocFailedHook( void )
{
  /* Called if a call to pvPortMalloc() fails because there is insufficient
  free memory available in the FreeRTOS heap.  pvPortMalloc() is called
  internally by FreeRTOS API functions that create tasks, queues, software
  timers, and semaphores.  The size of the FreeRTOS heap is set by the
  configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
  for( ;; );
}

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

void vApplicationIdleHook( void )
{
  volatile size_t xFreeStackSpace;

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

static void showError(void) {
  gpio_clear(GPIOC, GPIO13); // led on permanently
}

int main(void)
{
  /* Configure the NVIC, LED outputs and button inputs. */
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
  // interrupt priority moet onder configMAX_SYSCALL_INTERRUPT_PRIORITY (95d=0x5F) liggen
  // anders kan je geen RTOS API calls doen vanuit exti0_isr!!
  // test :nvic_set_priority(NVIC_EXTI0_IRQ, 0x4F); -> dan krijg je 
  // assert failed in lib/FreeRTOS-platformio/FreeRTOS-Kernel/portable/GCC/ARM_CM3/port.c:688
  // nvic_set_priority schrijft 'priority' naar het register as is, ie. zonder shift-left (priobits)!
  nvic_set_priority(NVIC_EXTI0_IRQ, 255); // set lowest priority
  nvic_clear_pending_irq(NVIC_EXTI0_IRQ); // just in case
  nvic_enable_irq(NVIC_EXTI0_IRQ);

  #if ( configGENERATE_RUN_TIME_STATS == 1 )
    runtime_stats_timer_setup();
  #endif

  usart_setup();  
  printf("\nfreeRTOS Learning code\n");
  printf("rcc_ahb_frequency = %lu\n",rcc_ahb_frequency);
  printf("rcc_apb1_frequency = %lu\n",rcc_apb1_frequency);
  printf("rcc_apb2_frequency = %lu\n",rcc_apb2_frequency);  

  // we gaan de tasks voor verschillende test cases gebruiken, de task parameter is de testcase id
  // case 1 : queues send+receive, bij elke receive togglet de led
  // case 2 : timer one-shot, de timer start bij een button-press en doet led aan, timer callback (expiry) doet led uit
  // case 3 : send+receive met task notifications ipv queues
  uint32_t testCaseId;

  testCaseId = 4;
  // initial setup per testcase
  switch (testCaseId) {
    case 1:
      /* Create the queue. */
      xQueue = xQueueCreate( mainQUEUE_LENGTH, sizeof( unsigned long ) );
      if (!xQueue) showError();
      break;
    case 2:
      /* Create the software timer that is responsible for turning off the LED
      if the button is not pushed within 5000ms */
      xLEDTimer = xTimerCreate("LEDTimer", /* A text name, purely to help debugging. */
                    pdMS_TO_TICKS(5000),   /* The timer period, in this case 5000ms (5s). */
                    pdFALSE,               /* This is a one-shot timer, so xAutoReload is set to pdFALSE. */
                    ( void * ) 0,          /* The ID is not used, so can be set to anything. */
                    vLEDTimerCallback      /* The callback function that switches the LED off. */
                  );
      //xTimerStart(xLEDTimer, 0); // dat mag blijkbaar al vóór vTaskStartScheduler
      // timer is hier nog dormant, en wordt geactiveerd als de eerste keer de PA0 int wordt getriggerd
      break;
    case 4:
      // for test gaan we bepaalde stats via message doorsturen naar de ReceiveTask
      // de receive task moet dan op 2 objecten kunnen blocken, dus kunnen we niet meer port_MAXDELAY gebruiken
      xTaskStatsMessageBuffer = xMessageBufferCreate(250);
      break;
    default:
      break;
  }
  xTaskStatsTimer = xTimerCreate("StatsTmr", /* A text name, purely to help debugging. */
                pdMS_TO_TICKS(5000),   /* The timer period, in this case 5000ms (5s). */
                pdTRUE,               /* This is a one-shot timer, so xAutoReload is set to pdFALSE. */
                ( void * ) 0,          /* The ID is not used, so can be set to anything. */
                vStatsTimerCallback      /* The callback function */
              );
  xTimerStart(xTaskStatsTimer, 0); // dat mag blijkbaar al vóór vTaskStartScheduler

  // need lots of stack for printf with % format strings (measured 90x4)
  // otherwise stack overflow and anything can happen (had xQueue variable overwritten and access address 0 in queue.c)
  // so system can hang before stack overflow is caught by the hook!
  xTaskCreate( prvReceiveTask, "RX", 200 /*configMINIMAL_STACK_SIZE*/, (void*) testCaseId, mainQUEUE_RECEIVE_TASK_PRIORITY, &hRxTask );
  xTaskCreate( prvSendTask, "TX", 200 /*configMINIMAL_STACK_SIZE*/, (void*)testCaseId, mainQUEUE_RECEIVE_TASK_PRIORITY /*mainQUEUE_SEND_TASK_PRIORITY*/, &hTxTask );

  /* Start the tasks and timer running. */
  vTaskStartScheduler();

  /* If all is well, the scheduler will now be running, and the following line
  will never be reached.  If the following line does execute, then there was
  insufficient FreeRTOS heap memory available for the idle and/or timer tasks
  to be created.  See the memory management section on the FreeRTOS web site
  for more details. */
  for( ;; );
  return 0;
} // main