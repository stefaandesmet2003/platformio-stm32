/*
 * FreeRTOS V202107.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */


#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/
//sds
//#define vPortSVCHandler SVC_Handler
//#define xPortPendSVHandler PendSV_Handler
//#define xPortSysTickHandler SysTick_Handler
// these must be correct for freeRTOS to work under opencm3!!
#define vPortSVCHandler sv_call_handler
#define xPortPendSVHandler pend_sv_handler
#define xPortSysTickHandler sys_tick_handler


#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      ( 72000000UL )
//NODIG? #define configSYSTICK_CLOCK_HZ                  1000000
//OLD #define configTICK_RATE_HZ				( ( TickType_t ) 1000 )
#define configTICK_RATE_HZ                      1000
//sds : configMAX_PRIORITIES mag hier 3 zijn om ram te sparen
#define configMAX_PRIORITIES                    5
#define configMINIMAL_STACK_SIZE                70 //sds 128
#define configMAX_TASK_NAME_LEN                 10 //sds 16
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   3 // sds default is 1, maar dan geen Indexed()...
#define configUSE_MUTEXES                       0
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           0
#define configUSE_ALTERNATIVE_API               0 /* Deprecated! */
//sds #define configQUEUE_REGISTRY_SIZE               0 //sds stond 10, maar dit is enkel voor debug
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIME_SLICING                  1  // sds hier stond 0, maar default is 1
#define configUSE_NEWLIB_REENTRANT              0
#define configENABLE_BACKWARD_COMPATIBILITY     0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 0 // sds hier stond 5, maar default is 0
#define configSTACK_DEPTH_TYPE                  uint16_t
#define configMESSAGE_BUFFER_LENGTH_TYPE        size_t

/* Memory allocation related definitions. */
#define configSUPPORT_STATIC_ALLOCATION         		0
#define configSUPPORT_DYNAMIC_ALLOCATION        		1
//sds : configTOTAL_HEAP_SIZE : total RAM - stack voor ISR's
// heap = statically allocated ram, dus zie je in de RAM usage stats bij build
// ram usage = heap + alle andere static variables
#define configTOTAL_HEAP_SIZE												( ( size_t ) ( 7 * 1024 ) ) 
#define configAPPLICATION_ALLOCATED_HEAP            0
#define configSTACK_ALLOCATION_FROM_SEPARATE_HEAP   0

/* Hook function related definitions. */
//sds #define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
//sds #define configCHECK_FOR_STACK_OVERFLOW          0
//sds #define configUSE_MALLOC_FAILED_HOOK            0
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

/* Run time and task stats gathering related definitions. */
//sds #define configGENERATE_RUN_TIME_STATS           0
//sds #define configUSE_TRACE_FACILITY                0
//sds #define configUSE_STATS_FORMATTING_FUNCTIONS    0

/* Co-routine related definitions. */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         1

/* Software timer related definitions. */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               3
#define configTIMER_QUEUE_LENGTH                5 // sds stond 10
//sds #define configTIMER_TASK_STACK_DEPTH            configMINIMAL_STACK_SIZE

/* Interrupt nesting behaviour configuration. */
/* Use the system definition, if there is one */
#ifdef __NVIC_PRIO_BITS
	#define configPRIO_BITS       __NVIC_PRIO_BITS
#else
	#define configPRIO_BITS       4        /* 15 priority levels */
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY			15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY	5

/* The lowest priority. */
#define configKERNEL_INTERRUPT_PRIORITY 	( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
/* Priority 5, or 95 as only the top four bits are implemented. */
/* !!!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!!!
See http://www.FreeRTOS.org/RTOS-Cortex-M3-M4.html. */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_API_CALL_INTERRUPT_PRIORITY  (configMAX_SYSCALL_INTERRUPT_PRIORITY)

/* Define to trap errors during development. */
//sds #define configASSERT( x ) if( ( x ) == 0 ) { taskDISABLE_INTERRUPTS(); for( ;; ); }
// route assert to a function that prints info before locking up
extern void vAssertCalled (const char *pcFile, uint32_t ulLine);
#define configASSERT( x ) if( ( x ) == 0 ) { vAssertCalled( __FILE__, __LINE__ ); }

/* FreeRTOS MPU specific definitions. */
#define configINCLUDE_APPLICATION_DEFINED_PRIVILEGED_FUNCTIONS 0
#define configTOTAL_MPU_REGIONS                                8 /* Default value. */
#define configTEX_S_C_B_FLASH                                  0x07UL /* Default value. */
#define configTEX_S_C_B_SRAM                                   0x07UL /* Default value. */
#define configENFORCE_SYSTEM_CALLS_FROM_KERNEL_ONLY            1

/* ARMv8-M secure side port related definitions. */
#define secureconfigMAX_SECURE_CONTEXTS         5

/* Optional functions - most linkers will remove unused functions anyway. */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_xResumeFromISR                  1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     0
#define INCLUDE_xTaskGetIdleTaskHandle          0
#define INCLUDE_eTaskGetState                   0
#define INCLUDE_xEventGroupSetBitFromISR        1
//sds #define INCLUDE_xTimerPendFunctionCall          0
#define INCLUDE_xTaskAbortDelay                 0
#define INCLUDE_xTaskGetHandle                  0
#define INCLUDE_xTaskResumeFromISR              1

/* A header file that defines trace macro can be included here. */

//sds specifiek voor learning code
#define configUSE_IDLE_HOOK          	 					1
#define configUSE_MALLOC_FAILED_HOOK						1

#define configQUEUE_REGISTRY_SIZE               0 // 10 // nog niet gebruikt in learning code

// sds: increased stack because we want to do printfs from timer task
#define configTIMER_TASK_STACK_DEPTH   					250

// sds: for stack overflow test
#define configCHECK_FOR_STACK_OVERFLOW 					2
#define INCLUDE_uxTaskGetStackHighWaterMark2 		1 // nodig als je de functie wil gebruiken

// sds: for vTaskList
#define configUSE_TRACE_FACILITY								1
#define configUSE_STATS_FORMATTING_FUNCTIONS 		1

// sds: for runtime stats (hoeveel tijd elke task gebruikt)
#define configGENERATE_RUN_TIME_STATS						1

// sds : for deferred interrupt to work
// this included additional freeRTOS code!
#define INCLUDE_xTimerPendFunctionCall 					1

extern volatile unsigned long ulHighFrequencyTimerTicks;
/* ulHighFrequencyTimerTicks is already being incremented at 20KHz.  Just set
its value back to 0. */
// freeRTOS will call this from vTaskStartScheduler()
#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS() ( ulHighFrequencyTimerTicks = 0UL )
#define portGET_RUN_TIME_COUNTER_VALUE()	ulHighFrequencyTimerTicks

// OLD
#define configUSE_APPLICATION_TASK_TAG	0

#endif /* FREERTOS_CONFIG_H */

