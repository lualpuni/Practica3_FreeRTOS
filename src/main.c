/* Standard includes. */
#include <stdint.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"

/* TODO Add any manufacture supplied header files necessary for CMSIS functions
to be available here. */
#include "stm32f10x.h"
#include "STM32vldiscovery.h"

/* Priorities at which the tasks are created.  The event semaphore task is
given the maximum priority of ( configMAX_PRIORITIES - 1 ) to ensure it runs as
soon as the semaphore is given. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define	mainQUEUE_SEND_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainEVENT_SEMAPHORE_TASK_PRIORITY	( configMAX_PRIORITIES - 1 )

/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the portTICK_RATE_MS constant. */
#define mainQUEUE_SEND_PERIOD_MS			( 200 / portTICK_RATE_MS )

/* The period of the example software timer, specified in milliseconds, and
converted to ticks using the portTICK_RATE_MS constant. */
#define mainSOFTWARE_TIMER_PERIOD_MS		( 1000 / portTICK_RATE_MS )

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH					( 20 )

static void prvSetupHardware( void );
static void Task_led( void *pvParameters );
static void Task_debounce( void *pvParameters );

static xQueueHandle xQueue;
static xSemaphoreHandle event_1;
static xSemaphoreHandle event_2;


static void prvSetupHardware( void );
static void GPIO_Conf(void);
void EXTILine1_Config(void);
void EXTILine2_Config(void);

void USART1_Init(void);

GPIO_InitTypeDef  GPIO_Leds;
GPIO_InitTypeDef  GPIO_Buttons;
EXTI_InitTypeDef  EXTI_InitStructure;


int main(void)
{
	prvSetupHardware();
	vSemaphoreCreateBinary( event_1 );
	vSemaphoreCreateBinary( event_2 );

	xQueue = xQueueCreate( 	mainQUEUE_LENGTH, sizeof( uint32_t ) );
	xTaskCreate(Task_led,( signed char * ) "LEDS",configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate(Task_debounce,( signed char * ) "DEBOUNCE",configMINIMAL_STACK_SIZE+1, NULL, 1, NULL );
	vTaskStartScheduler();

	for( ;; );
}
//-------------

static void Task_led( void *pvParameters )
{
uint32_t ReceivedValue;

	for( ;; )
	{
		if (xQueueReceive( xQueue, &ReceivedValue, portMAX_DELAY ))
		{
			switch(ReceivedValue)
			{
			case 1:
				STM32vldiscovery_LEDToggle(LED3);
				break;
			case 2:
				STM32vldiscovery_LEDToggle(LED4);
				break;
			default:
				break;
			}
		}
		else
		{

		}
		vTaskDelay(200);
	}
	vTaskDelete(NULL);
}

/*-----------------------------------------------------------*/
static void Task_debounce( void *pvParameters )
{
	for( ;; )
		{
		EXTI_ClearFlag(EXTI_Line1);
		EXTI_ClearFlag(EXTI_Line2);

		xSemaphoreGive(event_1);
		xSemaphoreGive(event_2);

		vTaskDelay(150);

		}

}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	USART1_Init(); //USART initialization

    GPIO_Conf();
	EXTILine1_Config();
	EXTILine2_Config();

	STM32vldiscovery_LEDOn(0);
	STM32vldiscovery_LEDOn(1);

}

static void GPIO_Conf(void)
{

	/* GPIOA and GPIOB clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                         RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	//GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	//GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	//GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA , GPIO_PinSource0);
}

void EXTILine1_Config(void)
{

  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // Enable clocks first so peripherals work, AFIO required for EXTI
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

  // Configure IT sources
  GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPD; // You sure? For a button I'd pull up

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Connect sources to EXTI Lines

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource1);

 // Configure Button EXTI lines
  EXTI_InitStructure.EXTI_Line = EXTI_Line1;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // Enable  EXTI Interrupts
  NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void EXTILine2_Config(void)
{

  EXTI_InitTypeDef EXTI_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  // Enable clocks first so peripherals work, AFIO required for EXTI
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

  // Configure IT sources
  GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPD; // You sure? For a button I'd pull up

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  // Connect sources to EXTI Lines

  GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource2);

 // Configure Button EXTI lines
  EXTI_InitStructure.EXTI_Line = EXTI_Line2;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);

  // Enable  EXTI Interrupts
  NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/* Handle PD1 interrupt */
void EXTI1_IRQHandler(void) {
	uint32_t value = 1;
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line1) != RESET) {
    	if(xSemaphoreTakeFromISR(event_1,portMAX_DELAY))
    	{
    		xQueueSendFromISR( xQueue, &value, 0 );
    	}
        EXTI_ClearITPendingBit(EXTI_Line1);
    }
}

/* Handle PD2 interrupt */
void EXTI2_IRQHandler(void) {
	uint32_t value = 2;
    /* Make sure that interrupt flag is set */
    if (EXTI_GetITStatus(EXTI_Line2) != RESET) {
    	if(xSemaphoreTakeFromISR(event_2,portMAX_DELAY))
    	{
    		xQueueSendFromISR( xQueue, &value, 0 );
		}
        EXTI_ClearITPendingBit(EXTI_Line2);
    }
}


void USART1_Init(void){
	 /* USART configuration structure for USART1 */
    USART_InitTypeDef usart1_init_struct;

	NVIC_InitTypeDef NVIC_InitStructure;
    /* Bit configuration structure for GPIOA PIN9 and PIN10 */
    GPIO_InitTypeDef gpioa_init_struct;

    /* Enalbe clock for USART1, AFIO and GPIOA */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_GPIOA, ENABLE);

    /* GPIOA PIN9 alternative function Tx */
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_9;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpioa_init_struct);
    /* GPIOA PIN9 alternative function Rx */
    gpioa_init_struct.GPIO_Pin = GPIO_Pin_10;
    gpioa_init_struct.GPIO_Speed = GPIO_Speed_50MHz;
    gpioa_init_struct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpioa_init_struct);


    /* Baud rate 9600, 8-bit data, One stop bit
     * No parity, Do both Rx and Tx, No HW flow control
     */
    usart1_init_struct.USART_BaudRate = 9600;
    usart1_init_struct.USART_WordLength = USART_WordLength_8b;
    usart1_init_struct.USART_StopBits = USART_StopBits_1;
    usart1_init_struct.USART_Parity = USART_Parity_No ;
    usart1_init_struct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    usart1_init_struct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    /* Configure USART1 */
    USART_Init(USART1, &usart1_init_struct);
    /* Enable RXNE interrupt */
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    /* Enable USART1 global interrupt */

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		 // we want to configure the USART2 interrupts
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;// this sets the priority group of the USART2 interrupts
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		 // this sets the subpriority inside the group
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			 // the USART2 interrupts are globally enabled
	NVIC_Init(&NVIC_InitStructure);

	/* Enable USART1 */
    USART_Cmd(USART1, ENABLE);

}
	// this is the interrupt request handler (IRQ) for ALL USART2 interrupts
void USART1_IRQHandler(void){
	if( USART_GetITStatus(USART1, USART_IT_RXNE) != RESET ){
		char character;
		uint32_t value;
		character = (USART_ReceiveData(USART1));
		if (character == '1' ||character == '2')
		{
			value = (uint32_t) character - 48; //Convert ascii to integer
			xQueueSendFromISR( xQueue, &value, 0 );
		}
	}
}

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
static uint32_t ulCount = 0;


}
/*-----------------------------------------------------------*/
