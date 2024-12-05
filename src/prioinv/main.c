/*************************************************************************
 * Programa de prueba basado para LM3S6965.
 *
 * Basado en el demo provisto por FreeRTOS, para ejecutar sobre QEMU.
 *
 * Please ensure to read http://www.freertos.org/portlm3sx965.html
 * which provides information on configuring and running this demo for the
 * various Luminary Micro EKs.
 *************************************************************************/

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Standard includes. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>

/* Hardware library includes. */
#include "hw_memmap.h"
#include "hw_types.h"
#include "hw_sysctl.h"
#include "hw_uart.h"
#include "sysctl.h"
#include "gpio.h"
#include "grlib.h"
#include "osram128x64x4.h"
#include "uart.h"
#include "bitmap.h"

/*-----------------------------------------------------------*/

/* Dimensions the buffer for text messages. */
#define mainMAX_MSG_LEN                     25

/* Constants used when writing strings to the display. */
#define mainCHARACTER_HEIGHT                ( 9 )
#define mainMAX_ROWS_128                    ( mainCHARACTER_HEIGHT * 14 )
#define mainMAX_ROWS_96                     ( mainCHARACTER_HEIGHT * 10 )
#define mainMAX_ROWS_64                     ( mainCHARACTER_HEIGHT * 7 )
#define mainFULL_SCALE                      ( 15 )
#define ulSSI_FREQUENCY                     ( 3500000UL )

/* Semáforo Binario */
SemaphoreHandle_t xSemaforo;

/* Tasks periods. */
#define TASK1_PERIOD 5000
#define TASK2_PERIOD 8000
#define TASK3_PERIOD 12000

/* Tasks WCETs. */
#define TASK1_WCET 1000
#define TASK2_WCET 1000
#define TASK3_WCET 2000

/*-----------------------------------------------------------*/

/*
 * Configure the hardware for the demo.
 */
static void prvSetupHardware( void );

/*
 * Basic polling UART write function.
 */
static void prvPrintString( const char * pcString );

/*
 * Busy wait the specified number of ticks.
 */
static void vBusyWait( TickType_t ticks );

/*
 * Periodic task.
 */
void prvTaskLowPriority(void *pvParameters);
void prvTaskMidPriority(void *pvParameters);
void prvTaskHighPriority(void *pvParameters);

/*-----------------------------------------------------------*/

/* Functions to access the OLED.  The one used depends on the dev kit
being used. */
void ( *vOLEDInit )( uint32_t ) = NULL;
void ( *vOLEDStringDraw )( const char *, uint32_t, uint32_t, unsigned char ) = NULL;
void ( *vOLEDImageDraw )( const unsigned char *, uint32_t, uint32_t, uint32_t, uint32_t ) = NULL;
void ( *vOLEDClear )( void ) = NULL;

/*-----------------------------------------------------------*/

struct xTaskStruct {
    TickType_t wcet;
    TickType_t period;
};

typedef struct xTaskStruct xTask;

xTask task1 = {TASK1_WCET, TASK1_PERIOD};
xTask task2 = {TASK2_WCET, TASK2_PERIOD};
xTask task3 = {TASK3_WCET, TASK3_PERIOD};

/*************************************************************************
 * Main
 *************************************************************************/
int main( void )
{
	/* Initialise the trace recorder. */
	vTraceEnable( TRC_INIT );

    prvSetupHardware();

    /* Map the OLED access functions to the driver functions that are appropriate
    for the evaluation kit being used. */
    //configASSERT( ( HWREG( SYSCTL_DID1 ) & SYSCTL_DID1_PRTNO_MASK ) == SYSCTL_DID1_PRTNO_6965 );
    vOLEDInit = OSRAM128x64x4Init;
    vOLEDStringDraw = OSRAM128x64x4StringDraw;
    vOLEDImageDraw = OSRAM128x64x4ImageDraw;
    vOLEDClear = OSRAM128x64x4Clear;

    /* Initialise the OLED and display a startup message. */
    vOLEDInit( ulSSI_FREQUENCY );

    /* Print Hello World! to the OLED display. */
    static char cMessage[ mainMAX_MSG_LEN ];
    sprintf(cMessage, "Hello World!");
    vOLEDStringDraw( cMessage, 0, 0, mainFULL_SCALE );

    /* Crear el semáforo binario */
    xSemaforo = xSemaphoreCreateBinary();
    if (xSemaforo == NULL)
    {
        prvPrintString("Error: No se pudo crear el semáforo.\n\r");
        while (1);
    }
    xSemaphoreGive(xSemaforo); // Liberar el semáforo inicialmente


    /* Print "Start!" to the UART. */
    prvPrintString("Start!\n\r");

    /* Creates the periodic tasks. */
    xTaskCreate(prvTaskLowPriority, "T1", configMINIMAL_STACK_SIZE + 50, (void *)&task1, 1, NULL);   // Prioridad baja
    xTaskCreate(prvTaskMidPriority, "T2", configMINIMAL_STACK_SIZE + 50, (void *)&task2, 2, NULL);  // Prioridad media
    xTaskCreate(prvTaskHighPriority, "T3", configMINIMAL_STACK_SIZE + 50, (void *)&task3, 3, NULL); // Prioridad alta

    vTraceEnable( TRC_START );

    /* Launch the scheduler. */
    vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task. */
    for( ;; );
}
/*-----------------------------------------------------------*/

void prvSetupHardware( void )
{
    /* If running on Rev A2 silicon, turn the LDO voltage up to 2.75V.  This is
    a workaround to allow the PLL to operate reliably. */
    if( DEVICE_IS_REVA2 )
    {
        SysCtlLDOSet( SYSCTL_LDO_2_75V );
    }

    /* Set the clocking to run from the PLL at 50 MHz */
    SysCtlClockSet( SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_8MHZ );

    /* Initialise the UART - QEMU usage does not seem to require this
    initialisation. */
    SysCtlPeripheralEnable( SYSCTL_PERIPH_UART0 );
    UARTEnable( UART0_BASE );
}
/*-----------------------------------------------------------*/

static void prvPrintString( const char * pcString )
{
    while( *pcString != 0x00 )
    {
        UARTCharPut( UART0_BASE, *pcString );
        pcString++;
    }
}
/*-----------------------------------------------------------*/

static void vBusyWait( TickType_t ticks )
{
    TickType_t elapsedTicks = 0;
    TickType_t currentTick = 0;
    while ( elapsedTicks < ticks ) {
        currentTick = xTaskGetTickCount();
        while ( currentTick == xTaskGetTickCount() ) {
            asm("nop");
        }
        elapsedTicks++;
    }
}
/*-----------------------------------------------------------*/

void prvTaskLowPriority(void *pvParameters)
{
    xTask *task = (xTask *)pvParameters;

    for (;;)
    {
        /* Toma el semáforo */
        if (xSemaphoreTake(xSemaforo, portMAX_DELAY) == pdTRUE)
        {
            prvPrintString("Low: Semáforo tomado.\n\r");

            /* Simular trabajo */
            vBusyWait(task->wcet);

            /* Libera el semáforo */
            prvPrintString("Low: Liberando semáforo.\n\r");
            xSemaphoreGive(xSemaforo);
        }

        vTaskDelay(task->period);
    }
}

void prvTaskMidPriority(void *pvParameters)
{
    xTask *task = (xTask *)pvParameters;

    for (;;)
    {
        prvPrintString("Mid: Ejecutándose.\n\r");

        /* Simular trabajo */
        vBusyWait(task->wcet);

        vTaskDelay(task->period);
    }
}

void prvTaskHighPriority(void *pvParameters)
{
    xTask *task = (xTask *)pvParameters;

    for (;;)
    {
        prvPrintString("High: Intentando tomar el semáforo.\n\r");

        /* Intentar tomar el semáforo */
        if (xSemaphoreTake(xSemaforo, portMAX_DELAY) == pdTRUE)
        {
            prvPrintString("High: Semáforo tomado.\n\r");

            /* Trabajo rápido */
            vBusyWait(task->wcet);

            /* Libera el semáforo */
            prvPrintString("High: Liberando semáforo.\n\r");
            xSemaphoreGive(xSemaforo);
        }

        vTaskDelay(task->period);
    }
}

/*-----------------------------------------------------------*/

void vAssertCalled( const char *pcFile, uint32_t ulLine )
{
    volatile uint32_t ulSetTo1InDebuggerToExit = 0;
    {
        while( ulSetTo1InDebuggerToExit == 0 )
        {
            /* Nothing to do here.  Set the loop variable to a non zero value in
            the debugger to step out of this function to the point that caused
            the assertion. */
            ( void ) pcFile;
            ( void ) ulLine;
        }
    }
}

char* _sbrk_r (struct _reent *r, int incr)
{
    /* Just to keep the linker quiet. */
    ( void ) r;
    ( void ) incr;

    /* Check this function is never called by forcing an assert() if it is. */
    //configASSERT( incr == -1 );

    return NULL;
}

int __error__(char *pcFilename, unsigned long ulLine) {
    return 0;
}

