/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Environment includes */
#include "DriverLib.h"

/* Constants for random number generation */
#define TEMP_MIN               12
#define TEMP_MAX               40
#define GENERATION_DELAY_MS    200

/* Global filter-window size */
static int g_filterWindowSize = 10;

/* Maximum number of samples the filter buffer can hold */
#define MAX_FILTER_WINDOW_SIZE 100

/* Graph configuration */
#define LCD_WIDTH              64
#define LCD_HEIGHT             16
#define GRAPH_X_OFFSET         20

/* Top-like task statistics display configuration */
#define TOP_COLUMN_WIDTH       15

/* Task priorities */
#define TASK_PRIORITY_GEN      ( tskIDLE_PRIORITY + 3 )
#define TASK_PRIORITY_FILTER   ( tskIDLE_PRIORITY + 3 )
#define TASK_PRIORITY_GRAPH    ( tskIDLE_PRIORITY + 1 )
#define TASK_PRIORITY_UART     ( tskIDLE_PRIORITY + 1 )
#define TASK_PRIORITY_TOP      ( tskIDLE_PRIORITY + 2 )

/* Forward declarations */
static void prvSetupHardware(void);
static void vRandomGenTask(void *pvParameters);
static void vLowPassFilterTask(void *pvParameters);
static void vGraphTask(void *pvParameters);
static void vUartReceiveTask(void *pvParameters);
static int  prvGenerateRandomNumber(int min, int max);
static void prvIntToString(int value, char *buffer);
static void vTop(void *pvParameters);
void printViaUART(char *string);
void printPadded(const char *str, int totalWidth);

/* Random number seed */
static uint32_t ulRandomSeed = 12345;

/* Inter-task communication queues */
static QueueHandle_t xSensorValueQueue;
static QueueHandle_t xFilteredValueQueue;

/**
 * @brief Main entry point: sets up hardware, creates tasks/queues, and starts the scheduler.
 */
int main(void)
{
    prvSetupHardware();

    xTaskCreate(vRandomGenTask,     "RandGen",       configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY_GEN,    NULL);
    xTaskCreate(vLowPassFilterTask, "LowPassFilter", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY_FILTER, NULL);
    xTaskCreate(vGraphTask,         "Graph",         configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY_GRAPH,  NULL);
    xTaskCreate(vTop,               "Top",           configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY_TOP - 1, NULL);

    xSensorValueQueue   = xQueueCreate(10, sizeof(int));
    xFilteredValueQueue = xQueueCreate(10, sizeof(int));
    if (xFilteredValueQueue == NULL)
    {
        OSRAMStringDraw("Queue Fail", 0, 0);
        while (1);
    }

    xTaskCreate(vUartReceiveTask, "UART_RCV", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY_UART, NULL);

    vTaskStartScheduler();
    return 0;
}

/**
 * @brief Initializes system clock, GPIO, LCD, and UART peripherals.
 */
static void prvSetupHardware(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    OSRAMInit(false);
    OSRAMClear();
    OSRAMStringDraw("Starting...", 0, 0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIODirModeSet(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_DIR_MODE_HW);
    UARTConfigSet(UART0_BASE, 19200, (UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE));

    HWREG(UART0_BASE + UART_O_LCR_H) &= ~0x10;
}

/**
 * @brief Generates a pseudo-random integer in the range [min, max].
 */
static int prvGenerateRandomNumber(int min, int max)
{
    ulRandomSeed = (ulRandomSeed * 1664525U + 1013904223U);
    return (ulRandomSeed % (max - min + 1)) + min;
}

/**
 * @brief Converts an integer to its decimal string representation.
 */
static void prvIntToString(int value, char *buffer)
{
    char temp[10];
    int i = 0, j = 0;
    if (value == 0)
    {
        buffer[j++] = '0';
        buffer[j] = '\0';
        return;
    }
    if (value < 0)
    {
        buffer[j++] = '-';
        value = -value;
    }
    while (value > 0)
    {
        temp[i++] = '0' + (value % 10);
        value /= 10;
    }
    while (i > 0)
    {
        buffer[j++] = temp[--i];
    }
    buffer[j] = '\0';
}

/**
 * @brief Task that generates random sensor-like values and sends them to a queue.
 */
static void vRandomGenTask(void *pvParameters)
{
    int randomNumber;
    for (;;)
    {
        randomNumber = prvGenerateRandomNumber(TEMP_MIN, TEMP_MAX);
        xQueueSend(xSensorValueQueue, &randomNumber, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(GENERATION_DELAY_MS));
    }
}

/**
 * @brief Task implementing a low-pass filter over a dynamic window.
 */
static void vLowPassFilterTask(void *pvParameters)
{
    static int buffer[MAX_FILTER_WINDOW_SIZE] = {0};
    static int oldWindowSize = 0;
    static int index = 0;
    static int sum = 0;
    static int sampleCount = 0;
    int value, filteredValue;

    for (;;)
    {
        if (xQueueReceive(xSensorValueQueue, &value, portMAX_DELAY) == pdPASS)
        {
            if (g_filterWindowSize != oldWindowSize)
            {
                oldWindowSize = g_filterWindowSize;
                sum = 0;
                index = 0;
                sampleCount = 0;
                for (int i = 0; i < g_filterWindowSize; i++)
                {
                    buffer[i] = 0;
                }
            }
            sum -= buffer[index];
            buffer[index] = value;
            sum += value;
            index = (index + 1) % g_filterWindowSize;
            if (sampleCount < g_filterWindowSize)
            {
                sampleCount++;
            }
            filteredValue = sum / sampleCount;
            xQueueSend(xFilteredValueQueue, &filteredValue, portMAX_DELAY);
        }
    }
}

/**
 * @brief Task that reads filtered values from a queue and displays them on the LCD.
 */
static void vGraphTask(void *pvParameters)
{
    int value;
    int xPos = GRAPH_X_OFFSET + 1;
    OSRAMClear();
    for (;;)
    {
        if (xQueueReceive(xFilteredValueQueue, &value, portMAX_DELAY) == pdPASS)
        {
            char buffer[10];
            prvIntToString(value, buffer);
            OSRAMStringDraw(buffer, 8, 1);
            OSRAMStringDraw("t", 14, 0);
            unsigned char yAxis[] = {0xFF, 0xFF};
            OSRAMImageDraw(yAxis, 20, 0, 1, 2);

            uint8_t graphData[2] = {0x00, 0x80};
            uint16_t heigth = 1 + (value - TEMP_MIN) * 15 / (TEMP_MAX - TEMP_MIN);
            if (heigth > 7)
            {
                graphData[0] |= 0x01 << (7 - (heigth - 7));
            }
            else
            {
                graphData[1] |= 0x01 << (7 - heigth);
            }
            OSRAMImageDraw(graphData, xPos, 0, 1, 2);

            uint8_t xAxis[2] = {0x00, 0x80};
            for (int i = xPos + 1; i < 96; i++)
            {
                OSRAMImageDraw(xAxis, i, 0, 1, 2);
            }
            xPos = (xPos < 96) ? xPos + 1 : GRAPH_X_OFFSET + 1;
        }
    }
}

/**
 * @brief Task that receives UART input and updates the filter window size.
 */
static void vUartReceiveTask(void *pvParameters)
{
    static char inputBuf[5] = {0};
    int idx = 0;
    for (;;)
    {
        while ((HWREG(UART0_BASE + UART_O_FR) & UART_FR_RXFE) == 0)
        {
            char c = (char)(HWREG(UART0_BASE + UART_O_DR) & 0xFF);
            if ((c >= '0') && (c <= '9'))
            {
                if (idx < 4)
                {
                    inputBuf[idx++] = c;
                }
            }
            else if ((c == '\n') || (c == '\r'))
            {
                int number = 0;
                for (int i = 0; i < idx; i++)
                {
                    number = number * 10 + (inputBuf[i] - '0');
                }
                if ((number > 0) && (number <= MAX_FILTER_WINDOW_SIZE))
                {
                    g_filterWindowSize = number;
                }
                idx = 0;
                inputBuf[0] = '\0';
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/**
 * @brief Task that prints task statistics in a top-like format via UART.
 */
static void vTop(void *pvParameters)
{
    TaskStatus_t *pxTaskStatusArray;
    volatile UBaseType_t uxArraySize, x;
    unsigned long ulTotalRunTime, ulStatsAsPercentage;
    char buffer[10];
    char percentStr[10];
    uint32_t ulRunTimeCounter;

    uxArraySize = uxTaskGetNumberOfTasks();
    pxTaskStatusArray = pvPortMalloc(uxArraySize * sizeof(TaskStatus_t));

    while (1)
    {
        if (pxTaskStatusArray != NULL)
        {
            printViaUART("\n");
            printPadded("Task", TOP_COLUMN_WIDTH);
            printPadded("CpuTicks", TOP_COLUMN_WIDTH);
            printPadded("Percentage", TOP_COLUMN_WIDTH);
            printPadded("Stack", TOP_COLUMN_WIDTH);
            printViaUART("\r\n");
            for (x = 0; x < TOP_COLUMN_WIDTH * 4; x++)
            {
                printViaUART("-");
            }
            printViaUART("\r\n");

            uxArraySize = uxTaskGetSystemState(pxTaskStatusArray, uxArraySize, &ulTotalRunTime);
            ulTotalRunTime /= 100UL; /* For percentage calculations */

            if (ulTotalRunTime > 0)
            {
                for (x = 0; x < uxArraySize; x++)
                {
                    ulStatsAsPercentage = pxTaskStatusArray[x].ulRunTimeCounter / ulTotalRunTime;
                    ulRunTimeCounter = pxTaskStatusArray[x].ulRunTimeCounter;

                    printPadded(pxTaskStatusArray[x].pcTaskName, TOP_COLUMN_WIDTH);
                    prvIntToString(ulRunTimeCounter, buffer);
                    printPadded(buffer, TOP_COLUMN_WIDTH);

                    if (ulStatsAsPercentage > 0UL)
                    {
                        prvIntToString(ulStatsAsPercentage, percentStr);
                        int i = 0;
                        while (percentStr[i] != '\0')
                        {
                            i++;
                        }
                        percentStr[i] = '%';
                        percentStr[i + 1] = '\0';
                        printPadded(percentStr, TOP_COLUMN_WIDTH);
                    }
                    else
                    {
                        printPadded("<1%", TOP_COLUMN_WIDTH);
                    }

                    prvIntToString(uxTaskGetStackHighWaterMark(pxTaskStatusArray[x].xHandle), buffer);
                    printViaUART(buffer);
                    printViaUART(" words\r\n");
                }
            }
        }
        else
        {
            printViaUART("Struct pxTaskStatusArray NULL\r\n");
        }
        vPortFree(pxTaskStatusArray);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/**
 * @brief Prints a string padded with spaces to reach a fixed width.
 */
void printPadded(const char *str, int totalWidth)
{
    int count = 0;
    while (str[count] != '\0')
    {
        char c[2] = { str[count], '\0' };
        printViaUART(c);
        count++;
    }
    while (count < totalWidth)
    {
        printViaUART(" ");
        count++;
    }
}

/**
 * @brief Sends a null-terminated string via UART.
 */
void printViaUART(char *string)
{
    while (*string != '\0')
    {
        UARTCharPut(UART0_BASE, *string);
        string++;
    }
}
