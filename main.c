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

/* Global filter-window size (was a macro, now a variable). */
static int g_filterWindowSize = 1;

/* Maximum number of samples the filter buffer can hold. */
#define MAX_FILTER_WINDOW_SIZE 100

/* Graph configuration */
#define LCD_WIDTH              64
#define LCD_HEIGHT             16
#define GRAPH_X_OFFSET         20

/* Task priorities */
#define TASK_PRIORITY_GEN      ( tskIDLE_PRIORITY + 1 )
#define TASK_PRIORITY_FILTER   ( tskIDLE_PRIORITY + 2 )
#define TASK_PRIORITY_GRAPH    ( tskIDLE_PRIORITY + 2 )
#define TASK_PRIORITY_UART     ( tskIDLE_PRIORITY + 2 )

/* Forward declarations */
static void prvSetupHardware(void);
static void vRandomGenTask(void *pvParameters);
static void vLowPassFilterTask(void *pvParameters);
static void vGraphTask(void *pvParameters);
static void vUartReceiveTask(void *pvParameters);
static int  prvGenerateRandomNumber(int min, int max);
static void prvIntToString(int value, char *buffer);

/* Random number seed */
static uint32_t ulRandomSeed = 12345;

/* Inter-task communication queues */
static QueueHandle_t xSensorValueQueue;
static QueueHandle_t xFilteredValueQueue;

/**
 * @brief Application entry point. Sets up hardware, creates tasks and queues, and starts the scheduler.
 */
int main(void)
{
    prvSetupHardware();

    xTaskCreate(vRandomGenTask,     "RandGen",       configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY_GEN,    NULL);
    xTaskCreate(vLowPassFilterTask, "LowPassFilter", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY_FILTER, NULL);
    xTaskCreate(vGraphTask,         "Graph",         configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY_GRAPH,  NULL);

    xSensorValueQueue   = xQueueCreate(10, sizeof(int));
    xFilteredValueQueue = xQueueCreate(10, sizeof(int));
    if (xFilteredValueQueue == NULL)
    {
        OSRAMStringDraw("Queue Fail", 0, 0);
        while(1);
    }

    xTaskCreate(vUartReceiveTask,   "UART_RCV",      configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY_UART,   NULL);

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
 * @brief Generates a pseudo-random integer between 'min' and 'max', inclusive.
 *
 * @param min Lower bound of the random range.
 * @param max Upper bound of the random range.
 * @return Random integer in [min, max].
 */
static int prvGenerateRandomNumber(int min, int max)
{
    ulRandomSeed = (ulRandomSeed * 1664525U + 1013904223U);
    return (ulRandomSeed % (max - min + 1)) + min;
}

/**
 * @brief Converts an integer value to its decimal string representation.
 *
 * @param value  The integer to convert.
 * @param buffer The output buffer for the resulting string.
 */
static void prvIntToString(int value, char *buffer)
{
    char temp[10];
    int i = 0, j = 0;
    if (value == 0) 
    {
        buffer[j++] = '0';
        buffer[j]   = '\0';
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
 * @brief Continuously generates random sensor-like values and sends them to a queue.
 *
 * @param pvParameters Unused.
 */
static void vRandomGenTask(void *pvParameters)
{
    int randomNumber;
    for(;;)
    {
        randomNumber = prvGenerateRandomNumber(TEMP_MIN, TEMP_MAX);
        xQueueSend(xSensorValueQueue, &randomNumber, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(GENERATION_DELAY_MS));
    }
}

/**
 * @brief Implements a low-pass filtering mechanism with a dynamic window size.
 *
 * The first samples in a new window size "warm up" the filter:
 * for sampleCount < window size, the average is over the actual number of samples.
 *
 * @param pvParameters Unused.
 */
static void vLowPassFilterTask(void *pvParameters)
{
    static int buffer[MAX_FILTER_WINDOW_SIZE] = {0};
    static int oldWindowSize = 0;
    static int index = 0;
    static int sum = 0;
    static int sampleCount = 0;

    int value, filteredValue;

    for(;;)
    {
        if (xQueueReceive(xSensorValueQueue, &value, portMAX_DELAY) == pdPASS)
        {
            if (g_filterWindowSize != oldWindowSize)
            {
                oldWindowSize = g_filterWindowSize;
                sum = 0;
                index = 0;
                sampleCount = 0;
                for(int i = 0; i < g_filterWindowSize; i++){
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
 * @brief Reads filtered values from a queue and plots them on the LCD graph.
 *
 * @param pvParameters Unused.
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
 * @brief Receives digits over UART, parses them into an integer, and updates g_filterWindowSize.
 *
 * This task waits for newline or carriage return to confirm the entered number.
 * If valid, g_filterWindowSize is updated in range [1, MAX_FILTER_WINDOW_SIZE].
 *
 * @param pvParameters Unused.
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
