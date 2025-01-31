/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Environment includes */
#include "DriverLib.h"

/* Constants for random number generation */
#define TEMP_MIN               12
#define TEMP_MAX               40
#define GENERATION_DELAY_MS    1000

/* Filter configuration */
#define FILTER_WINDOW_SIZE     30 /* Number of samples for low-pass filter */

/* Graph configuration */
#define LCD_WIDTH              64      
#define LCD_HEIGHT             16       
#define GRAPH_X_OFFSET         20     

/* Task priorities */
#define TASK_PRIORITY_GEN      (tskIDLE_PRIORITY + 1)
#define TASK_PRIORITY_FILTER   (tskIDLE_PRIORITY + 2)
#define TASK_PRIORITY_GRAPH    (tskIDLE_PRIORITY + 2)

/* Function prototypes */
static void prvSetupHardware(void);
static void vRandomGenTask(void *pvParameters);
static void vLowPassFilterTask(void *pvParameters);
static void vGraphTask(void *pvParameters);

/* Random number seed */
static uint32_t ulRandomSeed = 12345;

/* Queue handles for inter-task communication */
static QueueHandle_t xSensorValueQueue;
static QueueHandle_t xFilteredValueQueue;

/* Function prototypes */
static int prvGenerateRandomNumber(int min, int max);
static void prvIntToString(int value, char *buffer);

int main(void)
{
    /* Initialize hardware */
    prvSetupHardware();
    
    /* Create the tasks */
    xTaskCreate(vRandomGenTask, "RandGen", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY_GEN, NULL);
    xTaskCreate(vLowPassFilterTask, "LowPassFilter", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY_FILTER, NULL);
    xTaskCreate(vGraphTask, "Graph", configMINIMAL_STACK_SIZE, NULL, TASK_PRIORITY_GRAPH, NULL);
    
    /* Create the queues */
    xSensorValueQueue = xQueueCreate(10, sizeof(int));
    xFilteredValueQueue = xQueueCreate(10, sizeof(int));
    if (xFilteredValueQueue == NULL)
    {
        OSRAMStringDraw("Queue Fail", 0, 0);
        while(1);
    }
    
    /* Start the scheduler */
    vTaskStartScheduler();
    
    return 0;
}

static void prvSetupHardware(void)
{
    SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    OSRAMInit(false);
    OSRAMClear();
    OSRAMStringDraw("Starting...", 0, 0);
}

static int prvGenerateRandomNumber(int min, int max)
{
    ulRandomSeed = (ulRandomSeed * 1664525U + 1013904223U);
    return (ulRandomSeed % (max - min + 1)) + min;
}

static void prvIntToString(int value, char *buffer)
{
    char temp[10];
    int i = 0, j = 0;
    if (value == 0) {
        buffer[j++] = '0';
        buffer[j] = '\0';
        return;
    }
    if (value < 0) {
        buffer[j++] = '-';
        value = -value;
    }
    while (value > 0) {
        temp[i++] = '0' + (value % 10);
        value /= 10;
    }
    while (i > 0) {
        buffer[j++] = temp[--i];
    }
    buffer[j] = '\0';
}

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

static void vLowPassFilterTask(void *pvParameters)
{
    int buffer[FILTER_WINDOW_SIZE] = {0};
    int index = 0, sum = 0, value, filteredValue;
    for(;;)
    {
        if (xQueueReceive(xSensorValueQueue, &value, portMAX_DELAY) == pdPASS)
        {
            sum -= buffer[index];
            buffer[index] = value;
            sum += value;
            index = (index + 1) % FILTER_WINDOW_SIZE;
            filteredValue = sum / FILTER_WINDOW_SIZE;
            xQueueSend(xFilteredValueQueue, &filteredValue, portMAX_DELAY);
        }
    }
}

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
