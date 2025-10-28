#include <string.h>
#include <sys/time.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "../library/include/conn.h"
#include "../library/include/logServer.h"

#include "gatewayUart.h"
#include "board.h"

#define UARTGWBUFFERSIZE			        1460    

#define TEST_GET_FIRMWARE_AND_SERIAL        0x000000000001
#define TEST_GET_INSTANTANEOUS_AND_STATE    0x000000000002
#define TEST_GET_ENERGY                     0x000000000004
#define TEST_GET_DATETIME                   0x000000000008
#define TEST_SET_DATETIME                   0x000000000010

#define TEST_GET_LOAD_PROFILE1              0x000000000020
#define TEST_GET_LOAD_PROFILE2              0x000000000040
#define TEST_GET_LOAD_PROFILE1_PERIOD       0x000000000080
#define TEST_GET_LOAD_PROFILE2_PERIOD       0x000000000100
#define TEST_GET_LOAD_PROFILE1_CHANNELS     0x000000000200
#define TEST_GET_LOAD_PROFILE2_CHANNELS     0x000000000400

#define MIN_SERIAL_TIMEOUT                  150      // Timeout in [ms]. Total timeout = 25 * MIN_SERIAL_TIMEOUT

static tcpSerial serial[MAX_SERIAL_PORTS];

#define MAX_SIZE_FRAME  1024
char frameHDLC[MAX_SIZE_FRAME];
int frameSize;
TaskHandle_t printTaskHd;
SemaphoreHandle_t xMutex = NULL;
uart_config_t uart_config;

static int GatewayInitSerial(uint8_t index);
static void GatewayDriverCtrl(uint32_t value, tcpSerial *pSerial);
static int GatewayCreateTask(uint32_t index);
static int GatewayWriteUart(uint8_t *pBuf, uint16_t size);
static int GatewayReadUart(uint8_t *pBuf, uint16_t size);
static int GatewayConfigUart(uint8_t modeDLMS, uint16_t baudrate);

void GatewayUartFxn(void *arg0);

static int GatewayConfigSerialPort(uint8_t index, uint8_t modeDLMS, uint16_t baudrate);

void print_frames_task(void *arg0)
{
    BaseType_t xResult;
    uint32_t ulNotifiedValue;

    memset(frameHDLC, 0, MAX_SIZE_FRAME);

    /* In a production environment, this task control must be improved. */
    while (true) 
    {            
        /* Check new data */
        xResult = xTaskNotifyWait( pdFALSE,     /* Don't clear bits on entry. */
                    ULONG_MAX,                  /* Clear all bits on exit. */
                    &ulNotifiedValue,           /* Stores the notified value. */
                    portMAX_DELAY);

        if(xResult == pdPASS)
        {
            xSemaphoreTake(xMutex, portMAX_DELAY);

            printf(frameHDLC);            
            vTaskDelay(5 / portTICK_PERIOD_MS);

            memset(frameHDLC, 0, MAX_SIZE_FRAME);
            frameSize = 0;

            xSemaphoreGive(xMutex);
        }
    }
}

void GatewayUartFxn(void *arg0)
{  
    char                    buffer[64];
    METER_CONNECTION        conn;
    METER_DATA              data;
    time_t                  timer;
    int                     i, c;
    struct tm*              tm_info;
    DRIVER_ERROR_CODES      err;
    uint32_t dateTimeEnd;
    uint64_t testMask;

    init_params(&conn);

    /* Configure meter, passwords and serial callback */
    conn.model = METER_MODEL_MT880;
    
    /* Set test mask */
    testMask = (
                //TEST_GET_FIRMWARE_AND_SERIAL |
                TEST_GET_INSTANTANEOUS_AND_STATE |
                //TEST_GET_ENERGY |   

                //TEST_GET_DATETIME |
                //TEST_SET_DATETIME |                  
               
                //TEST_GET_LOAD_PROFILE1 |
                //TEST_GET_LOAD_PROFILE2 |
                //TEST_GET_LOAD_PROFILE1_PERIOD |
                //TEST_GET_LOAD_PROFILE2_PERIOD |
                //TEST_GET_LOAD_PROFILE1_CHANNELS |
                //TEST_GET_LOAD_PROFILE2_CHANNELS |                
                0);
    
    /* Set serial callbacks */
    conn.uartRead = &GatewayReadUart;
    conn.uartWrite = &GatewayWriteUart;
    conn.uartConfig = &GatewayConfigUart;
   
    /* Overwrite physical address */
    conn.serverPhyAddress = 31;
    //conn.serverPhyAddress = 79;

    /* Force HDLC address size */
    conn.addressSize = 2;

    /* Max send retry */
    conn.retry = 2;

    /* Create objects for logging */
    xMutex = xSemaphoreCreateMutex();   
    xTaskCreate(print_frames_task, "print_frames_task", 4096, NULL, 2, &printTaskHd);

    /* Infinite loop */
    while (1)
    {      
        /* Start log server */
        conn.log = log_open(printTaskHd, &xMutex, frameHDLC, &frameSize, MAX_SIZE_FRAME);

        //vTaskDelay(5000 / portTICK_PERIOD_MS);
        
        ESP_LOGI(TAG_GW, "--------- Meter model ---------");    
        switch (conn.model) {
 
            case METER_MODEL_MT880:
                ESP_LOGI(TAG_GW, "ISKRA MT880 3 phase");   
                strcpy(conn.pass, "12345678");
                break;
        }

        if (testMask & TEST_GET_FIRMWARE_AND_SERIAL) 
        {
            ESP_LOGI(TAG_GW, "--------- Firmware and serial ---------");

            init_data(&data);

            /* Firmware and serial */
            if ((err = requestFirmwareAndSerial(&conn, &data)) != 0) {
                ESP_LOGE(TAG_GW, "requestFirmwareAndSerial error %d", err);  
            }

            /* Serial meter */
            ESP_LOGI(TAG_GW, "Serial %s", data.serial);    

            /* Firmware */
            ESP_LOGI(TAG_GW, "Firmware %s", data.firmware);

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        if (testMask & TEST_GET_INSTANTANEOUS_AND_STATE) 
        {
            ESP_LOGI(TAG_GW, "--------- Instantaneous and state data ---------");

            init_data(&data);        
            /* Instant measures */
            if ((err = requestInstantMeasures(&conn, &data)) != 0) {
                ESP_LOGE(TAG_GW, "requestInstantMeasures error %d", err);
            }

            /* Clock */
            timer = data.datetime;
            tm_info = localtime(&timer);
            memset(buffer, 0, sizeof(buffer));
            strftime(buffer, 26, "%Y-%m-%d %H:%M:%S", tm_info);
            ESP_LOGI(TAG_GW, "Clock %s", buffer);

            /* Serial meter */
            ESP_LOGI(TAG_GW, "Serial %s", data.serial);    

            /* Firmware */
            ESP_LOGI(TAG_GW, "Firmware %s", data.firmware);

            /* Voltage */
            ESP_LOGI(TAG_GW, "Register\tVotage A\t%8.2f", data.voltage[METER_PHASE_A]);
            ESP_LOGI(TAG_GW, "Register\tVotage B\t%8.2f", data.voltage[METER_PHASE_B]);
            ESP_LOGI(TAG_GW, "Register\tVotage C\t%8.2f", data.voltage[METER_PHASE_C]);
            
            /* Current */
            ESP_LOGI(TAG_GW, "Register\tCurrent A\t%8.2f", data.current[METER_PHASE_A]);
            ESP_LOGI(TAG_GW, "Register\tCurrent B\t%8.2f", data.current[METER_PHASE_B]);
            ESP_LOGI(TAG_GW, "Register\tCurrent C\t%8.2f", data.current[METER_PHASE_C]);

            /* Frequency */
            ESP_LOGI(TAG_GW, "Register\tFrequency\t%8.2f", data.frequency);

            /* Power factor */
            ESP_LOGI(TAG_GW, "Register\tPower factor A\t%8.2f", data.power_factor[METER_PHASE_A]);
            ESP_LOGI(TAG_GW, "Register\tPower factor B\t%8.2f", data.power_factor[METER_PHASE_B]);
            ESP_LOGI(TAG_GW, "Register\tPower factor C\t%8.2f", data.power_factor[METER_PHASE_C]);

            /* Power */
            ESP_LOGI(TAG_GW, "Register\tActive power total import\t%8.2f", data.active_power_total_import);
            ESP_LOGI(TAG_GW, "Register\tActive power total export\t%8.2f", data.active_power_total_export);
            ESP_LOGI(TAG_GW, "Register\tReactive power total import\t%8.2f", data.reactive_power_total_import);
            ESP_LOGI(TAG_GW, "Register\tReactive power total export\t%8.2f", data.reactive_power_total_export);
            ESP_LOGI(TAG_GW, "Register\tApparent power total \t%8.2f", data.apparent_power_total);

            /* Reactive power in quadrants */ 
            ESP_LOGI(TAG_GW, "Register\tReactive power total QI\t%8.2f", data.reactive_power_total_qi);
            ESP_LOGI(TAG_GW, "Register\tReactive power total QII\t%8.2f", data.reactive_power_total_qii);
            ESP_LOGI(TAG_GW, "Register\tReactive power total QIII\t%8.2f", data.reactive_power_total_qiii);
            ESP_LOGI(TAG_GW, "Register\tReactive power total QIV\t%8.2f", data.reactive_power_total_qiv);

            /* Power A */
            ESP_LOGI(TAG_GW, "Register\tActive power import A\t%8.2f", data.active_power_import[METER_PHASE_A]);
            ESP_LOGI(TAG_GW, "Register\tActive power export A\t%8.2f", data.active_power_export[METER_PHASE_A]);
            ESP_LOGI(TAG_GW, "Register\tReactive power import A\t%8.2f", data.reactive_power_import[METER_PHASE_A]);
            ESP_LOGI(TAG_GW, "Register\tReactive power export A\t%8.2f", data.reactive_power_export[METER_PHASE_A]);
            
            /* Power B */
            ESP_LOGI(TAG_GW, "Register\tActive power import B\t%8.2f", data.active_power_import[METER_PHASE_B]);
            ESP_LOGI(TAG_GW, "Register\tActive power export B\t%8.2f", data.active_power_export[METER_PHASE_B]);
            ESP_LOGI(TAG_GW, "Register\tReactive power import B\t%8.2f", data.reactive_power_import[METER_PHASE_B]);
            ESP_LOGI(TAG_GW, "Register\tReactive power export B\t%8.2f", data.reactive_power_export[METER_PHASE_B]);
            
            /* Power C */
            ESP_LOGI(TAG_GW, "Register\tActive power import C\t%8.2f", data.active_power_import[METER_PHASE_C]);
            ESP_LOGI(TAG_GW, "Register\tActive power export C\t%8.2f", data.active_power_export[METER_PHASE_C]);
            ESP_LOGI(TAG_GW, "Register\tReactive power import C\t%8.2f", data.reactive_power_import[METER_PHASE_C]);
            ESP_LOGI(TAG_GW, "Register\tReactive power export C\t%8.2f", data.reactive_power_export[METER_PHASE_C]);
                                    
            /* Angle */
            ESP_LOGI(TAG_GW, "Register\tVoltage Angle B to A\t%8.2f", data.voltageAngleB2A);
            ESP_LOGI(TAG_GW, "Register\tVoltage Angle C to A\t%8.2f", data.voltageAngleC2A);
            //vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        if (testMask & TEST_GET_ENERGY)
        {
            ESP_LOGI(TAG_GW, "--------- Energy data ---------");

            init_data(&data);
            /* Energy measures */
            if ((err = requestEnergyMeasures(&conn, &data)) != 0) {
                ESP_LOGE(TAG_GW, "requestEnergyMeasures error %d", err);   
            }

            /* Clock */
            timer = data.datetime;
            tm_info = localtime(&timer);
            memset(buffer, 0, sizeof(buffer));
            strftime(buffer, 26, "%Y-%m-%d %H:%M:%S", tm_info);
            ESP_LOGI(TAG_GW, "Clock %s", buffer);

            /* Serial meter */
            ESP_LOGI(TAG_GW, "Serial %s", data.serial);    

            /* Firmware */
            ESP_LOGI(TAG_GW, "Firmware %s", data.firmware);

            /* Energy */
            ESP_LOGI(TAG_GW, "Register\tActive energy total import\t%8.2f", data.active_energy_total_import);
            ESP_LOGI(TAG_GW, "Register\tActive energy total export\t%8.2f", data.active_energy_total_export);
            ESP_LOGI(TAG_GW, "Register\tReactive energy total import\t%8.2f", data.reactive_energy_total_import);
            ESP_LOGI(TAG_GW, "Register\tReactive energy total export\t%8.2f", data.reactive_energy_total_export);

            vTaskDelay(1000 / portTICK_PERIOD_MS);  
        }

        if (testMask & TEST_GET_LOAD_PROFILE1)
        {
            ESP_LOGI(TAG_GW, "--------- Load profile 1 ---------");

            time_t now;
            time(&now);
            now -= 18000;

            //dateTimeEnd = now;
            dateTimeEnd = 1730665200;

            init_data(&data);
            /* Get load profile 1 */
            if ((err = requestLoadProfile1(&conn, &data, dateTimeEnd, 2)) != 0) {
                ESP_LOGE(TAG_GW, "requestLoadProfile1 error %d", err);    
            }

            for (i = 0; i < data.totalIntervals; i++) 
            {
                timer = data.dateTimeOfInterval[i];
                tm_info = localtime(&timer);
                memset(buffer, 0, sizeof(buffer));
                strftime(buffer, 26, "%Y-%m-%d %H:%M:%S", tm_info);
                ESP_LOGI(TAG_GW, "Date and time of interval %s", buffer);
                
                for (c = 0; c < data.totalChannels; c++) 
                {
                    ESP_LOGI(TAG_GW, "OBIS %s\tValue\t%8.2f", data.obisChannels + (c * CHANNEL_OBIS_LENGTH), (data.valueOfInterval + c)[i * MAX_CHANNELS]);
                    vTaskDelay(50 / portTICK_PERIOD_MS);
                }
            }
            free(data.dateTimeOfInterval);
            free(data.valueOfInterval);
            free(data.obisChannels);

            vTaskDelay(10000 / portTICK_PERIOD_MS);
        }

        if (testMask & TEST_GET_LOAD_PROFILE2)
        {
            ESP_LOGI(TAG_GW, "--------- Load profile 2 ---------");

            time_t now;
            time(&now);
            now -= 18000;

            //dateTimeEnd = now;
            dateTimeEnd = 1730667600;

            init_data(&data);
            /* Get load profile 2 */
            if ((err = requestLoadProfile2(&conn, &data, dateTimeEnd, 2)) != 0) {
                ESP_LOGE(TAG_GW, "requestLoadProfile2 error %d", err);    
            }

            for (i = 0; i < data.totalIntervals; i++) 
            {
                timer = data.dateTimeOfInterval[i];
                tm_info = localtime(&timer);
                memset(buffer, 0, sizeof(buffer));
                strftime(buffer, 26, "%Y-%m-%d %H:%M:%S", tm_info);
                ESP_LOGI(TAG_GW, "Date and time of interval %s", buffer);
                
                for (c = 0; c < data.totalChannels; c++) 
                {
                  ESP_LOGI(TAG_GW, "OBIS %s\tValue\t%8.2f", data.obisChannels + (c * CHANNEL_OBIS_LENGTH), (data.valueOfInterval + c)[i * MAX_CHANNELS]);
                  vTaskDelay(50 / portTICK_PERIOD_MS);
                }                
            }
            free(data.dateTimeOfInterval);
            free(data.valueOfInterval);
            free(data.obisChannels);

            vTaskDelay(10000 / portTICK_PERIOD_MS);
        }

        if (testMask & TEST_GET_LOAD_PROFILE1_CHANNELS)
        {
            ESP_LOGI(TAG_GW, "--------- Get channels Load profile 1 ---------");

            init_data(&data);
            /* Get channels load profile 1 */
            if ((err = requestChannelsLoadProfile1(&conn, &data)) != 0) {
                ESP_LOGE(TAG_GW, "requestChannelsLoadProfile1 error %d", err);    
            }
  
            for (c = 0; c < data.totalChannels; c++) 
            {
                ESP_LOGI(TAG_GW, "Capture object [%d] OBIS %s Class %d, Index %d Scale %d Unit %d",
                c, data.channels[c].obis, data.channels[c].class, data.channels[c].attribute, data.channels[c].scale, data.channels[c].unit);
            }                              

            free(data.channels);

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        if (testMask & TEST_GET_LOAD_PROFILE2_CHANNELS)
        {
            ESP_LOGI(TAG_GW, "--------- Get channels Load profile 2 ---------");

            init_data(&data);
            /* Get channels load profile 2 */
            if ((err = requestChannelsLoadProfile2(&conn, &data)) != 0) {
                ESP_LOGE(TAG_GW, "requestChannelsLoadProfile2 error %d", err);    
            }
    
            for (c = 0; c < data.totalChannels; c++) 
            {
                ESP_LOGI(TAG_GW, "Capture object [%d] OBIS %s Class %d, Index %d Scale %d Unit %d",
                c, data.channels[c].obis, data.channels[c].class, data.channels[c].attribute, data.channels[c].scale, data.channels[c].unit);
            }                    

            free(data.channels);

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        if (testMask & TEST_GET_LOAD_PROFILE1_PERIOD) 
        {
            ESP_LOGI(TAG_GW, "--------- Get load profile 1 capture period value ---------");

            init_data(&data);        
            /* Load profile capture period value */
            if ((err = requestCapturePeriodLoadProfile1(&conn, &data)) != 0) {
                ESP_LOGE(TAG_GW, "requestLoadProfileCapturePeriod 1 error %d", err);
            }

            ESP_LOGI(TAG_GW, "Capture period for load profile 1 %d", data.capturePeriod);

            vTaskDelay(1000 / portTICK_PERIOD_MS);        
        }

        if (testMask & TEST_GET_LOAD_PROFILE2_PERIOD) 
        {
            ESP_LOGI(TAG_GW, "--------- Get load profile 2 capture period value ---------");

            init_data(&data);        
            /* Load profile capture period value */
            if ((err = requestCapturePeriodLoadProfile2(&conn, &data)) != 0) {
                ESP_LOGE(TAG_GW, "requestLoadProfileCapturePeriod 2 error %d", err);
            }

            ESP_LOGI(TAG_GW, "Capture period for load profile 2 %d", data.capturePeriod);

            vTaskDelay(1000 / portTICK_PERIOD_MS);            
        }  

        if (testMask & TEST_GET_DATETIME) 
        {
            ESP_LOGI(TAG_GW, "--------- Get datetime ---------");

            init_data(&data);        
            /* Instant measures */
            if ((err = requestDateTime(&conn, &data)) != 0) {
                ESP_LOGE(TAG_GW, "requestDateTime error %d", err);
            }

            /* Clock */
            timer = data.datetime;
            tm_info = localtime(&timer);
            memset(buffer, 0, sizeof(buffer));
            strftime(buffer, 26, "%Y-%m-%d %H:%M:%S", tm_info);
            ESP_LOGI(TAG_GW, "Clock %s", buffer);
           
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        /* Overwrite client write password */
        switch (conn.model) {

            case METER_MODEL_MT880:
                strcpy(conn.pass, "12345678");
                break;
        }

        if (testMask & TEST_SET_DATETIME)
        {
            ESP_LOGI(TAG_GW, "--------- Set datetime ---------");

            time_t now;
            time(&now);
            //now = 1730495262;
            now -= 18000;

            /* 1718352000 */
            if ((err = setDateTime(&conn, now)) != 0) {
                ESP_LOGE(TAG_GW, "setDateTime error %d", err);
            } else {        
                ESP_LOGI(TAG_GW, "Set datetime OK");
            } 

            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        /* Close log and free resources */
        log_close(conn.log);

        UBaseType_t val = uxTaskGetStackHighWaterMark(NULL);
        ESP_LOGW(TAG_GW, "Task use %d Heap available %d", val, xPortGetFreeHeapSize());
        
        // while (true) 
        // {
        //    vTaskDelay(5000 / portTICK_PERIOD_MS);
        // }        
    }
	
	vTaskDelete(NULL);
}

int GatewayInit(void)
{
    uint8_t index;

    memset(serial, 0, sizeof(serial));

    for(index = 0; index < MAX_SERIAL_PORTS; index++) {
        /* Install Driver */
        if(GatewayInitSerial(index)) return (-1);
        /* Configure uart */
        if(GatewayConfigSerialPort(index, true, 9600)) return(-2);
    }

    for(index = 0; index < MAX_SERIAL_PORTS; index++) {
        if(GatewayCreateTask(index)) return (-3);
    }

    return(0);
}

static void GatewayDriverCtrl(uint32_t value, tcpSerial *pSerial) {
    /* Only for RS285 drivers */
    if (pSerial->type == TYPE_RS485) {
        gpio_set_level(pSerial->driver, value);
    }
}

static int GatewayInitSerial(uint8_t index)
{
    tcpSerial *pSerial;

    if (index >= MAX_SERIAL_PORTS) return (-1);

    pSerial = serial + index;

    /* Link state */
    pSerial->portIndex = index;

    /* Uart state */
    pSerial->serialStatus = SERIAL_FREE;

    /* Serial Hardware */
    pSerial->type = HW_type[index];
    pSerial->uartHW = HW_uart[index];
    pSerial->tx_pin = HW_tx[index];
    pSerial->rx_pin = HW_rx[index];
    pSerial->driver = HW_driver[index];

    uart_driver_install(pSerial->uartHW, UARTGWBUFFERSIZE * 2, UARTGWBUFFERSIZE * 2, 0, NULL, 0);

    ESP_LOGI(TAG_GW, "Init serial objects");

    return 0;
}

static int GatewayConfigSerialPort(uint8_t index, uint8_t modeDLMS, uint16_t baudrate)
{
    tcpSerial *pSerial;

    if (index >= MAX_SERIAL_PORTS) return (-1);
    pSerial = serial + index;

    if (modeDLMS) 
    {
        uart_config.baud_rate = baudrate;
        uart_config.data_bits = UART_DATA_8_BITS;
        uart_config.parity = UART_PARITY_DISABLE;
        uart_config.stop_bits = UART_STOP_BITS_1;
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uart_config.source_clk = UART_SCLK_APB;
    }
    else
    {
        uart_config.baud_rate = baudrate;
        uart_config.data_bits = UART_DATA_7_BITS;
        uart_config.parity = UART_PARITY_EVEN;
        uart_config.stop_bits = UART_STOP_BITS_1;
        uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
        uart_config.source_clk = UART_SCLK_APB;
    }

    ESP_LOGI(TAG_GW, "Uart reconfigure BR %d DB %d PR %d SB %d", 
    uart_config.baud_rate, 
    uart_config.data_bits, 
    uart_config.parity, 
    uart_config.stop_bits);

    pSerial->waitTime = MIN_SERIAL_TIMEOUT / portTICK_PERIOD_MS;
    uart_param_config(pSerial->uartHW, &uart_config);
    uart_set_pin(pSerial->uartHW, pSerial->tx_pin, pSerial->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    gpio_reset_pin(pSerial->driver);
    gpio_set_direction(pSerial->driver, GPIO_MODE_OUTPUT);
    GatewayDriverCtrl(Board_PIN_LOW, pSerial);   

    ESP_LOGI(TAG_GW, "Configured serial port");

    return(0);
}

static int GatewayCreateTask(uint32_t index)
{
    tcpSerial *pSerial;

    if (index >= MAX_SERIAL_PORTS) return (-1);
    pSerial = serial + index;

    xTaskCreate(GatewayUartFxn, "uartPortTask", 10000, (void*)index, configMAX_PRIORITIES - 1, &pSerial->evtUart);  

    ESP_LOGI(TAG_GW, "Uart Task created");

    return 0;
}

static int GatewayWriteUart(uint8_t *pBuf, uint16_t size)
{
    tcpSerial *pSerial;
    int rc;

    pSerial = serial + 0;

    if (!pSerial->uartHW) {
        return -1;
    }

   /* send frame */
    GatewayDriverCtrl(Board_PIN_HIGH, pSerial);
    rc = uart_write_bytes(pSerial->uartHW, (char*) pBuf, size);
    uart_wait_tx_done(pSerial->uartHW, portMAX_DELAY);
    GatewayDriverCtrl(Board_PIN_LOW, pSerial);

    return(rc);
}

static int GatewayReadUart(uint8_t *pBuf, uint16_t size)
{
    tcpSerial *pSerial;
    int rc = 0;

    pSerial = serial + 0;

    if (!pSerial->uartHW) {
        return -1;
    }

    rc = uart_read_bytes(pSerial->uartHW, (uint8_t*) pBuf, size, pSerial->waitTime);

    return rc;
}

static int GatewayConfigUart(uint8_t modeDLMS, uint16_t baudrate)
{
    int rc = 0;

    rc = GatewayConfigSerialPort(0, modeDLMS, baudrate);

    return rc;
}