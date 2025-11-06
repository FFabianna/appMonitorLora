#ifndef _BOARD_H_
#define _BOARD_H_

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define MAX_SERIAL_PORTS 1

#define Board_PIN_HIGH  0x01
#define Board_PIN_LOW   0x00

// =======================
// CONFIGURACIÓN DE UART
// =======================

typedef enum {
    SERIAL_FREE = 0,
    REMOTE_ACTIVE,
} SERIAL_STATUS;

typedef enum {
    TYPE_RS232 = 0,   // UART estándar (RAK3272 usa este)
    TYPE_RS485
} SERIAL_TYPE;

typedef struct {
    uart_port_t uartHW;
    TickType_t waitTime;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    gpio_num_t driver;
    SERIAL_TYPE type;
    SERIAL_STATUS serialStatus;
    TaskHandle_t evtUart;
    TaskHandle_t evtLink;
    uint8_t linkInstance;
    uint8_t portIndex;
    int txcnt;
    int rxcnt;
} tcpSerial;

// ==========================
// PINES DE HARDWARE LoRa
// ==========================

// UART para RAK3272
static SERIAL_TYPE HW_type[MAX_SERIAL_PORTS] = {TYPE_RS232};
static uart_port_t HW_uart[MAX_SERIAL_PORTS] = {UART_NUM_1};
static gpio_num_t HW_tx[MAX_SERIAL_PORTS] = {GPIO_NUM_23};
static gpio_num_t HW_rx[MAX_SERIAL_PORTS] = {GPIO_NUM_22};
static gpio_num_t HW_driver[MAX_SERIAL_PORTS] = {GPIO_NUM_NC};

// Pin de reset del módulo LoRa (controlado desde lorauart.c)
#define LORA_RST_PIN GPIO_NUM_21

#endif // _BOARD_H_
