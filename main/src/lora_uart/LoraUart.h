#ifndef _LORA_UART_H_
#define _LORA_UART_H_

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define BUFF_SIZE_TX 256
#define BUFF_SIZE_RX 256
#define QUEQUE_LENGTH 20
#define NUM_REGISTER_READ 8

typedef enum {
    sReset,
    sConfig_AT,
    sJoin,
    sSend_Lora_Frame,
    sSend_Complete,
    sSend_Error,
    sData,
    sData_complete,
    sSend_Data
} lora_signal_t;

typedef enum {
    Idle_sending,
    Send_Data,
    Wating_Send_Data
} LORA_SEND_STATE_ENUM;

typedef enum {
    Reset_lora,
    Send_Lora_AT_Conf,
    Wating_AT_Response,
    Wating_Make_Join,
    Idle_lora,
    Wating_Send_Lora_Frame
} LORA_STATE_ENUM;

typedef struct {
    lora_signal_t signal;
    int value;
    char *lora_tx;
    int lora_tx_length;
    char *lora_rx;
    int lora_rx_length;
    float *data;
} msg_t;

extern QueueHandle_t xQueue_uart_lora;
extern QueueHandle_t xQueue_lora_send_data;
extern QueueHandle_t xQueue_get_data;

// Arrays de strings para debug
extern const char* TO_LORA_STRING[];
extern const char* TO_LORA_SEND_STRING[];
extern const char* LORA_STATE_STRING[];
extern const char* LORA_SEND_STATE_STRING[];

// Inicialización
void init_uart(void);
void init_reset_gpio(void);

// Tareas principales
void lora_task(void *pvParameters);
void lora_send_data_task(void *pvParameters);
void uart_task(void *pvParameters);

// Utilidades
void create_lora_hex_string(int msj_num, float val1, float val2, char *dst);
void read_lora_credentials(void);
void reset_lora_func(void);

#endif