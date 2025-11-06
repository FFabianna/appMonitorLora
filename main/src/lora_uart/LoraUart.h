#ifndef _LORA_UART_H_
#define _LORA_UART_H_

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"

// ==========================
//  CONFIGURACIÓN GENERAL
// ==========================
#define BUFF_SIZE_TX 256
#define BUFF_SIZE_RX 256
#define QUEUE_LENGTH 20
#define NUM_REGISTER_READ 8
#define LORA_MAX_PAYLOAD 128
// ==========================
//  ENUMERACIONES
// ==========================

// Señales que se envían entre tareas
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

// Estados del envío de datos LoRa
typedef enum {
    Idle_sending,
    Send_Data,
    Wating_Send_Data
} LORA_SEND_STATE_ENUM;

// Estados de la máquina de estados principal LoRa
typedef enum {
    Reset_lora,
    Send_Lora_AT_Conf,
    Wating_AT_Response,
    Wating_Make_Join,
    Idle_lora,
    Wating_Send_Lora_Frame
} LORA_STATE_ENUM;

// ==========================
//  ESTRUCTURAS DE DATOS
// ==========================

typedef struct {
    lora_signal_t signal;   // Tipo de mensaje o evento
    int value;              // Valor opcional asociado
    char *lora_tx;          // Datos TX
    int lora_tx_length;     // Tamaño TX
    char *lora_rx;          // Datos RX
    int lora_rx_length;     // Tamaño RX
    uint8_t data[LORA_MAX_PAYLOAD];
    uint16_t length;
    float *data;            // Puntero a datos de sensores (si aplica)
} msg_t;

// ==========================
//  VARIABLES GLOBALES
// ==========================

extern QueueHandle_t xQueue_uart_lora;
extern QueueHandle_t xQueue_lora_send_data;
extern QueueHandle_t xQueue_get_data;

// Arrays de strings (para debug legible)
extern const char* TO_LORA_STRING[];
extern const char* TO_LORA_SEND_STRING[];
extern const char* LORA_STATE_STRING[];
extern const char* LORA_SEND_STATE_STRING[];

// ==========================
//  FUNCIONES PRINCIPALES
// ==========================

void init_uart(void);
void init_reset_gpio(void);

// Tareas RTOS
void lora_task(void *pvParameters);
void lora_send_data_task(void *pvParameters);
void uart_task(void *pvParameters);

// ==========================
//  UTILIDADES
// ==========================

/**
 * @brief Convierte dos floats en un payload hexadecimal TTN y genera comando AT+SEND
 */
void create_lora_hex_string(int msj_num, float val1, float val2, char *dst);

/**
 * @brief Lee credenciales LoRa desde memoria (si aplica)
 */
void read_lora_credentials(void);

/**
 * @brief Reinicia el módulo RAK3272 mediante GPIO controlado
 */
void reset_lora_func(void);

#endif // _LORA_UART_H_
