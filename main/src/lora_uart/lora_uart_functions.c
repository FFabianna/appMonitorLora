#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "LoraUart.h"

#define TAG "LORA_UART"

// Colas (ya declaradas en LoraUart.h)
QueueHandle_t xQueue_uart_lora;
QueueHandle_t xQueue_lora_send_data;
QueueHandle_t xQueue_get_data;

// Configuración UART para LoRa
#define LORA_UART_NUM UART_NUM_2
#define LORA_TX_PIN 33
#define LORA_RX_PIN 32
#define LORA_RESET_PIN 27

void init_uart(void) {
    ESP_LOGI(TAG, "Inicializando UART LoRa...");
    
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(LORA_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LORA_UART_NUM, LORA_TX_PIN, LORA_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(LORA_UART_NUM, 2048, 2048, 0, NULL, 0));
    
    ESP_LOGI(TAG, "UART LoRa inicializado en GPIO TX:%d, RX:%d", LORA_TX_PIN, LORA_RX_PIN);
}

void init_reset_gpio(void) {
    ESP_LOGI(TAG, "Inicializando GPIO reset...");
    
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LORA_RESET_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "GPIO reset inicializado en pin %d", LORA_RESET_PIN);
}

// Funciones placeholder para evitar errores de enlace
void uart_task(void *pvParameters) {
    ESP_LOGI(TAG, "uart_task iniciada");
    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void lora_task(void *pvParameters) {
    ESP_LOGI(TAG, "lora_task iniciada");
    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void lora_send_data_task(void *pvParameters) {
    ESP_LOGI(TAG, "lora_send_data_task iniciada");
    while(1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}