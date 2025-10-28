#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_sntp.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>


#include "src/lora_uart/LoraUart.h"
///

/* Include Driver version */
#include "../library/include/conn.h"
#include "gatewayUart.h"

#include "driver/uart.h"
#define UART_PORT_LORA UART_NUM_1   // usa el mismo puerto de tu init_uart()
#define BUF_SIZE 128


#define TAG_APP                     "METRUM"
#define TAG_LOG_SERVER              "LOG"
#define EXAMPLE_VERSION             "1.0.0.APP.ICESI"
extern const char*                   driver_version;
 

QueueHandle_t xQueue_uart_lora;
QueueHandle_t xQueue_lora_send_data;
QueueHandle_t xQueue_get_data;
///

void lora_check_info_task(void *pvParameters)
{
    const char *cmd = "AT+APPEUI=?\r\n";
    uint8_t rx_data[BUF_SIZE];
    
    ESP_LOGI("LORA_CHECK", "Enviando comando para leer APPEUI...");
    
    // Envía el comando AT
    uart_write_bytes(UART_PORT_LORA, cmd, strlen(cmd));

    // Espera un poco para que responda
    vTaskDelay(pdMS_TO_TICKS(500));

    // Lee lo que responda el módulo
    int len = uart_read_bytes(UART_PORT_LORA, rx_data, BUF_SIZE - 1, pdMS_TO_TICKS(1000));
    if (len > 0) {
        rx_data[len] = '\0';
        ESP_LOGI("LORA_CHECK", "Respuesta del módulo: %s", rx_data);
    } else {
        ESP_LOGW("LORA_CHECK", "No se recibió respuesta del módulo LoRa.");
    }

    vTaskDelete(NULL); // elimina la tarea después de ejecutarse
}

void app_main(void)
{
    ESP_LOGI(TAG_APP, "DLMS/COSEM Library %s", driver_version);
    ESP_LOGI(TAG_APP, "Example version %s", EXAMPLE_VERSION);

    /* Set logging level per module */
    esp_log_level_set(TAG_GW, ESP_LOG_INFO);
    esp_log_level_set(TAG_DLMS, ESP_LOG_INFO);
    esp_log_level_set(TAG_LOG_SERVER, ESP_LOG_INFO);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* Configuración SNTP */
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "1.co.pool.ntp.org");
    esp_sntp_setservername(1, "1.south-america.pool.ntp.org");
    esp_sntp_setservername(2, "0.south-america.pool.ntp.org");
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Inicializar DLMS */
    GatewayInit();

    /* Inicializar UART y GPIO para LoRa */
    init_uart();
    init_reset_gpio();

    /* Crear colas */
    xQueue_uart_lora = xQueueCreate(QUEQUE_LENGTH, sizeof(msg_t));
    xQueue_lora_send_data = xQueueCreate(QUEQUE_LENGTH, sizeof(msg_t));
    xQueue_get_data = xQueueCreate(QUEQUE_LENGTH, sizeof(msg_t));

    if (xQueue_uart_lora == NULL || xQueue_lora_send_data == NULL || xQueue_get_data == NULL) {
        ESP_LOGE(TAG_APP, "Error creando colas LoRa");
        return;
    }

    /* Crear tareas relacionadas con LoRa */
    xTaskCreate(uart_task, "uart_task", 4096, NULL, 10, NULL);
    xTaskCreate(lora_task, "lora_task", 4096, NULL, 10, NULL);
    xTaskCreate(lora_send_data_task, "lora_send_data_task", 4096, NULL, 10, NULL);

    xTaskCreate(lora_check_info_task, "lora_check_info_task", 2048, NULL, 5, NULL);
}

/*
 * main.c - Aplicación principal
 *
 * Este programa inicializa el entorno ESP-IDF (NVS, red, SNTP) y la librería DLMS 
 * para comunicación con el medidor. Además, configura la interfaz UART y GPIO 
 * para el módulo LoRa (RAK), crea las colas de FreeRTOS necesarias y lanza las 
 * tareas encargadas de manejar la comunicación LoRa (recepción UART, máquina de 
 * estados y envío de datos).
 *
 * Flujo general:
 *  1. Inicialización de logs, NVS, red y sincronización de tiempo.
 *  2. Arranque de la librería DLMS con GatewayInit().
 *  3. Configuración del módulo LoRa (UART + GPIO reset).
 *  4. Creación de colas de comunicación.
 *  5. Creación de tareas LoRa para manejo paralelo de UART y transmisión de datos.
 */

// ==== Diagnóstico LoRa: leer APPEUI ====
