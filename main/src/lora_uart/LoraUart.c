/*#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define TAG "LORA_TTN"

// Configuración UART LoRa
#define LORA_UART UART_NUM_2
#define TX_PIN 33
#define RX_PIN 32
#define RESET_PIN 27

// Credenciales TTN - ¡REEMPLAZA CON LAS TUYAS!
#define LORA_DEVEUI "70b3d57ed006c0e4"
#define LORA_APPEUI "ab870a94e0d624be"
#define LORA_APPKEY "95b97967b65f5a6738026ebb68fd325c"

void init_lora_uart(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_param_config(LORA_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LORA_UART, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(LORA_UART, 2048, 2048, 0, NULL, 0));
    
    ESP_LOGI(TAG, "UART LoRa inicializado");
}

void reset_lora(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RESET_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = 0,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    
    ESP_LOGI(TAG, "Resetando módulo LoRa...");
    gpio_set_level(RESET_PIN, 0);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gpio_set_level(RESET_PIN, 1);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "Reset completado");
}

bool send_at_command(const char* cmd, char* response, int response_size, int timeout_ms) {
    // Limpiar buffer de entrada antes de enviar
    uint8_t dummy[128];
    while (uart_read_bytes(LORA_UART, dummy, sizeof(dummy), 100 / portTICK_PERIOD_MS) > 0) {}
    
    ESP_LOGI(TAG, "Enviando: %s", cmd);
    int written = uart_write_bytes(LORA_UART, cmd, strlen(cmd));
    if (written != strlen(cmd)) {
        ESP_LOGE(TAG, "Error escribiendo comando UART");
        return false;
    }
    
    // Esperar respuesta
    vTaskDelay(timeout_ms / portTICK_PERIOD_MS);
    
    int len = uart_read_bytes(LORA_UART, (uint8_t*)response, response_size - 1, 
                             timeout_ms / portTICK_PERIOD_MS);
    if (len > 0) {
        response[len] = '\0';
        ESP_LOGI(TAG, "Respuesta: %s", response);
        return true;
    } else {
        ESP_LOGW(TAG, "Sin respuesta del módulo");
        response[0] = '\0';
        return false;
    }
}

bool wait_for_response(const char* expected, int timeout_seconds) {
    char buffer[512];
    int time_waited = 0;
    
    while (time_waited < timeout_seconds) {
        int len = uart_read_bytes(LORA_UART, (uint8_t*)buffer, sizeof(buffer) - 1, 
                                1000 / portTICK_PERIOD_MS);
        if (len > 0) {
            buffer[len] = '\0';
            ESP_LOGI(TAG, "Recibido: %s", buffer);
            
            if (strstr(buffer, expected)) {
                return true;
            }
            if (strstr(buffer, "ERROR") || strstr(buffer, "FAIL")) {
                ESP_LOGE(TAG, "Error detectado en respuesta");
                return false;
            }
        }
        time_waited++;
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    return false;
}

bool configure_lora_module(void) {
    char response[256];
    
    ESP_LOGI(TAG, "=== CONFIGURANDO MÓDULO LoRa ===");
    
    // 1. Verificar que el módulo responde
    if (!send_at_command("AT\r\n", response, sizeof(response), 2000)) {
        ESP_LOGE(TAG, "Módulo no responde a comandos AT");
        return false;
    }
    
    if (!strstr(response, "OK") && !strstr(response, "+++")) {
        ESP_LOGE(TAG, "Respuesta AT incorrecta: %s", response);
        return false;
    }
    
    // 2. Reset a configuración de fábrica (opcional, solo si hay problemas)
    // send_at_command("AT+FDEFAULT=1\r\n", response, sizeof(response), 3000);
    // vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    // 3. Configurar credenciales TTN
    const char* config_commands[] = {
        "AT+DEVEUI=" LORA_DEVEUI "\r\n",
        "AT+APPEUI=" LORA_APPEUI "\r\n",  
        "AT+APPKEY=" LORA_APPKEY "\r\n",
        "AT+NWM=1\r\n",      // Modo LoRaWAN
        "AT+NJM=1\r\n",      // OTAA
        "AT+CLASS=A\r\n",    // Clase A
        "AT+BAND=8\r\n",     // Banda EU868
        "AT+DR=5\r\n",       // Data Rate 5 (SF7, 125kHz)
        "AT+ADR=1\r\n",      // ADR activado
        "AT+CFM=1\r\n",      // Confirmación activada
    };
    
    for (int i = 0; i < sizeof(config_commands) / sizeof(config_commands[0]); i++) {
        if (!send_at_command(config_commands[i], response, sizeof(response), 2000)) {
            ESP_LOGE(TAG, "Error configurando comando: %s", config_commands[i]);
            return false;
        }
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
    // 4. Guardar configuración
    if (!send_at_command("AT+SAVE\r\n", response, sizeof(response), 3000)) {
        ESP_LOGE(TAG, "Error guardando configuración");
        return false;
    }
    
    ESP_LOGI(TAG, "Configuración completada y guardada");
    return true;
}

bool verify_configuration(void) {
    char response[256];
    ESP_LOGI(TAG, "=== VERIFICANDO CONFIGURACIÓN ===");
    
    const char* verify_commands[] = {
        "AT+DEVEUI=?\r\n",
        "AT+APPEUI=?\r\n", 
        "AT+APPKEY=?\r\n",
        "AT+BAND=?\r\n",
        "AT+NJM=?\r\n",
    };
    
    for (int i = 0; i < sizeof(verify_commands) / sizeof(verify_commands[0]); i++) {
        send_at_command(verify_commands[i], response, sizeof(response), 1500);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    
    return true;
}

bool join_ttn_network(void) {
    ESP_LOGI(TAG, "=== INTENTANDO JOIN A TTN ===");
    
    // Enviar comando JOIN
    char response[256];
    if (!send_at_command("AT+JOIN=1:0:10\r\n", response, sizeof(response), 1000)) {
        ESP_LOGE(TAG, "Error enviando comando JOIN");
        return false;
    }
    
    ESP_LOGI(TAG, "Esperando join... (hasta 30 segundos)");
    
    // Esperar respuesta de join
    if (wait_for_response("JOINED", 30)) {
        ESP_LOGI(TAG, "🎉 JOIN EXITOSO A TTN!");
        return true;
    } else {
        ESP_LOGE(TAG, "❌ JOIN FALLIDO - Timeout o error");
        return false;
    }
}

void send_test_data(void) {
    ESP_LOGI(TAG, "=== ENVIANDO DATOS DE PRUEBA ===");
    
    char response[256];
    const char* test_messages[] = {
        "AT+SEND=1:4:48656C6C6F\r\n",    // "Hello" en hex
        "AT+SEND=1:8:5465737444617461\r\n", // "TestData" en hex
        "AT+SEND=1:2:1234\r\n",          // Datos simples
    };
    
    for (int i = 0; i < sizeof(test_messages) / sizeof(test_messages[0]); i++) {
        if (send_at_command(test_messages[i], response, sizeof(response), 10000)) {
            if (strstr(response, "OK") || strstr(response, "SENDED")) {
                ESP_LOGI(TAG, "✅ Mensaje %d enviado correctamente", i + 1);
            } else {
                ESP_LOGW(TAG, "Respuesta inesperada al enviar: %s", response);
            }
        } else {
            ESP_LOGE(TAG, "Error enviando mensaje %d", i + 1);
        }
        vTaskDelay(10000 / portTICK_PERIOD_MS); // Esperar 10 segundos entre envíos
    }
}

void lora_ttn_task(void *pvParameters) {
    ESP_LOGI(TAG, "=== INICIANDO CONEXIÓN TTN ===");
    
    // 1. Reset inicial del módulo
    reset_lora();
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    // 2. Configurar módulo
    if (!configure_lora_module()) {
        ESP_LOGE(TAG, "Error en configuración. Abortando...");
        vTaskDelete(NULL);
        return;
    }
    
    // 3. Verificar configuración
    verify_configuration();
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    
    // 4. Intentar join a TTN
    if (join_ttn_network()) {
        // 5. Enviar datos de prueba si join fue exitoso
        send_test_data();
        
        // 6. Ciclo principal - enviar datos periódicamente
        ESP_LOGI(TAG, "=== MODO OPERACIÓN NORMAL ===");
        int counter = 0;
        while (1) {
            char payload[32];
            char command[64];
            
            // Crear payload simple con contador
            snprintf(payload, sizeof(payload), "%04X", counter++);
            snprintf(command, sizeof(command), "AT+SEND=1:2:%s\r\n", payload);
            
            char response[256];
            if (send_at_command(command, response, sizeof(response), 10000)) {
                ESP_LOGI(TAG, "Datos enviados: %s", payload);
            }
            
            // Esperar 30 segundos entre envíos
            vTaskDelay(30000 / portTICK_PERIOD_MS);
        }
    } else {
        ESP_LOGE(TAG, "No se pudo conectar a TTN. Verifica:");
        ESP_LOGI(TAG, "1. Credenciales en TTN Console");
        ESP_LOGI(TAG, "2. Gateway en rango");
        ESP_LOGI(TAG, "3. Región correcta (EU868)");
        ESP_LOGI(TAG, "4. Device activado en TTN");
    }
    
    vTaskDelete(NULL);
}
/*  
void app_main(void) {
    ESP_LOGI(TAG, "Iniciando aplicación LoRaWAN + TTN");
    
    // Inicializar UART
    init_lora_uart();
    
    // Iniciar tarea principal
    xTaskCreate(lora_ttn_task, "lora_ttn_task", 8192, NULL, 5, NULL);
}*/