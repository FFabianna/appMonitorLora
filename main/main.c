#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
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

#include "driver/uart.h"
#include "driver/gpio.h"

#include "src/lora_uart/LoraUart.h"
#include "gatewayUart.h"

#define TAG_APP "METRUM"
#define EXAMPLE_VERSION "1.0.0.APP.ICESI"

// Semáforo para acceso exclusivo al UART de LoRa
static SemaphoreHandle_t xLoraUartMutex;

// Función segura para escribir en UART
bool lora_uart_write_safe(const char* data, int timeout_ms) {
    if (xSemaphoreTake(xLoraUartMutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        uart_write_bytes(UART_NUM_2, data, strlen(data));
        xSemaphoreGive(xLoraUartMutex);
        return true;
    }
    return false;
}

// Función segura para leer del UART
int lora_uart_read_safe(uint8_t* buffer, int max_len, int timeout_ms) {
    int bytes_read = 0;
    if (xSemaphoreTake(xLoraUartMutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        bytes_read = uart_read_bytes(UART_NUM_2, buffer, max_len, pdMS_TO_TICKS(timeout_ms));
        xSemaphoreGive(xLoraUartMutex);
    }
    return bytes_read;
}

// Limpiar buffer de UART de forma segura
void lora_uart_clean_safe(int timeout_ms) {
    if (xSemaphoreTake(xLoraUartMutex, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        uint8_t dummy[128];
        int cleaned;
        do {
            cleaned = uart_read_bytes(UART_NUM_2, dummy, sizeof(dummy), pdMS_TO_TICKS(100));
        } while (cleaned > 0);
        xSemaphoreGive(xLoraUartMutex);
    }
}

// Enviar comando y esperar respuesta de forma segura
bool lora_send_command(const char* command, char* response, int response_size, int wait_time_ms) {
    lora_uart_clean_safe(1000);
    
    if (!lora_uart_write_safe(command, 2000)) {
        ESP_LOGE("LORA_TEST", "Error al enviar comando: %s", command);
        return false;
    }
    
    vTaskDelay(pdMS_TO_TICKS(wait_time_ms));
    
    int len = lora_uart_read_safe((uint8_t*)response, response_size - 1, wait_time_ms + 1000);
    if (len > 0) {
        response[len] = '\0';
        return true;
    }
    return false;
}

void lora_ttn_test_task(void *pvParameters) {
    ESP_LOGI("LORA_TEST", "=== TEST TTN CONFIGURACIÓN COMPLETA ===");
    
    char response[1024];
    bool credential_DevEUI_ok = false;
    bool credential_AppEUI_ok = false;
    bool credential_AppKey_ok = false;

    // 1. Reset y espera inicial
    ESP_LOGI("LORA_TEST", "Reset y espera inicial...");
    gpio_set_level(GPIO_NUM_27, 0);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    gpio_set_level(GPIO_NUM_27, 1);
    vTaskDelay(8000 / portTICK_PERIOD_MS);

    // 2. Limpiar buffer exhaustivamente
    ESP_LOGI("LORA_TEST", "Limpieza exhaustiva de buffer...");
    lora_uart_clean_safe(2000);

    // 3. CONFIGURACIÓN COMPLETA DEL MÓDULO LoRa
    ESP_LOGI("LORA_TEST", "=== CONFIGURACIÓN COMPLETA DEL MÓDULO LoRa ===");
    
    // Configuraciones esenciales para TTN
    const char* module_config[] = {
        // Configuración básica del módulo
        "AT+ATM=1\r\n",              // Modo AT
        "AT+CLASS=C\r\n",            // Clase C (recepción continua)
        "AT+NJM=1\r\n",              // Modo join por OTAA
        "AT+DR=5\r\n",               // Data Rate 5 (SF7, BW125kHz) - Óptimo para TTN
        "AT+BAND=6\r\n",             // Banda US915 (para Australia/Américas)
        "AT+CHMASK=0000\r\n",        // Máscara de canales (todos habilitados)
        "AT+CHE=1:2:3:4:5:6:7:8\r\n", // Habilitar canales específicos
        "AT+CFM=1\r\n",              // Confirmación habilitada
        "AT+RX2DR=8\r\n",            // Data Rate para segunda ventana de recepción
        "AT+TXP=15\r\n",             // Potencia máxima de transmisión (15 dBm)
        "AT+ADAPTIVE_DATA_RATE=0\r\n", // ADR deshabilitado inicialmente
        "AT+RETRIES=5\r\n",          // Número de reintentos
    };
    
    // Aplicar configuración del módulo
    for(int i = 0; i < 12; i++) {
        ESP_LOGI("LORA_TEST", "Configurando módulo: %s", module_config[i]);
        lora_uart_clean_safe(1000);
        lora_uart_write_safe(module_config[i], 2000);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        
        char config_response[256];
        int len = lora_uart_read_safe((uint8_t*)config_response, sizeof(config_response)-1, 3000);
        if(len > 0) {
            config_response[len] = '\0';
            ESP_LOGI("LORA_TEST", "Respuesta: %s", config_response);
        }
    }

    // 4. Verificar credenciales actuales
    ESP_LOGI("LORA_TEST", "=== VERIFICANDO CREDENCIALES ACTUALES ===");
    
    // Verificar DevEUI
    if (lora_send_command("AT+DEVEUI=?\r\n", response, sizeof(response), 4000)) {
        ESP_LOGI("LORA_TEST", "📖 DevEUI Actual: %s", response);
        if (strstr(response, "70B3D57ED0073AF5") || strstr(response, "70b3d57ed0073af5")) {
            ESP_LOGI("LORA_TEST", "✅ DevEUI configurado correctamente");
            credential_DevEUI_ok = true;
        } else {
            ESP_LOGE("LORA_TEST", "❌ DevEUI incorrecto o no configurado");
        }
    }

    // Verificar AppEUI
    if (lora_send_command("AT+APPEUI=?\r\n", response, sizeof(response), 4000)) {
        ESP_LOGI("LORA_TEST", "📖 AppEUI Actual: %s", response);
        if (strstr(response, "AB870A94E0D624BE") || strstr(response, "ab870a94e0d624be")) {
            ESP_LOGI("LORA_TEST", "✅ AppEUI configurado correctamente");
            credential_AppEUI_ok = true;
        } else {
            ESP_LOGE("LORA_TEST", "❌ AppEUI incorrecto o no configurado");
        }
    }

    // Verificar AppKey
    if (lora_send_command("AT+APPKEY=?\r\n", response, sizeof(response), 4000)) {
        ESP_LOGI("LORA_TEST", "📖 AppKey Actual: %s", response);
        if (strstr(response, "95B97967B65F5A67") || strstr(response, "95b97967b65f5a67")) {
            ESP_LOGI("LORA_TEST", "✅ AppKey configurado correctamente");
            credential_AppKey_ok = true;
        } else {
            ESP_LOGE("LORA_TEST", "❌ AppKey incorrecto o no configurado");
        }
    }

    // 5. Configurar credenciales solo si es necesario
    if (!credential_DevEUI_ok || !credential_AppEUI_ok || !credential_AppKey_ok) {
        ESP_LOGI("LORA_TEST", "=== CONFIGURANDO CREDENCIALES ===");
        
        const char* setup_commands[] = {
            "AT+DEVEUI=70B3D57ED0073AF5\r\n",
            "AT+APPEUI=AB870A94E0D624BE\r\n",  
            "AT+APPKEY=95B97967B65F5A6738026EBB68FD325C\r\n",
            "AT+SAVE\r\n",
        };  
        
        for(int i = 0; i < 4; i++) {
            ESP_LOGI("LORA_TEST", "Configurando: %s", setup_commands[i]);
            if (lora_send_command(setup_commands[i], response, sizeof(response), 5000)) {
                ESP_LOGI("LORA_TEST", "Respuesta: %s", response);
            }
            vTaskDelay(3000 / portTICK_PERIOD_MS);          
        }

        // Verificar configuración completa
        ESP_LOGI("LORA_TEST", "=== VERIFICACIÓN FINAL DE CONFIGURACIÓN ===");
        
        bool all_credentials_correct = true;
        
        // Verificar DevEUI final
        if (lora_send_command("AT+DEVEUI=?\r\n", response, sizeof(response), 4000)) {
            ESP_LOGI("LORA_TEST", "🔍 DevEUI Verificado: %s", response);
            if (strstr(response, "70B3D57ED0073AF5") || strstr(response, "70b3d57ed0073af5")) {
                ESP_LOGI("LORA_TEST", "✅ DevEUI configurado correctamente");
            } else {
                ESP_LOGE("LORA_TEST", "❌ DevEUI NO configurado correctamente");
                all_credentials_correct = false;
            }
        }

        // Verificar AppEUI final
        if (lora_send_command("AT+APPEUI=?\r\n", response, sizeof(response), 4000)) {
            ESP_LOGI("LORA_TEST", "🔍 AppEUI Verificado: %s", response);
            if (strstr(response, "AB870A94E0D624BE") || strstr(response, "ab870a94e0d624be")) {
                ESP_LOGI("LORA_TEST", "✅ AppEUI configurado correctamente");
            } else {
                ESP_LOGE("LORA_TEST", "❌ AppEUI NO configurado correctamente");
                all_credentials_correct = false;
            }
        }

        // Verificar AppKey final
        if (lora_send_command("AT+APPKEY=?\r\n", response, sizeof(response), 4000)) {
            ESP_LOGI("LORA_TEST", "🔍 AppKey Verificado: %s", response);
            if (strstr(response, "95B97967B65F5A67") || strstr(response, "95b97967b65f5a67")) {
                ESP_LOGI("LORA_TEST", "✅ AppKey configurado correctamente");
            } else {
                ESP_LOGE("LORA_TEST", "❌ AppKey NO configurado correctamente");
                all_credentials_correct = false;
            }
        }

        if (all_credentials_correct) {
            credential_DevEUI_ok = true;
            credential_AppEUI_ok = true;
            credential_AppKey_ok = true;
            ESP_LOGI("LORA_TEST", "🎉 TODAS las credenciales configuradas correctamente!");
        } else {
            ESP_LOGE("LORA_TEST", "❌ Algunas credenciales NO se configuraron correctamente");
        }
    } else {
        ESP_LOGI("LORA_TEST", "✅ Credenciales ya estaban configuradas correctamente");
    }

    // 6. Mostrar resumen de configuración
    ESP_LOGI("LORA_TEST", "=== RESUMEN DE CONFIGURACIÓN ===");
    ESP_LOGI("LORA_TEST", "📋 DevEUI: 70B3D57ED0073AF5");
    ESP_LOGI("LORA_TEST", "📋 AppEUI: AB870A94E0D624BE");
    ESP_LOGI("LORA_TEST", "📋 AppKey: 95B97967B65F5A6738026EBB68FD325C");
    ESP_LOGI("LORA_TEST", "📋 Clase: C");
    ESP_LOGI("LORA_TEST", "📋 Data Rate: 5 (SF7)");
    ESP_LOGI("LORA_TEST", "📋 Banda: US915");
    ESP_LOGI("LORA_TEST", "📋 Estado: %s", (credential_DevEUI_ok && credential_AppEUI_ok && credential_AppKey_ok) ? "CONFIGURADO ✅" : "NO CONFIGURADO ❌");

    // 7. VERIFICAR ESTADO DE LA RED ANTES DEL JOIN
    ESP_LOGI("LORA_TEST", "=== VERIFICANDO ESTADO DE LA RED ===");
    
    // Verificar RSSI (señal)
    lora_uart_clean_safe(1000);
    lora_uart_write_safe("AT+RSSI?\r\n", 2000);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    
    char network_response[512];
    int len = lora_uart_read_safe((uint8_t*)network_response, sizeof(network_response)-1, 5000);
    if(len > 0) {
        network_response[len] = '\0';
        ESP_LOGI("LORA_TEST", "📡 Estado de señal: %s", network_response);
    }

    // 8. INTENTAR JOIN CON CONFIGURACIÓN MEJORADA
    if (credential_DevEUI_ok && credential_AppEUI_ok && credential_AppKey_ok) {
        ESP_LOGI("LORA_TEST", "=== INTENTANDO JOIN CON CONFIGURACIÓN OPTIMIZADA ===");
        
        int max_join_attempts = 3;
        bool join_success = false;
        
        for(int join_attempt = 1; join_attempt <= max_join_attempts; join_attempt++) {
            ESP_LOGI("LORA_TEST", "🎯 Intento de JOIN %d/%d", join_attempt, max_join_attempts);
            
            lora_uart_clean_safe(2000);
            
            // Usar JOIN con parámetros optimizados
            if (lora_uart_write_safe("AT+JOIN=1:1:10:8\r\n", 2000)) {
                ESP_LOGI("LORA_TEST", "Comando JOIN enviado (auto-join habilitado)...");
                
                // Monitorear por más tiempo
                for(int wait_time = 0; wait_time < 60; wait_time++) {
                    char join_response[512];
                    int len = lora_uart_read_safe((uint8_t*)join_response, sizeof(join_response)-1, 1000);
                    
                    if(len > 0) {
                        join_response[len] = '\0';
                        ESP_LOGI("LORA_TEST", "JOIN [%d:%d]: %s", join_attempt, wait_time, join_response);
                        
                        if(strstr(join_response, "JOINED")) {
                            ESP_LOGI("LORA_TEST", "🎉🎉🎉 JOIN EXITOSO en intento %d!", join_attempt);
                            join_success = true;
                            
                            // Enviar mensaje de prueba inmediatamente
                            vTaskDelay(5000 / portTICK_PERIOD_MS);
                            lora_uart_write_safe("AT+SEND=1:48656C6C6F5F4C6F5261\r\n", 2000); // "Hello_LoRa"
                            ESP_LOGI("LORA_TEST", "📤 Mensaje de prueba enviado");
                            break;
                        } else if(strstr(join_response, "JOIN FAILED") || 
                                 strstr(join_response, "FAIL") ||
                                 strstr(join_response, "RX_TIMEOUT")) {
                            ESP_LOGW("LORA_TEST", "⚠️ JOIN falló: %s", join_response);
                            break;
                        } else if(strstr(join_response, "NO FREE CH")) {
                            ESP_LOGE("LORA_TEST", "❌ No hay canales disponibles");
                            break;
                        }
                    }
                    
                    // Mostrar progreso cada 10 segundos
                    if(wait_time % 10 == 0) {
                        ESP_LOGI("LORA_TEST", "⏳ Esperando join... %d/60 segundos", wait_time);
                    }
                    
                    vTaskDelay(1000 / portTICK_PERIOD_MS);
                }
            }
            
            if(join_success) break;
            
            // Espera entre intentos
            if(join_attempt < max_join_attempts) {
                ESP_LOGI("LORA_TEST", "🔄 Reintentando JOIN en 10 segundos...");
                vTaskDelay(10000 / portTICK_PERIOD_MS);
            }
        }
        
        if(!join_success) {
            ESP_LOGE("LORA_TEST", "❌❌❌ TODOS LOS INTENTOS DE JOIN FALLARON");
            ESP_LOGI("LORA_TEST", "🔍 Diagnóstico:");
            ESP_LOGI("LORA_TEST", "   • Verifica cobertura LoRaWAN en tu área");
            ESP_LOGI("LORA_TEST", "   • Mueve el dispositivo cerca de una ventana");
            ESP_LOGI("LORA_TEST", "   • Revisa gateways cercanos en TTN Mapper");
            ESP_LOGI("LORA_TEST", "   • Confirma activación en TTN Console");
        }
    } else {
        ESP_LOGE("LORA_TEST", "❌ No se puede hacer JOIN - Credenciales incorrectas");
    }

    ESP_LOGI("LORA_TEST", "=== FIN DEL TEST LORA ===");
    vTaskDelete(NULL);
}

void app_main(void) {
    ESP_LOGI(TAG_APP, "Iniciando aplicación DLMS + LoRaWAN");
    ESP_LOGI(TAG_APP, "Example version %s", EXAMPLE_VERSION);

    // Crear semáforo para UART
    xLoraUartMutex = xSemaphoreCreateMutex();
    if (xLoraUartMutex == NULL) {
        ESP_LOGE(TAG_APP, "Error creando semáforo");
        return;
    }

    // Inicialización básica
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Configuración SNTP
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "pool.ntp.org");
    sntp_init();
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Inicializar DLMS (para el medidor)
    GatewayInit();

    // Inicializar LoRa (esto configurará el UART y GPIO)
    init_uart();
    init_reset_gpio();

    // Pequeño delay para que todo se inicialice
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Iniciar test LoRa con más stack
    xTaskCreate(lora_ttn_test_task, "lora_ttn_test", 8192, NULL, 4, NULL);

    ESP_LOGI(TAG_APP, "Aplicación iniciada correctamente");
    
    // Tarea principal puede continuar con otras operaciones
    while (1) {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
        // Aquí puedes agregar otras operaciones periódicas
    }
}