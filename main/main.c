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
#include <stdint.h>

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

// Empaquetado de datos del medidor en payload binario y conversión a HEX
static void u16_to_hex_be(uint16_t v, char* out)
{
    static const char* hex = "0123456789ABCDEF";
    out[0] = hex[(v >> 12) & 0xF];
    out[1] = hex[(v >> 8) & 0xF];
    out[2] = hex[(v >> 4) & 0xF];
    out[3] = hex[(v) & 0xF];
}

static uint16_t encode_fp(float value, float scale)
{
    int32_t iv = (int32_t)(value * scale + (value >= 0 ? 0.5f : -0.5f));
    if (iv < 0) iv = 0;
    if (iv > 0xFFFF) iv = 0xFFFF;
    return (uint16_t)iv;
}

// Construye payload HEX con: VA,VB,VC, IA,IB,IC, Freq, PtotImp (16 bytes)
static bool build_meter_payload_hex(char* hex_out, size_t hex_out_size)
{
    // 14 valores → 28 bytes → 56 caracteres HEX + terminador
    if (hex_out_size < (28*2 + 1)) return false;

    GW_MeterSnapshot snap;
    int tries = 0;

    while (tries < 10) {
        if (GW_GetLatestSnapshot(&snap)) break;
        vTaskDelay(pdMS_TO_TICKS(500));
        tries++;
    }
    if (tries >= 10) return false;

    // Preparamos los 14 valores a enviar
    uint16_t v[14];
    int i = 0;

    // Voltajes
    v[i++] = encode_fp(snap.voltage[0], 100.0f);
    v[i++] = encode_fp(snap.voltage[1], 100.0f);
    v[i++] = encode_fp(snap.voltage[2], 100.0f);

    // Corrientes
    v[i++] = encode_fp(snap.current[0], 100.0f);
    v[i++] = encode_fp(snap.current[1], 100.0f);
    v[i++] = encode_fp(snap.current[2], 100.0f);

    // Frecuencia
    v[i++] = encode_fp(snap.frequency, 100.0f);

    // Power factor A/B/C
    v[i++] = encode_fp(snap.power_factor[0], 100.0f);
    v[i++] = encode_fp(snap.power_factor[1], 100.0f);
    v[i++] = encode_fp(snap.power_factor[2], 100.0f);

    // Totales (energia activa, reactiva, aparente)
    v[i++] = encode_fp(snap.active_power_total_import, 10.0f);
    v[i++] = encode_fp(snap.active_power_total_export, 10.0f);
    v[i++] = encode_fp(snap.reactive_power_total_import, 10.0f);
    v[i++] = encode_fp(snap.reactive_power_total_export, 10.0f);
    v[i++] = encode_fp(snap.apparent_power_total, 10.0f);

    // Convertir a HEX
    char* p = hex_out;

    for (int k = 0; k < i; k++) {
        u16_to_hex_be(v[k], p);
        p += 4;
    }

    *p = '\0';
    return true;
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

    const char* module_config[] = {
        "AT+ATM=1\r\n",              // Modo AT
        "AT+CLASS=C\r\n",            // Clase C (recepción continua)
        "AT+NJM=1\r\n",              // Modo join por OTAA
        "AT+DR=3\r\n",               // Data Rate SF7 (DR3 en US915)
        "AT+BAND=6\r\n",             // Banda US915
        "AT+CHMASK=00FF\r\n",        // Habilitar canales 0–7
        "AT+CFM=1\r\n",              // Confirmación habilitada
        "AT+RX2DR=8\r\n",            // DR para RX2
        "AT+TXP=15\r\n",             // Potencia máxima (15 dBm)
        "AT+ADAPTIVE_DATA_RATE=0\r\n",
        "AT+RETRIES=5\r\n"
    };

    for (size_t i = 0; i < sizeof(module_config) / sizeof(module_config[0]); i++) {
        ESP_LOGI("LORA_TEST", "Configurando módulo: %s", module_config[i]);
        lora_uart_clean_safe(1000);
        lora_uart_write_safe(module_config[i], 2000);
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        char config_response[256];
        int len = lora_uart_read_safe((uint8_t*)config_response, sizeof(config_response) - 1, 3000);
        if (len > 0) {
            config_response[len] = '\0';
            ESP_LOGI("LORA_TEST", "Respuesta: %s", config_response);
        }
    }

    // 4. Verificar credenciales actuales
    ESP_LOGI("LORA_TEST", "=== VERIFICANDO CREDENCIALES ACTUALES ===");

    if (lora_send_command("AT+DEVEUI=?\r\n", response, sizeof(response), 4000)) {
        ESP_LOGI("LORA_TEST", "📖 DevEUI Actual: %s", response);
        credential_DevEUI_ok = strstr(response, "70B3D57ED0073AF5") || strstr(response, "70b3d57ed0073af5");
    }

    if (lora_send_command("AT+APPEUI=?\r\n", response, sizeof(response), 4000)) {
        ESP_LOGI("LORA_TEST", "📖 AppEUI Actual: %s", response);
        credential_AppEUI_ok = strstr(response, "AB870A94E0D624BE") || strstr(response, "ab870a94e0d624be");
    }

    if (lora_send_command("AT+APPKEY=?\r\n", response, sizeof(response), 4000)) {
        ESP_LOGI("LORA_TEST", "📖 AppKey Actual: %s", response);
        credential_AppKey_ok = strstr(response, "95B97967B65F5A67") || strstr(response, "95b97967b65f5a67");
    }

    if (!credential_DevEUI_ok || !credential_AppEUI_ok || !credential_AppKey_ok) {
        ESP_LOGI("LORA_TEST", "=== CONFIGURANDO CREDENCIALES ===");
        const char* setup_commands[] = {
            "AT+DEVEUI=70B3D57ED0073AF5\r\n",
            "AT+APPEUI=AB870A94E0D624BE\r\n",
            "AT+APPKEY=95B97967B65F5A6738026EBB68FD325C\r\n",
            "AT+SAVE\r\n",
        };
        for (int i = 0; i < 4; i++) {
            ESP_LOGI("LORA_TEST", "Configurando: %s", setup_commands[i]);
            lora_send_command(setup_commands[i], response, sizeof(response), 5000);
            vTaskDelay(3000 / portTICK_PERIOD_MS);
        }
    }

    // 5. Intentar JOIN
    ESP_LOGI("LORA_TEST", "=== INTENTANDO JOIN CON TTN ===");

    int max_join_attempts = 3;
    bool join_success = false;

    for (int join_attempt = 1; join_attempt <= max_join_attempts; join_attempt++) {
        ESP_LOGI("LORA_TEST", "🎯 Intento de JOIN %d/%d", join_attempt, max_join_attempts);

        lora_uart_clean_safe(2000);
        if (lora_uart_write_safe("AT+JOIN=1:0:10:8\r\n", 2000)) {
            ESP_LOGI("LORA_TEST", "Comando JOIN enviado...");
            for (int wait_time = 0; wait_time < 60; wait_time++) {
                char join_response[512];
                int len = lora_uart_read_safe((uint8_t*)join_response, sizeof(join_response) - 1, 1000);
                if (len > 0) {
                    join_response[len] = '\0';
                    ESP_LOGI("LORA_TEST", "JOIN [%d:%d]: %s", join_attempt, wait_time, join_response);
                    if (strstr(join_response, "JOINED")) {
                        ESP_LOGI("LORA_TEST", "🎉 JOIN EXITOSO — comenzando envío continuo");
                        join_success = true;

                        vTaskDelay(5000 / portTICK_PERIOD_MS);

                        // 🔁 Bucle infinito de envío cada 60 s
                        while (true) {
                            lora_uart_clean_safe(1000);
                            char hex_payload[33];
                            if (build_meter_payload_hex(hex_payload, sizeof(hex_payload))) {
                                char cmd[64];
                                snprintf(cmd, sizeof(cmd), "AT+SEND=1:%s\r\n", hex_payload);
                                if (lora_uart_write_safe(cmd, 2000)) {
                                    ESP_LOGI("LORA_TEST", "📤 Enviado a TTN: %s", hex_payload);
                                } else {
                                    ESP_LOGE("LORA_TEST", "❌ Error enviando por LoRa");
                                }
                            } else {
                                ESP_LOGW("LORA_TEST", "⚠️ No hay snapshot válido del medidor");
                            }
                            vTaskDelay(pdMS_TO_TICKS(60000));  // 1 minuto
                        }
                    }
                }
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
        }
        if (join_success) break;
        if (join_attempt < max_join_attempts) {
            ESP_LOGI("LORA_TEST", "🔄 Reintentando JOIN en 10 segundos...");
            vTaskDelay(10000 / portTICK_PERIOD_MS);
        }
    }

    if (!join_success) {
        ESP_LOGE("LORA_TEST", "❌ No se logró conexión con TTN");
    }

    // Esta línea ya no se alcanza (la tarea nunca termina)
    // vTaskDelete(NULL);
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
