#include "loraUart.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "string.h"
#include "../leds/leds.h"
#include "board.h"

QueueHandle_t xQueue_lora_send_data = NULL;

#define UART_NUM                UART_NUM_2
#define BAUD_RATE               115200
#define UART_PIN_TX             33
#define UART_PIN_RX             32
#define RESET_LORA              GPIO_NUM_27
#define LORA_ON                 1
#define LORA_OFF                0
#define GPIO_OUTPUT_PIN_SEL_LORA (1ULL<<RESET_LORA)
#define JOIN_MAX_RETRY          8
#define SEND_MAX_RETRY          3
#define SIZE_ARRAY(arr) (sizeof(arr)/sizeof((arr)[0]))
int board_count = 1;

static const char *TAG = "UART_LORA";

// Arrays de strings para debug
const char* TO_LORA_STRING[] = {
    "sReset", "sConfig_AT", "sJoin", "sSend_Lora_Frame",
    "sSend_Complete", "sSend_Error", "sData", "sData_complete", "sSend_Data"
};

const char* TO_LORA_SEND_STRING[] = {
    "sReset", "sConfig_AT", "sJoin", "sSend_Lora_Frame",
    "sSend_Complete", "sSend_Error", "sData", "sData_complete", "sSend_Data"
};

const char* LORA_STATE_STRING[] = {
    "Reset_lora", "Send_Lora_AT_Conf", "Wating_AT_Response",
    "Wating_Make_Join", "Idle_lora", "Wating_Send_Lora_Frame"
};

const char* LORA_SEND_STATE_STRING[] = {
    "Idle_sending", "Send_Data", "Wating_Send_Data"
};

/*-------------------------------------------------------
 * Configuración AT para RAK3272 AU915 - TTN (Clase A)
 *------------------------------------------------------*/
static const char Lora_at_config[][50] = {
    {"AT+ATM\r\n"},                        // Modo AT
    {"AT+NJM=1\r\n"},                      // OTAA
    {"AT+CLASS=A\r\n"},                    // Clase A (TTN)
    {"AT+DR=3\r\n"},                       // DataRate (SF9BW125)
    {"AT+BAND=8\r\n"},                     // AU915
    {"AT+MASK=0001\r\n"},                  // SubBand 1 (canales 8-15)
    {"AT+CHE=8:9:10:11:12:13:14:15\r\n"},  // Canales TTN AU915
    {"AT+CFM=1\r\n"},                      // Uplink confirmado
    {"AT+DEVEUI=70B3D57ED0073AF5\r\n"},    // Tu DevEUI
    {"AT+APPEUI=AB870A94E0D624BE\r\n"},    // Tu AppEUI
    {"AT+APPKEY=95B97967B65F5A6738026EBB68FD325C\r\n"}, // Tu AppKey
    {"AT+RX2DR=8\r\n"},                    // RX2 DR (SF12BW500)
    {"AT+RX2FQ=923300000\r\n"},            // RX2 Freq (923.3 MHz)
    {"AT+JOIN=1:0:10:8\r\n"}               // Join OTAA (10 intentos)
};

/*-------------------------------------------------------
 * Queues externas
 *------------------------------------------------------*/
static QueueHandle_t uart_queue;
extern QueueHandle_t xQueue_uart_lora;
extern QueueHandle_t xQueue_get_data;
extern QueueHandle_t xQueue_lora_send_data;


/*-------------------------------------------------------
 * Tarea de envío de datos LoRa
 *------------------------------------------------------*/
void lora_send_data_task(void *pvParameters)
{
    msg_t In_msg, Out_msg;
    int count = 0;
    int count_lora_send_attemps = 0;
    LORA_SEND_STATE_ENUM state, state_next;
    Out_msg.lora_tx = (char *) malloc(BUFF_SIZE_TX);
    state_next = Idle_sending;
    float * data = NULL;

    for (;;) {
        if (xQueueReceive(xQueue_lora_send_data, &In_msg, portMAX_DELAY)) {
            state = state_next;
            printf("Lora_Send rcvd [%s(%d)] in state [%s]\n",
                TO_LORA_SEND_STRING[In_msg.signal], In_msg.value, LORA_SEND_STATE_STRING[state]);
            fflush(stdout);

            switch (state) {
                case Idle_sending:
                    if (In_msg.signal == sData_complete) {
                        state_next = Send_Data;
                        count = 0;
                        data = In_msg.data;
                        Out_msg.signal = sSend_Data;
                        xQueueSendToBack(xQueue_lora_send_data, (void*)&Out_msg, 0);
                    }
                    break;

                case Send_Data:
                    if (In_msg.signal == sSend_Data) {
                        create_lora_hex_string(count + 1, data[count * 2], data[count * 2 + 1], Out_msg.lora_tx);
                        Out_msg.signal = sSend_Lora_Frame;
                        xQueueSendToBack(xQueue_uart_lora, (void*)&Out_msg, 0);
                        state_next = Wating_Send_Data;
                    }
                    break;

                case Wating_Send_Data:
                    if (In_msg.signal == sSend_Complete) {
                        count++;
                        count_lora_send_attemps = 0;
                        if (count >= NUM_REGISTER_READ / 2) {
                            state_next = Idle_sending;
                            count = 0;
                        } else {
                            Out_msg.signal = sSend_Data;
                            xQueueSendToBack(xQueue_lora_send_data, (void*)&Out_msg, 0);
                            state_next = Send_Data;
                        }
                    } else if (In_msg.signal == sSend_Error) {
                        count_lora_send_attemps++;
                        if (count_lora_send_attemps > SEND_MAX_RETRY) {
                            count_lora_send_attemps = 0;
                            count++;
                            if (count >= NUM_REGISTER_READ / 2) {
                                count = 0;
                                state_next = Idle_sending;
                            } else {
                                Out_msg.signal = sSend_Data;
                                xQueueSendToBack(xQueue_lora_send_data, (void*)&Out_msg, 0);
                                state_next = Send_Data;
                            }
                        } else {
                            state_next = Send_Data;
                            Out_msg.signal = sSend_Data;
                            xQueueSendToBack(xQueue_lora_send_data, (void*)&Out_msg, 0);
                        }
                    }
                    break;

                default:
                    state_next = Idle_sending;
                    break;
            }
            printf("Lora_Send next state is [%s]\n", LORA_SEND_STATE_STRING[state_next]);
            fflush(stdout);
        }
    }
    free(Out_msg.lora_tx);
    ESP_LOGI(TAG, "Task Delete");
    vTaskDelete(NULL);
}

/*-------------------------------------------------------
 * Tarea principal de manejo LoRa
 *------------------------------------------------------*/
void lora_task(void *pvParameters)
{
    msg_t In_msg, Out_msg;
    int count = 0;
    LORA_STATE_ENUM state, state_next;
    Out_msg.lora_tx = (char *) malloc(BUFF_SIZE_TX);
    Out_msg.signal = sReset;
    Out_msg.value = 0;

    read_lora_credentials();
    board_led_operation(LED_R, LED_ON);
    state_next = Reset_lora;
    xQueueSendToBack(xQueue_uart_lora, (void*)&Out_msg, 0);

    for (;;) {
        if (xQueueReceive(xQueue_uart_lora, &In_msg, portMAX_DELAY)) {
            state = state_next;
            printf("Lora_Task rcvd [%s(%d)] in state [%s]\n",
                TO_LORA_STRING[In_msg.signal], In_msg.value, LORA_STATE_STRING[state]);
            fflush(stdout);

            switch (state) {
                case Reset_lora:
                    if (In_msg.signal == sReset) {
                        xQueueReset(xQueue_uart_lora);
                        reset_lora_func();
                        count = 0;
                    } else if (In_msg.signal == sData) {
                        count++;
                        if (count >= 1) {
                            count = 0;
                            state_next = Send_Lora_AT_Conf;
                            Out_msg.signal = sConfig_AT;
                            xQueueSendToBack(xQueue_uart_lora, (void*)&Out_msg, 0);
                        }
                    }
                    break;

                case Send_Lora_AT_Conf:
                    if (In_msg.signal == sConfig_AT) {
                        strcpy(Out_msg.lora_tx, Lora_at_config[count]);
                        Out_msg.lora_tx_length = uart_write_bytes(UART_NUM, (const void *) Out_msg.lora_tx, strlen(Out_msg.lora_tx));
                        ESP_LOGI(TAG, "length: %d Msj:%s", Out_msg.lora_tx_length, Out_msg.lora_tx);
                        count++;
                        state_next = Wating_AT_Response;
                    }
                    break;

                case Wating_AT_Response:
                    if (In_msg.signal == sData) {
                        if (!strcmp(In_msg.lora_rx, "OK\r\n")) {
                            if (count >= SIZE_ARRAY(Lora_at_config)) {
                                state_next = Wating_Make_Join;
                                count = 0;
                            } else {
                                state_next = Send_Lora_AT_Conf;
                                Out_msg.signal = sConfig_AT;
                                xQueueSendToBack(xQueue_uart_lora, (void*)&Out_msg, 0);
                            }
                        } else {
                            state_next = Reset_lora;
                            Out_msg.signal = sReset;
                            count = 0;
                            xQueueSendToBack(xQueue_uart_lora, (void*)&Out_msg, 0);
                        }
                    }
                    break;

                case Wating_Make_Join:
                    if (In_msg.signal == sData) {
                        if (!strcmp(In_msg.lora_rx, "+EVT:JOINED\r\n")) {
                            ESP_LOGI(TAG, "JOIN COMPLETE");
                            board_led_operation(LED_R, LED_OFF);
                            Out_msg.signal = sJoin;
                            xQueueSendToBack(xQueue_get_data, (void*)&Out_msg, 0);
                            state_next = Idle_lora;
                        } else if (!strcmp(In_msg.lora_rx, "+EVT:JOIN_FAILED_RX_TIMEOUT\r\n")) {
                            count++;
                            if (count > JOIN_MAX_RETRY) {
                                state_next = Reset_lora;
                                Out_msg.signal = sReset;
                                count = 0;
                                xQueueSendToBack(xQueue_uart_lora, (void*)&Out_msg, 0);
                            }
                        }
                    }

                    break;

                case Idle_lora:
                    if (In_msg.signal == sSend_Lora_Frame) {
                        printf("%s", In_msg.lora_tx);
                        Out_msg.lora_tx_length = uart_write_bytes(UART_NUM, (const void *) In_msg.lora_tx, strlen(In_msg.lora_tx));
                        board_led_operation(LED_B, LED_ON);
                        state_next = Wating_Send_Lora_Frame;
                    }

					else if (In_msg.signal == sSend_Data) {
						// Construir comando AT para enviar a TTN
						char at_command[256];
						snprintf(at_command, sizeof(at_command), "AT+SEND=10:%s\r\n", In_msg.lora_tx);

						// Enviar por UART al RAK3272
						Out_msg.lora_tx_length = uart_write_bytes(UART_NUM, at_command, strlen(at_command));

						ESP_LOGI(TAG, " Enviando trama directa a TTN: %s", at_command);

						// Feedback visual (LED azul)
						board_led_operation(LED_B, LED_ON);
						vTaskDelay(200 / portTICK_PERIOD_MS);
						board_led_operation(LED_B,s LED_OFF);

						// Esperar confirmación
						state_next = Wating_Send_Lora_Frame;
                    break;

                case Wating_Send_Lora_Frame:
                    if (In_msg.signal == sData) {
                        if (!strncmp(In_msg.lora_rx, "+EVT:TX_DONE", 12) || !strcmp(In_msg.lora_rx, "+EVT:SEND_CONFIRMED_OK\r\n")) {
                            ESP_LOGI(TAG, "TX Done");
                            board_led_operation(LED_B, LED_OFF);
                            board_led_operation(LED_G, LED_ON);
                            vTaskDelay(500 / portTICK_PERIOD_MS);
                            board_led_operation(LED_G, LED_OFF);
                            Out_msg.signal = sSend_Complete;
                            xQueueSendToBack(xQueue_lora_send_data, (void*)&Out_msg, 0);
                            state_next = Idle_lora;
                        } else if (!strncmp(In_msg.lora_rx, "+EVT:SEND_CONFIRMED_FAILED", 26)) {
                            board_led_operation(LED_R, LED_ON);
                            vTaskDelay(500 / portTICK_PERIOD_MS);
                            board_led_operation(LED_R, LED_OFF);
                            Out_msg.signal = sSend_Error;
                            xQueueSendToBack(xQueue_lora_send_data, (void*)&Out_msg, 0);
                            state_next = Idle_lora;
                        }
                    }
                    break;
            }
            printf("LORA next state is [%s]\n", LORA_STATE_STRING[state_next]);
            fflush(stdout);
        }
    }
}

/*-------------------------------------------------------
 * UART Task y Funciones auxiliares
 *------------------------------------------------------*/
void uart_task(void *pvParameters)
{
    uart_event_t event;
    char *data = (char *) malloc(BUFF_SIZE_RX);
    msg_t Out_msg;

    for (;;) {
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            bzero(data, BUFF_SIZE_RX);
            switch (event.type) {
                case UART_DATA:
                    if (event.size < 3) {
                        uart_flush(UART_NUM);
                        break;
                    }
                    uart_read_bytes(UART_NUM, data, event.size, pdMS_TO_TICKS(100));
                    uart_flush(UART_NUM);
                    Out_msg.lora_rx = data;
                    Out_msg.lora_rx_length = event.size;
                    xQueueSendToBack(xQueue_uart_lora, (void*)&Out_msg, 0);
                    printf("Data_receive len (%d): %s\n", event.size, data);
                    break;
                default:
                    uart_flush(UART_NUM);
                    break;
            }
        }
    }
}

/*-------------------------------------------------------
 * Inicialización UART
 *------------------------------------------------------*/
void init_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_PIN_TX, UART_PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUFF_SIZE_RX, BUFF_SIZE_TX, QUEQUE_LENGTH, &uart_queue, ESP_INTR_FLAG_LEVEL1);
}

/*-------------------------------------------------------
 * Test UART simple
 *------------------------------------------------------*/
void test_lora_uart(void *pvParameters)
{
    const char *cmd = "AT\r\n";
    uint8_t rx_buffer[128];

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "🔍 Enviando comando AT de prueba...");

    uart_write_bytes(UART_NUM_2, cmd, strlen(cmd));
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    int len = uart_read_bytes(UART_NUM_2, rx_buffer, sizeof(rx_buffer) - 1, 100 / portTICK_PERIOD_MS);
    if (len > 0) {
        rx_buffer[len] = '\0';
        ESP_LOGI(TAG, "✅ Respuesta recibida: %s", (char *)rx_buffer);
    } else {
        ESP_LOGW(TAG, "⚠️ No hubo respuesta del módulo LoRa");
    }
    vTaskDelete(NULL);
}

/*-------------------------------------------------------
 * GPIO Reset y Funciones de control
 *------------------------------------------------------*/
void init_reset_gpio(void)
{
    gpio_config_t gpio = {
        .pin_bit_mask = GPIO_OUTPUT_PIN_SEL_LORA,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&gpio);
    gpio_set_level(RESET_LORA, LORA_ON);
}

void reset_lora_func(void)
{
    gpio_set_level(RESET_LORA, LORA_OFF);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gpio_set_level(RESET_LORA, LORA_ON);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

/*-------------------------------------------------------
 * Lectura de credenciales LoRa
 *------------------------------------------------------*/
void read_lora_credentials(void)
{
    reset_lora_func();
    printf("Read Lora Credentials\n");
    char lora_tx[20] = {0};
    strcpy(lora_tx, "AT+DEVEUI=?\r\n");
    uart_write_bytes(UART_NUM, lora_tx, strlen(lora_tx));
    strcpy(lora_tx, "AT+APPEUI=?\r\n");
    uart_write_bytes(UART_NUM, lora_tx, strlen(lora_tx));
    strcpy(lora_tx, "AT+APPKEY=?\r\n");
    uart_write_bytes(UART_NUM, lora_tx, strlen(lora_tx));
    vTaskDelay(100 / portTICK_PERIOD_MS);
    xQueueReset(xQueue_uart_lora);
}

/*-------------------------------------------------------
 * Creación del payload LoRa (float a HEX)
 *------------------------------------------------------*/
void create_lora_hex_string(int msj_num, float float_val_1, float float_val_2, char *dst)
{
    uint8_t bytes[8];
    memcpy(&bytes[0], &float_val_1, 4);
    memcpy(&bytes[4], &float_val_2, 4);

    char hex_payload[20];
    sprintf(hex_payload, "%02X%02X%02X%02X%02X%02X%02X%02X",
        bytes[0], bytes[1], bytes[2], bytes[3],
        bytes[4], bytes[5], bytes[6], bytes[7]);

    sprintf(dst, "AT+SEND=10:%s\r\n", hex_payload);
    printf("TTN Payload: %s\nAT Command: %s", hex_payload, dst);
}
