

#include "LoraUart.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "string.h"
#include "../leds/leds.h"
#include "board.h"

#define	UART_NUM 							UART_NUM_2
#define BAUD_RATE 						115200

#define UART_PIN_TX 					33
#define UART_PIN_RX 					32
#define RESET_LORA 						GPIO_NUM_27
#define LORA_ON  							1
#define LORA_OFF 							0
#define GPIO_OUTPUT_PIN_SEL_LORA 	(1ULL<<RESET_LORA)
#define	JOIN_MAX_RETRY				8
#define SEND_MAX_RETRY				3
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

static const char Lora_at_config  [][50] = {
	{"AT+ATM\r\n"},
	{"AT+CLASS=C\r\n"},                // Clase C para TTN
	{"AT+NJM=1\r\n"},
	{"AT+DR=5\r\n"},                   
	{"AT+BAND=8\r\n"},                 
	{"AT+MASK=0000\r\n"},              
	{"AT+CHE=8:9:10:11:12:13:14:15\r\n"},   // Canales EU868
	{"AT+CFM=1\r\n"},
    {"AT+DEVEUI=70B3D57ED0073ACB\r\n"},        
    {"AT+APPKEY=95B97967B65F5A6738026EBB68FD325C\r\n"},        
    {"AT+APPEUI=AB870A94E0D624BE\r\n"},        
	{"AT+RX2DR=8\r\n"},
	{"AT+JOIN=1:0:10:8\r\n"},
};



static QueueHandle_t uart_queue;
extern QueueHandle_t xQueue_uart_lora;
extern QueueHandle_t xQueue_get_data;
extern QueueHandle_t xQueue_lora_send_data;


void lora_send_data_task(void *pvParameters)
{
	msg_t In_msg;
	msg_t	Out_msg;
	int count = 0;
	int count_lora_send_attemps = 0;
	LORA_SEND_STATE_ENUM state, state_next;
	Out_msg.lora_tx = (char *) malloc(BUFF_SIZE_TX);
	state_next = Idle_sending;
	float * data = NULL;
	for ( ; ; )
	{
		if (xQueueReceive(xQueue_lora_send_data, &In_msg, portMAX_DELAY))
		{
			state = state_next;
			printf ( "\t\t\t\t\tLora_Send	 rcvd [%s(%d)] in state [%s]\n", TO_LORA_SEND_STRING[In_msg.signal], In_msg.value, LORA_SEND_STATE_STRING[state] );
			fflush ( stdout );
			switch (state)
			{
			case Idle_sending:
				switch (In_msg.signal) {
				case sData_complete:
					state_next = Send_Data;
					count = 0;
					data = In_msg.data;
					Out_msg.signal = sSend_Data;
					xQueueSendToBack(xQueue_lora_send_data, (void*)&Out_msg, ( TickType_t ) 0);
					break;
				default:
					break;
				}
				break;
			case Send_Data:
				switch (In_msg.signal)
				{
				case sSend_Data:
					create_lora_hex_string(count + 1, data[count * 2], data[count * 2 + 1], Out_msg.lora_tx);
					Out_msg.signal = sSend_Lora_Frame;
					xQueueSendToBack(xQueue_uart_lora, (void*)&Out_msg, ( TickType_t ) 0);
					state_next = Wating_Send_Data;
					break;
				default:
					break;
				}
				break;
			case Wating_Send_Data:
				switch (In_msg.signal) {
				case sSend_Complete:
					count++;
					count_lora_send_attemps = 0;
					if (count >= NUM_REGISTER_READ/ 2)
					{
						state_next = Idle_sending;
						count = 0;
					}else{
						Out_msg.signal = sSend_Data;
						xQueueSendToBack(xQueue_lora_send_data, (void*)&Out_msg, ( TickType_t ) 0);
						state_next = Send_Data;
					}
					break;
				case sSend_Error:
					count_lora_send_attemps++;
					if (count_lora_send_attemps > SEND_MAX_RETRY)
					{
						count_lora_send_attemps = 0;
						count++;
						if (count >= NUM_REGISTER_READ /2)
						{
							count = 0;
							state_next = Idle_sending;
						} else {
							Out_msg.signal = sSend_Data;
							xQueueSendToBack(xQueue_lora_send_data, (void*)&Out_msg, ( TickType_t ) 0);
							state_next = Send_Data;
						}
					}
					else {
						state_next = Send_Data;
						Out_msg.signal = sSend_Data;
						xQueueSendToBack(xQueue_lora_send_data, (void*)&Out_msg, ( TickType_t ) 0);
					}	
					break;
				default:
					break;
				}
				break;
			default:
				state_next = Idle_sending;
				break;
			}
			printf ( "\t\t\t\t\tLora_Send next state is [%s]\n", LORA_SEND_STATE_STRING[state_next] );
			fflush ( stdout );
		}
	}
	free(Out_msg.lora_tx);
	ESP_LOGI(TAG, "Task Delete");
	vTaskDelete(NULL);
}

void lora_task(void *pvParameters) {
	msg_t In_msg;
	msg_t	Out_msg;
	int count = 0;
	LORA_STATE_ENUM state, state_next;
	Out_msg.lora_tx = (char *) malloc(BUFF_SIZE_TX);
	Out_msg.signal = sReset;
	Out_msg.value = 0;

	read_lora_credentials();
	board_led_operation(LED_R, LED_ON);
	state_next = Reset_lora;
	xQueueSendToBack(xQueue_uart_lora, (void*)&Out_msg, ( TickType_t ) 0);
	for ( ; ; )
	{
		if (xQueueReceive(xQueue_uart_lora, &In_msg, portMAX_DELAY))
		{
			state = state_next;
			printf ( "\t\t\t\t\tLora_Task	 rcvd [%s(%d)] in state [%s]\n", TO_LORA_STRING[In_msg.signal], In_msg.value, LORA_STATE_STRING[state] );
			fflush ( stdout );
			switch (state) {
			case Reset_lora:
				switch (In_msg.signal) {
				case sReset:
					xQueueReset(xQueue_uart_lora);
					reset_lora_func();
					count = 0;
					break;
				case sData:
					count++;
					if (count >= 1)
					{
						count = 0;
						state_next = Send_Lora_AT_Conf;
						Out_msg.signal = sConfig_AT;
						xQueueSendToBack(xQueue_uart_lora, (void*)&Out_msg, ( TickType_t ) 0);
					}
					break;
				default:
					break;
				}
				break;
			case Send_Lora_AT_Conf:
				switch (In_msg.signal) {
				case sConfig_AT:
					strcpy(Out_msg.lora_tx, Lora_at_config[count]);
					Out_msg.lora_tx_length = uart_write_bytes(UART_NUM, (const void *) Out_msg.lora_tx, strlen(Out_msg.lora_tx));
					ESP_LOGI(TAG, "length: %d Msj:%s", Out_msg.lora_tx_length , Out_msg.lora_tx);
					count++;
					state_next = Wating_AT_Response;
					break;
				default:
					break;
				}
				break;
			case Wating_AT_Response:
				switch (In_msg.signal) {
				case sData:
					if (!(strcmp(In_msg.lora_rx, "OK\r\n")))
					{
						if (count >= SIZE_ARRAY(Lora_at_config))
						{
							state_next = Wating_Make_Join;
							count = 0;
						}
						else {
							state_next = Send_Lora_AT_Conf;
							Out_msg.signal = sConfig_AT;
							xQueueSendToBack(xQueue_uart_lora, (void*)&Out_msg, ( TickType_t ) 0);
						}
					} else {
						state_next = Reset_lora;
						Out_msg.signal = sReset;
						count = 0;
						xQueueSendToBack(xQueue_uart_lora, (void*)&Out_msg, ( TickType_t ) 0);
					}
					break;
				default:
					break;
				}
				break;
			case Wating_Make_Join:
				switch (In_msg.signal) {
				case sData:
					if (!(strcmp(In_msg.lora_rx, "+EVT:JOINED\r\n")))
					{
						ESP_LOGI(TAG, "JOIN COMPLETE IN ATTTEMP: %d", count);
						board_led_operation(LED_R, LED_OFF);
						count = 0;
						Out_msg.signal = sJoin;
						xQueueSendToBack(xQueue_get_data, (void*)&Out_msg, ( TickType_t ) 0);
						state_next = Idle_lora;
					}
					else if (!(strcmp(In_msg.lora_rx, "+EVT:JOIN_FAILED_RX_TIMEOUT\r\n")))
					{
						count ++;
						ESP_LOGI(TAG, "LORA RETRY JOIN ATTTEMP: %d", count);
						if (count > JOIN_MAX_RETRY)
						{
							state_next = Reset_lora;
							Out_msg.signal = sReset;
							count = 0;
							xQueueSendToBack(xQueue_uart_lora, (void*)&Out_msg, ( TickType_t ) 0);
						}
					}
					else
					{
						state_next = Reset_lora;
						Out_msg.signal = sReset;
						count = 0;
						xQueueSendToBack(xQueue_uart_lora, (void*)&Out_msg, ( TickType_t ) 0);
					}
					break;
				default:
					break;
				}
				break;
			case Idle_lora:
				switch (In_msg.signal) {
				case sSend_Lora_Frame:
					printf("%s", In_msg.lora_tx);
					Out_msg.lora_tx_length = uart_write_bytes(UART_NUM, (const void *) In_msg.lora_tx, strlen(In_msg.lora_tx));
					board_led_operation(LED_B, LED_ON);
					state_next = Wating_Send_Lora_Frame;
					break;
				case sData:
					break;
				default:
					break;
				}
				break;
			case Wating_Send_Lora_Frame:
				switch (In_msg.signal) {
				case sData:
					if (!(strncmp(In_msg.lora_rx, "OK", 2)))
					{
						ESP_LOGI(TAG, "Send_Data: %d", count);
						state_next = Wating_Send_Lora_Frame;
						break;
					}
					else if (!(strcmp(In_msg.lora_rx, "+EVT:TX_DONE\r\n")) || !(strcmp(In_msg.lora_rx, "+EVT:SEND_CONFIRMED_OK\r\n")))
					{
						ESP_LOGI(TAG, "Lora_Send_Complete: %d", count);
						state_next = Idle_lora;
						board_led_operation(LED_B, LED_OFF);
						board_led_operation(LED_G, LED_ON);
						vTaskDelay(1000 / portTICK_PERIOD_MS);
						board_led_operation(LED_G, LED_OFF);
						Out_msg.signal = sSend_Complete;
						xQueueSendToBack(xQueue_lora_send_data, (void*)&Out_msg, ( TickType_t ) 0);
						break;
					} 
					else if (!(strncmp(In_msg.lora_rx, "+EVT:SEND_CONFIRMED_FAILED", 26)))
					{
						board_led_operation(LED_B, LED_OFF);
						board_led_operation(LED_R, LED_ON);
						vTaskDelay(1000 / portTICK_PERIOD_MS);
						board_led_operation(LED_R, LED_OFF);
						Out_msg.signal = sSend_Error;
						xQueueSendToBack(xQueue_lora_send_data, (void*)&Out_msg, ( TickType_t ) 0);						
						state_next = Idle_lora;
						break;
					}
					else if (!(strcmp(In_msg.lora_rx, "+BC:DONE\r\n")) || !(strcmp(In_msg.lora_rx, "+BC:LOCKED\r\n")))
					{
						ESP_LOGI(TAG, "Beacon MSJ");
						break;
					}
					else if ( !(strcmp(In_msg.lora_rx, "AT_BUSY_ERROR\r\n")) )
					{
						ESP_LOGI(TAG, "Lora Ocupado: %s", In_msg.lora_rx);
						board_led_operation(LED_B, LED_OFF);
						board_led_operation(LED_R, LED_ON);
						vTaskDelay(1000/portTICK_PERIOD_MS);
						board_led_operation(LED_R, LED_OFF);
						vTaskDelay(10000 / portTICK_PERIOD_MS);
						Out_msg.signal = sSend_Error;
						xQueueSendToBack(xQueue_lora_send_data, (void*)&Out_msg, ( TickType_t ) 0);	
						state_next = Idle_lora;
						break;
					}
					else
					{
						ESP_LOGI(TAG, "Send_Data_Error: %s", In_msg.lora_rx);
						board_led_operation(LED_R, LED_OFF);
						board_led_operation(LED_G, LED_OFF);
						board_led_operation(LED_B, LED_OFF);
						vTaskDelay(1000 / portTICK_PERIOD_MS);
						Out_msg.signal = sSend_Error;
						xQueueSendToBack(xQueue_lora_send_data, (void*)&Out_msg, ( TickType_t ) 0);	
						state_next = Idle_lora;
					}
					break;
				default:
					break;
				}
				break;
			default:
				break;
			}
			printf ( "\t\t\t\t\tLORA next state is [%s]\n", LORA_STATE_STRING[state_next] );
			fflush ( stdout );
		}
	}
	free(Out_msg.lora_tx);
	ESP_LOGI(TAG, "Task Delete");
	vTaskDelete(NULL);
}


void uart_task(void *pvParameters)
{
	uart_event_t event;
	char *data = (char *) malloc(BUFF_SIZE_RX);
	msg_t Out_msg;
	for ( ; ; )
	{
		if (xQueueReceive(uart_queue, (void *) &event, portMAX_DELAY))
		{
			bzero(data, BUFF_SIZE_RX);
			switch (event.type) {
			case UART_DATA:
				if (event.size < 3)
				{
					uart_flush(UART_NUM);
					break;
				}
				uart_read_bytes(UART_NUM, data, event.size, pdMS_TO_TICKS(100));
				uart_flush(UART_NUM);
				Out_msg.lora_rx = data;
				Out_msg.lora_rx_length = event.size;
				xQueueSendToBack(xQueue_uart_lora, (void*)&Out_msg, ( TickType_t ) 0);
				printf("Data_recieve len (%d): \n%s", event.size, data);
				fflush ( stdout );
				break;
			case UART_BREAK:
				ESP_LOGI(TAG, "break Uart");
				uart_driver_delete(UART_NUM);
				init_uart();
				break;
			case UART_BUFFER_FULL:
				uart_flush(UART_NUM);
				uart_flush_input(UART_NUM);
				break;
			case UART_FIFO_OVF:
				uart_flush(UART_NUM);
				uart_flush_input(UART_NUM);
				break;
			case UART_FRAME_ERR:
				ESP_LOGI(TAG, "Frame Error");
				uart_flush(UART_NUM);
				break;
			case UART_PARITY_ERR:
				ESP_LOGI(TAG, "Parity Error");
				uart_flush(UART_NUM);
				break;
			case UART_DATA_BREAK:
				ESP_LOGI(TAG, "Data Error");
				uart_flush(UART_NUM);
				break;
			case UART_PATTERN_DET:
				ESP_LOGI(TAG, "Patron Detectado");
				uart_flush(UART_NUM);
				break;
			default:
				break;
			}
		}
	}
	free(data);
	ESP_LOGI(TAG, "Task Delete");
	vTaskDelete(NULL);
}

void init_uart( void )
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
	uart_driver_install(UART_NUM, BUFF_SIZE_RX, BUFF_SIZE_TX, QUEQUE_LENGTH , &uart_queue, ESP_INTR_FLAG_LEVEL1);
}

void test_lora_uart(void *pvParameters) {
    const char *cmd = "AT\r\n";
    uint8_t rx_buffer[128];

    vTaskDelay(2000 / portTICK_PERIOD_MS); // espera a que el módulo arranque
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



void init_reset_gpio( void )
{
	gpio_config_t gpio =
	{
		.pin_bit_mask = GPIO_OUTPUT_PIN_SEL_LORA,    /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
		.mode = GPIO_MODE_OUTPUT,               /*!< GPIO mode: set input/output mode                     */
		.pull_up_en = GPIO_PULLUP_DISABLE,      /*!< GPIO pull-up                                         */
		.pull_down_en = GPIO_PULLDOWN_ENABLE, 	/*!< GPIO pull-down                                       */
		.intr_type = GPIO_INTR_DISABLE,      		/*!< GPIO interrupt type                                  */
	};
	gpio_config(&gpio);
	gpio_set_level(RESET_LORA, LORA_ON);
}

void reset_lora_func( void )
{
	gpio_set_level(RESET_LORA, LORA_OFF);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	gpio_set_level(RESET_LORA, LORA_ON);
	vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void read_lora_credentials( void )
{

	reset_lora_func();
	printf("Read Lora read_lora_credentials\n");
	char lora_tx[20] = {0};
	strcpy(lora_tx, "AT+DEVEUI=?\r\n");
	uart_write_bytes(UART_NUM, (const void *) lora_tx, strlen(lora_tx));
	strcpy(lora_tx, "AT+APPEUI=?\r\n");
	uart_write_bytes(UART_NUM, (const void *) lora_tx, strlen(lora_tx));
	strcpy(lora_tx, "AT+APPKEY=?\r\n");
	uart_write_bytes(UART_NUM, (const void *) lora_tx, strlen(lora_tx));
	vTaskDelay(100 / portTICK_PERIOD_MS);
	xQueueReset(xQueue_uart_lora);
}

void create_lora_hex_string(int msj_num, float float_val_1, float float_val_2, char *dst)
{
	if (strlen(dst) > 1000)
	{
		return;
	}
	
	// Convertir floats a bytes para mejor compatibilidad con TTN
	uint8_t bytes[8];
	memcpy(&bytes[0], &float_val_1, 4);
	memcpy(&bytes[4], &float_val_2, 4);
	
	// Crear payload hexadecimal más legible para TTN
	char hex_payload[20];
	sprintf(hex_payload, "%02X%02X%02X%02X%02X%02X%02X%02X", 
		bytes[0], bytes[1], bytes[2], bytes[3],
		bytes[4], bytes[5], bytes[6], bytes[7]);
	
	// Crear comando AT+SEND con el payload
	sprintf(dst, "AT+SEND=10:%s\r\n", hex_payload);
	
	printf("TTN Payload: %s\n", hex_payload);
	printf("AT Command: %s", dst);
}
