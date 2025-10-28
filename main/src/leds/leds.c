/***********
 * Creado por:  PDG sistema fotovoltaico
  * Date:        2025-2
    * Descripción: Control de leds
 **************/

#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "leds.h"

#define TAG "LEDS"



struct _led_state led_state[LEDS_NUM] = {
	//{ LED_OFF, LED_OFF, LED_R, 	"LED_RED"  	},
	{ LED_OFF, LED_OFF, LED_G, 	"LED_GREEN"	},
	{ LED_OFF, LED_OFF, LED_B, 	"LED_BLUE"	},
};


void leds_init(void) {

	gpio_config_t io_conf = {};
	//Disable Interrpts
	io_conf.intr_type = GPIO_INTR_DISABLE;
	//set as output mode
	io_conf.mode = GPIO_MODE_OUTPUT;
	//bit mask of the pins that you want to set
	//io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
	io_conf.pull_down_en = 0;
	//disable pull-up mode
	io_conf.pull_up_en = 0;
	//configure GPIO with the given settings
	gpio_config(&io_conf);
	for (int i = 0; i < LEDS_NUM; i++) {
		gpio_set_level(led_state[i].pin, LED_OFF);
		led_state[i].previous = LED_OFF;
	}
}



void board_led_operation(uint8_t pin, uint8_t onoff)
{
	for (int i = 0; i < LEDS_NUM; i++) {
		if (led_state[i].pin != pin) {
			continue;
		}
		if (onoff == led_state[i].previous) {
			ESP_LOGW(TAG, "led %s is already %s",
			         led_state[i].name, (onoff ? "ON" : "OFF"));
			return;
		}
		gpio_set_level(pin, onoff);
		led_state[i].previous = onoff;
		return;
	}

	ESP_LOGE(TAG, "LED is not found!");
}