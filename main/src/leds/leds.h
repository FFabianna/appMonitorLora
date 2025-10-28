/***********
 * Creado por:  PDG sistema fotovoltaico
  * Date:        2025-2
    * Descripción: Control de leds
 **************/
#ifndef LEDS_H
#define LEDS_H

/************* (Definiciones de pines asociados a los leds) *************/
//#define LED_R 			GPIO_NUM_16
#define LED_G 			GPIO_NUM_2
#define LED_B 		    GPIO_NUM_4

/************* (Definiciones de control) *************/
#define LED_ON  	1
#define LED_OFF 	0
#define LEDS_NUM 	3

/************* (Mascara de selección de pines) *************/
//#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<LED_R) | (1ULL<<LED_G) | (1ULL<<LED_B) )


/************* (Definición de estructura de LEDS) *************/
struct _led_state {
    uint8_t current;
    uint8_t previous;
    uint8_t pin;
    char *name;
};


/***************************
 * Function
 * leds_init: Función que inicializa los leds en estado apagado.
 *   @param:
 *      Sin parametros
 ***********/

void leds_init(void);


/***************************
 * Function
 * board_led_operation: Función que cambia el estado del led en caso de ser necesario.
 * @param
 *         pin: Pin del led a cambiar de estado.
 *         onoff: Estado al que se desea llevar el Led (LED_ON/LED_OFF).
 ***********/

void board_led_operation(uint8_t pin, uint8_t onoff);

#endif
