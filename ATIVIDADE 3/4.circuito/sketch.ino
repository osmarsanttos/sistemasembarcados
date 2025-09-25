#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// Configuração dos LEDs
#define LED1 GPIO_NUM_2
#define LED2 GPIO_NUM_3
#define LED3 GPIO_NUM_4
#define LED4 GPIO_NUM_5

// Tempo de atualização em milissegundos
#define DELAY_MS 500

// Vetor com os LEDs para facilitar loops
const gpio_num_t leds[] = {LED1, LED2, LED3, LED4};

// Função para inicializar os LEDs como saída e desligá-los
void leds_init() {
    for (int i = 0; i < 4; i++) {
        gpio_reset_pin(leds[i]);
        gpio_set_direction(leds[i], GPIO_MODE_OUTPUT);
        gpio_set_level(leds[i], 0);  // LEDs iniciam apagados
    }
}

// Função para acender um LED específico
void led_on(gpio_num_t led) {
    gpio_set_level(led, 1);
}

// Função para apagar um LED específico
void led_off(gpio_num_t led) {
    gpio_set_level(led, 0);
}

// Função para apagar todos os LEDs
void leds_off_all() {
    for (int i = 0; i < 4; i++) {
        led_off(leds[i]);
    }
}

// Função da Fase 1: Contador Binário de 0 a 15
void fase1_contador_binario() {
    for (int i = 0; i < 16; i++) {
        for (int bit = 0; bit < 4; bit++) {
            gpio_set_level(leds[bit], (i >> bit) & 0x01);
        }
        vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);
    }
}

// Função da Fase 2: Sequência de Varredura
void fase2_varredura() {
    // Acender do LED1 ao LED4
    for (int i = 0; i < 4; i++) {
        leds_off_all();
        led_on(leds[i]);
        vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);
    }
    // Acender do LED4 ao LED1
    for (int i = 3; i >= 0; i--) {
        leds_off_all();
        led_on(leds[i]);
        vTaskDelay(DELAY_MS / portTICK_PERIOD_MS);
    }
}

void app_main(void) {
    leds_init();

    while (1) {
        // Fase 1: Contador Binário
        fase1_contador_binario();

        // Fase 2: Sequência de Varredura
        fase2_varredura();
    }
}
