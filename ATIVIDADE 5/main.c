#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

// Define os pinos dos LEDs (4 bits)
#define LED0 GPIO_NUM_2
#define LED1 GPIO_NUM_4
#define LED2 GPIO_NUM_5
#define LED3 GPIO_NUM_18

// Define os pinos dos botões
#define BTN_INC GPIO_NUM_12
#define BTN_DEC GPIO_NUM_13

// Tempo de debounce
#define DEBOUNCE_TIME_MS 200

uint8_t contador = 0;

void atualizar_leds(uint8_t valor) {
    gpio_set_level(LED0, valor & 0x01);
    gpio_set_level(LED1, (valor >> 1) & 0x01);
    gpio_set_level(LED2, (valor >> 2) & 0x01);
    gpio_set_level(LED3, (valor >> 3) & 0x01);
}

void app_main(void) {
    // Configura LEDs como saída
    gpio_set_direction(LED0, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT);

    // Configura botões como entrada com pull-down
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_INC) | (1ULL << BTN_DEC),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 0,
        .pull_down_en = 1,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Inicializa LEDs
    atualizar_leds(contador);

    while (1) {
        // Verifica botão de incremento
        if (gpio_get_level(BTN_INC)) {
            contador = (contador + 1) % 16;  // Overflow: 15 -> 0
            atualizar_leds(contador);
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME_MS));
        }

        // Verifica botão de decremento
        if (gpio_get_level(BTN_DEC)) {
            contador = (contador == 0) ? 15 : contador - 1;  // Underflow: 0 -> 15
            atualizar_leds(contador);
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_TIME_MS));
        }

        vTaskDelay(pdMS_TO_TICKS(10)); // Pequeno delay para evitar sobrecarga
    }
}
