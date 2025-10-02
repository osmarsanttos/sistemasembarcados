#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"

#define LED1_GPIO 4
#define LED2_GPIO 5
#define LED3_GPIO 6
#define LED4_GPIO 7
#define BUZZER_GPIO 8

#define LED_FREQUENCY 1000   // 1 kHz para LEDs
#define BUZZER_BASE_FREQ 1000 // 1 kHz inicial
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_RES LEDC_TIMER_10_BIT // resolução de 10 bits
#define DELAY_MS 50                // atraso entre steps

// Número de steps no fade (quanto maior, mais suave)
#define FADE_STEPS 50

// --- Funções auxiliares ---
void set_led_duty(ledc_channel_t channel, int duty) {
    ledc_set_duty(LEDC_MODE, channel, duty);
    ledc_update_duty(LEDC_MODE, channel);
}

void set_buzzer_freq(int freq) {
    ledc_set_freq(LEDC_MODE, LEDC_TIMER_1, freq);
}

// --- Configuração inicial ---
void init_pwm() {
    // Configura timer para LEDs
    ledc_timer_config_t led_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LED_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&led_timer);

    // Configura timer para buzzer
    ledc_timer_config_t buzzer_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_RES,
        .timer_num = LEDC_TIMER_1,
        .freq_hz = BUZZER_BASE_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&buzzer_timer);

    // Configura canais dos LEDs
    ledc_channel_config_t channels[4] = {
        {.channel = LEDC_CHANNEL_0, .duty = 0, .gpio_num = LED1_GPIO, .speed_mode = LEDC_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER},
        {.channel = LEDC_CHANNEL_1, .duty = 0, .gpio_num = LED2_GPIO, .speed_mode = LEDC_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER},
        {.channel = LEDC_CHANNEL_2, .duty = 0, .gpio_num = LED3_GPIO, .speed_mode = LEDC_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER},
        {.channel = LEDC_CHANNEL_3, .duty = 0, .gpio_num = LED4_GPIO, .speed_mode = LEDC_MODE, .hpoint = 0, .timer_sel = LEDC_TIMER},
    };

    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&channels[i]);
    }

    // Configura canal do buzzer
    ledc_channel_config_t buzzer_channel = {
        .channel = LEDC_CHANNEL_4,
        .duty = 512, // duty 50% para som
        .gpio_num = BUZZER_GPIO,
        .speed_mode = LEDC_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_1
    };
    ledc_channel_config(&buzzer_channel);
}

// --- Fases ---
void fading_sincronizado() {
    printf("Fase 1: Fading sincronizado\n");
    for (int duty = 0; duty < (1 << LEDC_RES); duty += (1 << LEDC_RES) / FADE_STEPS) {
        for (int i = 0; i < 4; i++) set_led_duty(i, duty);
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
    for (int duty = (1 << LEDC_RES); duty > 0; duty -= (1 << LEDC_RES) / FADE_STEPS) {
        for (int i = 0; i < 4; i++) set_led_duty(i, duty);
        vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
    }
}

void fading_sequencial() {
    printf("Fase 2: Fading sequencial\n");
    for (int led = 0; led < 4; led++) {
        for (int duty = 0; duty < (1 << LEDC_RES); duty += (1 << LEDC_RES) / FADE_STEPS) {
            set_led_duty(led, duty);
            vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
        }
        for (int duty = (1 << LEDC_RES); duty > 0; duty -= (1 << LEDC_RES) / FADE_STEPS) {
            set_led_duty(led, duty);
            vTaskDelay(pdMS_TO_TICKS(DELAY_MS));
        }
    }
}

void teste_buzzer() {
    printf("Fase 3: Teste buzzer\n");
    for (int f = 500; f <= 2000; f += 100) {
        set_buzzer_freq(f);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    for (int f = 2000; f >= 500; f -= 100) {
        set_buzzer_freq(f);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// --- Main ---
void app_main() {
    init_pwm();

    while (1) {
        fading_sincronizado();
        fading_sequencial();
        teste_buzzer();
    }
}
