#include <stdio.h>
#include "lcd1602_i2c.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "math.h"
#include "lcd1602_i2c.h"

#define TAG "TEMP_ADC"

// ---- Configurações Gerais ----
#define DEFAULT_ALARM_TEMP 25
#define ADC_CHANNEL ADC_CHANNEL_6   // GPIO7 (exemplo Wokwi)
#define ADC_ATTEN ADC_ATTEN_DB_11
#define ADC_WIDTH ADC_WIDTH_BIT_12

#define I2C_MASTER_SDA 8
#define I2C_MASTER_SCL 9
#define I2C_MASTER_FREQ_HZ 100000

#define BUTTON_INC 3
#define BUTTON_DEC 4
#define BUZZER_PIN 5

#define LED1 10
#define LED2 11
#define LED3 12
#define LED4 13

// ---- Variáveis globais ----
float alarm_temp = DEFAULT_ALARM_TEMP;
float ntc_temp = 0.0;
bool alarm_on = false;

QueueHandle_t gpio_evt_queue = NULL;

// ---- Funções auxiliares ----
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

float adc_to_temp(uint32_t adc) {
    // Conversão simulada para NTC 10k
    float voltage = (adc * 3.3) / 4095.0;
    float tempC = (100.0 * voltage); // Simples mapeamento linear (simulação)
    return tempC;
}

void update_leds(float diff) {
    gpio_set_level(LED1, diff <= 20);
    gpio_set_level(LED2, diff <= 15);
    gpio_set_level(LED3, diff <= 10);
    gpio_set_level(LED4, diff <= 2);
}

void set_pwm(bool on) {
    if (on) ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 512);
    else ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// ---- Tarefas ----
void button_task(void *arg) {
    uint32_t io_num;
    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            static uint32_t last_time = 0;
            uint32_t now = xTaskGetTickCount();
            if (now - last_time < pdMS_TO_TICKS(250)) continue; // debounce
            last_time = now;

            if (io_num == BUTTON_INC) alarm_temp += 5;
            else if (io_num == BUTTON_DEC) alarm_temp -= 5;

            printf("Temperatura de alarme: %.1f °C\n", alarm_temp);
        }
    }
}

void temp_task(void *arg) {
    lcd1602_i2c_init(I2C_MASTER_NUM, I2C_MASTER_SDA, I2C_MASTER_SCL, 0x27, 2, 16);
    lcd1602_clear();
    lcd1602_set_cursor(0, 0);
    lcd1602_printf("Iniciando...");

    while (1) {
        uint32_t adc_reading = adc1_get_raw(ADC_CHANNEL);
        ntc_temp = adc_to_temp(adc_reading);
        float diff = alarm_temp - ntc_temp;

        lcd1602_clear();
        lcd1602_set_cursor(0, 0);
        lcd1602_printf("Temp: %.1f C", ntc_temp);
        lcd1602_set_cursor(1, 0);
        lcd1602_printf("Alarme: %.1f C", alarm_temp);

        if (ntc_temp >= alarm_temp) {
            alarm_on = true;
            set_pwm(true);
            gpio_set_level(LED1, 1);
            gpio_set_level(LED2, 1);
            gpio_set_level(LED3, 1);
            gpio_set_level(LED4, 1);
            vTaskDelay(pdMS_TO_TICKS(200));
            gpio_set_level(LED1, 0);
            gpio_set_level(LED2, 0);
            gpio_set_level(LED3, 0);
            gpio_set_level(LED4, 0);
        } else {
            alarm_on = false;
            set_pwm(false);
            update_leds(diff);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main() {
    esp_log_level_set(TAG, ESP_LOG_INFO);
    printf("Sistema de leitura NTC iniciado.\n");

    // Configura ADC
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);

    // Configura LEDs
    gpio_reset_pin(LED1); gpio_reset_pin(LED2);
    gpio_reset_pin(LED3); gpio_reset_pin(LED4);
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED4, GPIO_MODE_OUTPUT);

    // Configura botões
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_INC) | (1ULL << BUTTON_DEC),
        .pull_up_en = 1
    };
    gpio_config(&io_conf);

    // Fila de interrupção
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_INC, gpio_isr_handler, (void*) BUTTON_INC);
    gpio_isr_handler_add(BUTTON_DEC, gpio_isr_handler, (void*) BUTTON_DEC);

    // Configura PWM para buzzer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = 2000
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = BUZZER_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&ledc_channel);

    // Cria tarefas
    xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);
    xTaskCreate(temp_task, "temp_task", 4096, NULL, 5, NULL);
}
