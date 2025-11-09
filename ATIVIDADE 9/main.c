
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "lcd1602_i2c.h"

// NOVA API ADC
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static const char *TAG = "MAIN_STATE";

// -------------------- PINOUT --------------------
#define SDA_GPIO            12
#define SCL_GPIO            13
#define NTC_GPIO_CHANNEL    ADC_CHANNEL_3    // GPIO4
#define BUTTON_INC_GPIO     10
#define BUTTON_DEC_GPIO     16
#define BUZZER_GPIO         42
#define LED_GPIO_1          40
#define LED_GPIO_2          45
#define LED_GPIO_3          48
#define LED_GPIO_4          47
#define SD_MOSI_GPIO        23
#define SD_MISO_GPIO        19
#define SD_SCK_GPIO         18
#define SD_CS_GPIO          5

#define I2C_NUM             I2C_NUM_0
#define I2C_FREQ_HZ         100000
#define LCD_I2C_ADDR        0x27

// -------------------- PARAMETROS --------------------
#define DEFAULT_ALARM_TEMP_C    25.0f
#define BUTTON_DEBOUNCE_MS     250
#define LEDC_MODE              LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL           LEDC_CHANNEL_0
#define LEDC_TIMER             LEDC_TIMER_0
#define LEDC_DUTY_RES          LEDC_TIMER_10_BIT
#define LEDC_FREQ_HZ           2000

// -------------------- VARIÁVEIS GLOBAIS --------------------
static volatile float alarm_temp = DEFAULT_ALARM_TEMP_C;
static volatile float ntc_temp = 0.0f;
static volatile bool sd_mounted = false;

static QueueHandle_t gpio_evt_queue = NULL;
static SemaphoreHandle_t sd_mutex = NULL;

// Calibração ADC
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali = NULL;

// -------------------- MÁQUINA DE ESTADOS --------------------
typedef enum {
    STATE_READ_TEMP,
    STATE_UPDATE_LCD,
    STATE_UPDATE_LEDS,
    STATE_CHECK_ALARM,
    STATE_SAVE_SD
} system_state_t;

// -------------------- FUNÇÕES AUXILIARES --------------------
static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// Inicializa o ADC e calibração
static void init_adc(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    adc_oneshot_config_channel(adc_handle, NTC_GPIO_CHANNEL, &cfg);

    // Calibração
    adc_cali_curve_fitting_config_t cali_cfg = {
    .unit_id = ADC_UNIT_1,
    .atten = ADC_ATTEN_DB_12,
    .bitwidth = ADC_BITWIDTH_DEFAULT
};
esp_err_t ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali);

    if (ret == ESP_OK) ESP_LOGI(TAG, "Calibração ADC criada.");
    else ESP_LOGW(TAG, "Sem calibração ADC.");
}

static float adc_read_temp(void) {
    int raw;
    adc_oneshot_read(adc_handle, NTC_GPIO_CHANNEL, &raw);
    int mv = 0;
    if (adc_cali) adc_cali_raw_to_voltage(adc_cali, raw, &mv);
    else mv = raw; // fallback
    float tempC = (float)mv / 10.0f; // Simulação (0–3300mV → 0–330°C)
    return tempC;
}

// SDCard inicialização
static esp_err_t sdcard_mount(void) {
    sdmmc_card_t *card;
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_GPIO;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI_GPIO,
        .miso_io_num = SD_MISO_GPIO,
        .sclk_io_num = SD_SCK_GPIO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000
    };
    ESP_ERROR_CHECK(spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA));

    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    esp_err_t ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_cfg, &card);
    if (ret == ESP_OK) {
        sdmmc_card_print_info(stdout, card);
        sd_mounted = true;
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "Falha ao montar SD: %s", esp_err_to_name(ret));
        return ret;
    }
}

// Salva leitura no SD
static void sd_save_reading(float temp, float alarm) {
    if (!sd_mounted) return;
    if (xSemaphoreTake(sd_mutex, pdMS_TO_TICKS(200)) != pdTRUE) return;

    FILE *f = fopen("/sdcard/temp_log.txt", "a");
    if (!f) {
        ESP_LOGE(TAG, "Erro abrindo arquivo!");
        xSemaphoreGive(sd_mutex);
        return;
    }
    int64_t ms = esp_timer_get_time() / 1000;
    fprintf(f, "%lld ms, Temp: %.2f C, Alarm: %.2f C\n", (long long)ms, temp, alarm);
    fclose(f);
    xSemaphoreGive(sd_mutex);
}

// -------------------- TAREFAS --------------------
static void button_task(void *arg) {
    uint32_t io_num;
    static uint32_t last_tick_inc = 0, last_tick_dec = 0;

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            uint32_t now = xTaskGetTickCount();
            if (io_num == BUTTON_INC_GPIO && now - last_tick_inc > pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
                alarm_temp += 5;
                ESP_LOGI(TAG, "Botão + : alarme = %.1f°C", alarm_temp);
                last_tick_inc = now;
            } else if (io_num == BUTTON_DEC_GPIO && now - last_tick_dec > pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
                alarm_temp -= 5;
                ESP_LOGI(TAG, "Botão - : alarme = %.1f°C", alarm_temp);
                last_tick_dec = now;
            }
        }
    }
}

static void system_task(void *arg) {
    system_state_t state = STATE_READ_TEMP;

    lcd1602_i2c_init(I2C_NUM, SDA_GPIO, SCL_GPIO, LCD_I2C_ADDR, 2, 16);
    lcd1602_clear();
    lcd1602_set_cursor(0, 0);
    lcd1602_printf("Iniciando...");

    while (1) {
        switch (state) {
        case STATE_READ_TEMP:
            ntc_temp = adc_read_temp();
            state = STATE_UPDATE_LCD;
            break;

        case STATE_UPDATE_LCD:
            lcd1602_clear();
            lcd1602_set_cursor(0, 0);
            lcd1602_printf("Temp: %.1f C", ntc_temp);
            lcd1602_set_cursor(1, 0);
            lcd1602_printf("Alarme: %.1f C", alarm_temp);
            state = STATE_UPDATE_LEDS;
            break;

        case STATE_UPDATE_LEDS: {
            float diff = alarm_temp - ntc_temp;
            if (ntc_temp >= alarm_temp) {
                for (int i = 0; i < 2; i++) {
                    gpio_set_level(LED_GPIO_1, 1);
                    gpio_set_level(LED_GPIO_2, 1);
                    gpio_set_level(LED_GPIO_3, 1);
                    gpio_set_level(LED_GPIO_4, 1);
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 512);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    vTaskDelay(pdMS_TO_TICKS(200));

                    gpio_set_level(LED_GPIO_1, 0);
                    gpio_set_level(LED_GPIO_2, 0);
                    gpio_set_level(LED_GPIO_3, 0);
                    gpio_set_level(LED_GPIO_4, 0);
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    vTaskDelay(pdMS_TO_TICKS(200));
                }
            } else {
                gpio_set_level(LED_GPIO_1, diff <= 20);
                gpio_set_level(LED_GPIO_2, diff <= 15);
                gpio_set_level(LED_GPIO_3, diff <= 10);
                gpio_set_level(LED_GPIO_4, diff <= 2);
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            }
            state = STATE_CHECK_ALARM;
            break;
        }

        case STATE_CHECK_ALARM:
            if (ntc_temp >= alarm_temp) {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 512);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            } else {
                ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
                ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            }
            state = STATE_SAVE_SD;
            break;

        case STATE_SAVE_SD:
            sd_save_reading(ntc_temp, alarm_temp);
            vTaskDelay(pdMS_TO_TICKS(500));
            state = STATE_READ_TEMP;
            break;
        }
    }
}

// -------------------- MAIN --------------------
void app_main(void) {
    ESP_LOGI(TAG, "Sistema iniciado");

    init_adc();

    // LEDs
    gpio_reset_pin(LED_GPIO_1); gpio_reset_pin(LED_GPIO_2);
    gpio_reset_pin(LED_GPIO_3); gpio_reset_pin(LED_GPIO_4);
    gpio_set_direction(LED_GPIO_1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GPIO_2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GPIO_3, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GPIO_4, GPIO_MODE_OUTPUT);

    // Botões
    gpio_config_t btn_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << BUTTON_INC_GPIO) | (1ULL << BUTTON_DEC_GPIO),
        .pull_up_en = 1
    };
    gpio_config(&btn_conf);

    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_INC_GPIO, gpio_isr_handler, (void*)BUTTON_INC_GPIO);
    gpio_isr_handler_add(BUTTON_DEC_GPIO, gpio_isr_handler, (void*)BUTTON_DEC_GPIO);

    // PWM Buzzer
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_cfg);

    ledc_channel_config_t ch_cfg = {
        .channel = LEDC_CHANNEL,
        .duty = 0,
        .gpio_num = BUZZER_GPIO,
        .speed_mode = LEDC_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER
    };
    ledc_channel_config(&ch_cfg);

    // SDCard
    sd_mutex = xSemaphoreCreateMutex();
    if (sdcard_mount() != ESP_OK)
        ESP_LOGW(TAG, "SDCard não montado.");

    // Cria tarefas
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, NULL);
    xTaskCreate(system_task, "system_task", 8192, NULL, 8, NULL);
}
