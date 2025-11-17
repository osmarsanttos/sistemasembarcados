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
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_spiffs.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "lcd1602_i2c.h"

static const char *TAG = "WOKWI";

// -------------------- PINOUT --------------------
#define SDA_GPIO            12
#define SCL_GPIO            13

#define NTC_GPIO_CHANNEL    ADC_CHANNEL_3  // NTC simulado com potenciômetro
#define BUTTON_INC_GPIO     10
#define BUTTON_DEC_GPIO     16

#define BUZZER_GPIO         42

#define SEG_A_GPIO  7
#define SEG_B_GPIO  8
#define SEG_C_GPIO  9
#define SEG_D_GPIO  10
#define SEG_E_GPIO  11
#define SEG_F_GPIO  12
#define SEG_G_GPIO  13

// PWM
#define LEDC_MODE              LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL           LEDC_CHANNEL_0
#define LEDC_TIMER             LEDC_TIMER_0
#define LEDC_DUTY_RES          LEDC_TIMER_10_BIT
#define LEDC_FREQ_HZ           2000

// -------------------- VARIÁVEIS GLOBAIS --------------------
static float alarm_temp = 25.0f;
static float ntc_temp = 0.0f;

static QueueHandle_t gpio_evt_queue = NULL;   
static SemaphoreHandle_t alarm_mutex = NULL;

// ADC
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali = NULL;


// -------------------- ISR --------------------
static void IRAM_ATTR gpio_isr_handler(void *arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}


// -------------------- SPIFFS --------------------
static void init_spiffs(void) {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    ESP_ERROR_CHECK(esp_vfs_spiffs_register(&conf));

    size_t total, used;
    esp_spiffs_info(NULL, &total, &used);
    ESP_LOGI(TAG, "SPIFFS: total=%d, used=%d", total, used);
}

static void spiffs_save(float temp, float alarm) {
    FILE *f = fopen("/spiffs/temp_log.txt", "a");
    if (!f) {
        ESP_LOGE(TAG, "Erro abrindo log");
        return;
    }

    int64_t ms = esp_timer_get_time() / 1000;
    fprintf(f, "%lld ms, T=%.2f C, ALARM=%.2f C\n",
            (long long)ms, temp, alarm);

    fclose(f);
}


// -------------------- ADC --------------------
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

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT
    };

    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &adc_cali) == ESP_OK)
        ESP_LOGI(TAG, "Calibração ADC OK");
    else
        ESP_LOGW(TAG, "ADC sem calibração");
}

static float adc_read_temp(void) {
    int raw = 0;
    adc_oneshot_read(adc_handle, NTC_GPIO_CHANNEL, &raw);

    int mv = raw * 3300 / 4095; // Wokwi não calibra ADC

    float t = (float)mv / 10.0f; // Simulação
    return t;
}


// -------------------- 7-SEG --------------------
static void seg_write(bool a,bool b,bool c,bool d,bool e,bool f,bool g) {
    gpio_set_level(SEG_A_GPIO,a);
    gpio_set_level(SEG_B_GPIO,b);
    gpio_set_level(SEG_C_GPIO,c);
    gpio_set_level(SEG_D_GPIO,d);
    gpio_set_level(SEG_E_GPIO,e);
    gpio_set_level(SEG_F_GPIO,f);
    gpio_set_level(SEG_G_GPIO,g);
}

static void seg_display(char c) {
    if (c=='0') seg_write(1,1,1,1,1,1,0);
    else if (c=='3') seg_write(1,1,1,1,0,0,1);
    else if (c=='7') seg_write(1,1,1,0,0,0,0);
    else if (c=='D') seg_write(0,1,1,1,1,0,1);
    else if (c=='F') seg_write(1,0,0,0,1,1,1);
    else seg_write(0,0,0,0,0,0,0);
}


// -------------------- TASK: Botões --------------------
static void button_task(void *arg) {
    uint32_t io_num;
    TickType_t last_inc = 0, last_dec = 0;

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            TickType_t now = xTaskGetTickCount();

            if (io_num == BUTTON_INC_GPIO &&
                now - last_inc > pdMS_TO_TICKS(250)) {

                xSemaphoreTake(alarm_mutex, portMAX_DELAY);
                alarm_temp += 5;
                xSemaphoreGive(alarm_mutex);

                last_inc = now;
                ESP_LOGI(TAG, "INC -> alarm=%.1f", alarm_temp);
            }

            if (io_num == BUTTON_DEC_GPIO &&
                now - last_dec > pdMS_TO_TICKS(250)) {

                xSemaphoreTake(alarm_mutex, portMAX_DELAY);
                alarm_temp -= 5;
                xSemaphoreGive(alarm_mutex);

                last_dec = now;
                ESP_LOGI(TAG, "DEC -> alarm=%.1f", alarm_temp);
            }
        }
    }
}


// -------------------- TASK: ADC --------------------
static void adc_task(void *arg) {
    while (1) {
        ntc_temp = adc_read_temp();
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


// -------------------- TASK: LCD --------------------
static void lcd_task(void *arg) {
    lcd1602_i2c_init(I2C_NUM_0, SDA_GPIO, SCL_GPIO, 0x27, 2, 16);
    lcd1602_clear();

    while (1) {
        float alarm;

        xSemaphoreTake(alarm_mutex, portMAX_DELAY);
        alarm = alarm_temp;
        xSemaphoreGive(alarm_mutex);

        lcd1602_clear();
        lcd1602_set_cursor(0, 0);
        lcd1602_printf("Temp: %.1f C", ntc_temp);
        lcd1602_set_cursor(1, 0);
        lcd1602_printf("Alarm: %.1f C", alarm);

        vTaskDelay(pdMS_TO_TICKS(400));
    }
}


// -------------------- TASK: 7-SEG --------------------
static void seg_task(void *arg) {
    while (1) {
        float alarm;

        xSemaphoreTake(alarm_mutex, portMAX_DELAY);
        alarm = alarm_temp;
        xSemaphoreGive(alarm_mutex);

        float diff = alarm - ntc_temp;

        if (ntc_temp >= alarm) {
            seg_display('F');
            vTaskDelay(300/portTICK_PERIOD_MS);
            seg_display(' ');
            vTaskDelay(300/portTICK_PERIOD_MS);
            continue;
        }

        if (diff >= 20) seg_display('0');
        else if (diff >= 15) seg_display('3');
        else if (diff >= 10) seg_display('7');
        else if (diff >= 2) seg_display('D');
        else seg_display(' ');

        vTaskDelay(200/portTICK_PERIOD_MS);
    }
}


// -------------------- TASK: Buzzer --------------------
static void buzzer_task(void *arg) {
    while (1) {
        float alarm;

        xSemaphoreTake(alarm_mutex, portMAX_DELAY);
        alarm = alarm_temp;
        xSemaphoreGive(alarm_mutex);

        if (ntc_temp >= alarm)
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 512);
        else
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);

        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(150));
    }
}


// -------------------- TASK: SPIFFS SAVE --------------------
static void spiffs_task(void *arg) {
    while (1) {
        float alarm;

        xSemaphoreTake(alarm_mutex, portMAX_DELAY);
        alarm = alarm_temp;
        xSemaphoreGive(alarm_mutex);

        spiffs_save(ntc_temp, alarm);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}



// -------------------- MAIN --------------------
void app_main(void) {

    alarm_mutex = xSemaphoreCreateMutex();

    // SPIFFS
    init_spiffs();

    // ADC
    init_adc();

    // 7-seg GPIO config
    int segs[]={SEG_A_GPIO,SEG_B_GPIO,SEG_C_GPIO,SEG_D_GPIO,
                SEG_E_GPIO,SEG_F_GPIO,SEG_G_GPIO};
    for(int i=0;i<7;i++){
        gpio_set_direction(segs[i], GPIO_MODE_OUTPUT);
        gpio_set_level(segs[i],0);
    }

    // Botões
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    gpio_config_t btn = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL<<BUTTON_INC_GPIO) | (1ULL<<BUTTON_DEC_GPIO),
        .pull_up_en = 1
    };
    gpio_config(&btn);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_INC_GPIO, gpio_isr_handler, (void*)BUTTON_INC_GPIO);
    gpio_isr_handler_add(BUTTON_DEC_GPIO, gpio_isr_handler, (void*)BUTTON_DEC_GPIO);

    // PWM Buzzer
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer);

    ledc_channel_config_t ch = {
        .channel = LEDC_CHANNEL,
        .duty = 0,
        .gpio_num = BUZZER_GPIO,
        .speed_mode = LEDC_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER
    };
    ledc_channel_config(&ch);


    // Criação das tasks
    xTaskCreate(button_task, "button_task", 4096, NULL, 10, NULL);
    xTaskCreate(adc_task,    "adc_task",    4096, NULL, 9,  NULL);
    xTaskCreate(lcd_task,    "lcd_task",    4096, NULL, 8,  NULL);
    xTaskCreate(seg_task,    "seg_task",    4096, NULL, 7,  NULL);
    xTaskCreate(buzzer_task, "buzzer_task", 4096, NULL, 6,  NULL);
    xTaskCreate(spiffs_task, "spiffs_task", 4096, NULL, 5,  NULL);

    ESP_LOGI(TAG, "Sistema pronto.");
}
