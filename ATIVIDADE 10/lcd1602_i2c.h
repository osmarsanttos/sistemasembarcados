#ifndef LCD1602_I2C_H
#define LCD1602_I2C_H

#include "driver/i2c.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ==== Configuração padrão ====
#define I2C_MASTER_NUM I2C_NUM_0

// ==== Protótipos de funções ====
void lcd1602_i2c_init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint8_t addr, int rows, int cols);
void lcd1602_clear(void);
void lcd1602_set_cursor(uint8_t row, uint8_t col);
void lcd1602_printf(const char *fmt, ...);

// ==== Implementação ====
static uint8_t lcd_addr;
static i2c_port_t lcd_i2c_port;
static int lcd_cols, lcd_rows;

#define LCD_CMD_CLEAR 0x01
#define LCD_CMD_HOME  0x02
#define LCD_CMD_ENTRY_MODE 0x04
#define LCD_CMD_DISPLAY_CONTROL 0x08
#define LCD_CMD_FUNCTION_SET 0x20
#define LCD_CMD_SET_DDRAM 0x80

static void lcd_write_cmd(uint8_t cmd) {
    uint8_t data[2] = {0x00, cmd};
    i2c_master_write_to_device(lcd_i2c_port, lcd_addr, data, 2, pdMS_TO_TICKS(100));
}

static void lcd_write_data(uint8_t data) {
    uint8_t buf[2] = {0x40, data};
    i2c_master_write_to_device(lcd_i2c_port, lcd_addr, buf, 2, pdMS_TO_TICKS(100));
}

void lcd1602_i2c_init(i2c_port_t i2c_num, gpio_num_t sda, gpio_num_t scl, uint8_t addr, int rows, int cols) {
    lcd_i2c_port = i2c_num;
    lcd_addr = addr;
    lcd_cols = cols;
    lcd_rows = rows;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };
    i2c_param_config(i2c_num, &conf);
    i2c_driver_install(i2c_num, I2C_MODE_MASTER, 0, 0, 0);

    vTaskDelay(pdMS_TO_TICKS(50));
    lcd_write_cmd(LCD_CMD_FUNCTION_SET | 0x08);
    lcd_write_cmd(LCD_CMD_DISPLAY_CONTROL | 0x04);
    lcd_write_cmd(LCD_CMD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd1602_clear(void) {
    lcd_write_cmd(LCD_CMD_CLEAR);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd1602_set_cursor(uint8_t row, uint8_t col) {
    static const uint8_t row_offsets[] = {0x00, 0x40, 0x14, 0x54};
    lcd_write_cmd(LCD_CMD_SET_DDRAM | (col + row_offsets[row]));
}

void lcd1602_printf(const char *fmt, ...) {
    char buf[32];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    for (int i = 0; i < strlen(buf); i++) {
        lcd_write_data(buf[i]);
    }
}

#endif
