/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"

/* Custom includes */
#include "sensor/ccs811.h"

#define I2C_ESP_ADDR (CCS811_ADDRESS)
#define LED_DELAY (500)
#define MEAS_DELAY (3000)
#define CORE_0 (0U)
#define CORE_1 (1U)
#define BLICK_TASK_CORE (CORE_0)
#define WDT_TIMEOUT_MS (1000)

static uint8_t initialized = 0U;
static i2c_master_bus_handle_t i2c_bus_handler;
static i2c_master_dev_handle_t i2c_htu21d_handler;

void configure_app();
void blink_task();

void app_main(void)
{
    int64_t now = 0U;
    int64_t then = 0U;
    uint8_t state = 0U;
    uint8_t hwId = 0U;
    uint8_t hwVer = 0U;
    CCS811_ERR_TYPE errStat = 0U;
    CCS811_STATUS status;
    CCS811_MEASUREMENT data;
    CCS811_MEAS_MODE mode;
    mode.drive_mode = 1;

    printf("Starting app_main()... \n");

    esp_task_wdt_add(NULL);

    configure_app();

    //HTU21D_init(&i2c_htu21d_handler);
    ccs811_init(&i2c_htu21d_handler);
    ccs811_set_mode(mode);

    while(1U)
    {
        esp_task_wdt_reset();
        blink_task();

        now = esp_timer_get_time() / 1000U; // convert microsec to millis

        if(now <= 0)
        {
            then = 0U;
        }

        if((now - then) >= MEAS_DELAY)
        {
            state = !state;
            then = now;

            ccs811_get_status(&status);
            ccs811_get_mode(&mode);

            ccs811_read_measurement(&data);

            ccs811_get_hw_ver(&hwVer);
            ccs811_get_hw_id(&hwId);
            ccs811_get_err_stat(&errStat);
            
            printf("Status.app_valid: %d \n", status.app_valid);
            printf("status.data_ready: %d \n", status.data_ready);
            printf("status.data_ready: %d \n", status.data_ready);
            printf("status.fw_mode: %d \n", status.fw_mode);
            printf("status.error: %d \n", status.error);
            printf("mode: %d \n", mode.flags);
            printf("data.eCO2: %d \n", data.eCO2);
            printf("data.TVOC: %d \n", data.TVOC);
            printf("data.status: %d \n", data.status);
            printf("data.error_id: %d \n", data.error_id);
            printf("data.raw_data: %d \n", data.raw_data);
            printf("HW ID: %d \n", hwId);
            printf("HW version: %d \n", hwVer);
            printf("Err stat: %d \n\n", errStat);
        }

        vTaskDelay(1U);
    }
}

void configure_app()
{
    i2c_master_bus_config_t i2c_conf = {
        .i2c_port = -1,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .glitch_ignore_cnt = 70,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .intr_priority = 0,
        .flags.enable_internal_pullup = 1
    };
    
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ESP_ADDR,
        .scl_speed_hz = 100000,
    };

    gpio_reset_pin(GPIO_NUM_14);
    gpio_set_direction(GPIO_NUM_14, GPIO_MODE_OUTPUT);

    /* I2C config */
    if(ESP_OK != i2c_new_master_bus(&i2c_conf, &i2c_bus_handler))
        esp_restart();

    if(ESP_OK != i2c_master_bus_add_device(i2c_bus_handler, &dev_cfg, &i2c_htu21d_handler))
        esp_restart();

    initialized = 1U;
}

void blink_task()
{
    static int64_t now = 0U;
    static int64_t then = 0U;
    static uint8_t state = 0U;

    now = esp_timer_get_time() / 1000U; // convert microsec to millis

    if(now <= 0)
    {
        then = 0U;
    }

    if((now - then) >= LED_DELAY)
    {
        state = !state;
        then = now;

        gpio_set_level(GPIO_NUM_14, state);
    }
}
