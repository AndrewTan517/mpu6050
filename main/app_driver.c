/*  Temperature Sensor demo implementation using RGB LED and timer

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>
#include <sdkconfig.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h> 
#include <esp_rmaker_standard_params.h> 

#include <app_reset.h>
#include <ws2812_led.h>
#include "app_priv.h"

#include "mpu6050.h"

/* This is the button that is used for toggling the power */
#define BUTTON_GPIO          CONFIG_EXAMPLE_BOARD_BUTTON_GPIO
#define BUTTON_ACTIVE_LEVEL  0
/* This is the GPIO on which the power will be set */
#define OUTPUT_GPIO    19

// static TimerHandle_t sensor_timer;

#define DEFAULT_SATURATION  100
#define DEFAULT_BRIGHTNESS  50

#define WIFI_RESET_BUTTON_TIMEOUT       3
#define FACTORY_RESET_BUTTON_TIMEOUT    10

// static uint16_t g_hue;
// static uint16_t g_saturation = DEFAULT_SATURATION;
// static uint16_t g_value = DEFAULT_BRIGHTNESS;
// static float g_temperature;

float app_get_current_temperature(mpu6050_handle_t sensor)
{
    // return g_temperature;

    mpu6050_temp_value_t temp_value[1];
    ESP_ERROR_CHECK(mpu6050_get_temp(sensor, temp_value));

    return temp_value->temp;
}

float app_get_current_acceleration_x(mpu6050_handle_t sensor)
{
    mpu6050_acce_value_t acce_value[3];      
    ESP_ERROR_CHECK(mpu6050_get_acce(sensor, acce_value));

    return acce_value->acce_x;
}

float app_get_current_acceleration_y(mpu6050_handle_t sensor)
{
    mpu6050_acce_value_t acce_value[3];      
    ESP_ERROR_CHECK(mpu6050_get_acce(sensor, acce_value));

    return acce_value->acce_y;
}

float app_get_current_acceleration_z(mpu6050_handle_t sensor)
{
    mpu6050_acce_value_t acce_value[3];      
    ESP_ERROR_CHECK(mpu6050_get_acce(sensor, acce_value));

    return acce_value->acce_z;
}

float app_get_current_gyro_x(mpu6050_handle_t sensor)
{
    mpu6050_gyro_value_t gyro_value[3];
    ESP_ERROR_CHECK(mpu6050_get_gyro(sensor, gyro_value));

    return gyro_value->gyro_x;
}

float app_get_current_gyro_y(mpu6050_handle_t sensor)
{
    mpu6050_gyro_value_t gyro_value[3];
    ESP_ERROR_CHECK(mpu6050_get_gyro(sensor, gyro_value));

    return gyro_value->gyro_y;
}

float app_get_current_gyro_z(mpu6050_handle_t sensor)
{
    mpu6050_gyro_value_t gyro_value[3];
    ESP_ERROR_CHECK(mpu6050_get_gyro(sensor, gyro_value));

    return gyro_value->gyro_z;
}

esp_err_t app_sensor_init(void)
{
    // esp_err_t err = ws2812_led_init();
    // if (err != ESP_OK) {
    //     return err;
    // }

    // g_temperature = DEFAULT_TEMPERATURE;
    // TimerHandle_t sensor_timer = xTimerCreate("app_sensor_update_tm", (REPORTING_PERIOD * 1000) / portTICK_PERIOD_MS,
    //                         pdTRUE, NULL, app_sensor_update);
    // if (sensor_timer) {
    //     xTimerStart(sensor_timer, 0);    
    //     // g_hue = (100 - g_temperature) * 2;
    //     // ws2812_led_set_hsv(g_hue, g_saturation, g_value);
    //     // return ESP_OK;
    // }
    // return ESP_FAIL;

    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void app_driver_init()
{
    app_sensor_init();
    // app_reset_button_register(app_reset_button_create(BUTTON_GPIO, BUTTON_ACTIVE_LEVEL),
    //             WIFI_RESET_BUTTON_TIMEOUT, FACTORY_RESET_BUTTON_TIMEOUT);
}
