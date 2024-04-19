/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#pragma once
#include <stdint.h>
#include <stdbool.h>

#include "mpu6050.h"

#define DEFAULT_TEMPERATURE 25.0
#define REPORTING_PERIOD    0.5 /* Seconds */

#define ESP_RMAKER_PARAM_ACCELERATION_X   "esp.param.accelerationx"
#define ESP_RMAKER_PARAM_ACCELERATION_Y   "esp.param.accelerationy"
#define ESP_RMAKER_PARAM_ACCELERATION_Z   "esp.param.accelerationz"
#define ESP_RMAKER_PARAM_GYRO_X           "esp.param.gyrox"
#define ESP_RMAKER_PARAM_GYRO_Y           "esp.param.gyroy"
#define ESP_RMAKER_PARAM_GYRO_Z           "esp.param.gyroz"

#define CONFIG_I2C_MASTER_SCL 7
#define CONFIG_I2C_MASTER_SDA 6

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define MPU6050_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU6050 sensor */
#define MPU6050_WHO_AM_I_REG_ADDR           0x75        /*!< Register addresses of the "who am I" register */

// extern esp_rmaker_device_t *mpu6050_device;

extern esp_rmaker_device_t *mpu6050_device_temp;
extern esp_rmaker_device_t *mpu6050_device_acce_x;
extern esp_rmaker_device_t *mpu6050_device_acce_y;
extern esp_rmaker_device_t *mpu6050_device_acce_z;
extern esp_rmaker_device_t *mpu6050_device_gyro_x;
extern esp_rmaker_device_t *mpu6050_device_gyro_y;
extern esp_rmaker_device_t *mpu6050_device_gyro_z;

void app_driver_init(void);
float app_get_current_temperature();
float app_get_current_acceleration_x();
float app_get_current_acceleration_y();
float app_get_current_acceleration_z();
float app_get_current_gyro_x();
float app_get_current_gyro_y();
float app_get_current_gyro_z();
