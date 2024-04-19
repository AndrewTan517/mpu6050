/* Temperature Sensor Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>

#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_types.h> 
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>

#include <app_wifi.h>
#include <app_insights.h>

#include "app_priv.h"

#include "mpu6050.h"

static const char *TAG = "app_main";

#define ESP_RMAKER_DEVICE_ACCELEROMETER_X   "esp.device.accelerometerx"
#define ESP_RMAKER_DEVICE_ACCELEROMETER_Y   "esp.device.accelerometery"
#define ESP_RMAKER_DEVICE_ACCELEROMETER_Z   "esp.device.accelerometerz"
#define ESP_RMAKER_DEVICE_GYROSCOPE_X       "esp.device.gyroscopex"
#define ESP_RMAKER_DEVICE_GYROSCOPE_Y       "esp.device.gyroscopey"
#define ESP_RMAKER_DEVICE_GYROSCOPE_Z       "esp.device.gyroscopez"

#define ESP_RMAKER_DEF_ACCELERATION_X_NAME     "Acceleration x"
#define ESP_RMAKER_DEF_ACCELERATION_Y_NAME     "Acceleration y"
#define ESP_RMAKER_DEF_ACCELERATION_Z_NAME     "Acceleration z"
#define ESP_RMAKER_DEF_GYRO_X_NAME             "Gyro x"
#define ESP_RMAKER_DEF_GYRO_Y_NAME             "Gyro y"
#define ESP_RMAKER_DEF_GYRO_Z_NAME             "Gyro z"

#define ESP_RMAKER_UI_TEXT              "esp.ui.text"

// esp_rmaker_device_t *mpu6050_device;

esp_rmaker_device_t *mpu6050_device_temp;
esp_rmaker_device_t *mpu6050_device_acce_x;
esp_rmaker_device_t *mpu6050_device_acce_y;
esp_rmaker_device_t *mpu6050_device_acce_z;
esp_rmaker_device_t *mpu6050_device_gyro_x;
esp_rmaker_device_t *mpu6050_device_gyro_y;
esp_rmaker_device_t *mpu6050_device_gyro_z;

esp_rmaker_param_t *esp_rmaker_acceleration_x_param_create(const char *param_name, float val)
{
    esp_rmaker_param_t *param = esp_rmaker_param_create(param_name, ESP_RMAKER_PARAM_ACCELERATION_X,
            esp_rmaker_float(val), PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    if (param) {
        esp_rmaker_param_add_ui_type(param, ESP_RMAKER_UI_TEXT);
    }
    return param;
}

esp_rmaker_param_t *esp_rmaker_acceleration_y_param_create(const char *param_name, float val)
{
    esp_rmaker_param_t *param = esp_rmaker_param_create(param_name, ESP_RMAKER_PARAM_ACCELERATION_Y,
            esp_rmaker_float(val), PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    if (param) {
        esp_rmaker_param_add_ui_type(param, ESP_RMAKER_UI_TEXT);
    }
    return param;
}

esp_rmaker_param_t *esp_rmaker_acceleration_z_param_create(const char *param_name, float val)
{
    esp_rmaker_param_t *param = esp_rmaker_param_create(param_name, ESP_RMAKER_PARAM_ACCELERATION_Z,
            esp_rmaker_float(val), PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    if (param) {
        esp_rmaker_param_add_ui_type(param, ESP_RMAKER_UI_TEXT);
    }
    return param;
}

esp_rmaker_param_t *esp_rmaker_gyro_x_param_create(const char *param_name, float val)
{
    esp_rmaker_param_t *param = esp_rmaker_param_create(param_name, ESP_RMAKER_PARAM_GYRO_X,
            esp_rmaker_float(val), PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    if (param) {
        esp_rmaker_param_add_ui_type(param, ESP_RMAKER_UI_TEXT);
    }
    return param;
}

esp_rmaker_param_t *esp_rmaker_gyro_y_param_create(const char *param_name, float val)
{
    esp_rmaker_param_t *param = esp_rmaker_param_create(param_name, ESP_RMAKER_PARAM_GYRO_Y,
            esp_rmaker_float(val), PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    if (param) {
        esp_rmaker_param_add_ui_type(param, ESP_RMAKER_UI_TEXT);
    }
    return param;
}

esp_rmaker_param_t *esp_rmaker_gyro_z_param_create(const char *param_name, float val)
{
    esp_rmaker_param_t *param = esp_rmaker_param_create(param_name, ESP_RMAKER_PARAM_GYRO_Z,
            esp_rmaker_float(val), PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    if (param) {
        esp_rmaker_param_add_ui_type(param, ESP_RMAKER_UI_TEXT);
    }
    return param;
}

esp_rmaker_device_t *esp_rmaker_accelerometer_x_device_create(const char *dev_name,
        void *priv_data, float acceleration_x)
{
    esp_rmaker_device_t *device = esp_rmaker_device_create(dev_name, ESP_RMAKER_DEVICE_ACCELEROMETER_X, priv_data);
    if (device) {
        esp_rmaker_device_add_param(device, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, dev_name));
        esp_rmaker_param_t *primary = esp_rmaker_acceleration_x_param_create(ESP_RMAKER_DEF_ACCELERATION_X_NAME, acceleration_x);
        esp_rmaker_device_add_param(device, primary);
        esp_rmaker_device_assign_primary_param(device, primary);
	}
    return device;
}

esp_rmaker_device_t *esp_rmaker_accelerometer_y_device_create(const char *dev_name,
        void *priv_data, float acceleration_y)
{
    esp_rmaker_device_t *device = esp_rmaker_device_create(dev_name, ESP_RMAKER_DEVICE_ACCELEROMETER_Y, priv_data);
    if (device) {
        esp_rmaker_device_add_param(device, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, dev_name));
        esp_rmaker_param_t *primary = esp_rmaker_acceleration_y_param_create(ESP_RMAKER_DEF_ACCELERATION_Y_NAME, acceleration_y);
        esp_rmaker_device_add_param(device, primary);
        esp_rmaker_device_assign_primary_param(device, primary);
    }
    return device;
}

esp_rmaker_device_t *esp_rmaker_accelerometer_z_device_create(const char *dev_name,
        void *priv_data, float acceleration_z)
{
    esp_rmaker_device_t *device = esp_rmaker_device_create(dev_name, ESP_RMAKER_DEVICE_ACCELEROMETER_Z, priv_data);
    if (device) {
        esp_rmaker_device_add_param(device, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, dev_name));
        esp_rmaker_param_t *primary = esp_rmaker_acceleration_z_param_create(ESP_RMAKER_DEF_ACCELERATION_Z_NAME, acceleration_z);
        esp_rmaker_device_add_param(device, primary);
        esp_rmaker_device_assign_primary_param(device, primary);
    }
    return device;
}

esp_rmaker_device_t *esp_rmaker_gyroscope_x_device_create(const char *dev_name,
        void *priv_data, float gyro_x)
{
    esp_rmaker_device_t *device = esp_rmaker_device_create(dev_name, ESP_RMAKER_DEVICE_GYROSCOPE_X, priv_data);
    if (device) {
        esp_rmaker_device_add_param(device, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, dev_name));
        esp_rmaker_param_t *primary = esp_rmaker_gyro_x_param_create(ESP_RMAKER_DEF_GYRO_X_NAME, gyro_x);
        esp_rmaker_device_add_param(device, primary);
        esp_rmaker_device_assign_primary_param(device, primary);
    }
    return device;
}

esp_rmaker_device_t *esp_rmaker_gyroscope_y_device_create(const char *dev_name,
        void *priv_data, float gyro_y)
{
    esp_rmaker_device_t *device = esp_rmaker_device_create(dev_name, ESP_RMAKER_DEVICE_GYROSCOPE_Y, priv_data);
    if (device) {
        esp_rmaker_device_add_param(device, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, dev_name));
        esp_rmaker_param_t *primary = esp_rmaker_gyro_y_param_create(ESP_RMAKER_DEF_GYRO_Y_NAME, gyro_y);
        esp_rmaker_device_add_param(device, primary);
        esp_rmaker_device_assign_primary_param(device, primary);
    }
    return device;
}

esp_rmaker_device_t *esp_rmaker_gyroscope_z_device_create(const char *dev_name,
        void *priv_data, float gyro_z)
{
    esp_rmaker_device_t *device = esp_rmaker_device_create(dev_name, ESP_RMAKER_DEVICE_GYROSCOPE_Z, priv_data);
    if (device) {
        esp_rmaker_device_add_param(device, esp_rmaker_name_param_create(ESP_RMAKER_DEF_NAME_PARAM, dev_name));
        esp_rmaker_param_t *primary = esp_rmaker_gyro_z_param_create(ESP_RMAKER_DEF_GYRO_Z_NAME, gyro_z);
        esp_rmaker_device_add_param(device, primary);
        esp_rmaker_device_assign_primary_param(device, primary);
    }
    return device;
}

mpu6050_handle_t app_sensor_create(mpu6050_acce_fs_t ACCE_FS, mpu6050_gyro_fs_t GYRO_FS)
{
    mpu6050_handle_t sensor = mpu6050_create(I2C_MASTER_NUM, MPU6050_SENSOR_ADDR);
    mpu6050_acce_fs_t acce_fs = ACCE_FS;
    mpu6050_gyro_fs_t gyro_fs = GYRO_FS;
    ESP_ERROR_CHECK(mpu6050_config(sensor, acce_fs, gyro_fs));
    ESP_ERROR_CHECK(mpu6050_wake_up(sensor));

    return sensor;
}

void app_sensor_delete(mpu6050_handle_t sensor)
{
    ESP_ERROR_CHECK(mpu6050_sleep(sensor));
    mpu6050_delete(sensor);
}

static void app_sensor_update(mpu6050_handle_t sensor)
{
    // static float delta = 0.5;
    // g_temperature += delta;
    // if (g_temperature > 99) {
    //     delta = -0.5;
    // } else if (g_temperature < 1) {
    //     delta = 0.5;
    // }
    // g_hue = (100 - g_temperature) * 2;
    // ws2812_led_set_hsv(g_hue, g_saturation, g_value);
    // esp_rmaker_param_update_and_report(
    //         esp_rmaker_device_get_param_by_type(mpu6050_device, ESP_RMAKER_PARAM_TEMPERATURE),
    //         esp_rmaker_float(g_temperature));

    // mpu6050_handle_t sensor = app_sensor_create(ACCE_FS_8G, GYRO_FS_1000DPS);
    
    float values[7] = {app_get_current_temperature(sensor),
    app_get_current_acceleration_x(sensor),
    app_get_current_acceleration_y(sensor),
    app_get_current_acceleration_z(sensor),
    app_get_current_gyro_x(sensor),
    app_get_current_gyro_y(sensor),
    app_get_current_gyro_z(sensor)};

    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(mpu6050_device_temp, ESP_RMAKER_PARAM_TEMPERATURE),
            esp_rmaker_float(values[0]));
    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(mpu6050_device_acce_x, ESP_RMAKER_PARAM_ACCELERATION_X),
            esp_rmaker_float(values[1]));
    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(mpu6050_device_acce_y, ESP_RMAKER_PARAM_ACCELERATION_Y),
            esp_rmaker_float(values[2]));
    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(mpu6050_device_acce_z, ESP_RMAKER_PARAM_ACCELERATION_Z),
            esp_rmaker_float(values[3]));
    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(mpu6050_device_gyro_x, ESP_RMAKER_PARAM_GYRO_X),
            esp_rmaker_float(values[4]));
    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(mpu6050_device_gyro_y, ESP_RMAKER_PARAM_GYRO_Y),
            esp_rmaker_float(values[5]));
    esp_rmaker_param_update_and_report(
            esp_rmaker_device_get_param_by_type(mpu6050_device_gyro_z, ESP_RMAKER_PARAM_GYRO_Z),
            esp_rmaker_float(values[6]));
}

void app_main()
{
    /* Initialize Application specific hardware drivers and
     * set initial state.
     */
    app_driver_init();    

    /* Initialize NVS. */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    /* Initialize Wi-Fi. Note that, this should be called before esp_rmaker_node_init()
     */
    app_wifi_init();
    
    /* Initialize the ESP RainMaker Agent.
     * Note that this should be called after app_wifi_init() but before app_wifi_start()
     * */
    esp_rmaker_config_t rainmaker_cfg = {
        .enable_time_sync = false,
    };
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "ESP RainMaker Device", "MPU6050");
    if (!node) {
        ESP_LOGE(TAG, "Could not initialise node. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    mpu6050_handle_t sensor = app_sensor_create(ACCE_FS_8G, GYRO_FS_1000DPS);

    /* Create a device and add the relevant parameters to it */
    // mpu6050_device = esp_rmaker_temp_sensor_device_create("Temperature Sensor", NULL, app_get_current_temperature());
    // esp_rmaker_node_add_device(node, mpu6050_device);

    mpu6050_device_temp = esp_rmaker_temp_sensor_device_create("Temperature", NULL, app_get_current_temperature(sensor));
    esp_rmaker_node_add_device(node, mpu6050_device_temp);
    mpu6050_device_acce_x = esp_rmaker_accelerometer_x_device_create("Acceleration x", NULL, app_get_current_acceleration_x(sensor));
    esp_rmaker_node_add_device(node, mpu6050_device_acce_x);
    mpu6050_device_acce_y = esp_rmaker_accelerometer_y_device_create("Acceleration y", NULL, app_get_current_acceleration_y(sensor));
    esp_rmaker_node_add_device(node, mpu6050_device_acce_y);
    mpu6050_device_acce_z = esp_rmaker_accelerometer_z_device_create("Acceleration z", NULL, app_get_current_acceleration_z(sensor));
    esp_rmaker_node_add_device(node, mpu6050_device_acce_z);
    mpu6050_device_gyro_x = esp_rmaker_gyroscope_x_device_create("Gyro x", NULL, app_get_current_gyro_x(sensor));
    esp_rmaker_node_add_device(node, mpu6050_device_gyro_x);
    mpu6050_device_gyro_y = esp_rmaker_gyroscope_y_device_create("Gyro y", NULL, app_get_current_gyro_y(sensor));
    esp_rmaker_node_add_device(node, mpu6050_device_gyro_y);
    mpu6050_device_gyro_z = esp_rmaker_gyroscope_z_device_create("Gyro z", NULL, app_get_current_gyro_z(sensor));
    esp_rmaker_node_add_device(node, mpu6050_device_gyro_z);

    /* Enable OTA */
    esp_rmaker_ota_enable_default();

    /* Enable Insights. Requires CONFIG_ESP_INSIGHTS_ENABLED=y */
    app_insights_enable();

    /* Start the ESP RainMaker Agent */
    esp_rmaker_start();

    /* Start the Wi-Fi.
     * If the node is provisioned, it will start connection attempts,
     * else, it will start Wi-Fi provisioning. The function will return
     * after a connection has been successfully established
     */
    err = app_wifi_start(POP_TYPE_RANDOM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not start Wifi. Aborting!!!");
        vTaskDelay(5000/portTICK_PERIOD_MS);
        abort();
    }

    while(1)
    {
        app_sensor_update(sensor);
        
        vTaskDelay(REPORTING_PERIOD * 1000/portTICK_PERIOD_MS);
    }    

    app_sensor_delete(sensor);
}
