/* vl53l1 Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "vl53l1_esp32.h"

#define I2C_MASTER_SCL_IO CONFIG_SCL_IO //make menuconfig 
#define I2C_MASTER_SDA_IO CONFIG_SDA_IO
#define I2C_MASTER_NUM 0 //CONFIG_I2C_NUM
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_FREQ_HZ 400000

#define LED_RED_IO CONFIG_LED_RED_IO  //make menuconfig 
#define LED_GREEN_IO CONFIG_LED_GREEN_IO
static const char *TAG = "app_main";

VL53L1_Dev_t vl53l1_dev;
OBJECT_DETECT object_person;

static void i2c_init(void)
{
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE,
                       0);
}



void app_main(void)
{
    
    //I2C  
    i2c_init();

    // led
    gpio_pad_select_gpio(LED_RED_IO);
    gpio_pad_select_gpio(LED_GREEN_IO);
    gpio_set_direction(LED_RED_IO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GREEN_IO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_RED_IO, 0);
    gpio_set_level(LED_GREEN_IO, 0);
    // xshut high
    gpio_pad_select_gpio(CONFIG_XSHUT_IO);
    gpio_set_direction(CONFIG_XSHUT_IO, GPIO_MODE_OUTPUT); //
    gpio_set_level(CONFIG_XSHUT_IO, 1);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    vl53l1_dev.I2cHandle = I2C_MASTER_NUM; 
    vl53l1_dev.I2cDevAddr = 0x52;
    object_person.pvl53l1_dev = &vl53l1_dev;
    object_person.base_height = 0;   //when this param is 0, base height is automatically obtained
    object_person.min_object_height = 15.0;


    vl53l1_init(&object_person);

    while (1)
    {
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
    

}
