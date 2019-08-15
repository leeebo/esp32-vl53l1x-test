/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "soc/gpio_struct.h"

#define DIST_QSIZE 128
#define I2C_MASTER_SCL_IO               CONFIG_SCL_IO
#define I2C_MASTER_SDA_IO               CONFIG_SDA_IO
#define I2C_MASTER_NUM                  CONFIG_I2C_NUM
#define I2C_MASTER_TX_BUF_DISABLE       0
#define I2C_MASTER_RX_BUF_DISABLE       0
#define I2C_MASTER_FREQ_HZ              400000





static void i2c_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE,
                       0);
}

extern void rn_task(void *arg);

void app_main(void)
{
   
    i2c_init();
    
   // xTaskCreate(udp_task, "udp_task", 2048, NULL, 4, NULL);
    xTaskCreate(rn_task, "rn_task", 2048, NULL, 7, NULL);

    gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT); //led
    int level = 0;
    while (true) {
        gpio_set_level(GPIO_NUM_25, level);
        level = !level;
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}

