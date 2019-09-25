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
#define I2C_MASTER_NUM  CONFIG_I2C_NUM
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_FREQ_HZ 400000

#define LED_RED_IO CONFIG_LED_RED_IO  //make menuconfig 
#define LED_GREEN_IO CONFIG_LED_GREEN_IO
static const char *TAG = "vl53l1";

volatile float height_current; //current height （cm）
volatile int is_detected;      //detected == 1  undetected == 0  error<0  : undetected == -1 base_height changed
TaskHandle_t hauto_detect_task;
volatile bool is_init = false;

static void auto_detect_task(void * pvParameters) ;

static float get_base_height(VL53L1_Dev_t *dev)
{
    float heightCurrent = 0;
    int countForAvg = 5;
    float heightForAvg = 0;
    VL53L1_RangingMeasurementData_t RangingData;
    for (int i = 0; i < countForAvg;)
    {

        int status = VL53L1_WaitMeasurementDataReady(dev); //5 HZ
        if (!status)
        {
            status = VL53L1_GetRangingMeasurementData(dev, &RangingData);
            if (status == 0)
            {
                heightCurrent = RangingData.RangeMilliMeter / 10.0;
                heightForAvg += heightCurrent;
                i++;
                ESP_LOGI(TAG, "height %3.1f \n", heightCurrent);
            }

            status = VL53L1_ClearInterruptAndStartMeasurement(dev); //clear Interrupt start next measurement
        }
    }
    ESP_LOGI(TAG, "height base = %3.1f \n", heightForAvg / countForAvg);
    return heightForAvg / countForAvg;
}

/** 
* @brief init  vl53l1 module ,creat a object detect task
* @param base_height - base height should be acquired when no objects in the measurement area 
* when this param is 0, base height is automatically obtained
* @param min_object_height - the minimum height to trigger a report
*
*/
void vl53l1_init(OBJECT_DETECT *object)
{
    if(is_init){
        return;
    }

    uint8_t byteData;
    uint16_t wordData;
    VL53L1_RdByte(object->pvl53l1_dev, 0x010F, &byteData);
    ESP_LOGI(TAG, "VL53L1X Model_ID: %02X\n\r", byteData);
    VL53L1_RdByte(object->pvl53l1_dev, 0x0110, &byteData);
    ESP_LOGI(TAG, "VL53L1X Module_Type: %02X\n\r", byteData);
    VL53L1_RdWord(object->pvl53l1_dev, 0x010F, &wordData);
    ESP_LOGI(TAG, "VL53L1X: %02X\n\r", wordData);

    VL53L1_UserRoi_t Roi0;
    Roi0.TopLeftX = 0; //set ROI according to requirement
    Roi0.TopLeftY = 15;
    Roi0.BotRightX = 15;
    Roi0.BotRightY = 0;
    int status = VL53L1_WaitDeviceBooted(object->pvl53l1_dev);
    status = VL53L1_DataInit(object->pvl53l1_dev);                                       //performs the device initialization
    status = VL53L1_StaticInit(object->pvl53l1_dev);                                     // load device settings specific for a given use case.
    status = VL53L1_SetDistanceMode(object->pvl53l1_dev, VL53L1_DISTANCEMODE_LONG);      //Max distance in dark:Short:136cm Medium:290cm long:360cm
    status = VL53L1_SetMeasurementTimingBudgetMicroSeconds(object->pvl53l1_dev, 140000); //140 ms is the timing budget which allows the maximum distance of 4 m (in the dark on a white chart)
    status = VL53L1_SetInterMeasurementPeriodMilliSeconds(object->pvl53l1_dev, 200);     //period of time between two consecutive measurements 100ms
    status = VL53L1_SetUserROI(object->pvl53l1_dev, &Roi0); //SET region of interest
    status = VL53L1_StartMeasurement(object->pvl53l1_dev);
    
    if (status)
    {
        ESP_LOGE(TAG, "VL53L1_StartMeasurement failed \n");
        return ;

    }else
    {
        ESP_LOGD(TAG, "VL53L1 StartMeasurement  \n");
    }

    if (object->base_height == 0)
    {
        object->base_height= get_base_height(object->pvl53l1_dev);  //base height is automatically obtained
    }

    xTaskCreate(auto_detect_task, "auto_detect_task", 2048, object, 5, hauto_detect_task);  
    is_init = true ;
    
}


/** 
* @brief deinit  vl53l1 module ,delete auto_detect_task task
*
*/
void vl53l1_deinit(OBJECT_DETECT *object)
{
    if (!is_init)
    {
        return;
    }
    vTaskDelete(hauto_detect_task);
    is_init = false;
}

/**
* @brief get height value
* 
*/
float vl53l1_get_height(void){
   
    if (!is_init)
    {
        return 0;
    }

    return height_current;
}


/**
* @brief get height value
* @return == 1：detected    return== 0：undetected   return == -2：base_height changed  return == -1：vl53l1 not init 
*/
int vl53l1_get_detect_result(void){
    
    if (!is_init)
    {
        return -1;
    }

    return is_detected;
}




/* Autonomous ranging loop*/
/**
 * @brief auto_detect_task 
 * set the is_detected flag when object is detected
 * set the height_current when measurement update
 */
static void auto_detect_task(void * pvParameters) 
{
    OBJECT_DETECT *object = (OBJECT_DETECT *)pvParameters;
    ESP_LOGI(TAG, "Autonomous Detect\n");

    VL53L1_RangingMeasurementData_t RangingData;
    float heightCurrent = 0;
    static int detectedCount = 0;
    // static bool ledRedState = 0;
    // static bool ledGreenState = 0;

    do
    {                                                      // polling mode
        int status = VL53L1_WaitMeasurementDataReady(object->pvl53l1_dev); //5 HZ
        if (!status)
        {
            status = VL53L1_GetRangingMeasurementData(object->pvl53l1_dev, &RangingData);
            if (status == 0)
            {
#if 0
                ESP_LOGI(TAG, "%d,%d,%.2f,%.2f\n", RangingData.RangeStatus,
                         RangingData.RangeMilliMeter,
                         (RangingData.SignalRateRtnMegaCps / 65536.0),
                         RangingData.AmbientRateRtnMegaCps / 65336.0);
#else
                heightCurrent = RangingData.RangeMilliMeter / 10.0;
                height_current = heightCurrent;

                if (object->base_height - heightCurrent > object->min_object_height)
                {
                    /* code */
                    if (detectedCount <  object->min_detect_times) //1s
                    {
                        detectedCount++;
                    }
                    else
                    {
                        is_detected = 1;

                        //trigger open action

                        // if (ledRedState == 0)
                        // {
                        //     ESP_LOGD(TAG, "detected ! do something here  \n");
                        //     gpio_set_level(LED_RED_IO, 1);
                        //     ledRedState = 1;
                        // }
                        // if (ledGreenState == 1)
                        // {
                        //     gpio_set_level(LED_GREEN_IO, 0);
                        //     ledGreenState = 0;
                        // }
                    }
                }
                else 
                {
                    if (detectedCount > 0) //
                    {
                        detectedCount--;
                    }
                    else
                    {
                        is_detected = 0;

                        //trigger close action

                        // if (ledGreenState == 0)
                        // {
                        //     ESP_LOGD(TAG, "disappeared !  do something here \n");
                        //     gpio_set_level(LED_GREEN_IO, 1);
                        //     ledGreenState = 1;
                        // }
                        // if (ledRedState == 1)
                        // {
                        //     gpio_set_level(LED_RED_IO, 0);
                        //     ledRedState = 0;
                        // }
                    }
                }

                if (heightCurrent - object->base_height> (object->min_object_height / 2.0)){

                    ESP_LOGW(TAG, "NEED MODIFY BASE HEIGHT \n");
                    is_detected = -2;  
                }

                //ESP_LOGD(TAG, "height %3.1f detectedCount = %d object is detected = %d \n", heightCurrent, detectedCount , is_detected);
#endif
            }

            status = VL53L1_ClearInterruptAndStartMeasurement(object->pvl53l1_dev); //clear Interrupt start next measurement
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    } while (1);


}


