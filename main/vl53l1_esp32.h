#ifndef _VL53L0_ESP32_H_
#define _VL53L0_ESP32_H_


#include "vl53l1_api.h"

typedef struct {

    VL53L1_Dev_t *pvl53l1_dev;  //define VL53L1_Dev_t first and set this pointer 
    float base_height;         //if set 0, base height is automatically obtained
    float min_object_height;    //the minimum height to trigger a report

    float height_current;  //current height （cm）
    int  is_detected;   //detected == 1  undetected == 0  error<0  : undetected == -1 base_height changed 

}OBJECT_DETECT;

/** 
* @brief init  vl53l1 module ,and launch a  auto_detect_task task
* @param object.base_height - base height should be acquired when no objects in the measurement area 
* when this param is 0, base height is automatically obtained
* @param object.min_object_height - the minimum height to trigger a report
*
*/
void vl53l1_init(OBJECT_DETECT *object);


/**
* @brief measurement height once manually
* dont call this function when auto_detect_task is running
*
*/
float get_height_once(VL53L1_Dev_t *dev);


/**
 * @brief auto_detect_task 
 * set the is_detected flag when object is detected
 * set the height_current when measurement update
 */

void auto_detect_task(void * pvParameters);




#endif