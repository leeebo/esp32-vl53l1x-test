#ifndef _VL53L0_ESP32_H_
#define _VL53L0_ESP32_H_


#include "vl53l1_api.h"

typedef struct {

    VL53L1_Dev_t *pvl53l1_dev;  //define VL53L1_Dev_t first and set this pointer 
    float base_height;         //if set 0, base height is automatically obtained
    float min_object_height;    //the minimum height to trigger a report
    int min_detect_times;   // time (ms)= min_detect_times * 200

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
* @brief deinit  vl53l1 module ,delete auto_detect_task task
*
*/
void vl53l1_deinit(OBJECT_DETECT *object);

/**
* @brief get height value
* 
*/
float vl53l1_get_height(void);


/**
* @brief get height value
* @return == 1：detected    return== 0：undetected   return == -2：base_height changed  return == -1：vl53l1 not init 
*/
int vl53l1_get_detect_result(void);



#endif