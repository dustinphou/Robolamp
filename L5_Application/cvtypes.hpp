/*
 * cvtypes.hpp
 *
 *  Created on: May 1, 2016
 *      Author: Brandon Zhen
 */

#ifndef L5_APPLICATION_CVTYPES_HPP_
#define L5_APPLICATION_CVTYPES_HPP_

#include "tasks.hpp"

enum sharedObject_id
{
    CV_QueueHandle_id,
    FRAME_QueueHandle_id,
    PWM_QueueHandle_id,
    ERR_QueueHandle_id,
};

enum ERR_id
{
    roboLampHandler_cmdParams_scanf,
    roboLampHandler_xQueueSend_To_visionTask,
    visionTask_xQueueReceive_From_roboLampHandler,
    visionTask_xQueueSend_To_CV_Core,
    CV_Core_xQueueReceive_From_visionTask,
    CV_Core_xQueueSend_To_motorTask,
    LEDTask_xQueueSend_To_motorTask,
    PWMTask_xQueueReceiveFrom_CV_Core,
    PWMTask_setDegree_ExceedLimit_p2_0,
    PWMTask_setDegree_ExceedLimit_p2_1,
    PWMTask_setDegree_ExceedLimit_p2_2,
    PWMTask_setDegree_ExceedLimit_p2_3,
    PWMTask_setDegree_ExceedLimit_p2_4,
    PWMTask_setDegree_ExceedLimit_p2_5,
    PWMTask_setPercent_ExceedLimit_p2_0,
    PWMTask_setPercent_ExceedLimit_p2_1,
    PWMTask_setPercent_ExceedLimit_p2_2,
    PWMTask_setPercent_ExceedLimit_p2_3,
    PWMTask_setPercent_ExceedLimit_p2_4,
    PWMTask_setPercent_ExceedLimit_p2_5,
    PWMTask_setDegree_UndefinedPin,
    PWMTask_setPercent_UndefinedPin,
    PWMTask_UndefinedType,
    errorTask_xQueueReceive_From_generic,
};

/**
 * reportError reports an ERR_id to errorTask without ever blocking.
 * Example: reportError(errorTask_xQueueReceiveFrom_generic);
 */
void reportError(ERR_id error);

struct CV_t
{
        uint16_t coordx;    // The horizontal coordinates of the point. Ex: 123
        uint16_t coordy;    // The vertical coordinates of the point. Ex: 456
        uint16_t framex;    // The horizontal width of the frame. Ex: 1920
        uint16_t framey;    // The vertical height of the frame. Ex: 1080
};

struct FRAME_t
{
        float coordx;   // The percentage horizontal coordinates of the point. Ex: Left = -100% | Right = +100%
        float coordy;   // The percentage vertical coordinates of the point. Ex: Up = +100% | Down = -100%
};

enum pwmPin {
    p2_0,       ///< Base Servo
    p2_1,       ///< Head Servo
    p2_2,       ///<
    p2_3,       ///<
    p2_4,       ///<
    p2_5,       ///< Super LED
};

enum pwmType {
    pwmDegree,  ///< The PWM signal is in degrees       Ex: (Clockwise = -90 | Counter-clockwise = +90) from the top of the motor
    pwmPercent, ///< The PWM signal is in percentages   Ex: (+0 = min% | +100 = max%)
};

struct PWM_t
{
        pwmPin pin;
        pwmType type;
        float value;    ///< The PWM signal in pwmType
};

#endif /* L5_APPLICATION_CVTYPES_HPP_ */
