/*
 * cvtypes.hpp
 *
 *  Created on: May 1, 2016
 *      Author: Brandon Zhen
 */

#ifndef L5_APPLICATION_CVTYPES_HPP_
#define L5_APPLICATION_CVTYPES_HPP_

#include "tasks.hpp"

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

struct PWM_t
{
        float p2_0;     // The PWM signal in degrees for P2.0 Ex: (Clockwise = -90 | Counter-clockwise = +90) from the top of the motor
        float p2_1;     // The PWM signal in degrees for P2.1 Ex: (Clockwise = -90 | Counter-clockwise = +90) from the top of the motor
        float p2_2;     // The PWM signal in degrees for P2.2 Ex: (Clockwise = -90 | Counter-clockwise = +90) from the top of the motor
        float p2_3;     // The PWM signal in degrees for P2.3 Ex: (Clockwise = -90 | Counter-clockwise = +90) from the top of the motor
        float p2_4;     // The PWM signal in degrees for P2.4 Ex: (Clockwise = -90 | Counter-clockwise = +90) from the top of the motor
        float p2_5;     // The PWM signal in degrees for P2.5 Ex: (Clockwise = -90 | Counter-clockwise = +90) from the top of the motor
};

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
    roboLampHandler_xQueueSendTo_visionTask,
    visionTask_xQueueReceiveFrom_roboLampHandler,
    visionTask_xQueueSendTo_CV_Core,
    CV_Core_xQueueReceiveFrom_visionTask,
    CV_Core_xQueueSendTo_motorTask,
    motorTask_xQueueReceiveFrom_CV_Core,
    motorTask_setDegreeExceedLimit_p2_0,
    motorTask_setDegreeExceedLimit_p2_1,
    motorTask_setDegreeExceedLimit_p2_2,
    motorTask_setDegreeExceedLimit_p2_3,
    motorTask_setDegreeExceedLimit_p2_4,
    motorTask_setDegreeExceedLimit_p2_5,
    errorTask_xQueueReceiveFrom_generic,
};

/**
 * reportError reports an ERR_id to errorTask without ever blocking.
 * Example: reportError(errorTask_xQueueReceiveFrom_generic);
 */
void reportError(ERR_id error);

#endif /* L5_APPLICATION_CVTYPES_HPP_ */
