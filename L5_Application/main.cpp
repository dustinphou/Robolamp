/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 * 			FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 * 			@see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 */

/* RTOS Libraries */
#include "tasks.hpp"
#include "examples/examples.hpp"

/* Standard Template Libraries */

/* LPC Libraries */
#include "io.hpp"
#include "lpc_pwm.hpp"
#include <fstream>

/* Custom Libraries */
#include "cvtypes.hpp"  ///< Contains all structures and enumerations used by CV_Core

/**
 * CV_Core is the brain of all logic.
 */
class CV_Core : public scheduler_task
{
        QueueHandle_t CV_QueueHandle;           ///< Contains data of type CV_t from roboLampHandler to visionTask
        QueueHandle_t FRAME_QueueHandle;        ///< Contains incoming data of type FRAME_t in percentages from visionTask
        QueueHandle_t PWM_QueueHandle;          ///< Contains outgoing data of type PWM_t in degrees from CV_Core and LEDTask to PWMTask
        QueueHandle_t ERR_QueueHandle;          ///< Contains data of type ERR_id from * to errorTask
        TickType_t FRAME_ReceiveTimeout;        ///< Max xTicksToWait for xQueueReceive
        TickType_t PWM_SendTimeout;             ///< Max xTicksToWait for xQueueSend

        float PWM_UpdateFrequencyInHz;          ///< Frequency at which to Update PWM_*DegreeToSend in Hertz
        float PWM_UpdateStepPercentage;         ///< Percentage of degrees to add per Update [Value is between 0 and 100]

        float PWM_BaseDegreeTarget;             ///< Target PWM_t->value to Update PWM_BaseDegreeToSend to (Base Servo)
        float PWM_BaseDegreeLeftLimit;          ///< Max leftward degree of Base Servo
        float PWM_BaseDegreeRightLimit;         ///< Max rightward degree of Base Servo
        PWM_t PWM_BaseDegreeToSend;             ///< PWM_t to xQueueSend to Pin 2.0 per Update (Base Servo)

        float PWM_HeadDegreeTarget;             ///< Target PWM_t->value to Update PWM_HeadDegreeToSend to (Head Servo)
        float PWM_HeadDegreeUpperLimit;         ///< Max upward degree of Head Servo
        float PWM_HeadDegreeLowerLimit;         ///< Max downward degree of Head Servo
        PWM_t PWM_HeadDegreeToSend;             ///< PWM_t to xQueueSend to Pin 2.1 per Update (Head Servo)

        float CAM_ViewAngleHorizontal;          ///< Horizontal view angle of the camera
        float CAM_ViewAngleVertical;            ///< Vertical view angle of the camera
        float CAM_DegreeBeforeStartFollowing;   ///< Degree of displacement from origin before starting to follow the target
        float CAM_DegreeBeforeStopFollowing;    ///< Degree of displacement from origin before stopping to follow the target

    public:
        CV_Core(uint8_t priority) : scheduler_task("core", 2048, priority),
            CV_QueueHandle(xQueueCreate(1, sizeof(CV_t))),          ///< CV_QueueHandle
            FRAME_QueueHandle(xQueueCreate(4, sizeof(FRAME_t))),    ///< FRAME_QueueHandle
            PWM_QueueHandle(xQueueCreate(1, sizeof(PWM_t))),        ///< PWM_QueueHandle
            ERR_QueueHandle(xQueueCreate(1, sizeof(ERR_id))),       ///< ERR_QueueHandle
            FRAME_ReceiveTimeout(0 * portTICK_PERIOD_MS),           ///< 0ms
            PWM_SendTimeout(0 * portTICK_PERIOD_MS),                ///< 0ms

            PWM_UpdateFrequencyInHz(50),                            ///< 50 Hz to match servo PWM frequency
            PWM_UpdateStepPercentage(5),                            ///< 30% of degrees added per Update

            PWM_BaseDegreeTarget(0),                                ///< Target PWM_t->value to Update PWM_BaseDegreeToSend to (Base Servo)
            PWM_BaseDegreeLeftLimit(90),                            ///< Max leftward degree of Base Servo
            PWM_BaseDegreeRightLimit(90),                           ///< Max rightward degree of Base Servo
            PWM_BaseDegreeToSend{p2_0, pwmDegree, 0},               ///< Pin 2.0 (Base Servo), Value type is in degrees, Initial value

            PWM_HeadDegreeTarget(0),                                ///< Target PWM_t->value to Update PWM_HeadDegreeToSend to (Head Servo)
            PWM_HeadDegreeUpperLimit(0),                            ///< Max upward degree of Head Servo
            PWM_HeadDegreeLowerLimit(90),                           ///< Max downward degree of Head Servo
            PWM_HeadDegreeToSend{p2_1, pwmDegree, 0},               ///< Pin 2.1 (Head Servo), Value type is in degrees, Initial value

            CAM_ViewAngleHorizontal(48),                            ///< 48 degrees
            CAM_ViewAngleVertical(36),                              ///< 36 degrees
            CAM_DegreeBeforeStartFollowing(5),                      ///< 5 degrees
            CAM_DegreeBeforeStopFollowing(5)                        ///< 5 degrees
        {
            addSharedObject(CV_QueueHandle_id, CV_QueueHandle);         ///< Shares CV_QueueHandle
            addSharedObject(FRAME_QueueHandle_id, FRAME_QueueHandle);   ///< Shares FRAME_QueueHandle
            addSharedObject(PWM_QueueHandle_id, PWM_QueueHandle);       ///< Shares PWM_QueueHandle
            addSharedObject(ERR_QueueHandle_id, ERR_QueueHandle);       ///< Shares ERR_QueueHandle
        }

        bool taskEntry(void)
        {
            xQueueSend(PWM_QueueHandle, &PWM_BaseDegreeToSend, PWM_SendTimeout);  ///< Send initial PWM_t
            xQueueSend(PWM_QueueHandle, &PWM_HeadDegreeToSend, PWM_SendTimeout);  ///< Send initial PWM_t

            return true;
        }

        bool run(void *p)
        {
            FRAME_t frame;
            if (pdTRUE == xQueueReceive(FRAME_QueueHandle, &frame, FRAME_ReceiveTimeout)) {
                float conversionRatio_DegreeX = ((CAM_ViewAngleHorizontal/100)/2);
                float conversionRatio_DegreeY = ((CAM_ViewAngleVertical/100)/2);

                float FRAME_DegreeX = conversionRatio_DegreeX * frame.coordx;                       // Range: -24 degrees to +24 degrees
                float FRAME_DegreeY = conversionRatio_DegreeY * frame.coordy;                       // Range: -18 degrees to +18 degrees

                float FRAME_OffsetX = ((FRAME_DegreeX > 0) ? (FRAME_DegreeX) : (-FRAME_DegreeX));   // Absolute distance from origin
                float FRAME_OffsetY = ((FRAME_DegreeY > 0) ? (FRAME_DegreeY) : (-FRAME_DegreeY));   // Absolute distance from origin

                if (FRAME_OffsetX > CAM_DegreeBeforeStartFollowing) {
                    PWM_BaseDegreeTarget = PWM_BaseDegreeToSend.value - FRAME_DegreeX;              // Inverted Rotation Logic
                    if (PWM_BaseDegreeTarget < (-PWM_BaseDegreeRightLimit)) // Servo right is negative
                        PWM_BaseDegreeTarget = (-PWM_BaseDegreeRightLimit); // Servo right is negative
                    if (PWM_BaseDegreeTarget > (+PWM_BaseDegreeLeftLimit )) // Servo left is positive
                        PWM_BaseDegreeTarget = (+PWM_BaseDegreeLeftLimit ); // Servo left is positive
                }
                if (FRAME_OffsetY > CAM_DegreeBeforeStartFollowing) {
                    PWM_HeadDegreeTarget = PWM_HeadDegreeToSend.value - FRAME_DegreeY;              // Inverted Rotation Logic
                    if (PWM_HeadDegreeTarget < (-PWM_HeadDegreeUpperLimit)) // Servo up is negative
                        PWM_HeadDegreeTarget = (-PWM_HeadDegreeUpperLimit); // Servo up is negative
                    if (PWM_HeadDegreeTarget > (+PWM_HeadDegreeLowerLimit)) // Servo down is positive
                        PWM_HeadDegreeTarget = (+PWM_HeadDegreeLowerLimit); // Servo down is positive
                }
            }

            float PWM_BaseStep = PWM_BaseDegreeTarget - PWM_BaseDegreeToSend.value;                 // Vector distance to target
            float PWM_HeadStep = PWM_HeadDegreeTarget - PWM_HeadDegreeToSend.value;                 // Vector distance to target

            float PWM_BaseOffset = ((PWM_BaseStep > 0) ? (PWM_BaseStep) : (-PWM_BaseStep));         // Absolute distance to target
            float PWM_HeadOffset = ((PWM_HeadStep > 0) ? (PWM_HeadStep) : (-PWM_HeadStep));         // Absolute distance to target

            if (PWM_BaseOffset > CAM_DegreeBeforeStopFollowing) {
                PWM_BaseDegreeToSend.value += (PWM_UpdateStepPercentage / 100) * (PWM_BaseStep);
                if (errQUEUE_FULL == xQueueSend(PWM_QueueHandle, &PWM_BaseDegreeToSend, PWM_SendTimeout))
                    reportError(CV_Core_xQueueSend_To_motorTask);
            }
            if (PWM_HeadOffset > CAM_DegreeBeforeStopFollowing) {
                PWM_HeadDegreeToSend.value += (PWM_UpdateStepPercentage / 100) * (PWM_HeadStep);
                if (errQUEUE_FULL == xQueueSend(PWM_QueueHandle, &PWM_HeadDegreeToSend, PWM_SendTimeout))
                    reportError(CV_Core_xQueueSend_To_motorTask);
            }
            vTaskDelay((1 / PWM_UpdateFrequencyInHz) * 1000 * portTICK_PERIOD_MS);
            return true;
        }
};

/**
 * LEDTask controls all LED signals.
 */
class LEDTask : public scheduler_task
{
        QueueHandle_t PWM_QueueHandle;      ///< Contains outgoing data of type PWM_t
        TickType_t PWM_SendTimeout;         ///< Max xTicksToWait for xQueueReceive

        PWM_t PWM_LEDPercentage;            ///< PWM_t to xQueueSend

        float LED_UpdateFrequencyInHz;      ///< Frequency at which to Update PWM_LEDPercentage in Hertz
        float LED_UpdateStepInPercentage;   ///< Percentage of light to add per Update [Value is between 0 and 100]
        float LED_MinOutputPercentage;      ///< Minimum Brightness of LED [Value is between 0 and 100]
        float LED_MaxOutputPercentage;      ///< Maximum Brightness of LED [Value is between 0 and 100]
        float ENV_MinInputPercentage;       ///< Minimum Brightness of Environment [Value is between 0 and 100]
        float ENV_MaxInputPercentage;       ///< Maximum Brightness of Environment [Value is between 0 and 100]

    public:
        LEDTask(uint8_t priority) : scheduler_task("LED", 2048, priority),
            PWM_QueueHandle(getSharedObject(PWM_QueueHandle_id)),   ///< PWM_QueueHandle
            PWM_SendTimeout(0 * portTICK_PERIOD_MS),                ///< 0ms

            PWM_LEDPercentage{p2_5, pwmPercent, 10},                ///< Pin 2.5 (Super LED), Value type is in percentages, Initial value

            LED_UpdateFrequencyInHz(10),    ///< Update Frequency = 10 Hz
            LED_UpdateStepInPercentage(10), ///< Update Step = 10 %
            LED_MinOutputPercentage(50),    ///< Minimum LED Brightness
            LED_MaxOutputPercentage(100),   ///< Maximum LED Brightness
            ENV_MinInputPercentage(0),      ///< Minimum Environment Brightness
            ENV_MaxInputPercentage(25)      ///< Maximum Environment Brightness
        {
            /* Nothing to init */
        }

        bool run(void *p)
        {
            float conversionRatio = ((LED_MaxOutputPercentage-LED_MinOutputPercentage)/(ENV_MaxInputPercentage-ENV_MinInputPercentage));
            float nextBrightness = ((conversionRatio)*(LS.getPercentValue())) + (LED_MinOutputPercentage);
            PWM_LEDPercentage.value += ((LED_UpdateStepInPercentage / 100) * (nextBrightness - PWM_LEDPercentage.value));
            if (PWM_LEDPercentage.value < LED_MinOutputPercentage)
                PWM_LEDPercentage.value = LED_MinOutputPercentage;
            if (PWM_LEDPercentage.value > LED_MaxOutputPercentage)
                PWM_LEDPercentage.value = LED_MaxOutputPercentage;
            if (errQUEUE_FULL == xQueueSend(PWM_QueueHandle, &PWM_LEDPercentage, PWM_SendTimeout))
                reportError(LEDTask_xQueueSend_To_motorTask);
            vTaskDelay((1 / LED_UpdateFrequencyInHz) * 1000 * portTICK_PERIOD_MS);
            return true;
        }
};

/**
 * visionTask controls all CV signals.
 */
class visionTask : public scheduler_task
{
        QueueHandle_t CV_QueueHandle;       ///< Contains incoming data of type CV_t from roboLampHandler
        QueueHandle_t FRAME_QueueHandle;    ///< Contains outgoing data of type FRAME_t to CV_Core
        TickType_t CV_ReceiveTimeout;       ///< Max xTicksToWait for xQueueReceive
        TickType_t FRAME_SendTimeout;       ///< Max xTicksToWait for xQueueSend

    public:
        visionTask(uint8_t priority) : scheduler_task("vision", 2048, priority),
            CV_QueueHandle(getSharedObject(CV_QueueHandle_id)),         ///< CV_QueueHandle
            FRAME_QueueHandle(getSharedObject(FRAME_QueueHandle_id)),   ///< FRAME_QueueHandle
            CV_ReceiveTimeout(1 * 1000 * portTICK_PERIOD_MS),           ///< 1 * 1000ms
            FRAME_SendTimeout(0 * portTICK_PERIOD_MS)                   ///< 0ms
        {
            /* Nothing to init */
        }

        bool run(void *p)
        {
            CV_t raw;
            FRAME_t frame;
            if (pdTRUE == xQueueReceive(CV_QueueHandle, &raw, CV_ReceiveTimeout)) {
                frame.coordx = ((((float)raw.coordx/(float)raw.framex)*(200))-(100));
                frame.coordy = ((((float)raw.coordy/(float)raw.framey)*(200))-(100));

                if (errQUEUE_FULL == xQueueSend(FRAME_QueueHandle, &frame, FRAME_SendTimeout))
                    reportError(visionTask_xQueueSend_To_CV_Core);
            }
            else /* errQUEUE_Empty */ {
                reportError(visionTask_xQueueReceive_From_roboLampHandler);
            }
            return true;
        }
};

/**
 * PWMTask controls all PWM signals in degrees. (See PWM_t)
 */
class PWMTask : public scheduler_task
{
        QueueHandle_t PWM_QueueHandle;  ///< Contains incoming data of type PWM_t in degrees
        TickType_t PWM_ReceiveTimeout;  ///< Max xTicksToWait for xQueueReceive

        PWM PWM_Base;   ///< P2.0 Base Servo
        float min2_0;   ///< Min PWM Percentage
        float max2_0;   ///< Max PWM Percentage
        float rot2_0;   ///< Max Degree of Servo

        PWM PWM_Head;   ///< P2.1 Head Servo
        float min2_1;   ///< Min PWM Percentage
        float max2_1;   ///< Max PWM Percentage
        float rot2_1;   ///< Max Degree of Servo

        PWM PWM_LED;    ///< P2.5 Super LED
        float min2_5;   ///< Min PWM Percentage
        float max2_5;   ///< Max PWM Percentage

    public:
        PWMTask(uint8_t priority) : scheduler_task("PWM", 2048, priority),
            PWM_QueueHandle(getSharedObject(PWM_QueueHandle_id)),   ///< PWM_QueueHandle
            PWM_ReceiveTimeout(1 * 1000 * portTICK_PERIOD_MS),      ///< 1 * 1000ms

            PWM_Base(PWM(PWM::pwm1, 50)),   ///< P2.0 Base Servo
            min2_0(2.5),                    ///< Min PWM Percentage
            max2_0(12.5),                   ///< Max PWM Percentage
            rot2_0(180),                    ///< Max Degree of Servo

            PWM_Head(PWM(PWM::pwm2, 50)),   ///< P2.1 Head Servo
            min2_1(2.5),                    ///< Min PWM Percentage
            max2_1(12.5),                   ///< Max PWM Percentage
            rot2_1(180),                    ///< Max Degree of Servo

            PWM_LED(PWM(PWM::pwm6, 1024)),  ///< P2.5 Super LED
            min2_5(0),                      ///< Min PWM Percentage
            max2_5(100)                     ///< Max PWM Percentage
        {
            /* Nothing to init */
        }

        /**
         * Sets the PWM based on the degree.
         * If servo=50Hz, min=5.0, max=10.0, and rot=180, then you can use the following :
         *      - Left    : degreeToPercent(-90, 5.0, 10.0, 180);   // -90 degrees = 5.0 % of 20ms = 1.0ms
         *      - Neutral : degreeToPercent(  0, 5.0, 10.0, 180);   //   0 degrees = 7.5 % of 20ms = 1.5ms
         *      - Right   : degreeToPercent( 90, 5.0, 10.0, 180);   //  90 degrees = 10  % of 20ms = 2.0ms
         *
         * You can use micro-steps to position the servo motor by using
         * degreeToPercent(0.1, 5.0, 10.0, 180), degreeToPercent(0.2, 5.0, 10.0, 180) ... degreeToPercent(34.5, 5.0, 10.0, 180) etc.
         */
        bool setDegree(PWM &pwm, float degree, float min, float max, float rot)
        {
            if (degree < -rot/2) {
                pwm.set(min);
                return false;
            }
            else if (degree > rot/2) {
                pwm.set(max);
                return false;
            }
            else {
                return pwm.set( ((degree/(rot/2))*((max-min)/2)) + ((max+min)/2) );
            }
        }

        /**
         * Sets the PWM based on the percentage.
         * If servo=50Hz, min=5.0, and max=10.0, then you can use the following :
         *      - Left    : degreeToPercent(  0, 5.0, 10.0);    //   0 percent = 5.0 % of 20ms = 1.0ms
         *      - Neutral : degreeToPercent( 50, 5.0, 10.0);    //  50 percent = 7.5 % of 20ms = 1.5ms
         *      - Right   : degreeToPercent(100, 5.0, 10.0);    // 100 percent = 10  % of 20ms = 2.0ms
         *
         * You can use micro-steps to position the servo motor by using
         * degreeToPercent(0.1, 5.0, 10.0), degreeToPercent(0.2, 5.0, 10.0) ... degreeToPercent(34.5, 5.0, 10.0) etc.
         */
        bool setPercent(PWM &pwm, float degree, float min, float max)
        {
            if (degree < 0) {
                pwm.set(min);
                return false;
            }
            else if (degree > 100) {
                pwm.set(max);
                return false;
            }
            else {
                return pwm.set( ((degree/100)*(max-min)) + (min) );
            }
        }

        bool run(void *p)
        {
            PWM_t signal;
            if (pdTRUE == xQueueReceive(PWM_QueueHandle, &signal, PWM_ReceiveTimeout)) {
                switch (signal.type) {
                    case pwmDegree:
                        switch (signal.pin) {
                            case p2_0:
                                if (false == setDegree(PWM_Base, signal.value, min2_0, max2_0, rot2_0))
                                    reportError(PWMTask_setDegree_ExceedLimit_p2_0);
                                break;
                            case p2_1:
                                if (false == setDegree(PWM_Head, signal.value, min2_1, max2_1, rot2_1))
                                    reportError(PWMTask_setDegree_ExceedLimit_p2_1);
                                    break;
                            default:
                                reportError(PWMTask_setDegree_UndefinedPin);
                                break;
                        }
                        break;
                    case pwmPercent:
                        switch (signal.pin) {
                            case p2_0:
                                if (false == setPercent(PWM_Base, signal.value, min2_0, max2_0))
                                    reportError(PWMTask_setPercent_ExceedLimit_p2_0);
                                break;
                            case p2_1:
                                if (false == setPercent(PWM_Head, signal.value, min2_1, max2_1))
                                    reportError(PWMTask_setPercent_ExceedLimit_p2_1);
                                break;
                            case p2_5:
                                if (false == setPercent(PWM_LED, signal.value, min2_5, max2_5))
                                    reportError(PWMTask_setPercent_ExceedLimit_p2_5);
                                break;
                            default:
                                reportError(PWMTask_setPercent_UndefinedPin);
                                break;
                        }
                        break;
                    default:
                        reportError(PWMTask_UndefinedType);
                        break;
                }
            }
            else /* errQUEUE_EMPTY */ {
                reportError(PWMTask_xQueueReceiveFrom_CV_Core);
            }
            return true;
        }
};

/**
 * errorTask controls all error signals.
 */
class errorTask : public scheduler_task
{
        QueueHandle_t ERR_QueueHandle;  ///< Contains incoming data of type ERR_id
        TickType_t ERR_ReadTimeout;     ///< Max xTicksToWait for xQueueReceive
        TickType_t ERR_DisplayDuration; ///< Minimum xTicksToDelay LED Display update

    public:
        errorTask(uint8_t priority) : scheduler_task("error", 2048, priority),
            ERR_QueueHandle(getSharedObject(ERR_QueueHandle_id)),   ///< ERR_QueueHandle
            ERR_ReadTimeout(portMAX_DELAY),                         ///< (2^32)-1 ticks
            ERR_DisplayDuration(1 * 1000 * portTICK_PERIOD_MS)      ///< 1 * 1000ms
        {
            LD.clear();     ///< Begin with blank LED Display
        }

        /**
         * Sets the LED Display to a two character value.
         * Example: LED_Display("HI");
         */
        void LED_Display(const char *value)
        {
            LD.setLeftDigit(value[0]);
            LD.setRightDigit(value[1]);
        }

        bool run(void *p)
        {
            ERR_id error;
            if (pdTRUE == xQueueReceive(ERR_QueueHandle, &error, ERR_ReadTimeout)) {
                switch (error) {
                    case roboLampHandler_cmdParams_scanf:
                        LED_Display("HI");
                        break;
                    case roboLampHandler_xQueueSend_To_visionTask:
                        LED_Display("HO");
                        break;
                    case visionTask_xQueueReceive_From_roboLampHandler:
                        LED_Display("VI");
                        break;
                    case visionTask_xQueueSend_To_CV_Core:
                        LED_Display("VO");
                        break;
                    case CV_Core_xQueueReceive_From_visionTask:
                        LED_Display("CI");
                        break;
                    case CV_Core_xQueueSend_To_motorTask:
                        LED_Display("CO");
                        break;
                    case LEDTask_xQueueSend_To_motorTask:
                        LED_Display("LO");
                        break;
                    case PWMTask_xQueueReceiveFrom_CV_Core:
                        LED_Display("MI");
                        break;
                    case PWMTask_setDegree_ExceedLimit_p2_0:
                    case PWMTask_setPercent_ExceedLimit_p2_0:
                        LED_Display("P0");
                        break;
                    case PWMTask_setDegree_ExceedLimit_p2_1:
                    case PWMTask_setPercent_ExceedLimit_p2_1:
                        LED_Display("P1");
                        break;
                    case PWMTask_setDegree_ExceedLimit_p2_2:
                    case PWMTask_setPercent_ExceedLimit_p2_2:
                        LED_Display("P2");
                        break;
                    case PWMTask_setDegree_ExceedLimit_p2_3:
                    case PWMTask_setPercent_ExceedLimit_p2_3:
                        LED_Display("P3");
                        break;
                    case PWMTask_setDegree_ExceedLimit_p2_4:
                    case PWMTask_setPercent_ExceedLimit_p2_4:
                        LED_Display("P4");
                        break;
                    case PWMTask_setDegree_ExceedLimit_p2_5:
                    case PWMTask_setPercent_ExceedLimit_p2_5:
                        LED_Display("P5");
                        break;
                    case PWMTask_setDegree_UndefinedPin:
                    case PWMTask_setPercent_UndefinedPin:
                    case PWMTask_UndefinedType:
                        LED_Display("PU");
                        break;
                    case errorTask_xQueueReceive_From_generic:
                        LED_Display("EI");
                        break;
                    default:
                        LED_Display("??");
                        break;
                }
                vTaskDelay(ERR_DisplayDuration);
                xQueueReset(ERR_QueueHandle);   ///< Always returns pdPASS here
                LD.clear();
            }
            else {
                reportError(errorTask_xQueueReceive_From_generic);
            }
            return true;
        }
};

/**
 * The main() creates tasks or "threads".  See the documentation of scheduler_task class at scheduler_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */
int main(void)
{
    scheduler_add_task(new CV_Core(PRIORITY_LOW));
    scheduler_add_task(new LEDTask(PRIORITY_LOW));
    scheduler_add_task(new visionTask(PRIORITY_MEDIUM));
    scheduler_add_task(new PWMTask(PRIORITY_MEDIUM));
    scheduler_add_task(new errorTask(PRIORITY_MEDIUM));

    /**
     * A few basic tasks for this bare-bone system :
     *      1.  Terminal task provides gateway to interact with the board through UART terminal.
     *      2.  Remote task allows you to use remote control to interact with the board.
     *      3.  Wireless task responsible to receive, retry, and handle mesh network.
     *
     * Disable remote task if you are not using it.  Also, it needs SYS_CFG_ENABLE_TLM
     * such that it can save remote control codes to non-volatile memory.  IR remote
     * control codes can be learned by typing the "learn" terminal command.
     */
    scheduler_add_task(new terminalTask(PRIORITY_HIGH));

    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
    scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));

    /* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
    #if 0
    scheduler_add_task(new periodicSchedulerTask());
    #endif

    /* The task for the IR receiver */
    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));

    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
     * task to always be responsive so you can poke around in case something goes wrong.
     */

    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
    #if 0
        scheduler_add_task(new example_io_demo());
    #endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
    #if 0
        scheduler_add_task(new example_task());
        scheduler_add_task(new example_alarm());
        scheduler_add_task(new example_logger_qset());
        scheduler_add_task(new example_nv_vars());
    #endif

    /**
	 * Try the rx / tx tasks together to see how they queue data to each other.
	 */
    #if 0
        scheduler_add_task(new queue_tx());
        scheduler_add_task(new queue_rx());
    #endif

    /**
     * Another example of shared handles and producer/consumer using a queue.
     * In this example, producer will produce as fast as the consumer can consume.
     */
    #if 0
        scheduler_add_task(new producer());
        scheduler_add_task(new consumer());
    #endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This does two things for us:
     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
     *
     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
     * @code
     *     // Assuming Wifly is on Uart3
     *     addCommandChannel(Uart3::getInstance(), false);
     * @endcode
     */
    #if 0
        Uart3 &u3 = Uart3::getInstance();
        u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
    #endif

    scheduler_start(); ///< This shouldn't return
    return -1;
}
