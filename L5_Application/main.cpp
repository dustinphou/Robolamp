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
#include "tasks.hpp"
#include "examples/examples.hpp"

#include "io.hpp"
#include "lpc_pwm.hpp"

#include "cvtypes.hpp"  ///< Contains all structures and enumerations used by CV_Core

/**
 * CV_Core is the brain of all logic.
 */
class CV_Core : public scheduler_task
{
        QueueHandle_t CV_QueueHandle;       ///< Contains data of type CV_t from roboLampHandler to visionTask
        QueueHandle_t FRAME_QueueHandle;    ///< Contains incoming data of type FRAME_t in percentages from visionTask
        QueueHandle_t PWM_QueueHandle;      ///< Contains outgoing data of type PWM_t in degrees to motorTask
        QueueHandle_t ERR_QueueHandle;      ///< Contains data of type ERR_id from * to errorTask
        TickType_t FRAME_ReceiveTimeout;    ///< Max xTicksToWait for xQueueReceive
        TickType_t PWM_SendTimeout;         ///< Max xTicksToWait for xQueueSend
        PWM_t PWM_Degree;                   ///< Previous PWM_t to motorTask

    public:
        CV_Core(uint8_t priority) : scheduler_task("core", 2048, priority),
            CV_QueueHandle(xQueueCreate(1, sizeof(CV_t))),          ///< CV_QueueHandle
            FRAME_QueueHandle(xQueueCreate(4, sizeof(FRAME_t))),    ///< FRAME_QueueHandle
            PWM_QueueHandle(xQueueCreate(1, sizeof(PWM_t))),        ///< PWM_QueueHandle
            ERR_QueueHandle(xQueueCreate(1, sizeof(ERR_id))),       ///< ERR_QueueHandle
            FRAME_ReceiveTimeout(1 * 1000 * portTICK_PERIOD_MS),    ///< 1 * 1000ms
            PWM_SendTimeout(0 * portTICK_PERIOD_MS),                ///< 0ms
            PWM_Degree{0,   ///< Initial degrees for p2_0
                       0,   ///< Initial degrees for p2_1
                       0,   ///< Initial degrees for p2_2
                       0,   ///< Initial degrees for p2_3
                       0,   ///< Initial degrees for p2_4
                       0}   ///< Initial degrees for p2_5
        {
            addSharedObject(CV_QueueHandle_id, CV_QueueHandle);         ///< Shares CV_QueueHandle
            addSharedObject(FRAME_QueueHandle_id, FRAME_QueueHandle);   ///< Shares FRAME_QueueHandle
            addSharedObject(PWM_QueueHandle_id, PWM_QueueHandle);       ///< Shares PWM_QueueHandle
            addSharedObject(ERR_QueueHandle_id, ERR_QueueHandle);       ///< Shares ERR_QueueHandle
            xQueueSend(PWM_QueueHandle, &PWM_Degree, PWM_SendTimeout);  ///< Send initial PWM_t
        }

        bool run(void *p)
        {
            FRAME_t frame;
            if (pdTRUE == xQueueReceive(FRAME_QueueHandle, &frame, FRAME_ReceiveTimeout)) {

                // Todo: Logic & Limits <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
                PWM_Degree.p2_0 -= frame.coordx / 100;  ///< Base Servo (Inverted Directionality)
                PWM_Degree.p2_1 += frame.coordy / 100;  ///< Camera Servo (Todo: Undefined Directionality)
                if (PWM_Degree.p2_0 > 90)
                    PWM_Degree.p2_0 = 90;
                if (PWM_Degree.p2_0 <-90)
                    PWM_Degree.p2_0 =-90;
                if (PWM_Degree.p2_1 > 90)
                    PWM_Degree.p2_1 = 90;
                if (PWM_Degree.p2_1 <-90)
                    PWM_Degree.p2_1 =-90;

                if (errQUEUE_FULL == xQueueSend(PWM_QueueHandle, &PWM_Degree, PWM_SendTimeout))
                    reportError(CV_Core_xQueueSendTo_motorTask);
            }
            else /* errQUEUE_Empty */ {
                reportError(CV_Core_xQueueReceiveFrom_visionTask);
            }
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
                frame.coordy = ((((float)raw.coordx/(float)raw.framex)*(200))-(100));
                if (errQUEUE_FULL == xQueueSend(FRAME_QueueHandle, &frame, FRAME_SendTimeout))
                    reportError(visionTask_xQueueSendTo_CV_Core);
            }
            else /* errQUEUE_Empty */ {
                reportError(visionTask_xQueueReceiveFrom_roboLampHandler);
            }
            return true;
        }
};

/**
 * motorTask controls all PWM signals.
 */
class motorTask : public scheduler_task
{
        PWM servo2[6];  ///< P2.0-P2.5
        float min2[6];  ///< Min PWM Percentage
        float max2[6];  ///< Max PWM Percentage
        float deg2[6];  ///< Max Degree of Servo
        enum pwmNum {
            p2_0 = 0,   ///< Dustin
            p2_1 = 1,   ///< James
            p2_2 = 2,   ///< Brandon
            p2_3 = 3,
            p2_4 = 4,
            p2_5 = 5,
        };
        QueueHandle_t PWM_QueueHandle;  ///< Contains incoming data of type PWM_t in degrees
        TickType_t PWM_ReceiveTimeout;  ///< Max xTicksToWait for xQueueReceive

    public:
        motorTask(uint8_t priority) : scheduler_task("motor", 2048, priority),
            servo2{PWM(PWM::pwm1, 50),  ///< P2.0 Dustin
                   PWM(PWM::pwm2, 50),  ///< P2.1 James
                   PWM(PWM::pwm3, 50),  ///< P2.2 Brandon
                   PWM(PWM::pwm4, 50),  ///< P2.3
                   PWM(PWM::pwm5, 50),  ///< P2.4
                   PWM(PWM::pwm6, 50)}, ///< P2.5
            min2{2.5,   ///< P2.0 Dustin
                 2.5,   ///< P2.1 James
                 2.5,   ///< P2.2 Brandon
                 5.0,   ///< P2.3
                 5.0,   ///< P2.4
                 5.0},  ///< P2.5
            max2{12.5,  ///< P2.0 Dustin
                 12.5,  ///< P2.1 James
                 12.5,  ///< P2.2 Brandon
                 10.0,  ///< P2.3
                 10.0,  ///< P2.4
                 10.0}, ///< P2.5
            deg2{180,   ///< P2.0 Dustin
                 180,   ///< P2.1 James
                 180,   ///< P2.2 Brandon
                 180,   ///< P2.3
                 180,   ///< P2.4
                 180},  ///< P2.5
            PWM_QueueHandle(getSharedObject(PWM_QueueHandle_id)),   ///< PWM_QueueHandle
            PWM_ReceiveTimeout(1 * 1000 * portTICK_PERIOD_MS)       ///< 1 * 1000ms
        {
            PWM_t init_degree;                                                  ///< Initial PWM state
            xQueueReceive(PWM_QueueHandle, &init_degree, PWM_ReceiveTimeout);   ///< Get initial PWM state
            set(p2_0, init_degree.p2_0);    ///< Set initial PWM state
            set(p2_1, init_degree.p2_1);    ///< Set initial PWM state
            set(p2_2, init_degree.p2_2);    ///< Set initial PWM state
            set(p2_3, init_degree.p2_3);    ///< Set initial PWM state
            set(p2_4, init_degree.p2_4);    ///< Set initial PWM state
            set(p2_5, init_degree.p2_5);    ///< Set initial PWM state
        }

        /**
         * Sets the PWM based on the degree.
         * If servo2[0]=50Hz, min2[0]=5.0, max2[0]=10.0, and deg2[0]=180, then you can use the following :
         *      - Left    : set(p2_0, -90);     // -90 degrees = 5.0 % of 20ms = 1.0ms
         *      - Neutral : set(p2_0,   0);     //   0 degrees = 7.5 % of 20ms = 1.5ms
         *      - Right   : set(p2_0,  90);     //  90 degrees = 10  % of 20ms = 2.0ms
         *
         * You can use micro-steps to position the servo motor by using
         * set(p2_0, 0.1), set(p2_0, 0.2) ... set(p2_0, 34.5) etc.
         */
        bool set(pwmNum pinNum, float degree)
        {
            if (degree < -deg2[pinNum]/2) {
                servo2[pinNum].set(min2[pinNum]);
                return false;
            }
            else if (degree > deg2[pinNum]/2) {
                servo2[pinNum].set(max2[pinNum]);
                return false;
            }
            else {
                return servo2[pinNum].set(((degree/(deg2[pinNum]/2))*((max2[pinNum]-min2[pinNum])/2))+((max2[pinNum]+min2[pinNum])/2));
            }
        }

        bool run(void *p)
        {
            PWM_t degree;
            if (pdTRUE == xQueueReceive(PWM_QueueHandle, &degree, PWM_ReceiveTimeout)) {
                if (false == set(p2_0, degree.p2_0))
                    reportError(motorTask_setDegreeExceedLimit_p2_0);
                if (false == set(p2_1, degree.p2_1))
                    reportError(motorTask_setDegreeExceedLimit_p2_1);
                if (false == set(p2_2, degree.p2_2))
                    reportError(motorTask_setDegreeExceedLimit_p2_2);
                if (false == set(p2_3, degree.p2_3))
                    reportError(motorTask_setDegreeExceedLimit_p2_3);
                if (false == set(p2_4, degree.p2_4))
                    reportError(motorTask_setDegreeExceedLimit_p2_4);
                if (false == set(p2_5, degree.p2_5))
                    reportError(motorTask_setDegreeExceedLimit_p2_5);
            }
            else /* errQUEUE_EMPTY */ {
                reportError(motorTask_xQueueReceiveFrom_CV_Core);
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
            ERR_DisplayDuration(10 * 1000 * portTICK_PERIOD_MS)     ///< 10 * 1000ms
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
                    case roboLampHandler_xQueueSendTo_visionTask:
                        LED_Display("HO");
                        break;
                    case visionTask_xQueueReceiveFrom_roboLampHandler:
                        LED_Display("VI");
                        break;
                    case visionTask_xQueueSendTo_CV_Core:
                        LED_Display("VO");
                        break;
                    case CV_Core_xQueueReceiveFrom_visionTask:
                        LED_Display("CI");
                        break;
                    case CV_Core_xQueueSendTo_motorTask:
                        LED_Display("CO");
                        break;
                    case motorTask_xQueueReceiveFrom_CV_Core:
                        LED_Display("MI");
                        break;
                    case motorTask_setDegreeExceedLimit_p2_0:
                        LED_Display("P0");
                        break;
                    case motorTask_setDegreeExceedLimit_p2_1:
                        LED_Display("P1");
                        break;
                    case motorTask_setDegreeExceedLimit_p2_2:
                        LED_Display("P2");
                        break;
                    case motorTask_setDegreeExceedLimit_p2_3:
                        LED_Display("P3");
                        break;
                    case motorTask_setDegreeExceedLimit_p2_4:
                        LED_Display("P4");
                        break;
                    case motorTask_setDegreeExceedLimit_p2_5:
                        LED_Display("P5");
                        break;
                    case errorTask_xQueueReceiveFrom_generic:
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
                reportError(errorTask_xQueueReceiveFrom_generic);
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
    scheduler_add_task(new visionTask(PRIORITY_MEDIUM));
    scheduler_add_task(new motorTask(PRIORITY_MEDIUM));
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
