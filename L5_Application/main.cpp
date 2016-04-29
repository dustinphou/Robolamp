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

#include "iostream"
#include "lpc_pwm.hpp"

class motorTask : public scheduler_task
{
        PWM servo2[6];  ///< P2.0-P2.5
        float min2[6];  ///< Min PWM Percentage
        float max2[6];  ///< Max PWM Percentage
        float deg2[6];  ///< Max Degree of Servo
        typedef enum {
            p2_0=0,
            p2_1=1,
            p2_2=2,
            p2_3=3,
            p2_4=4,
            p2_5=5
        } pwmNum;
    public:
        motorTask(uint8_t priority) : scheduler_task("motor", 4096, priority),
        servo2{PWM(PWM::pwm1, 50),  ///< P2.0
               PWM(PWM::pwm2, 50),  ///< P2.1
               PWM(PWM::pwm3, 50),  ///< P2.2
               PWM(PWM::pwm4, 50),  ///< P2.3
               PWM(PWM::pwm5, 50),  ///< P2.4
               PWM(PWM::pwm6, 50)}, ///< P2.5
        min2{5.0,   ///< P2.0
             5.0,   ///< P2.1
             5.0,   ///< P2.2
             5.0,   ///< P2.3
             5.0,   ///< P2.4
             5.0},  ///< P2.5
        max2{10.,   ///< P2.0
             10.,   ///< P2.1
             10.,   ///< P2.2
             10.,   ///< P2.3
             10.,   ///< P2.4
             10.},  ///< P2.5
        deg2{180,   ///< P2.0
             180,   ///< P2.1
             180,   ///< P2.2
             180,   ///< P2.3
             180,   ///< P2.4
             180}   ///< P2.5
        {
            /* Nothing to init */
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
            if (degree < -deg2[pinNum]/2 || degree > deg2[pinNum]/2)
                return false;
            return servo2[pinNum].set(((max2[pinNum]+min2[pinNum])/2)+((degree/(deg2[pinNum]/2))*((max2[pinNum]-min2[pinNum])/2)));
        }

        bool run(void *p)
        {
            set(p2_0, -90);
            vTaskDelay(1000);
            set(p2_0,   0);
            vTaskDelay(1000);
            set(p2_0,  90);
            vTaskDelay(1000);
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
    scheduler_add_task(new motorTask(PRIORITY_LOW));

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
