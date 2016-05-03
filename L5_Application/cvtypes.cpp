/*
 * cvtypes.cpp
 *
 *  Created on: May 3, 2016
 *      Author: Brandon Zhen
 */

#include "cvtypes.hpp"

#include "tasks.hpp"

void reportError(ERR_id error)
{
    TickType_t ERR_SendTimeout = 0 * portTICK_PERIOD_MS;    // 0ms
    xQueueSend(scheduler_task::getSharedObject(ERR_QueueHandle_id), &error, ERR_SendTimeout);
}
