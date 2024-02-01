#ifndef ROBOT_TASK_H
#define ROBOT_TASK_H

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "robot.h"
#include "ins_task.h"
#include "motor_task.h"
#include "referee_task.h"
#include "master_process.h"
#include "daemon.h"
#include "HT04.h"
#include "buzzer.h"

#include "bsp_log.h"

void OSTaskInit();

extern osThreadId insTaskHandle;
extern osThreadId robotTaskHandle;
extern osThreadId motorTaskHandle;
extern osThreadId daemonTaskHandle;
extern osThreadId uiTaskHandle;

#endif // !


