/* 
 * FileName - The C head file of the myTask.c driver
 * NOTE: This file is based on HAL library of stm32 platform
 *       
 *
 * Copyright (c) 2020-, FOSH Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes           mail
 * 2020-07-26     ElonJoker         version1.1      2649853081@qq.com
 */
 
 #ifndef  _MYTASK_H
#define  _MYTASK_H

#include "pid.h"
#include "bsp_can.h"
#include "mytype.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "tim.h"
#include "RC.h"

#define NORMAL_MODE    '1'
#define GIVEBULLET     '2'
#define CLIMB_MODE      '3'
#define GETBULLET_1_MODE     '4'
#define GETBULLET_10_MODE    '5'
#define GETBULLET_4_MODE     '6'
#define RESCUE_MODE     '7'
#define OPEN_MODE       '8'
#define RESCUE_CAR_MODE '9'

static int  set_spd[5];//设置速度
static uint8_t mode='1';//传输所选择的模式
static int key_sta,key_cnt;//按键的状态和按下次数
static float vx,vy,wz;  //遥控器发出的速度

void moveTaskFunction(void const * argument);
void rescueTaskeFunction(void const * argument);
void msgSendTaskFunction(void const * argument); 

#endif
