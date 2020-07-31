/* 
 * MYTASK.C - The C file of the MYTASK.h
 * NOTE: 
 *
 * Copyright (c) 2020-, FOSH Project
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author            Notes             mail
 * 2020-07-04     ElonJoker         rebuild version   2649853081@qq.com
 * 2020-07-10     ElonJoker         修改救援方案      
 * 2020-07-26     ElonJoker         测试重构后的CAN   
 * 2020-07-27     ElonJoker         加入PID           
 * 2020-07-29     ElonJoker         强制调用回调函数，暂时解决CAN接收中断一次的毛病           
 */
#include "myTask.h"

int testCode = 1;//为1时关掉遥控器判断，进行测试
/**
    * @brief  底盘移动任务功能代码
    * @note   执行移动任务
    * @author 占建
    * @param  None
    * @retval None
    */
		
void moveTaskFunction(void const * argument)
{	
  			for(int i=0; i<4; i++)
			{
				PID_struct_init(&pid_spd[i],POSITION_PID,16000,16000,25,0.05,0);
			}
	/* Infinite loop */	
		  for(;;)
		{
	 HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);	//强制调动回调函数接收信息，貌似反应不够快！！！	
	// 对四个轮子的PID 进行调节
			for(int i=0; i<4; i++)
			{
				pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, set_spd[i]);
			}
			setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
									         pid_spd[1].pos_out,
									         pid_spd[2].pos_out,
									         pid_spd[3].pos_out,IDMARK_ONE_FOUR);			
//按键设置电机的基础马力(速度)配置
	switch(key_sta)
	{
		case 0:	//no key
			if( 0 == HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) )
			{
				key_sta = 1;
			}
			break;
		case 1: //key down wait release.
			if( 0 == HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) )
			{
				key_sta = 2;
				key_cnt++;
			}
			else
			{
				key_sta = 0;
			}
			break;
		case 2: 
			if( 0 != HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_10) )
			{
				key_sta = 0;
			}
			break;
	}
	if(key_cnt>10)
		key_cnt = 0;
		 //获取遥控器的前后，左右控制器的输入数据
		vx = (float)rc.ch2 / 660 * 800;
		vy = -(float)rc.ch1 / 660 *800;
		wz = rc.wheel*5.2;
	//未启动移动模式则不移动
   if(mode=='4'||mode=='5')	
	 {
		 set_spd[0]=set_spd[1]=set_spd[2]=set_spd[3]=0;
	 }
	 else
	 {
	//麦克纳姆轮速度分解并设置电机速度
		set_spd[0] = -(-vy + vx+ wz)*(key_cnt+1);    
		set_spd[1] = -(-vy - vx+ wz)*(key_cnt+1);
	  set_spd[2] = -( vy + vx + wz)*(key_cnt+1);
	  set_spd[3] = -( vy- vx+ wz)*(key_cnt+1);	
	 }
	   osDelay(1);//加入延时，使程序能够执行，过长或过短都会导致电机PID不稳定
    }	
}


/**
    * @brief  实现救援方案
    * @note   代码中PWM参数值需要实际测试设定，确保抓稳，伸出，救援卡的电机控制待定
    * @author 占建
    * @param  None
    * @retval 
    */
void rescueTaskeFunction(void const * argument)
{
		for(;;)
	{
		 if(mode=='7'||mode=='8'||mode=='9'||testCode==1)
		  {

			      HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
		       	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,800);//自动重载值为1000，800的待调试！！！！！
				
		     if(mode=='9'||testCode==1)
				 {
			      osDelay(500);//等待半秒，确保抓住车辆
            HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);//强行调用回调函数
					 
            pid_calc(&pid_spd[4], moto_chassis[4].speed_rpm, 400);//固定的速度值，待调试！！！！！
					  setMotoSpeed(&hcan1, pid_spd[4].pos_out, 0,0,0,IDMARK_FIVE_EIGHT);//正转，伸出救援卡
					  osDelay(100);//确保感应卡伸出，与速度值联调，待调试！！！！！
					 
					 pid_calc(&pid_spd[4], moto_chassis[4].speed_rpm, 0);//停住等待感应
					 osDelay(1000);//确保感应生效，与感应时间有关，待输入！！！！！
					 
					 pid_calc(&pid_spd[4], moto_chassis[4].speed_rpm, -400);//固定的速度值，待调试！！！！！
					 setMotoSpeed(&hcan1, pid_spd[4].pos_out, 0,0,0,IDMARK_FIVE_EIGHT);//反转，收回救援卡
				 }

			 }
    else
		{
			 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,200);//收回爪子
		  setMotoSpeed(&hcan1, 0, 0,0,0,IDMARK_FIVE_EIGHT);//救援卡不动
		}
		 osDelay(1);//加入延时，使程序能够执行，过长或过短都会导致电机PID不稳定
	}
}

/**
    * @brief  云台和底盘通信任务实现函数
    * @note   执行通信任务
    * @author 占建
    * @param  None
    * @retval None
    */
void msgSendTaskFunction(void const * argument) 
{
	for(;;)
	{
		//判断此时的遥控器的模式
		if(rc_device_get_state(&rc,RC_S2_UP)&&rc_device_get_state(&rc,RC_S1_UP))//普通模式 右1 左1
			mode=NORMAL_MODE;
		
		if(rc_device_get_state(&rc,RC_S2_UP)&&rc_device_get_state(&rc,RC_S1_MID))//普通模式下，补弹 右1 左2
	    mode=GIVEBULLET;
		
		if(rc_device_get_state(&rc,RC_S2_UP)&&rc_device_get_state(&rc,RC_S1_DOWN))//普通模式下，登岛 右1 左3
			mode=CLIMB_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_UP)&&rc_device_get_state(&rc,RC_S2_MID))//取弹模式下，一排 右2  左1
			mode=GETBULLET_1_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_MID)&&rc_device_get_state(&rc,RC_S2_MID))//取弹模式下，十字 右2  左2
		   mode=GETBULLET_10_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_DOWN)&&rc_device_get_state(&rc,RC_S2_MID))//取弹模式下,四角 右2  左3
		   mode=GETBULLET_4_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_UP)&&rc_device_get_state(&rc,RC_S2_DOWN))//进入救援模式 右3 左1
			mode=RESCUE_MODE;
		
		if(rc_device_get_state(&rc,RC_S2_DOWN)&&rc_device_get_state(&rc,RC_S1_MID))//退出救援模式，拨弹 右3 左2
		   mode=OPEN_MODE;
    
		if(rc_device_get_state(&rc,RC_S2_DOWN)&&rc_device_get_state(&rc,RC_S1_DOWN))//救援卡模式，拨弹 右3 左3
		   mode=RESCUE_CAR_MODE;		
		
		//发送此时的模式和云台数据
		masge[0]='a';
		masge[6]='b';
		masge[5]=mode;
		HAL_UART_Transmit(&huart6,masge,sizeof(masge),50);
		osDelay(10);
	}
	
}







