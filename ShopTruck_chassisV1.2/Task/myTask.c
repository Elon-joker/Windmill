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
 * 2020-07-10     ElonJoker         �޸ľ�Ԯ����      
 * 2020-07-26     ElonJoker         �����ع����CAN   
 * 2020-07-27     ElonJoker         ����PID           
 * 2020-07-29     ElonJoker         ǿ�Ƶ��ûص���������ʱ���CAN�����ж�һ�ε�ë��           
 */
#include "myTask.h"

int testCode = 1;//Ϊ1ʱ�ص�ң�����жϣ����в���
/**
    * @brief  �����ƶ������ܴ���
    * @note   ִ���ƶ�����
    * @author ռ��
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
	 HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);	//ǿ�Ƶ����ص�����������Ϣ��ò�Ʒ�Ӧ�����죡����	
	// ���ĸ����ӵ�PID ���е���
			for(int i=0; i<4; i++)
			{
				pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, set_spd[i]);
			}
			setMotoSpeed(&hcan1, pid_spd[0].pos_out, 
									         pid_spd[1].pos_out,
									         pid_spd[2].pos_out,
									         pid_spd[3].pos_out,IDMARK_ONE_FOUR);			
//�������õ���Ļ�������(�ٶ�)����
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
		 //��ȡң������ǰ�����ҿ���������������
		vx = (float)rc.ch2 / 660 * 800;
		vy = -(float)rc.ch1 / 660 *800;
		wz = rc.wheel*5.2;
	//δ�����ƶ�ģʽ���ƶ�
   if(mode=='4'||mode=='5')	
	 {
		 set_spd[0]=set_spd[1]=set_spd[2]=set_spd[3]=0;
	 }
	 else
	 {
	//�����ķ���ٶȷֽⲢ���õ���ٶ�
		set_spd[0] = -(-vy + vx+ wz)*(key_cnt+1);    
		set_spd[1] = -(-vy - vx+ wz)*(key_cnt+1);
	  set_spd[2] = -( vy + vx + wz)*(key_cnt+1);
	  set_spd[3] = -( vy- vx+ wz)*(key_cnt+1);	
	 }
	   osDelay(1);//������ʱ��ʹ�����ܹ�ִ�У���������̶��ᵼ�µ��PID���ȶ�
    }	
}


/**
    * @brief  ʵ�־�Ԯ����
    * @note   ������PWM����ֵ��Ҫʵ�ʲ����趨��ȷ��ץ�ȣ��������Ԯ���ĵ�����ƴ���
    * @author ռ��
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
		       	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,800);//�Զ�����ֵΪ1000��800�Ĵ����ԣ���������
				
		     if(mode=='9'||testCode==1)
				 {
			      osDelay(500);//�ȴ����룬ȷ��ץס����
            HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);//ǿ�е��ûص�����
					 
            pid_calc(&pid_spd[4], moto_chassis[4].speed_rpm, 400);//�̶����ٶ�ֵ�������ԣ���������
					  setMotoSpeed(&hcan1, pid_spd[4].pos_out, 0,0,0,IDMARK_FIVE_EIGHT);//��ת�������Ԯ��
					  osDelay(100);//ȷ����Ӧ����������ٶ�ֵ�����������ԣ���������
					 
					 pid_calc(&pid_spd[4], moto_chassis[4].speed_rpm, 0);//ͣס�ȴ���Ӧ
					 osDelay(1000);//ȷ����Ӧ��Ч�����Ӧʱ���йأ������룡��������
					 
					 pid_calc(&pid_spd[4], moto_chassis[4].speed_rpm, -400);//�̶����ٶ�ֵ�������ԣ���������
					 setMotoSpeed(&hcan1, pid_spd[4].pos_out, 0,0,0,IDMARK_FIVE_EIGHT);//��ת���ջؾ�Ԯ��
				 }

			 }
    else
		{
			 HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
			__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,200);//�ջ�צ��
		  setMotoSpeed(&hcan1, 0, 0,0,0,IDMARK_FIVE_EIGHT);//��Ԯ������
		}
		 osDelay(1);//������ʱ��ʹ�����ܹ�ִ�У���������̶��ᵼ�µ��PID���ȶ�
	}
}

/**
    * @brief  ��̨�͵���ͨ������ʵ�ֺ���
    * @note   ִ��ͨ������
    * @author ռ��
    * @param  None
    * @retval None
    */
void msgSendTaskFunction(void const * argument) 
{
	for(;;)
	{
		//�жϴ�ʱ��ң������ģʽ
		if(rc_device_get_state(&rc,RC_S2_UP)&&rc_device_get_state(&rc,RC_S1_UP))//��ͨģʽ ��1 ��1
			mode=NORMAL_MODE;
		
		if(rc_device_get_state(&rc,RC_S2_UP)&&rc_device_get_state(&rc,RC_S1_MID))//��ͨģʽ�£����� ��1 ��2
	    mode=GIVEBULLET;
		
		if(rc_device_get_state(&rc,RC_S2_UP)&&rc_device_get_state(&rc,RC_S1_DOWN))//��ͨģʽ�£��ǵ� ��1 ��3
			mode=CLIMB_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_UP)&&rc_device_get_state(&rc,RC_S2_MID))//ȡ��ģʽ�£�һ�� ��2  ��1
			mode=GETBULLET_1_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_MID)&&rc_device_get_state(&rc,RC_S2_MID))//ȡ��ģʽ�£�ʮ�� ��2  ��2
		   mode=GETBULLET_10_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_DOWN)&&rc_device_get_state(&rc,RC_S2_MID))//ȡ��ģʽ��,�Ľ� ��2  ��3
		   mode=GETBULLET_4_MODE;
		
		if(rc_device_get_state(&rc,RC_S1_UP)&&rc_device_get_state(&rc,RC_S2_DOWN))//�����Ԯģʽ ��3 ��1
			mode=RESCUE_MODE;
		
		if(rc_device_get_state(&rc,RC_S2_DOWN)&&rc_device_get_state(&rc,RC_S1_MID))//�˳���Ԯģʽ������ ��3 ��2
		   mode=OPEN_MODE;
    
		if(rc_device_get_state(&rc,RC_S2_DOWN)&&rc_device_get_state(&rc,RC_S1_DOWN))//��Ԯ��ģʽ������ ��3 ��3
		   mode=RESCUE_CAR_MODE;		
		
		//���ʹ�ʱ��ģʽ����̨����
		masge[0]='a';
		masge[6]='b';
		masge[5]=mode;
		HAL_UART_Transmit(&huart6,masge,sizeof(masge),50);
		osDelay(10);
	}
	
}







