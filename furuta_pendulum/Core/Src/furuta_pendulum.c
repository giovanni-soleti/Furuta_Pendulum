/*
 * furuta_pendulum.c
 *
 *  Created on: Sep 24, 2020
 *  Author: Caramia Donato, D'Alessandro Vito Ivano, Soleti Giovanni, Venezia Antonio
 *  Group name: Team Thor
 *  Description: Furuta Pendulum source file. This file contains all the function prototypes
 *  used to implement the control system
 */
#include "furuta_pendulum.h"
int8_t sign(float variable)
{
	/*Function that determines the sign of a number*/
	if(variable>0)
		return 1;
	else
		return -1;
}
float swing_up()
{
	/*Function that implements the swing up non-linear controller*/
	float w0 = (float)sqrt((double)(mP*g*Lk)/Inertia);
	float E = ((mP*g*Lk)/2)*(((pow((double)(pendulum_speed/w0),2))) + cos(pendulum_angle) - 1);
	float torque_swing = Kv*(E-E0)*sign(pendulum_speed*cos(pendulum_angle));
	return torque_swing;
}

float catch()
{
	/*Function that implements the catch controller*/
	float proportional_error = -pendulum_angle;
	float derivative_error = -pendulum_speed;
	float P_term = Kp_catch * proportional_error;
	float D_term = Kd_catch * derivative_error;
	return (P_term + D_term);
}

float balancing_control()
{
	/*Motor PID on motor_speed*/
	float proportional_motor_error = setpoint_m - motor_speed;
	motor_speed_2 = motor_speed;
	float derivative_motor_error = -(motor_speed_2 - motor_speed_1)/time_elapsed;
	motor_speed_1 = motor_speed_2;
	float setpoint_PD_out = Kp_o * proportional_motor_error + Kd_o*derivative_motor_error;

	/*Pendulum PID*/
	float proportional_error = setpoint_PD_out - pendulum_angle;
	float derivative_error = -pendulum_speed;
	integral_error += (proportional_error*time_elapsed);
	float P_term = Kp * proportional_error;
	float I_term = Ki * integral_error;
	float D_term = Kd * derivative_error;

	return((P_term + I_term + D_term));
}

float calculate_control_action()
{
	/*Function that switch between controllers*/

	/* Swing-up control action in ]35°;-35°[ */
	if ((pendulum_angle < - 0.61) || (pendulum_angle > 0.61))
	{
		integral_error = 0;
		return (swing_up());

	}

	/* Catch control action in ]10°;35] || [-35°;-10°[ */
	else if ((pendulum_angle < - 0.174 && pendulum_speed > 0) || (pendulum_angle > 0.174 && pendulum_speed < 0))
	{
		integral_error = 0;
		return (catch());

	}

	/* Balancing control action in [-10°;10°] */
	else if (pendulum_angle > - 0.174 || pendulum_angle < 0.174)
	{
		return (balancing_control());
	}
	else
	{
		return 0;
	}
}
void actuate_control_action(float torque)
{
	/* Calculate duty cycle from torque */
	float v_out = Ra*torque/Ka + Ka*motor_speed;
	float duty = v_out/Vm;
	uint16_t ccr = (uint16_t)(fabs(duty)*(float)(1+__HAL_TIM_GET_AUTORELOAD(&htim4)));
	/* Duty cycle saturation block */
	if (duty > 1)
	{
		duty = 1;
	}
	else if (duty < -1)
	{
		duty = -1;
	}
	/* Setting duty cycle to PWM Channels */
	if (duty > 0) {
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,ccr);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3, ccr);
		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,0);
	}
}
void update_state()
{
	/* Calculate system states */
	count2_encoder_motor = __HAL_TIM_GET_COUNTER(&htim2);
	count2_encoder_pendulum = __HAL_TIM_GET_COUNTER(&htim3);
	float difference_count_motor = difference_count(count2_encoder_motor,count1_encoder_motor,htim2);
	float difference_count_pendulum = difference_count(count2_encoder_pendulum,count1_encoder_pendulum,htim3);
	count1_encoder_motor = count2_encoder_motor;
	count1_encoder_pendulum = count2_encoder_pendulum;
	motor_speed = ((difference_count_motor * 60)/(gear_ratio_motor * time_elapsed * CPR_encoder_motor))/9.54;
	pendulum_speed = ((difference_count_pendulum * 60)/(time_elapsed * CPR_encoder_pendulum))/9.54;
	pendulum_angle = normalized_angle(count2_encoder_pendulum,CPR_encoder_pendulum);
}
int32_t difference_count(uint32_t cnt2,uint32_t cnt1,TIM_HandleTypeDef htim)
{
	/* Function used to avoid count overflow */
	int32_t difference = cnt2 - cnt1;
	if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim) && (cnt2 > cnt1))
		{
			difference =  cnt2 - __HAL_TIM_GET_AUTORELOAD(&htim) - cnt1;
		}
	else if ((__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim) == 0) && (cnt1 > cnt2))
	{
			difference = cnt1 +  (__HAL_TIM_GET_AUTORELOAD(&htim) - cnt2);
	}

	return difference;
}

float normalized_angle(uint32_t count, uint32_t CPR)
{
	/* Function used to get normalized angle in [-pi;pi] from encoder */
	double angle = ((count * 2 * M_PI) / CPR) - M_PI;
	return ((float) atan2(sin(angle),cos(angle)));
}

void start_detection()
{
	/* Start counting in X4 Encoder mode rising and falling edges of encoders' output signals */
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	/* Time base start in IT mode */
	HAL_TIM_Base_Start_IT(&htim7);
	/* First count */
	count1_encoder_motor = __HAL_TIM_GET_COUNTER(&htim2);
	count1_encoder_pendulum = __HAL_TIM_GET_COUNTER(&htim3);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim -> Instance == TIM7)
		{
			update_state();
			float torque = calculate_control_action();
			actuate_control_action(torque);
		}
}
