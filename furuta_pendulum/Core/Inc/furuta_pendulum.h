/*
 * furuta_pendulum.h
 *
 *  Created on: Sep 24, 2020
 *  Author: Caramia Donato, D'Alessandro Vito Ivano, Soleti Giovanni, Venezia Antonio
 *  Group name: Team Thor
 *  Description: Furuta Pendulum header file. This file contains all the define,
 *  global variable and function macro used to implement the control system.
 */

#ifndef INC_FURUTA_PENDULUM_H_
#define INC_FURUTA_PENDULUM_H_
#include "main.h"
#include "tim.h"
#include "math.h"

#define time_elapsed 0.005		   /* Sampling time */

/* Hardware specs */
#define CPR_encoder_motor 64
#define CPR_encoder_pendulum 2400
#define gear_ratio_motor 70
#define Vm 12                      /* Motor Supply Voltage */
#define Ka 0.5                     /* Motor Torque Constant*/
#define Ra 2.1818                  /* Motor Armature Resistance*/

/*Swing Up*/
#define mP  0.04				   /* Pendulum mass */
#define g  9.81					   /* Gravity acceleration */
#define Lk  0.15				   /* Pendulum COM */
#define Inertia 0.0003			   /* Pendulum MoI */
#define Kv 28					   /* Swing up proportional coefficient */
#define E0 0					   /* Pendulum energy at the unstable equilibrium */

/* Catch Up */
#define Kp_catch -11.682		   /* Swing up proportional coefficient */
#define Kd_catch -0.4 			   /* Swing up derivation coefficient */

/*PID Inner Loop*/
#define setpoint_p 0               /*Setpoint pendulum position*/
#define Kp -10.620                 /*Proportional coefficient*/
#define Ki -90.382       		   /*Integral coefficient*/
#define Kd -0.312                  /*Derivative coefficient*/

/*PID Outer loop*/
#define Kp_o -0.007				   /*Proportional coefficient*/
#define Kd_o -0.000162	           /*Derivative coefficient*/

/*Global Variables*/
uint32_t count2_encoder_motor;
uint32_t count1_encoder_motor;
uint32_t count2_encoder_pendulum;
uint32_t count1_encoder_pendulum;
float motor_speed;
float pendulum_angle;
float pendulum_speed;
float integral_error;
float motor_speed_1;
float motor_speed_2;
float setpoint_m;

/*Function Macros*/
int8_t sign(float variable);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
int32_t difference_count(uint32_t cnt2,uint32_t cnt1,TIM_HandleTypeDef htim);
float normalized_angle(uint32_t count, uint32_t CPR);
void start_detection();
void update_state();
float control_action();
float PID();
void calculate_duty(float torque);
float balancing_control();
float catch();
float swing_up();

#endif /* INC_FURUTA_PENDULUM_H_ */
