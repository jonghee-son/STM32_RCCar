/*
 * motor.c
 *
 *  Created on: Oct 20, 2023
 *      Author: jongh
 */

#include "motor.h"

void motor1_forward(void)
{
	HAL_GPIO_WritePin(MOTOR_CONTROL, IN1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_CONTROL, IN2, GPIO_PIN_RESET);
}

void motor2_forward(void)
{
	HAL_GPIO_WritePin(MOTOR_CONTROL, IN3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MOTOR_CONTROL, IN4, GPIO_PIN_RESET);
}

void motor1_backward(void)
{
	HAL_GPIO_WritePin(MOTOR_CONTROL, IN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_CONTROL, IN2, GPIO_PIN_SET);
}

void motor2_backward(void)
{
	HAL_GPIO_WritePin(MOTOR_CONTROL, IN3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_CONTROL, IN4, GPIO_PIN_SET);
}

void go_forward(void)
{
	motor1_forward();
	motor2_forward();
}

void go_backward(void)
{
	motor1_backward();
	motor2_backward();
}

void go_left(void)
{
	motor1_backward();
	motor2_forward();
}

void go_right(void)
{
	motor1_forward();
	motor2_backward();
}

void stop(void) // Motor full stop
{
	HAL_GPIO_WritePin(MOTOR_CONTROL, IN1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_CONTROL, IN2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_CONTROL, IN3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MOTOR_CONTROL, IN4, GPIO_PIN_RESET);
}
