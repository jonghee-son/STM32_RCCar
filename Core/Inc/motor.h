/*
 * motor.h
 *
 *  Created on: Oct 20, 2023
 *      Author: jongh
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_CONTROL GPIOA
#define IN1 GPIO_PIN_0
#define IN2 GPIO_PIN_1
#define IN3 GPIO_PIN_2
#define IN4 GPIO_PIN_3

void motor1_forward(void);

void motor2_forward(void);

void motor1_backward(void);

void motor2_backward(void);

void go_forward(void);

void go_backward(void);

void go_left(void);

void go_right(void);

void stop(void);

#ifdef __cplusplus
}
#endif

#endif /* INC_MOTOR_H_ */
