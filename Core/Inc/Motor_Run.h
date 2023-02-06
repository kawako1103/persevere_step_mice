/*
 * Motor_Run.h
 *
 *  Created on: 2022/12/19
 *      Author: kawaguchitakahito
 */

#ifndef SRC_MOTOR_RUN_H_
#define SRC_MOTOR_RUN_H_

//励磁、PWM系のONOFF
void motor_excitation_on();
void motor_excitation_off();
void motor_pwm_on();
void motor_pwm_off();

//動作系 直進系、スラローム以外は励磁, PWM系のONOFF入っている
void trapezoid_accel_forward(float,float,float,float,float);
void step_ver_trapezoid_accel_forward(float,float,float,float,float);
void non_wall_control_trapezoid_accel_forward(float a0,float v0,float vM,float vE,float tx);
void trapezoid_accel_backward(float,float,float,float,float);
void trapezoid_accel_lturn(float,float,float,float,float);
void trapezoid_accel_rturn(float,float,float,float,float);
void slalom_trapezoid_accel_lturn(float,float,float,float,float,float);
void slalom_trapezoid_accel_rturn(float,float,float,float,float,float);
extern float accm;

#endif /* SRC_MOTOR_RUN_H_ */
