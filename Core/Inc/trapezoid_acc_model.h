/*
 * trapezoid_acc_model.h
 *
 *  Created on: Dec 14, 2022
 *      Author: kawaguchitakahito
 */

#ifndef SRC_TRAPEZOID_ACC_MODEL_H_
#define SRC_TRAPEZOID_ACC_MODEL_H_

void acc_dis();
void dec_dis();
void trapezoid_acc_interupt();
void trapezoid_angle_acc_interupt();

extern float acc;//加速度の定義
extern float v_start;//初速定義
extern float v_max;//最高速度定義
extern float v_end;//終端速度定義
extern float x;//設定距離
extern float x_acc;//加速距離
extern float x_dec;//減速距離
extern float Delta_x;
extern float dis;
extern float vel;
extern float dt;
extern int trapezoid_flg;//flgの宣言
extern int trapezoid_angle_flg;//flgの宣言
extern float g_motorCount_l;
extern float g_motorCount_r;
#endif /* SRC_TRAPEZOID_ACC_MODEL_H_ */
