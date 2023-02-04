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
void slalom_trapezoid_acc_interupt();

extern float acc;//加速度の定義
extern float v_start;//初速定義
extern float v_max;//最高速度定義
extern float v_end;//終端速度定義
extern float target_dis;//設定距離
extern float x_acc;//加速距離
extern float x_dec;//減速距離
extern float left_dis;//残り距離
//extern float Delta_x;
extern float dis;//変数として
extern float vel;//変数として

//超信地旋回用パラメータ
extern float angle_v_start;//初角速度定義
extern float angle_v_max;//最高角速度定義
extern float angle_v_end;//終端角速度定義
extern float target_angle;//設定角速度
extern float angle;//disに代入する形なので特に値を代入する必要ない
extern float angle_vel;//一旦
extern float angle_acc;//一旦
extern float angle_acc;//加速距離
extern float angle_dec;//減速距離

extern float dt;
extern int trapezoid_flg;//flgの宣言
extern int trapezoid_angle_flg;//flgの宣言
extern int slalom_trapezoid_flg;//flgの宣言
extern float g_motorCount_l;
extern float g_motorCount_r;
#endif /* SRC_TRAPEZOID_ACC_MODEL_H_ */
