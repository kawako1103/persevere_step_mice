/*
 * wall_control.h
 *
 *  Created on: Dec 16, 2022
 *      Author: kawaguchitakahito
 */

#ifndef SRC_WALL_CONTROL_H_
#define SRC_WALL_CONTROL_H_

void calWallControl();
void wall_control_interupt();
void near_wall_cutting();
//void front_wall_control();
extern int wall_control_flg;//flgの宣言
extern int wall_cut_control_flg;//壁切れ補正のやつ(実践応用編)
extern int wall_cut_control_start_flg;
extern int near_wall_cutting_flg;//壁に近過ぎたら壁制御を切るやつ
extern float v_l;
extern float v_r;
extern float PID_wall;
extern float CENTER_L;//機体が中心にいる時の左側のセンサ値(仮)
extern float CENTER_R;//機体が中心にいる時の右側のセンサ値（仮）
extern float THERESHOULD_L;//左壁判定の閾値（仮実測値からよろ）//壁がないとき69~71なので 70 少し大きめで100
extern float THERESHOULD_R;//右壁判定の閾値（仮実測値からよろ）//壁がないとき35~40なので38 少し大きめて70
extern float WALLREAD_L;
extern float WALLREAD_R;
extern float THERESHOULD_FL;//左壁判定の閾値（仮実測値からよろ）//ありで240/壁がないとき98なので 170少し小さめで140
extern float THERESHOULD_FR;//右壁判定の閾値（仮実測値からよろ）//ありで344/壁がないとき175 259.5少し小さめで220
extern float WALLREAD_FL;
extern float WALLREAD_FR;
extern float THERESHOULD_DIFF_L;//左壁の壁切れ判定の閾値(仮同上) 520-70=450 400にしておこう揺らぎを少なくしたかったらもう少し大きくして
extern float THERESHOULD_DIFF_R;//右壁の壁切れ判定の閾値(仮同上) 280-38=242 200にしておこう同上

extern float FrontWallR;
extern float FrontWallL;

#endif /* SRC_WALL_CONTROL_H_ */
