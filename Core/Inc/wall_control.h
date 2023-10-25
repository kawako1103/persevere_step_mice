/*
 * wall_control.h
 *
 *  Created on: Dec 16, 2022
 *      Author: kawaguchitakahito
 */

#ifndef SRC_WALL_CONTROL_H_
#define SRC_WALL_CONTROL_H_

void calWallControl(void);
void wall_control_interupt(void);
extern int wall_control_flg;//flgの宣言
extern float v_l;
extern float v_r;
extern float PID_wall;

#endif /* SRC_WALL_CONTROL_H_ */
