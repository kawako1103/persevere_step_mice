/*
 * explore_method.h
 *
 *  Created on: 2022/12/20
 *      Author: kawaguchitakahito
 */

#ifndef INC_EXPLORE_METHOD_H_
#define INC_EXPLORE_METHOD_H_
#define MAX_QUEUE_NUM 80 //x,yどちらもなので、2倍にした

void left_hand_method();
void left_hand_method_2();
void Print_Wall();
void left_hand_method_and_Print_Wall();
void step_number();
void before_step_number_revised();
void step_number_revised();
void after_step_number_revised();
void Print_Wall_2();
void wall_information_initialize();
void least_step_judgement_and_action_decision();
void action_based_on_direction_decision_and_coordinate_update();
void continual_ver_action_based_on_direction_decision_and_coordinate_update();
void slalom_continual_ver_action_based_on_direction_decision_and_coordinate_update();
void wall_sensor();
void adachi_method();
void continual_adachi_method();
void slalom_continual_adachi_method();


extern int Front_wall;
extern int Right_wall;
extern int Left_wall;

extern int s,k,l;
extern int aroundgoal;

//連続足立法のキュー配列に必要な要素 スタック構造体
typedef struct{
/* データの最前列 */
int head;
    /* データの最後尾 */
    int tail;
    /* スタックされているデータ */
    int data[MAX_QUEUE_NUM];
} STACK_T;
extern STACK_T g_stack;
#endif /* INC_EXPLORE_METHOD_H_ */
