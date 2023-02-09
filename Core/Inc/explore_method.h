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
//void pushStack_walk();
unsigned short popStack_walk();
void before_step_number_revised();
void step_number_revised();
void after_step_number_revised();
void short_step_number_revised();
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

extern int column[16];
extern int row[16];
//最短走行用の壁塞ぐ用の配列
extern int short_column[16];
extern int short_row[16];
extern int x;
extern int y;
extern int z;//(方角,z)=(北,0),(東,1),(南,2),(西,3),(-1になったら3に変換,4になったら0に変換あ(後で入れる))
extern int Dist_map[16][16];
extern int direction;//最短走行では別物を使いたい
extern int Front_wall;
extern int Right_wall;
extern int Left_wall;

//ゴール座標
extern int goal_x;
extern int goal_y;
extern int count;
extern int MIN;

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



