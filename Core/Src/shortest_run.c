/*
 * shortest_run.c
 *
 *  Created on: 2023/02/07
 *      Author: kawaguchitakahito
 */
#include "stdlib.h"
#include "PL_lcd.h"
#include "pl_timer.h"
#include "speaker.h"

#include "Motor_Run.h"
#include "PL_sensor.h"
#include "gpio.h"
#include "wall_control.h"
#include "stdio.h"
#include "explore_method.h"


int pass[255];
int n;
int nmax;
//int direction;
//Dist_map[16][16]=0;

///////
//探索で得た壁情報で歩数マップ作成(未探索の壁は全て塞ぐ)ここやらないと//壁を塞ぐというのは連続足立法でキュー配列を利用した時の歩数マップ作成の歩数更新の条件に探索したかどうかを入れた　これの裏返しが壁を埋めることになるのでは？
//void wall_block(void){
//	int s,k,l;
//	//	int p[16],q[16];
//	//	int i=0;//ここでしか使わないつもり
//	//	int value=0;//同上
//	//	int v,w;//同上
//		//一旦,x,y,zを別の変数に保管
//		s=x;
//		k=y;
//		l=z;
//	//    //足立法で255にしたところを一旦保存
//	//	for(x=0;x<=15;x++){
//	//			for(y=0;y<=15;y++){
//	//				if(Dist_map[x][y]==255){
//	//					p[i]=x;
//	//					q[i]=y;
//	//					value=i;
//	//					i++;
//	//				}
//	//			}
//	//		}//end
//
//	//ゴールの隣にきた時に壁があっても認識できるようにする
//	if(Dist_map[goal_x][goal_y]==255){
//		aroundgoal=1;//フラグみたいなもん
//	}
//
//	//歩数マップの初期化これは足立法の繰り返しでは入れてはいけないきがす、、いや入れないとダメだ　毎回やらないと壁があるところなのに大きい値と判断しちゃう気がする
//	for(x=0;x<=15;x++){
//		for(y=0;y<=15;y++){
//			Dist_map[x][y]=255;
//		}
//	}//end
//
//	//	Print_Wall_2();
//
//	//歩数マップの作成
//	Dist_map[goal_x][goal_y]=0;
//	for(count=0;count<=254;count++){
//		for(x=0;x<=15;x++){
//				for(y=0;y<=15;y++){
//					if(Dist_map[x][y]==count){
//						if((x!=15)&&(Dist_map[x+1][y]==255)&&((column[x]&(1<<(y)))!=(1<<(y)))){//xは東の方向に大きくなっていく x,yでの東//東壁がない
//							Dist_map[x+1][y]=count+1;
//						}
//						if((x!=0)&&(Dist_map[x-1][y]==255)&&((column[x-1]&(1<<(y)))!=(1<<(y)))){//西壁がない
//							Dist_map[x-1][y]=count+1;
//						}
//						if((y!=15)&&(Dist_map[x][y+1]==255)&&((row[y]&(1<<(x)))!=(1<<(x)))){//北壁がない
//							Dist_map[x][y+1]=count+1;
//						}
//						if((y!=0)&&(Dist_map[x][y-1]==255)&&((row[y-1]&(1<<(x)))!=(1<<(x)))){//南壁がない
//							Dist_map[x][y-1]=count+1;
//						}
//						}
//					//end
//					}
//				}
//			}
//
//	if(aroundgoal==1){
//		Dist_map[goal_x][goal_y]=255;
//		aroundgoal=0;//役目終わったら0にする
//	}else{
//		Dist_map[goal_x][goal_y]=0;
//	}
//	//	 //足立法で255に保存したところを元に戻す
//	//		for(i=0;i<=value;i++){
//	//			v=p[i];
//	//			w=q[i];
//	//			Dist_map[v][w]==255;
//	//			}//end
//
//	//x,y,zにこの関数の初めに入れといた数字を戻す
//		x=s;
//		y=k;
//		z=l;
//
//		//255を入れる作業//入れたら次のときのために初期化せよ
//		//北
//		if(z==0){
//			if(Front_wall==255){
//				Dist_map[x][y+1]=Front_wall;
//				Front_wall=0;
//			}
//			if(Right_wall==255){
//				Dist_map[x+1][y]=Right_wall;
//				Right_wall=0;
//			}
//			if(Left_wall==255){
//				Dist_map[x-1][y]=Left_wall;
//				Left_wall=0;
//			}
//		}
//		//東
//		else if(z==1){
//			if(Front_wall==255){
//				Dist_map[x+1][y]=Front_wall;
//				Front_wall=0;
//			}
//			if(Right_wall==255){
//				Dist_map[x][y-1]=Right_wall;
//				Right_wall=0;
//			}
//			if(Left_wall==255){
//				Dist_map[x][y+1]=Left_wall;
//				Left_wall=0;
//			}
//
//		//南
//		}else if(z==2){
//			if(Front_wall==255){
//				Dist_map[x][y-1]=Front_wall;
//				Front_wall=0;
//			}
//			if(Right_wall==255){
//				Dist_map[x-1][y]=Right_wall;
//				Right_wall=0;
//			}
//			if(Left_wall==255){
//				Dist_map[x+1][y]=Left_wall;
//				Left_wall=0;
//			}
//
//		//西
//		}else if(z==3){
//			if(Front_wall==255){
//				Dist_map[x-1][y]=Front_wall;
//				Front_wall=0;
//			}
//			if(Right_wall==255){
//				Dist_map[x][y+1]=Right_wall;
//				Right_wall=0;
//			}
//			if(Left_wall==255){
//				Dist_map[x][y-1]=Left_wall;
//				Left_wall=0;
//			}
//		}
//
//
//}

//
/////////
////歩数マップ作成改善後
//void short_step_number_revised(void){//歩数マップ作成　初期化は外にしてみた//壁を埋めるというのを探索したかどうかも歩数を増やす条件に入れることでそうでないところは255のままになるようにしたが、、
////別の関数によってMの座標は別の変数に補完されることとなりました。
////	int s,k,l;
//////	STACK_T stack;
////	//構造体の話をここで描いとくべきなんかな
//////	int p[16],q[16];
//////	int i=0;//ここでしか使わないつもり
//////	int value=0;//同上
//////	int v,w;//同上
////		//一旦,x,y,zを別の変数に保管
////		s=x;
////		k=y;
////		l=z;
////    //足立法で255にしたところを一旦保存
////	for(x=0;x<=15;x++){
////			for(y=0;y<=15;y++){
////				if(Dist_map[x][y]==255){
////					p[i]=x;
////					q[i]=y;
////					value=i;
////					i++;
////				}
////			}
////		}//end
//
////	//ゴールの隣にきた時に壁があっても認識できるようにする//before.に授ける
////	if(Dist_map[goal_x][goal_y]==255){
////		aroundgoal=1;//フラグみたいなもん
////	}
//
//	//歩数マップの初期化これは足立法の繰り返しでは入れてはいけないきがす、、いや入れないとダメだ　毎回やらないと壁があるところなのに大きい値と判断しちゃう気がする
//	for(x=0;x<=15;x++){
//		for(y=0;y<=15;y++){
//			Dist_map[x][y]=255;
//		}
//	}//end
//
////	Print_Wall_2();
//
//	//スタックの初期化
//	g_stack.head= 0;
//	g_stack.tail= 0;
//
//	//歩数マップの作成これは連続足立法のためのもの
//	Dist_map[goal_x][goal_y]=0;
//
//	pushStack_walk(&g_stack, goal_x);
//	pushStack_walk(&g_stack, goal_y);
//
//
//	for(count=0;count<=254;count++){
//
//		//スタックから座標をpopする
//		x=popStack_walk(&g_stack);//一旦、スタックが空ならbreakというのがこの関数に入っている信じて
//		y=popStack_walk(&g_stack);
//
//
//		if(x==65535||y==65535){
//					break;
//				}
////		for(x=0;x<=15;x++){
////				for(y=0;y<=15;y++){
////		if(x>=0&&x<=15||y>=0&&y<=15){
//						if(x!=15){//探索済かどうかを判定することで、探索済出ないところは255のままになるようにする
//						if((x!=15)&&(Dist_map[x+1][y]==255)&&((column[x]&(1<<(y)))!=(1<<(y)))&&((short_column[x]&(1<<y))==(1<<y))){//xは東の方向に大きくなっていく x,yでの東//東壁がない//探索済
//							Dist_map[x+1][y]=Dist_map[x][y]+1;
//							//代入した座標をスタックにpushする
//								pushStack_walk(&g_stack, x+1);
//								pushStack_walk(&g_stack, y);
//						}
//						}
//
//						if(x!=0){
//						if((x!=0)&&(Dist_map[x-1][y]==255)&&((column[x-1]&(1<<(y)))!=(1<<(y)))&&((short_column[x-1]&(1<<y))==(1<<y))){//西壁がない//探索済
//							Dist_map[x-1][y]=Dist_map[x][y]+1;
//							//代入した座標をスタックにpushする
//								pushStack_walk(&g_stack, x-1);
//								pushStack_walk(&g_stack, y);
//
//						}
//						}
//
//						if(y!=15){
//						if((y!=15)&&(Dist_map[x][y+1]==255)&&((row[y]&(1<<(x)))!=(1<<(x)))&&((short_row[y]&(1<<(x)))==(1<<(x)))){//北壁がない//探索済
//							Dist_map[x][y+1]=Dist_map[x][y]+1;
//							//代入した座標をスタックにpushする
//								pushStack_walk(&g_stack, x);
//								pushStack_walk(&g_stack, y+1);
//
//						}
//						}
//
//						if(y!=0){
//						if((y!=0)&&(Dist_map[x][y-1]==255)&&((row[y-1]&(1<<(x)))!=(1<<(x)))&&((short_row[y-1]&(1<<(x)))==(1<<(x)))){//南壁がない//探索済
//							Dist_map[x][y-1]=Dist_map[x][y]+1;
//							//代入した座標をスタックにpushする
//								pushStack_walk(&g_stack, x);
//								pushStack_walk(&g_stack, y-1);
//
//						}
//						}
////		}
////					//代入した座標をスタックにpushする
////					pushStack_walk(&stack, x);
////				    pushStack_walk(&stack, y);
////					//end
////					}
////				}
//			}
//
////	if(aroundgoal==1){
////		Dist_map[goal_x][goal_y]=255;
////		aroundgoal=0;//役目終わったら0にする
////	}else{
////		Dist_map[goal_x][goal_y]=0;
////	}
//////	 //足立法で255に保存したところを元に戻す
//////		for(i=0;i<=value;i++){
//////			v=p[i];
//////			w=q[i];
//////			Dist_map[v][w]==255;
//////			}//end
////
////	//x,y,zにこの関数の初めに入れといた数字を戻す
////		x=s;
////		y=k;
////		z=l;
////
////		printf("Front_wall=%d,Right_wall=%d,Left_wall=%d\n\r",Front_wall,Right_wall,Left_wall);
////		//255を入れる作業//入れたら次のときのために初期化せよ
////		//北
////		if(z==0){
////			if(Front_wall==255){
////				Dist_map[x][y+1]=Front_wall;
//////				Front_wall=0;//連続足立法の場合, while文中で5回繰り返されるので2回目以降に0になるのを防ぎたいこの関数全体を別の場所に移す
////			}
////			if(Right_wall==255){
////				Dist_map[x+1][y]=Right_wall;
////				Right_wall=0;
////			}
////			if(Left_wall==255){
////				Dist_map[x-1][y]=Left_wall;
////				Left_wall=0;
////			}
////		}
////		//東
////		else if(z==1){
////			if(Front_wall==255){
////				Dist_map[x+1][y]=Front_wall;
////				Front_wall=0;
////			}
////			if(Right_wall==255){
////				Dist_map[x][y-1]=Right_wall;
////				Right_wall=0;
////			}
////			if(Left_wall==255){
////				Dist_map[x][y+1]=Left_wall;
////				Left_wall=0;
////			}
////
////		//南
////		}else if(z==2){
////			if(Front_wall==255){
////				Dist_map[x][y-1]=Front_wall;
////				Front_wall=0;
////			}
////			if(Right_wall==255){
////				Dist_map[x-1][y]=Right_wall;
////				Right_wall=0;
////			}
////			if(Left_wall==255){
////				Dist_map[x+1][y]=Left_wall;
////				Left_wall=0;
////			}
////
////		//西
////		}else if(z==3){
////			if(Front_wall==255){
////				Dist_map[x-1][y]=Front_wall;
////				Front_wall=0;
////			}
////			if(Right_wall==255){
////				Dist_map[x][y+1]=Right_wall;
////				Right_wall=0;
////			}
////			if(Left_wall==255){
////				Dist_map[x][y-1]=Left_wall;
////				Left_wall=0;
////			}
////		}
////		Print_Wall_2;
//
//	}

////passの作成
/////////////////////
//short_step_number_revised();//壁を塞いた上での歩数マップ作成
////Pass[255]の作成
//int pass[255];
//
////Pass[0]~Pass[255]に軌道の順番を保存していく
//
////現在持っているマップ情報と加速上(仮想上の区画の境目からスタート)の位置と向きから前後左右で最も小さい歩数を判定(同じ場合は優先順位をつける前→右→左→後)
//
//x = 0;
//y = 1;
//z = 0;//仮想上の座標(0,1), 北向き(z = 0)からスタート
//
///////////////////////

void short_least_step_judgement_and_action_decision(){

//direction == 0;
if(z==0||z==4){//(x,y),北に対して、//前Dist_map[x][y+1]→右Dist_map[x+1][y]→左Dist_map[x-1][y]→後Dist_map[x][y-1]
	//前後左右最も小さい歩数を決定
	MIN=Dist_map[x][y+1];
	if(MIN>Dist_map[x+1][y]){
		MIN=Dist_map[x+1][y];
		}
	if((x>0)&&(MIN>Dist_map[x-1][y])){
		MIN=Dist_map[x-1][y];
	}else{
		MIN=MIN;
	}
	if((y>0)&&(MIN>Dist_map[x][y-1])){
			MIN=Dist_map[x][y-1];
	}else{
		MIN=MIN;
	}

//		if((y>0)&&(MIN>Dist_map[x][y-1])){
//				MIN=Dist_map[x][y-1];
//			}
//		else{
//			Dist_map[x][y-1]=255;
//		}

	//Mの向きを決めて行動決定に繋げる
	if(MIN==Dist_map[x][y+1]){
		pass[n]=pass[n]+2;//前向き
		direction=0;
	}else if(MIN==Dist_map[x+1][y]){
		pass[n]=-3;//右向き
		direction=1;
//			if((x>0)&&((row[y]&(1<<x))==(1<<x))&&((column[x-1]&(1<<y))==(1<<y))){//前壁かつ左壁あればそれぞれにケツあて(hip_adjustment=1;90前進/180°旋回/遅く90後進/30前進//90°左旋回/遅く90後進/120前進)
//				hip_adjustment=1;
//			}
	}else if((x>0)&&(MIN==Dist_map[x-1][y])){
		pass[n]=-2;//左向き
		direction=2;
//			if(((row[y]&(1<<x))==(1<<x))&&((column[x]&(1<<y))==(1<<y))){//前壁かつ右壁あればそれぞれにケツあて(hip_adjustment=2;90前進/180°旋回/遅く90後進/30前進/90°右旋回/遅く90後進/120前進)
//				hip_adjustment=2;
//			}
    }//else if((y>0)&&(MIN==Dist_map[x][y-1])){
//		direction=3;//後向き
//		if(((x>0)&&((row[y]&(1<<x))==(1<<x))&&((column[x-1]&(1<<y))==(1<<y)))||(((row[y]&(1<<x))==(1<<x))&&((column[x]&(1<<y))==(1<<y)))){//(前壁かつ右壁)or(前壁かつ左壁)あればそれぞれにケツあて(hip_adjustment=3;90前進/180°旋回/遅く90後進/120前進)
//			hip_adjustment=3;
//		}


	//}

	if(z==4){
		z=0;
		  }
}else if(z==1){//(x,y),東に対して、、//前Dist_map[x+1][y]→右Dist_map[x][y-1]→左Dist_map[x][y+1]→後Dist_map[x-1][y]
	//前後左右最も小さい歩数を決定
	MIN=Dist_map[x+1][y];
	if((y>0)&&(MIN>Dist_map[x][y-1])){
		MIN=Dist_map[x][y-1];
	}else{
		MIN=MIN;
	}
	if(MIN>Dist_map[x][y+1]){
		MIN=Dist_map[x][y+1];
	}
	if((x>0)&&(MIN>Dist_map[x-1][y])){
		MIN=Dist_map[x-1][y];
	}else{
		MIN=MIN;
	}
	if(MIN>Dist_map[x][y+1]){
		MIN=Dist_map[x][y+1];
	}
	if((x>0)&&(MIN>Dist_map[x-1][y])){
		MIN=Dist_map[x-1][y];
	}else{
		MIN=MIN;
	}
	//Mの向きを決めて行動決定に繋げる
	if(MIN==Dist_map[x+1][y]){
		pass[n]=pass[n]+2;//前向き
		direction=0;

	}else if((y>0)&&(MIN==Dist_map[x][y-1])){
		pass[n]=-3;//右向き
		direction=1;
//			if(((column[x]&(1<<y))==(1<<y))&&((row[y]&(1<<x))==(1<<x))){//前壁かつ左壁あればそれぞれにケツあて(hip_adjustment=1;90前進/180°旋回/90後進/30前進/90°左旋回/90後進/120前進)
//				hip_adjustment=1;
//			}
	}else if(MIN==Dist_map[x][y+1]){
		pass[n]=-2;//左向き
		direction=2;
//			if((y>0)&&((column[x]&(1<<y))==(1<<y))&&((row[y-1]&(1<<x))==(1<<x))){//前壁かつ右壁あればそれぞれにケツあて(hip_adjustment=2;90前進/180°旋回/90後進/30前進/90°右旋回/90後進/120前進)
//				hip_adjustment=2;
//			}

    }//else if((x>0)&&(MIN==Dist_map[x-1][y])){
//		direction=3;//後向き
//		if((((column[x]&(1<<y))==(1<<y))&&((row[y]&(1<<x))==(1<<x)))||((y>0)&&((column[x]&(1<<y))==(1<<y))&&((row[y-1]&(1<<x))==(1<<x)))){//(前壁かつ右壁)or(前壁かつ左壁)あればそれぞれにケツあて(hip_adjustment=3;90前進/180°旋回/90後進/120前進)
//			hip_adjustment=3;
//		}
//	}

}else if(z==2){//(x,y),南に対して、、//前Dist_map[x][y-1]→右Dist_map[x-1][y]→左Dist_map[x+1][y]→後Dist_map[x][y+1]
	//前後左右最も小さい歩数を決定
	if(y>0){
		MIN=Dist_map[x][y-1];
	}else {
		MIN=255;
	}
	if((x>0)&&(MIN>Dist_map[x-1][y])){
		MIN=Dist_map[x-1][y];
	}else {
		MIN=MIN;
	}
	if(MIN>Dist_map[x+1][y]){
		MIN=Dist_map[x+1][y];
	}
	if(MIN>Dist_map[x][y+1]){
		MIN=Dist_map[x][y+1];
	}
	//Mの向きを決めて行動決定に繋げる
	if((y>0)&&(MIN==Dist_map[x][y-1])){
		pass[n]=pass[n]+2;//前向き
		direction=0;
	}else if((x>0)&&(MIN==Dist_map[x-1][y])){
		pass[n]=-3;//右向き
		direction=1;
//			if((y>0)&&((row[y-1]&(1<<x))==(1<<x))&&((column[x]&(1<<y))==(1<<y))){//前壁かつ左壁あればそれぞれにケツあて(hip_adjustment=1;90前進/180°旋回/遅く90後進/30前進//90°左旋回/遅く90後進/120前進)
//				hip_adjustment=1;
//			}
	}else if(MIN==Dist_map[x+1][y]){
		pass[n]=-2;//左向き
		direction=2;
//			if((y>0)&&(x>0)&&((row[y-1]&(1<<x))==(1<<x))&&((column[x-1]&(1<<y))==(1<<y))){//前壁かつ右壁あればそれぞれにケツあて(hip_adjustment=2;90前進/180°旋回/遅く90後進/30前進/90°右旋回/遅く90後進/120前進)
//				hip_adjustment=2;
//			}

	}//else if(MIN==Dist_map[x][y+1]){
//		direction=3;//後向き
//		if(((y>0)&&((row[y-1]&(1<<x))==(1<<x))&&((column[x]&(1<<y))==(1<<y)))||((y>0)&&(x>0)&&((row[y-1]&(1<<x))==(1<<x))&&((column[x-1]&(1<<y))==(1<<y)))){//(前壁かつ右壁)or(前壁かつ左壁)あればそれぞれにケツあて(hip_adjustment=3;90前進/180°旋回/遅く90後進/120前進)
//			hip_adjustment=3;
//		}
//	}

}else if(z==3||z==-1){//(x,y),西に対して、、//前Dist_map[x-1][y]→右Dist_map[x][y+1]→左Dist_map[x][y-1]→後Dist_map[x+1][y]
	//前後左右最も小さい歩数を決定
	if(x>0){
		MIN=Dist_map[x-1][y];
	}else{
		MIN=Dist_map[x][y+1];
	}
	if(MIN>Dist_map[x][y+1]){
		MIN=Dist_map[x][y+1];
		}
	if((y>0)&&(MIN>Dist_map[x][y-1])){
	MIN=Dist_map[x][y-1];
	}
	if(MIN>Dist_map[x+1][y]){
		MIN=Dist_map[x+1][y];
	}
	//Mの向きを決めて行動決定に繋げる
	if((x>0)&&(MIN==Dist_map[x-1][y])){
		pass[n]=pass[n]+2;//前向き
		direction=0;
	}else if(MIN==Dist_map[x][y+1]){
		pass[n]=-3;//右向き
		direction=1;
//			if((x>0)&&(y>0)&&((column[x-1]&(1<<y))==(1<<y))&&((row[y-1]&(1<<x))==(1<<x))){//前壁かつ左壁あればそれぞれにケツあて(hip_adjustment=1;90前進/180°旋回/90後進/30前進/90°左旋回/90後進/120前進)
//				hip_adjustment=1;
//			}
	}else if((y>0)&&(MIN==Dist_map[x][y-1])){
		pass[n]=-2;//左向き
		direction=2;
//			if((x>0)&&(y>0)&&((column[x-1]&(1<<y))==(1<<y))&&((row[y+1]&(1<<x))==(1<<x))){//前壁かつ右壁あればそれぞれにケツあて(hip_adjustment=2;90前進/180°旋回/90後進/30前進/90°右旋回/90後進/120前進)
//				hip_adjustment=2;
//			}

	}//else if(MIN==Dist_map[x+1][y]){
//		direction=3;//後向き
//		if(((x>0)&&(y>0)&&((column[x-1]&(1<<y))==(1<<y))&&((row[y-1]&(1<<x))==(1<<x)))||((x>0)&&(y>0)&&((column[x-1]&(1<<y))==(1<<y))&&((row[y+1]&(1<<x))==(1<<x)))){//(前壁かつ右壁)or(前壁かつ左壁)あればそれぞれにケツあて(hip_adjustment=3;90前進/180°旋回/90後進/120前進)
//			hip_adjustment=3;
//		}
//	}
	//zを0~3に修正
	if(z==-1){
			  z=3;
		  }
	  }
	printf("MIN=%d,direction=%d",MIN,direction);
Dist_map[goal_x][goal_y]=0;
}


//前が最小

//左が最小 スラローム前提でやる

//右が最小 スラローム前提でやる

//仮想上の位置と向きの更新  ////()最短走行用の行動決定するところではなさそう
void short_action_based_on_direction_decision_and_coordinate_update(void){
	//(最小歩数判別→行動決定(least_step_judgement_and_action_decision(void)))→<動作>
	if(direction==0){//前向き
//		//180 mm直進,停止
//		  trapezoid_accel_forward(2000,100,500,100,180);

		  //座標更新
		  if(z==0||z==4){//北に対して、、
			  x=x;
			  y=y+1;
			  if(z==4){
				  z=0;
			  }
		  }
		  if(z==1){//東に対して、、
			  x=x+1;
			  y=y;
		  }
		  if(z==2){//南に対して、、
			  x=x;
			  y=y-1;
		  }
		  if(z==3||z==-1){//西に対して、、
			  x=x-1;
			  y=y;
			  if(z==-1){
				  z=3;
			  }
		  }
		  //向き更新
			  z=z;
		  }
	      //180 mm直進終了
	if(direction==1){//右向き
//		if(hip_adjustment==0){//条件に合わない時普通に
//		  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//		  trapezoid_accel_rturn(2000,100,400,80,90);//右90°曲がる
//		  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//		}else if(hip_adjustment==1){
////		  hip_drop();
//		}

		  //座標更新
		  if(z==0||z==4){//北に対して、、
			  x=x+1;
			  y=y;
			  if(z==4){
				  z=0;
			  }
		  }
		  if(z==1){//東に対して、、
			  x=x;
			  y=y-1;
		  }
		  if(z==2){//南に対して、、
			  x=x-1;
			  y=y;
		  }
		  if(z==3||z==-1){//西に対して、、
			  x=x;
			  y=y+1;
			  if(z==-1){
				  z=3;
			  }
		  }
		  //向き更新
			  z=z+1;
			  if(z==4){
				  z=0;
			  }


	}
	if(direction==2){//左向き
		  //90 mm直進,90°左旋回,90 mm直進,停止

//		if(hip_adjustment==0){//条件に合わない時普通に
//			  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//			  trapezoid_accel_lturn(2000,100,400,80,90);//左90°曲がる
//			  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//		}else if(hip_adjustment==2){
////		  hip_drop();
//		}
		  //座標更新
		  if(z==0||z==4){//北に対して、、
			  x=x-1;
			  y=y;
			  if(z==4){
				  z=0;
			  }
		  }
		  if(z==1){//東に対して、、
			  x=x;
			  y=y+1;
		  }
		  if(z==2){//南に対して、、
			  x=x+1;
			  y=y;
		  }
		  if(z==3||z==-1){//西に対して、、
			  x=x;
			  y=y-1;
			  if(z==-1){
				  z=3;
			  }
		  }
		  //向き更新
			  z=z-1;
			  if(z==-1){
				  z=3;
			  }
	}
	if(direction==3){//後ろ向き
	//90前進,180°左旋回,90前進,停止
//		if(hip_adjustment==0){//条件に合わない時普通に
//			  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//			  trapezoid_accel_lturn(2000,100,400,80,180);//左180°曲がる
//			  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//		}else if(hip_adjustment==3){
//		  hip_drop();
//		}
		  //座標更新
		  if(z==0||z==4){//北に対して、、
			  x=x;
			  y=y-1;
			  if(z==4){
				  z=0;
			  }
		  }
		  if(z==1){//東に対して、、
			  x=x-1;
			  y=y;
		  }
		  if(z==2){//南に対して、、
			  x=x;
			  y=y+1;
		  }
		  if(z==3||z==-1){//西に対して、、
			  x=x+1;
			  y=y;
			  if(z==-1){
				  z=3;
			  }
		  }
		  //向き更新
			  z=z-2;
			  if(z==-1){
				  z=3;
			  }else if(z==-2){
				  z=2;
		  }


}//終わり
}

//passを決定するところ
void imaginary_run_and_pass_determination(void){
	/////以下は別の関数に入れるべき
	  x = 0;
	  y = 1;
	  z = 0;//仮想上の座標(0,1), 北向き(z = 0)からスタート
	  n = 0;

	  while(1){
			//終了判定
			if((x==goal_x)&&(y==goal_y)){//ゴールに来たら終了　ここは仮想上だからね
//					trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
//					motor_pwm_off();
//					HAL_Delay(1000);//1秒経ってから励磁解除
//					motor_excitation_off();//励磁ストップ
				    nmax = n;//goalの区画に入る境目で終わるのでその後90進んでほしい それをnmaxとする //と思ったけど最後別途90進ませることにした
//				    pass[nmax]=1;
					break;//無限ループから脱出
			}
//			printf("行動前、座標x=%d,y=%d,z=%d\n\r",x,y,z);

			////保存済の壁情報をもとに仮想上の走行での壁認識として働く
			short_ver_wall_sensor();//保存している壁に対して255とか入れるやつ

			////歩数マップ作り
			before_step_number_revised();//
			short_step_number_revised();//歩数マップを作る(壁塞いだやつも含め)

			////歩数マップをもとに行動決定と仮想上での行動後、座標更新 passについてもここで
		  short_least_step_judgement_and_action_decision();//方向の決定(directionで向きを決めている)とpassへの値入れ
		  short_action_based_on_direction_decision_and_coordinate_update();//directionに従って座標と向きの更新x,y,zはここで変わっている
//		  printf("行動後、座標更新時x=%d,y=%d,z=%d\n\r",x,y,z);

		  ////////////////
		  ////////////////最初だけ出して一回止める
//		  Print_Wall_2();
		  ////////////////
		  ////////////////
//		  					 HAL_Delay(1000);
//		  n += 1; //goalの区画に入る境目で終わるのでその後90進んでほしい
		  ///////////////////////////////
		  	 	 char strBuffer[17] = {0};
		  	 	 sprintf(strBuffer, "n=%04d", n);
		  	 	 pl_lcd_pos(1, 0);
		  	 	 pl_lcd_puts(strBuffer);
		 //////////////////////////////////
		  n += 1; //goalの区画に入る境目で終わるのでその後90進んでほしい
	  }




}


//スラローム付き連続足立法の判断後の行動 passが正でどれだけ
void shortest_run_action_based_on_direction_decision_and_coordinate_update(float v){
	//(最小歩数判別→行動決定(least_step_judgement_and_action_decision(void)))→<動作>
	if(pass[n]>0){//前向き
		//半区画 90 mm 直進 速度を可変//90ずつ走っているが、距離の方を調整したほうがいいのかもしれない
		int i;
		for(i=1;i<=pass[n];i++){
		  trapezoid_accel_forward(2000,v,v,v,90);
		}
	}else if(pass[n]==-2){//左:速度は調整する必要があり(v = 500で調整したパラメータなので、)
		offset_slalom_trapezoid_accel_lturn(500,10000,100,460,80,96.9,90.8);//(500,10000,100,460,80,90); (500,17000,100,460,80,90);  (500,20000,100,460,80,90)
		non_wall_control_trapezoid_accel_forward(2000,v,v,v,38);
	}else if(pass[n]==-3){//右
		offset_slalom_trapezoid_accel_rturn(500,10000,100,460,80,97,79);////(500,10000,100,460,80,90); (500,17000,100,460,80,90);  (500,20000,100,460,80,90)
		non_wall_control_trapezoid_accel_forward(2000,v,v,v,38);
	}

}//終わり


void shortest_run(float v){

	  HAL_Delay(5000);
	  //初期動作
      motor_excitation_on();
	  motor_pwm_on();
	  trapezoid_accel_backward(600,100,200,100,80);//90back台形加速の関数遅くね
	  HAL_Delay(500);//
	  motor_pwm_off();
	  motor_pwm_on();
	  trapezoid_accel_forward(2000,100,500,v,120);//120む台形加速の関数



//	  for(n=0;;n++){
//	  		   shortest_run_action_based_on_direction_decision_and_coordinate_update(v);
//	  	  }
///

	  for(n=0;n<=nmax;n++){
		   shortest_run_action_based_on_direction_decision_and_coordinate_update(v);
	  }
	  //最後180進ませることに
	  trapezoid_accel_forward(2000,v,v,v,90);//90む台形加速の関数
	  trapezoid_accel_forward(2000,v,v,100,90);//90む台形加速の関数
		motor_pwm_off();
		HAL_Delay(1000);//1秒経ってから励磁解除
		motor_excitation_off();//励磁ストップ

}

void after_explore_shortes_run(float v){
	imaginary_run_and_pass_determination();//passの決定
	ring_step();
	 before_start_count();

	 pl_lcd_puts("Go!!!!");//最短走行速度500()
	 pl_lcd_pos(1,0);
	 pl_lcd_puts("500");
	shortest_run(v);//最短走行実施
}
