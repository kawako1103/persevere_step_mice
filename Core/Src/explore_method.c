/*
 * explore_method.c
 *
 *  Created on: 2022/12/20
 *      Author: kawaguchitakahito
 */
#include "stdlib.h"
#include "Motor_Run.h"
#include "PL_sensor.h"
#include "gpio.h"
#include "wall_control.h"
#include "stdio.h"
#include "explore_method.h"
#include "speaker.h"

//#define MAX_QUEUE_NUM 40

//<左手法>の概要/動作＋壁情報保存
//以下の閾値は,wall_control.hでグローバル変数化した
//float THERESHOULD_L=260;//左壁判定の閾値（仮実測値からよろ）//壁がないとき69~71なので 70 少し大きめで100
//float THERESHOULD_R=180;//右壁判定の閾値（仮実測値からよろ）//壁がないとき35~40なので38 少し大きめて70
//float THERESHOULD_FL=260;//左壁判定の閾値（仮実測値からよろ）//ありで240/壁がないとき98なので 170少し小さめで140
//float THERESHOULD_FR=180;//右壁判定の閾値（仮実測値からよろ）//ありで344/壁がないとき175 259.5少し小さめで220
int column[16];
int row[16];
int short_column[16];
int short_row[16];
int x=0;
int y=0;
int z=0;//(方角,z)=(北,0),(東,1),(南,2),(西,3),(-1になったら3に変換,4になったら0に変換あ(後で入れる))
int Dist_map[16][16];
int direction; //最小歩数→行動判定の際に使うMの向きを決める//directionは、(0,1,2,3)=(前,右,左,後)とする　方角ではないので注意
int Front_wall;
int Right_wall;
int Left_wall;
int hip_adjustment=0;

//ゴール座標をここに入力
int goal_x=8;
int goal_y=7;
int count;
int MIN;

int s,k,l;
int aroundgoal=0;

//キュー配列系の
///* データの最前列 */
int head;
///* データの最後尾 */
int tail;
/* スタックされているデータ */
int data[MAX_QUEUE_NUM];
STACK_T g_stack;

//壁情報の打ち出し//最短走行の壁を埋める　のつもり　探索した壁は埋めるというやつ
void buried_Print_Wall(void){
	int x,xx;//ここではスペースとして
	int y,i;//ここでしか使わないつもり
	printf("Print Start\n\r");
		for(y=15;y>=0;y--){
			for(x=0;x<16;x++){
				if((short_row[y]&(1<<x))==(1<<x)){
					printf("+---");
				}
				else if((short_row[y]&(0<<x))==(0<<x)){
					printf("+   ");
				}
			}
			printf("+");
			printf("\n\r");
			printf("|");
		for(xx=0;xx<16;xx++){
			if((short_column[xx]&(1<<y))==(1<<y)){
//				printf("%3d|",Dist_Map[xx][y]);
				printf("   |");//空白を3つ分
				}
			else if((short_column[xx]&(0<<y))==(0<<y)){
				printf("    ");//空白を3つ分
				}
		}
			printf("\n\r");
		}
	/*最後の行*/
	for(i=0;i<16;i++){
		printf("+---");
	}
	printf("+\n\r");
	printf("end\n\n\r");
}

//壁情報の打ち出し//左手法の時
void Print_Wall(void){
	int x,xx;//ここではスペースとして
	int y,i;//ここでしか使わないつもり
	printf("Print Start\n\r");
		for(y=15;y>=0;y--){
			for(x=0;x<16;x++){
				if((row[y]&(1<<x))==(1<<x)){
					printf("+---");
				}
				else if((row[y]&(0<<x))==(0<<x)){
					printf("+   ");
				}
			}
			printf("+");
			printf("\n\r");
			printf("|");
		for(xx=0;xx<16;xx++){
			if((column[xx]&(1<<y))==(1<<y)){
//				printf("%3d|",Dist_Map[xx][y]);
				printf("   |");//空白を3つ分
				}
			else if((column[xx]&(0<<y))==(0<<y)){
				printf("    ");//空白を3つ分
				}
		}
			printf("\n\r");
		}
	/*最後の行*/
	for(i=0;i<16;i++){
		printf("+---");
	}
	printf("+\n\r");
	printf("end\n\n\r");
}

////<左手法>の概要/動作＋壁情報保存+出力
//void left_hand_method_and_Print_Wall(void){
//	left_hand_method();
//    Print_Wall();
//}

//
void wall_information_initialize(void){
	int a;
	//わかってるところは閉じる
	a=0;
	column[a]=0b0000000000000001;
    row[a]=0b0000000000000000;
    short_column[a]=0b0000000000000001;
    short_row[a]=0b0000000000000000;

	 for(a=1;a<=15;a++){
	 column[a]=0b0000000000000000;
	 row[a]=0b0000000000000000;//壁情報の初期化ね
	 short_column[a]=0b0000000000000000;
	 short_row[a]=0b0000000000000000;
     }
	 ///端っこだけ壁入れとく
	 column[15]=0b1111111111111111;
	 row[15]=0b1111111111111111;

}


void Print_Wall_2(void){//歩数マップ(↑のstep_numberに準ずる)を確認する時に使う
	int x,xx,y,i;
	printf("Print Start\n\r");
		for(y=15;y>=0;y--){
			for(x=0;x<16;x++){
				if((row[y]&(1<<x))==(1<<x)){
					printf("+---");
				}
				else if((row[y]&(0<<x))==(0<<x)){
					printf("+   ");
				}
			}
			printf("+");
			printf("\n\r");
			printf("|");
		for(xx=0;xx<16;xx++){
			if((column[xx]&(1<<y))==(1<<y)){
				printf("%3d|",Dist_map[xx][y]);
				}
			else if((column[xx]&(0<<y))==(0<<y)){
				printf("%3d ",Dist_map[xx][y]);
				}
		}
			printf("\n\r");
		}
	/*最後の行*/
	for(i=0;i<16;i++){
		printf("+---");
	}
	printf("+\n\r");
	printf("end\n\n\r");
}

//連続足立法用に変更
void hip_drop(void){
	if(hip_adjustment==1){//前壁かつ左壁あればそれぞれにケツあて(hip_adjustment=1;90前進/180°旋回/遅く90後進/30前進//90°左旋回/遅く90後進/120前進)
		trapezoid_accel_forward(2000,500,500,100,70);//70む台形加速の関数
		motor_pwm_off();
		motor_pwm_on();
		  trapezoid_accel_lturn(2000,100,400,80,180);//左180°曲がる
		motor_pwm_off();
		motor_pwm_on();
		  trapezoid_accel_backward(1000,100,300,100,80);//80back台形加速の関数遅くね
		  motor_pwm_off();
		  motor_pwm_on();
		  trapezoid_accel_forward(2000,100,500,100,30);//30前進
		motor_pwm_off();
		motor_pwm_on();
		  trapezoid_accel_lturn(2000,100,400,80,90);//左90°曲がる
		motor_pwm_off();
		motor_pwm_on();
		  trapezoid_accel_backward(1000,100,300,100,90);//90back台形加速の関数遅くね
		  motor_pwm_off();
		  motor_pwm_on();
		  trapezoid_accel_forward(2000,100,500,500,120);//120む台形加速の関数
	}else if(hip_adjustment==2){//前壁かつ右壁あればそれぞれにケツあて(hip_adjustment=2;90前進/180°旋回/遅く90後進/30前進/90°右旋回/遅く90後進/120前進)
		  trapezoid_accel_forward(2000,500,500,100,70);//90む台形加速の関数
			motor_pwm_off();
			motor_pwm_on();
		  trapezoid_accel_lturn(2000,100,400,80,180);//左180°曲がる
			motor_pwm_off();
			motor_pwm_on();
		  trapezoid_accel_backward(1000,100,300,100,80);//90back台形加速の関数遅くね
		  motor_pwm_off();
		  motor_pwm_on();
		  trapezoid_accel_forward(2000,100,500,100,30);//30前進
			motor_pwm_off();
			motor_pwm_on();
		  trapezoid_accel_rturn(2000,100,400,80,90);//右90°曲がる
			motor_pwm_off();
			motor_pwm_on();
		  trapezoid_accel_backward(1000,100,300,100,90);//90back台形加速の関数遅くね
		    motor_pwm_off();
		    motor_pwm_on();
		  trapezoid_accel_forward(2000,100,500,500,120);//120む台形加速の関数
	}else if(hip_adjustment==3){//(前壁かつ右壁)or(前壁かつ左壁)あればそれぞれにケツあて(hip_adjustment=3;90前進/180°旋回/遅く90後進/120前進)
		  trapezoid_accel_forward(2000,500,500,100,70);//90む台形加速の関数
			motor_pwm_off();
			motor_pwm_on();
			////////////////////////
			//横にも位置合わせした方がいいかと思ったが、Mが中心に来ていないといけないからここは180°ターンでとりあえずの角度調整でいいかも　あとは壁制御で調整かな
			//なんかあった時用 もし入れるときは180°ターン~120進むまでを消して
			trapezoid_accel_lturn(2000,100,400,80,90);//左90°曲がる
			motor_pwm_off();
			motor_pwm_on();
//
			trapezoid_accel_backward(2000,100,200,100,80);
			motor_pwm_off();
			motor_pwm_on();

			non_wall_control_trapezoid_accel_forward(2000,100,300,100,30);//ここで中心にこれているか　左右で傾いていないか
			motor_pwm_off();
//			HAL_Delay(500);
			motor_pwm_on();
			trapezoid_accel_lturn(2000,100,400,80,90);//90°曲がる
			motor_pwm_off();
			motor_pwm_on();
			trapezoid_accel_backward(2000,100,200,100,80);
			motor_pwm_off();
			motor_pwm_on();
			trapezoid_accel_forward(2000,100,500,500,120);//ここで中心にこれているか　左右で傾いていないか

			//////////////////////////////
//		  trapezoid_accel_lturn(2000,100,400,80,180);//左180°曲がる
//			motor_pwm_off();
//			motor_pwm_on();
//		  trapezoid_accel_backward(1000,100,300,100,80);//90back台形加速の関数遅くね
//		  motor_pwm_off();
//		  motor_pwm_on();
//		  trapezoid_accel_forward(2000,100,500,500,120);//120む台形加速の関数
}
	hip_adjustment=0;
}

////前後左右最も小さい歩数の判定と行動決定に繋げる//2023 2/16 前or右端にいるときに17の配列を走らないように調整
void least_step_judgement_and_action_decision(void){
	if(z==0||z==4){//(x,y),北に対して、//前Dist_map[x][y+1]→右Dist_map[x+1][y]→左Dist_map[x-1][y]→後Dist_map[x][y-1]
		//前後左右最も小さい歩数を決定
		if(y<15){
		MIN=Dist_map[x][y+1];
		}else{
			MIN=255;
		}

		if((x<15)&&(MIN>Dist_map[x+1][y])){
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
		if((y<15)&&(MIN==Dist_map[x][y+1])){
			direction=0;//前向き
		}else if((x<15)&&(MIN==Dist_map[x+1][y])){
			direction=1;//右向き
//			if((x>0)&&((row[y]&(1<<x))==(1<<x))&&((column[x-1]&(1<<y))==(1<<y))){//前壁かつ左壁あればそれぞれにケツあて(hip_adjustment=1;90前進/180°旋回/遅く90後進/30前進//90°左旋回/遅く90後進/120前進)
//				hip_adjustment=1;
//			}
		}else if((x>0)&&(MIN==Dist_map[x-1][y])){
			direction=2;//左向き
//			if(((row[y]&(1<<x))==(1<<x))&&((column[x]&(1<<y))==(1<<y))){//前壁かつ右壁あればそれぞれにケツあて(hip_adjustment=2;90前進/180°旋回/遅く90後進/30前進/90°右旋回/遅く90後進/120前進)
//				hip_adjustment=2;
//			}

		}else if((y>0)&&(MIN==Dist_map[x][y-1])){
			direction=3;//後向き
			if(((x>0)&&((row[y]&(1<<x))==(1<<x))&&((column[x-1]&(1<<y))==(1<<y)))||(((row[y]&(1<<x))==(1<<x))&&((column[x]&(1<<y))==(1<<y)))){//(前壁かつ右壁)or(前壁かつ左壁)あればそれぞれにケツあて(hip_adjustment=3;90前進/180°旋回/遅く90後進/120前進)
				hip_adjustment=3;
			}


		}

		if(z==4){
			z=0;
			  }
	}else if(z==1){//(x,y),東に対して、、//前Dist_map[x+1][y]→右Dist_map[x][y-1]→左Dist_map[x][y+1]→後Dist_map[x-1][y]
		//前後左右最も小さい歩数を決定
		if(x<15){
			MIN=Dist_map[x+1][y];
		}else{
			MIN = 255;
		}
		if((y>0)&&(MIN>Dist_map[x][y-1])){
			MIN=Dist_map[x][y-1];
		}else{
			MIN=MIN;
		}
		if((y<15)&&(MIN>Dist_map[x][y+1])){
			MIN=Dist_map[x][y+1];
		}
		if((x>0)&&(MIN>Dist_map[x-1][y])){
			MIN=Dist_map[x-1][y];
		}else{
			MIN=MIN;
		}
		if((y<15)&&(MIN>Dist_map[x][y+1])){
			MIN=Dist_map[x][y+1];
		}
		if((x>0)&&(MIN>Dist_map[x-1][y])){
			MIN=Dist_map[x-1][y];
		}else{
			MIN=MIN;
		}
		//Mの向きを決めて行動決定に繋げる
		if((x<15)&&(MIN==Dist_map[x+1][y])){
			direction=0;//前向き
		}else if((y>0)&&(MIN==Dist_map[x][y-1])){
			direction=1;//右向き
//			if(((column[x]&(1<<y))==(1<<y))&&((row[y]&(1<<x))==(1<<x))){//前壁かつ左壁あればそれぞれにケツあて(hip_adjustment=1;90前進/180°旋回/90後進/30前進/90°左旋回/90後進/120前進)
//				hip_adjustment=1;
//			}
		}else if((y<15)&&(MIN==Dist_map[x][y+1])){
			direction=2;//左向き
//			if((y>0)&&((column[x]&(1<<y))==(1<<y))&&((row[y-1]&(1<<x))==(1<<x))){//前壁かつ右壁あればそれぞれにケツあて(hip_adjustment=2;90前進/180°旋回/90後進/30前進/90°右旋回/90後進/120前進)
//				hip_adjustment=2;
//			}

		}else if((x>0)&&(MIN==Dist_map[x-1][y])){
			direction=3;//後向き
			if((((column[x]&(1<<y))==(1<<y))&&((row[y]&(1<<x))==(1<<x)))||((y>0)&&((column[x]&(1<<y))==(1<<y))&&((row[y-1]&(1<<x))==(1<<x)))){//(前壁かつ右壁)or(前壁かつ左壁)あればそれぞれにケツあて(hip_adjustment=3;90前進/180°旋回/90後進/120前進)
				hip_adjustment=3;
			}
		}

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
		if((x<15)&&(MIN>Dist_map[x+1][y])){
			MIN=Dist_map[x+1][y];
		}
		if((y<15)&&(MIN>Dist_map[x][y+1])){
			MIN=Dist_map[x][y+1];
		}
		//Mの向きを決めて行動決定に繋げる
		if((y>0)&&(MIN==Dist_map[x][y-1])){
			direction=0;//前向き
		}else if((x>0)&&(MIN==Dist_map[x-1][y])){
			direction=1;//右向き
//			if((y>0)&&((row[y-1]&(1<<x))==(1<<x))&&((column[x]&(1<<y))==(1<<y))){//前壁かつ左壁あればそれぞれにケツあて(hip_adjustment=1;90前進/180°旋回/遅く90後進/30前進//90°左旋回/遅く90後進/120前進)
//				hip_adjustment=1;
//			}
		}else if((x<15)&&(MIN==Dist_map[x+1][y])){
			direction=2;//左向き
//			if((y>0)&&(x>0)&&((row[y-1]&(1<<x))==(1<<x))&&((column[x-1]&(1<<y))==(1<<y))){//前壁かつ右壁あればそれぞれにケツあて(hip_adjustment=2;90前進/180°旋回/遅く90後進/30前進/90°右旋回/遅く90後進/120前進)
//				hip_adjustment=2;
//			}

		}else if((y<15)&&(MIN==Dist_map[x][y+1])){
			direction=3;//後向き
			if(((y>0)&&((row[y-1]&(1<<x))==(1<<x))&&((column[x]&(1<<y))==(1<<y)))||((y>0)&&(x>0)&&((row[y-1]&(1<<x))==(1<<x))&&((column[x-1]&(1<<y))==(1<<y)))){//(前壁かつ右壁)or(前壁かつ左壁)あればそれぞれにケツあて(hip_adjustment=3;90前進/180°旋回/遅く90後進/120前進)
				hip_adjustment=3;
			}
		}

	}else if(z==3||z==-1){//(x,y),西に対して、、//前Dist_map[x-1][y]→右Dist_map[x][y+1]→左Dist_map[x][y-1]→後Dist_map[x+1][y]
		//前後左右最も小さい歩数を決定
		if(x>0){
			MIN=Dist_map[x-1][y];
		}else if(y<15){
			MIN=Dist_map[x][y+1];
		}else{
			MIN = 255;
		}

		if((y<15)&&(MIN>Dist_map[x][y+1])){
			MIN=Dist_map[x][y+1];
			}
		if((y>0)&&(MIN>Dist_map[x][y-1])){
			MIN=Dist_map[x][y-1];
		}
		if((x<15)&&(MIN>Dist_map[x+1][y])){
			MIN=Dist_map[x+1][y];
		}
		//Mの向きを決めて行動決定に繋げる
		if((x>0)&&(MIN==Dist_map[x-1][y])){
			direction=0;//前向き
		}else if((y<15)&&(MIN==Dist_map[x][y+1])){
			direction=1;//右向き
//			if((x>0)&&(y>0)&&((column[x-1]&(1<<y))==(1<<y))&&((row[y-1]&(1<<x))==(1<<x))){//前壁かつ左壁あればそれぞれにケツあて(hip_adjustment=1;90前進/180°旋回/90後進/30前進/90°左旋回/90後進/120前進)
//				hip_adjustment=1;
//			}
		}else if((y>0)&&(MIN==Dist_map[x][y-1])){
			direction=2;//左向き
//			if((x>0)&&(y>0)&&((column[x-1]&(1<<y))==(1<<y))&&((row[y+1]&(1<<x))==(1<<x))){//前壁かつ右壁あればそれぞれにケツあて(hip_adjustment=2;90前進/180°旋回/90後進/30前進/90°右旋回/90後進/120前進)
//				hip_adjustment=2;
//			}

		}else if((x<15)&&(MIN==Dist_map[x+1][y])){
			direction=3;//後向き
			if(((x>0)&&(y>0)&&((column[x-1]&(1<<y))==(1<<y))&&((row[y-1]&(1<<x))==(1<<x)))||((x>0)&&(y>0)&&((column[x-1]&(1<<y))==(1<<y))&&((row[y+1]&(1<<x))==(1<<x)))){//(前壁かつ右壁)or(前壁かつ左壁)あればそれぞれにケツあて(hip_adjustment=3;90前進/180°旋回/90後進/120前進)
				hip_adjustment=3;
			}
		}
		//zを0~3に修正
		if(z==-1){
				  z=3;
			  }
		  }
//	printf("MIN=%d,direction=%d",MIN,direction);
	Dist_map[goal_x][goal_y]=0;
}

//<足立法>の概要/動作?いや歩数マップってやつ
void step_number(void){//歩数マップ作成　初期化は外にしてみた

	int s,k,l;
//	int p[16],q[16];
//	int i=0;//ここでしか使わないつもり
//	int value=0;//同上
//	int v,w;//同上
		//一旦,x,y,zを別の変数に保管
		s=x;
		k=y;
		l=z;
//    //足立法で255にしたところを一旦保存
//	for(x=0;x<=15;x++){
//			for(y=0;y<=15;y++){
//				if(Dist_map[x][y]==255){
//					p[i]=x;
//					q[i]=y;
//					value=i;
//					i++;
//				}
//			}
//		}//end

	//ゴールの隣にきた時に壁があっても認識できるようにする
	if(Dist_map[goal_x][goal_y]==255){
		aroundgoal=1;//フラグみたいなもん
	}

	//歩数マップの初期化これは足立法の繰り返しでは入れてはいけないきがす、、いや入れないとダメだ　毎回やらないと壁があるところなのに大きい値と判断しちゃう気がする
	for(x=0;x<=15;x++){
		for(y=0;y<=15;y++){
			Dist_map[x][y]=255;
		}
	}//end

//	Print_Wall_2();

	//歩数マップの作成
	Dist_map[goal_x][goal_y]=0;
	for(count=0;count<=254;count++){
		for(x=0;x<=15;x++){
				for(y=0;y<=15;y++){
					if(Dist_map[x][y]==count){
						if((x!=15)&&(Dist_map[x+1][y]==255)&&((column[x]&(1<<(y)))!=(1<<(y)))){//xは東の方向に大きくなっていく x,yでの東//東壁がない
							Dist_map[x+1][y]=count+1;
						}
						if((x!=0)&&(Dist_map[x-1][y]==255)&&((column[x-1]&(1<<(y)))!=(1<<(y)))){//西壁がない
							Dist_map[x-1][y]=count+1;
						}
						if((y!=15)&&(Dist_map[x][y+1]==255)&&((row[y]&(1<<(x)))!=(1<<(x)))){//北壁がない
							Dist_map[x][y+1]=count+1;
						}
						if((y!=0)&&(Dist_map[x][y-1]==255)&&((row[y-1]&(1<<(x)))!=(1<<(x)))){//南壁がない
							Dist_map[x][y-1]=count+1;
						}
						}
					//end
					}
				}
			}

	if(aroundgoal==1){
		Dist_map[goal_x][goal_y]=255;
		aroundgoal=0;//役目終わったら0にする
	}else{
		Dist_map[goal_x][goal_y]=0;
	}
//	 //足立法で255に保存したところを元に戻す
//		for(i=0;i<=value;i++){
//			v=p[i];
//			w=q[i];
//			Dist_map[v][w]==255;
//			}//end

	//x,y,zにこの関数の初めに入れといた数字を戻す
		x=s;
		y=k;
		z=l;

		//255を入れる作業//入れたら次のときのために初期化せよ
		//北
		if(z==0){
			if(Front_wall==255){
				Dist_map[x][y+1]=Front_wall;
				Front_wall=0;
			}
			if(Right_wall==255){
				Dist_map[x+1][y]=Right_wall;
				Right_wall=0;
			}
			if(Left_wall==255){
				Dist_map[x-1][y]=Left_wall;
				Left_wall=0;
			}
		}
		//東
		else if(z==1){
			if(Front_wall==255){
				Dist_map[x+1][y]=Front_wall;
				Front_wall=0;
			}
			if(Right_wall==255){
				Dist_map[x][y-1]=Right_wall;
				Right_wall=0;
			}
			if(Left_wall==255){
				Dist_map[x][y+1]=Left_wall;
				Left_wall=0;
			}

		//南
		}else if(z==2){
			if(Front_wall==255){
				Dist_map[x][y-1]=Front_wall;
				Front_wall=0;
			}
			if(Right_wall==255){
				Dist_map[x-1][y]=Right_wall;
				Right_wall=0;
			}
			if(Left_wall==255){
				Dist_map[x+1][y]=Left_wall;
				Left_wall=0;
			}

		//西
		}else if(z==3){
			if(Front_wall==255){
				Dist_map[x-1][y]=Front_wall;
				Front_wall=0;
			}
			if(Right_wall==255){
				Dist_map[x][y+1]=Right_wall;
				Right_wall=0;
			}
			if(Left_wall==255){
				Dist_map[x][y-1]=Left_wall;
				Left_wall=0;
			}
		}
	}


////連続足立法のキュー配列に必要な要素 スタック構造体
//typedef struct{
///* データの最前列 */
//int head;
//    /* データの最後尾 */
//    int tail;
//    /* スタックされているデータ */
//    int data[MAX_QUEUE_NUM];
//} STACK_T;
//extern STACK_T g_stack;

//スタックのpush
void pushStack_walk(STACK_T *stack, unsigned short input){

    /* データをデータの最後尾の１つ後ろに格納 */
    stack->data[stack->tail] = input;

    /* データの最後尾を１つ後ろに移動 */
    stack->tail = stack->tail + 1;

    /* 巡回シフト */
    if(stack->tail == MAX_QUEUE_NUM) stack->tail = 0;

    /* スタックが満杯なら何もせず関数終了 */
    if(stack->tail == stack->head ){
    //printf("stack_full\n");
        return;
    }
}

//スタックのpop
unsigned short popStack_walk(STACK_T *stack){
	unsigned short ret = 0;

	/* スタックが空なら何もせずに関数終了 */
	if(stack->tail == stack->head){
	//printf("stack_empty\n");
		return 65535;
	}

	/* データの最前列からデータを取得 */
	ret = stack->data[stack->head];

	/* データの最前列を１つ前にずらす */
	stack->head = stack->head + 1;//これだと後ろじゃ？

	/* 巡回シフト */
	if(stack->head == MAX_QUEUE_NUM) stack->head = 0;

	/* 取得したデータを返却 */
	return ret;
}


void before_step_number_revised(void){
//	int s,k,l;
	//	STACK_T stack;
		//構造体の話をここで描いとくべきなんかな
	//	int p[16],q[16];
	//	int i=0;//ここでしか使わないつもり
	//	int value=0;//同上
	//	int v,w;//同上
			//一旦,x,y,zを別の変数に保管
			s=x;
			k=y;
			l=z;

			//ゴールの隣にきた時に壁があっても認識できるようにする
				if(Dist_map[goal_x][goal_y]==255){
					aroundgoal=1;//フラグみたいなもん
				}
}

//歩数マップ作成改善後
void step_number_revised(void){//歩数マップ作成　初期化は外にしてみた
//別の関数によってMの座標は別の変数に補完されることとなりました。
//	int s,k,l;
////	STACK_T stack;
//	//構造体の話をここで描いとくべきなんかな
////	int p[16],q[16];
////	int i=0;//ここでしか使わないつもり
////	int value=0;//同上
////	int v,w;//同上
//		//一旦,x,y,zを別の変数に保管
//		s=x;
//		k=y;
//		l=z;
//    //足立法で255にしたところを一旦保存
//	for(x=0;x<=15;x++){
//			for(y=0;y<=15;y++){
//				if(Dist_map[x][y]==255){
//					p[i]=x;
//					q[i]=y;
//					value=i;
//					i++;
//				}
//			}
//		}//end

//	//ゴールの隣にきた時に壁があっても認識できるようにする//before.に授ける
//	if(Dist_map[goal_x][goal_y]==255){
//		aroundgoal=1;//フラグみたいなもん
//	}

	//歩数マップの初期化これは足立法の繰り返しでは入れてはいけないきがす、、いや入れないとダメだ　毎回やらないと壁があるところなのに大きい値と判断しちゃう気がする
	for(x=0;x<=15;x++){
		for(y=0;y<=15;y++){
			Dist_map[x][y]=255;
		}
	}//end

//	Print_Wall_2();

	//スタックの初期化
	g_stack.head= 0;
	g_stack.tail= 0;

	//歩数マップの作成これは連続足立法のためのもの
	Dist_map[goal_x][goal_y]=0;

	pushStack_walk(&g_stack, goal_x);
	pushStack_walk(&g_stack, goal_y);


	for(count=0;count<=254;count++){

		//スタックから座標をpopする
		x=popStack_walk(&g_stack);//一旦、スタックが空ならbreakというのがこの関数に入っている信じて
		y=popStack_walk(&g_stack);


		if(x==65535||y==65535){
					break;
				}
//		for(x=0;x<=15;x++){
//				for(y=0;y<=15;y++){
//		if(x>=0&&x<=15||y>=0&&y<=15){
						if(x!=15){
						if((x!=15)&&(Dist_map[x+1][y]==255)&&((column[x]&(1<<(y)))!=(1<<(y)))){//xは東の方向に大きくなっていく x,yでの東//東壁がない
							Dist_map[x+1][y]=Dist_map[x][y]+1;
							//代入した座標をスタックにpushする
								pushStack_walk(&g_stack, x+1);
								pushStack_walk(&g_stack, y);
						}
						}

						if(x!=0){
						if((x!=0)&&(Dist_map[x-1][y]==255)&&((column[x-1]&(1<<(y)))!=(1<<(y)))){//西壁がない
							Dist_map[x-1][y]=Dist_map[x][y]+1;
							//代入した座標をスタックにpushする
								pushStack_walk(&g_stack, x-1);
								pushStack_walk(&g_stack, y);

						}
						}

						if(y!=15){
						if((y!=15)&&(Dist_map[x][y+1]==255)&&((row[y]&(1<<(x)))!=(1<<(x)))){//北壁がない
							Dist_map[x][y+1]=Dist_map[x][y]+1;
							//代入した座標をスタックにpushする
								pushStack_walk(&g_stack, x);
								pushStack_walk(&g_stack, y+1);

						}
						}

						if(y!=0){
						if((y!=0)&&(Dist_map[x][y-1]==255)&&((row[y-1]&(1<<(x)))!=(1<<(x)))){//南壁がない
							Dist_map[x][y-1]=Dist_map[x][y]+1;
							//代入した座標をスタックにpushする
								pushStack_walk(&g_stack, x);
								pushStack_walk(&g_stack, y-1);

						}
						}
//		}
//					//代入した座標をスタックにpushする
//					pushStack_walk(&stack, x);
//				    pushStack_walk(&stack, y);
//					//end
//					}
//				}
			}

//	if(aroundgoal==1){
//		Dist_map[goal_x][goal_y]=255;
//		aroundgoal=0;//役目終わったら0にする
//	}else{
//		Dist_map[goal_x][goal_y]=0;
//	}
////	 //足立法で255に保存したところを元に戻す
////		for(i=0;i<=value;i++){
////			v=p[i];
////			w=q[i];
////			Dist_map[v][w]==255;
////			}//end
//
//	//x,y,zにこの関数の初めに入れといた数字を戻す
//		x=s;
//		y=k;
//		z=l;
//
//		printf("Front_wall=%d,Right_wall=%d,Left_wall=%d\n\r",Front_wall,Right_wall,Left_wall);
//		//255を入れる作業//入れたら次のときのために初期化せよ
//		//北
//		if(z==0){
//			if(Front_wall==255){
//				Dist_map[x][y+1]=Front_wall;
////				Front_wall=0;//連続足立法の場合, while文中で5回繰り返されるので2回目以降に0になるのを防ぎたいこの関数全体を別の場所に移す
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
//		Print_Wall_2;

	}

void after_step_number_revised(){

	if(aroundgoal==1){
			Dist_map[goal_x][goal_y]=255;
			aroundgoal=0;//役目終わったら0にする
		}else{
			Dist_map[goal_x][goal_y]=0;
		}
	//	 //足立法で255に保存したところを元に戻す
	//		for(i=0;i<=value;i++){
	//			v=p[i];
	//			w=q[i];
	//			Dist_map[v][w]==255;
	//			}//end

		//x,y,zにこの関数の初めに入れといた数字を戻す
			x=s;
			y=k;
			z=l;

//			printf("Front_wall=%d,Right_wall=%d,Left_wall=%d\n\r",Front_wall,Right_wall,Left_wall);
			//255を入れる作業//入れたら次のときのために初期化せよ
			//北
			if(z==0){
				if(Front_wall==255){
					Dist_map[x][y+1]=Front_wall;
					Front_wall=0;//連続足立法の場合, while文中で5回繰り返されるので2回目以降に0になるのを防ぎたいこの関数全体を別の場所に移す
				}
				if(Right_wall==255){
					Dist_map[x+1][y]=Right_wall;
					Right_wall=0;
				}
				if(Left_wall==255){
					Dist_map[x-1][y]=Left_wall;
					Left_wall=0;
				}
			}
			//東
			else if(z==1){
				if(Front_wall==255){
					Dist_map[x+1][y]=Front_wall;
					Front_wall=0;
				}
				if(Right_wall==255){
					Dist_map[x][y-1]=Right_wall;
					Right_wall=0;
				}
				if(Left_wall==255){
					Dist_map[x][y+1]=Left_wall;
					Left_wall=0;
				}

			//南
			}else if(z==2){
				if(Front_wall==255){
					Dist_map[x][y-1]=Front_wall;
					Front_wall=0;
				}
				if(Right_wall==255){
					Dist_map[x-1][y]=Right_wall;
					Right_wall=0;
				}
				if(Left_wall==255){
					Dist_map[x+1][y]=Left_wall;
					Left_wall=0;
				}

			//西
			}else if(z==3){
				if(Front_wall==255){
					Dist_map[x-1][y]=Front_wall;
					Front_wall=0;
				}
				if(Right_wall==255){
					Dist_map[x][y+1]=Right_wall;
					Right_wall=0;
				}
				if(Left_wall==255){
					Dist_map[x][y-1]=Left_wall;
					Left_wall=0;
				}
			}
//			Print_Wall_2;

}

//最短走行用の歩数マップ作る関数

///////
//歩数マップ作成改善後
void short_step_number_revised(void){//歩数マップ作成　初期化は外にしてみた//壁を埋めるというのを探索したかどうかも歩数を増やす条件に入れることでそうでないところは255のままになるようにしたが、、
//別の関数によってMの座標は別の変数に補完されることとなりました。
//	int s,k,l;
////	STACK_T stack;
//	//構造体の話をここで描いとくべきなんかな
////	int p[16],q[16];
////	int i=0;//ここでしか使わないつもり
////	int value=0;//同上
////	int v,w;//同上
//		//一旦,x,y,zを別の変数に保管
//		s=x;
//		k=y;
//		l=z;
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

	//歩数マップの初期化これは足立法の繰り返しでは入れてはいけないきがす、、いや入れないとダメだ　毎回やらないと壁があるところなのに大きい値と判断しちゃう気がする
	for(x=0;x<=15;x++){
		for(y=0;y<=15;y++){
			Dist_map[x][y]=255;
		}
	}//end

//	Print_Wall_2();

	//スタックの初期化
	g_stack.head= 0;
	g_stack.tail= 0;

	//歩数マップの作成これは連続足立法のためのもの
	Dist_map[goal_x][goal_y]=0;

	pushStack_walk(&g_stack, goal_x);
	pushStack_walk(&g_stack, goal_y);


	for(count=0;count<=254;count++){

		//スタックから座標をpopする
		x=popStack_walk(&g_stack);//一旦、スタックが空ならbreakというのがこの関数に入っている信じて
		y=popStack_walk(&g_stack);


		if(x==65535||y==65535){
					break;
				}
//		for(x=0;x<=15;x++){
//				for(y=0;y<=15;y++){
//		if(x>=0&&x<=15||y>=0&&y<=15){
						if(x!=15){//探索済かどうかを判定することで、探索済出ないところは255のままになるようにする
						if((x!=15)&&(Dist_map[x+1][y]==255)&&((column[x]&(1<<(y)))!=(1<<(y)))&&((short_column[x]&(1<<y))==(1<<y))){//xは東の方向に大きくなっていく x,yでの東//東壁がない//探索済
							Dist_map[x+1][y]=Dist_map[x][y]+1;
							//代入した座標をスタックにpushする
								pushStack_walk(&g_stack, x+1);
								pushStack_walk(&g_stack, y);
						}
						}

						if(x!=0){
						if((x!=0)&&(Dist_map[x-1][y]==255)&&((column[x-1]&(1<<(y)))!=(1<<(y)))&&((short_column[x-1]&(1<<y))==(1<<y))){//西壁がない//探索済
							Dist_map[x-1][y]=Dist_map[x][y]+1;
							//代入した座標をスタックにpushする
								pushStack_walk(&g_stack, x-1);
								pushStack_walk(&g_stack, y);

						}
						}

						if(y!=15){
						if((y!=15)&&(Dist_map[x][y+1]==255)&&((row[y]&(1<<(x)))!=(1<<(x)))&&((short_row[y]&(1<<(x)))==(1<<(x)))){//北壁がない//探索済
							Dist_map[x][y+1]=Dist_map[x][y]+1;
							//代入した座標をスタックにpushする
								pushStack_walk(&g_stack, x);
								pushStack_walk(&g_stack, y+1);

						}
						}

						if(y!=0){
						if((y!=0)&&(Dist_map[x][y-1]==255)&&((row[y-1]&(1<<(x)))!=(1<<(x)))&&((short_row[y-1]&(1<<(x)))==(1<<(x)))){//南壁がない//探索済
							Dist_map[x][y-1]=Dist_map[x][y]+1;
							//代入した座標をスタックにpushする
								pushStack_walk(&g_stack, x);
								pushStack_walk(&g_stack, y-1);

						}
						}
//		}
//					//代入した座標をスタックにpushする
//					pushStack_walk(&stack, x);
//				    pushStack_walk(&stack, y);
//					//end
//					}
//				}
			}

	if(aroundgoal==1){
		Dist_map[goal_x][goal_y]=255;
		aroundgoal=0;//役目終わったら0にする
	}else{
		Dist_map[goal_x][goal_y]=0;
	}
//	 //足立法で255に保存したところを元に戻す
//		for(i=0;i<=value;i++){
//			v=p[i];
//			w=q[i];
//			Dist_map[v][w]==255;
//			}//end

	//x,y,zにこの関数の初めに入れといた数字を戻す
		x=s;
		y=k;
		z=l;

//		printf("Front_wall=%d,Right_wall=%d,Left_wall=%d\n\r",Front_wall,Right_wall,Left_wall);
		//255を入れる作業//入れたら次のときのために初期化せよ
		//北
		if(z==0){
			if(Front_wall==255){
				Dist_map[x][y+1]=Front_wall;
				Front_wall=0;//連続足立法の場合, while文中で5回繰り返されるので2回目以降に0になるのを防ぎたいこの関数全体を別の場所に移す
			}
			if(Right_wall==255){
				Dist_map[x+1][y]=Right_wall;
				Right_wall=0;
			}
			if(Left_wall==255){
				Dist_map[x-1][y]=Left_wall;
				Left_wall=0;
			}
		}
		//東
		else if(z==1){
			if(Front_wall==255){
				Dist_map[x+1][y]=Front_wall;
				Front_wall=0;
			}
			if(Right_wall==255){
				Dist_map[x][y-1]=Right_wall;
				Right_wall=0;
			}
			if(Left_wall==255){
				Dist_map[x][y+1]=Left_wall;
				Left_wall=0;
			}

		//南
		}else if(z==2){
			if(Front_wall==255){
				Dist_map[x][y-1]=Front_wall;
				Front_wall=0;
			}
			if(Right_wall==255){
				Dist_map[x-1][y]=Right_wall;
				Right_wall=0;
			}
			if(Left_wall==255){
				Dist_map[x+1][y]=Left_wall;
				Left_wall=0;
			}

		//西
		}else if(z==3){
			if(Front_wall==255){
				Dist_map[x-1][y]=Front_wall;
				Front_wall=0;
			}
			if(Right_wall==255){
				Dist_map[x][y+1]=Right_wall;
				Right_wall=0;
			}
			if(Left_wall==255){
				Dist_map[x][y-1]=Left_wall;
				Left_wall=0;
			}
		}
//		Print_Wall_2;

	}



void action_based_on_direction_decision_and_coordinate_update(void){
	//(最小歩数判別→行動決定(least_step_judgement_and_action_decision(void)))→<動作>
	if(direction==0){//前向き
		//180 mm直進,停止
		  trapezoid_accel_forward(2000,100,500,100,180);

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
		if(hip_adjustment==0){//条件に合わない時普通に
		  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
		  trapezoid_accel_rturn(2000,100,400,80,90);//右90°曲がる
		  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
		}else if(hip_adjustment==1){
//		  hip_drop();
		}

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

		if(hip_adjustment==0){//条件に合わない時普通に
			  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
			  trapezoid_accel_lturn(2000,100,400,80,90);//左90°曲がる
			  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
		}else if(hip_adjustment==2){
//		  hip_drop();
		}
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
		if(hip_adjustment==0){//条件に合わない時普通に
			  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
			  trapezoid_accel_lturn(2000,100,400,80,180);//左180°曲がる
			  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
		}else if(hip_adjustment==3){
		  hip_drop();
		}
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

//連続足立法の判断後の行動
void continual_ver_action_based_on_direction_decision_and_coordinate_update(void){
	//(最小歩数判別→行動決定(least_step_judgement_and_action_decision(void)))→<動作>
	if(direction==0){//前向き
		//160 mm直進,停止
		  trapezoid_accel_forward(2000,500,500,500,160);

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
		if(hip_adjustment==0){//条件に合わない時普通に
		  trapezoid_accel_forward(2000,500,500,100,70);//70む台形加速の関数 連続足立法ね
		  motor_pwm_off();
		  motor_pwm_on();
		  trapezoid_accel_rturn(2000,100,400,80,90);//右90°曲がる
		  motor_pwm_off();
		  motor_pwm_on();
		  trapezoid_accel_forward(2000,100,500,500,90);//90む台形加速の関数
		}else if(hip_adjustment==1){
//		  hip_drop();
		}

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

		if(hip_adjustment==0){//条件に合わない時普通に
			  trapezoid_accel_forward(2000,500,500,100,70);//70む台形加速の関数
			  motor_pwm_off();
			  motor_pwm_on();
			  trapezoid_accel_lturn(2000,100,400,80,90);//左90°曲がる
			  motor_pwm_off();
			  motor_pwm_on();
			  trapezoid_accel_forward(2000,100,500,500,90);//90む台形加速の関数

		}else if(hip_adjustment==2){
//		  hip_drop();
		}
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
		if(hip_adjustment==0){//条件に合わない時普通に
			  trapezoid_accel_forward(2000,500,500,100,70);//90む台形加速の関数
			  motor_pwm_off();
			  motor_pwm_on();
			  trapezoid_accel_lturn(2000,100,400,80,180);//左180°曲がる
			  motor_pwm_off();
			  motor_pwm_on();
			  trapezoid_accel_forward(2000,100,500,500,90);//90む台形加速の関数
		}else if(hip_adjustment==3){
		  hip_drop();
		}
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

//スラローム付き連続足立法の判断後の行動
void slalom_continual_ver_action_based_on_direction_decision_and_coordinate_update(void){
	float control_left_sensor;
	float control_right_sensor;
	float offset_adjustment_len_rslalom;
	float offset_adjustment_len_lslalom;

	//(最小歩数判別→行動決定(least_step_judgement_and_action_decision(void)))→<動作>
	if(direction==0){//前向き
		//160 mm直進,停止
		  trapezoid_accel_forward(2000,500,500,500,160);
//		  wall_cut_detection_trapezoid_accel_forward(2000,500,500,500,160);//壁切れ補正を入れた壁制御
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
		if(hip_adjustment==0){//条件に合わない時普通に
            ///////////////////////////
//			////横壁制御
//			if((float)g_sensor[1][0]>=WALLREAD_L){//左あり
//				control_left_sensor = g_sensor[1][0];//オフセットを調整するためのもの
//				offset_adjustment_len_rslalom =  ((37-0)/(901-595))*(control_left_sensor-595)+0 ;
//				///右に曲がるとき, 左センサ(センサ値x)に対しては左向きを正として y = ((37-0)/(901-595))*(x-595)+0 　その後, yだけ左によっているので, yだけオフセットを増やす 両符号
//				///R 0:599/30:485/60:275//74:178
//				///左に曲がるとき, 右センサ(センサ値x)に対しては右向きを正として y = ((37-0)/(599-388.5))*(x-388.5)+0
//			}else{
//				offset_adjustment_len_rslalom = 0;
//			}
			/////////////////////////////

			offset_slalom_trapezoid_accel_rturn(500,10000,100,460,80,97,79);///これでいく(20mm全身で左に5度くらい傾いているからあえて回転角度を下げている感じになっている)r(500,10000,100,460,80,98,68);//r(500,10000,100,460,80,93,75)→; (500,17000,100,460,80,90);  (500,20000,100,460,80,90)
//			offset_slalom_trapezoid_accel_rturn(500,10000,100,460,80,96,83);//基本t系にoffsetを調整できるやつで
			non_wall_control_trapezoid_accel_forward(2000,500,500,500,38);//横壁制御なしver.
			//
//			slalom_trapezoid_accel_rturn(500,10000,100,460,80,91,75);//(500,10000,100,460,80,90); (500,17000,100,460,80,90);  (500,20000,100,460,80,90)
//		    //////////////////////
//			non_wall_control_trapezoid_accel_forward(2000,500,500,500,20 + offset_adjustment_len_rslalom);//横壁制御分入れた
//			/////////////////////
//			slalom_trapezoid_accel_rturn(500,17000,100,460,80,93);//(500,10000,100,460,80,90); (500,17000,100,460,80,90);  (500,20000,100,460,80,90)
//			non_wall_control_trapezoid_accel_forward(2000,500,500,500,2);
//			slalom_trapezoid_accel_rturn(350,15000,250,500,230,90);
////			step_ver_trapezoid_accel_forward(2000,500,500,100,20);
//			trapezoid_accel_forward(2000,500,500,100,20);//90む台形加速の関数(2000,500,500,100,20);
//			slalom_trapezoid_accel_rturn(300,10000,100,460,80,90);
//			trapezoid_accel_forward(2000,500,500,100,20);//90む台形加速の関数(2000,500,500,100,20);

		}else if(hip_adjustment==1){
//		  hip_drop();
		}

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

		if(hip_adjustment==0){//条件に合わない時普通に

//			/////////////////////////
//			////横壁制御
//				if((float)g_sensor[2][0]>=WALLREAD_R){//右あり
//					control_right_sensor = g_sensor[2][0];//オフセットを調整するためのもの
//					offset_adjustment_len_lslalom = ((37-0)/(599-388.5))*(control_right_sensor-388.5)+0 ;
//					///右に曲がるとき, 左センサ(センサ値x)に対しては左向きを正として y = ((37-0)/(901-595))*(x-595)+0 　その後, yだけ左によっているので, yだけオフセットを増やす 両符号
//					///R 0:599/30:485/60:275//74:178
//					///左に曲がるとき, 右センサ(センサ値x)に対しては右向きを正として y = ((37-0)/(599-388.5))*(x-388.5)+0
//				}else{
//					offset_adjustment_len_lslalom = 0;
//				}
//			/////////////////////////
			  offset_slalom_trapezoid_accel_lturn(500,10000,100,460,80,96.9,90.8);//2/17 20:16これでいく
//		      offset_slalom_trapezoid_accel_lturn(500,10000,100,460,80,96,95);//これでいく
//			  offset_slalom_trapezoid_accel_lturn(500,10000,100,460,80,91.5,108);//基本t系にoffsetを調整できるやつで
//			  slalom_trapezoid_accel_lturn(500,10000,100,460,80,91,105);//(500,10000,100,460,80,90); (500,17000,100,460,80,90);  (500,20000,100,460,80,90)
			  non_wall_control_trapezoid_accel_forward(2000,500,500,500,38);
			  ////////横壁用の前進
//			  non_wall_control_trapezoid_accel_forward(2000,500,500,500,20 + offset_adjustment_len_lslalom);
//			 ////////////
//			  slalom_trapezoid_accel_lturn(500,17000,100,460,80,93);//(500,10000,100,460,80,90); (500,17000,100,460,80,90);  (500,20000,100,460,80,90)
//			  non_wall_control_trapezoid_accel_forward(2000,500,500,500,2);
//			  slalom_trapezoid_accel_lturn(350,15000,250,500,230,90);
//			  trapezoid_accel_forward(2000,500,500,100,20);//90む台形加速の関数(2000,500,500,100,20);
//			  slalom_trapezoid_accel_lturn(300,10000,100,460,80,90);
//			  trapezoid_accel_forward(2000,500,500,100,20);//90む台形加速の関数(2000,500,500,100,20);


		}else if(hip_adjustment==2){
//		  hip_drop();
		}
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
		if(hip_adjustment==0){//条件に合わない時普通に
			  trapezoid_accel_forward(2000,500,500,100,70);//90む台形加速の関数
			  motor_pwm_off();
			  motor_pwm_on();
			  trapezoid_accel_lturn(2000,100,400,80,180);//左180°曲がる
			  motor_pwm_off();
			  motor_pwm_on();
			  trapezoid_accel_forward(2000,100,500,500,90);//90む台形加速の関数
		}else if(hip_adjustment==3){
		  hip_drop();
		}
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

//横壁制御を入れたスラローム付き連続足立法の判断後の行動
void side_add_slalom_continual_ver_action_based_on_direction_decision_and_coordinate_update(void){
	float control_left_sensor;
	float control_right_sensor;
	float offset_adjustment_len_rslalom;
	float offset_adjustment_len_lslalom;

	//(最小歩数判別→行動決定(least_step_judgement_and_action_decision(void)))→<動作>
	if(direction==0){//前向き
		//160 mm直進,停止
		  trapezoid_accel_forward(2000,500,500,500,160);
//		  wall_cut_detection_trapezoid_accel_forward(2000,500,500,500,160);//壁切れ補正を入れた壁制御
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
		if(hip_adjustment==0){//条件に合わない時普通に
            /////////////////////////
			////横壁制御
			if((float)g_sensor[1][0]>=WALLREAD_L){//左あり
				control_left_sensor = g_sensor[1][0];//オフセットを調整するためのもの
				//↓左センサーをもとに後ろオフセットを調整
				offset_adjustment_len_lslalom =  -1*(0.000001)*pow(control_left_sensor,3)+0.0022*pow(control_left_sensor,2)-1.3589*control_left_sensor+309.07 + 37 ;
//				offset_adjustment_len_rslalom =  ((37-0)/(901-595))*(control_left_sensor-595)+0 ;
				///右に曲がるとき, 左センサ(センサ値x)に対しては左向きを正として y = ((37-0)/(901-595))*(x-595)+0 　その後, yだけ左によっているので, yだけオフセットを増やす 両符号
				///R 0:599/30:485/60:275//74:178
				///左に曲がるとき, 右センサ(センサ値x)に対しては右向きを正として y = ((37-0)/(599-388.5))*(x-388.5)+0
			}else{
				offset_adjustment_len_lslalom = 0;
			}
			/////////////////////////////

			offset_slalom_trapezoid_accel_rturn(500,10000,100,460,80,97,79);///これでいく(20mm全身で左に5度くらい傾いているからあえて回転角度を下げている感じになっている)r(500,10000,100,460,80,98,68);//r(500,10000,100,460,80,93,75)→; (500,17000,100,460,80,90);  (500,20000,100,460,80,90)
//			offset_slalom_trapezoid_accel_rturn(500,10000,100,460,80,96,83);//基本t系にoffsetを調整できるやつで
			non_wall_control_trapezoid_accel_forward(2000,500,500,500,38 + offset_adjustment_len_lslalom);//横壁制御なしver.
			//
//			slalom_trapezoid_accel_rturn(500,10000,100,460,80,91,75);//(500,10000,100,460,80,90); (500,17000,100,460,80,90);  (500,20000,100,460,80,90)
//		    //////////////////////
//			non_wall_control_trapezoid_accel_forward(2000,500,500,500,20 + offset_adjustment_len_rslalom);//横壁制御分入れた
//			/////////////////////
//			slalom_trapezoid_accel_rturn(500,17000,100,460,80,93);//(500,10000,100,460,80,90); (500,17000,100,460,80,90);  (500,20000,100,460,80,90)
//			non_wall_control_trapezoid_accel_forward(2000,500,500,500,2);
//			slalom_trapezoid_accel_rturn(350,15000,250,500,230,90);
////			step_ver_trapezoid_accel_forward(2000,500,500,100,20);
//			trapezoid_accel_forward(2000,500,500,100,20);//90む台形加速の関数(2000,500,500,100,20);
//			slalom_trapezoid_accel_rturn(300,10000,100,460,80,90);
//			trapezoid_accel_forward(2000,500,500,100,20);//90む台形加速の関数(2000,500,500,100,20);

		}else if(hip_adjustment==1){
//		  hip_drop();
		}

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

		if(hip_adjustment==0){//条件に合わない時普通に

//			/////////////////////////
			////横壁制御
				if((float)g_sensor[2][0]>=WALLREAD_R){//右あり
					control_right_sensor = g_sensor[2][0];//オフセットを調整するためのもの
					offset_adjustment_len_rslalom =  -7*(0.000001)*pow(control_left_sensor,3)+0.0068*pow(control_left_sensor,2)-2.4156*control_left_sensor+321.2 + 37 ;
					///右に曲がるとき, 左センサ(センサ値x)に対しては左向きを正として y = ((37-0)/(901-595))*(x-595)+0 　その後, yだけ左によっているので, yだけオフセットを増やす 両符号
					///R 0:599/30:485/60:275//74:178
					///左に曲がるとき, 右センサ(センサ値x)に対しては右向きを正として y = ((37-0)/(599-388.5))*(x-388.5)+0
				}else{
					offset_adjustment_len_rslalom= 0;
				}
//			/////////////////////////
			  offset_slalom_trapezoid_accel_lturn(500,10000,100,460,80,96.9,90.8);//2/17 20:16これでいく
//		      offset_slalom_trapezoid_accel_lturn(500,10000,100,460,80,96,95);//これでいく
//			  offset_slalom_trapezoid_accel_lturn(500,10000,100,460,80,91.5,108);//基本t系にoffsetを調整できるやつで
//			  slalom_trapezoid_accel_lturn(500,10000,100,460,80,91,105);//(500,10000,100,460,80,90); (500,17000,100,460,80,90);  (500,20000,100,460,80,90)
			  ////////横壁用の前進
			  non_wall_control_trapezoid_accel_forward(2000,500,500,500,38 + offset_adjustment_len_rslalom);
			  ////////横壁用の前進
//			  non_wall_control_trapezoid_accel_forward(2000,500,500,500,20 + offset_adjustment_len_lslalom);
//			 ////////////
//			  slalom_trapezoid_accel_lturn(500,17000,100,460,80,93);//(500,10000,100,460,80,90); (500,17000,100,460,80,90);  (500,20000,100,460,80,90)
//			  non_wall_control_trapezoid_accel_forward(2000,500,500,500,2);
//			  slalom_trapezoid_accel_lturn(350,15000,250,500,230,90);
//			  trapezoid_accel_forward(2000,500,500,100,20);//90む台形加速の関数(2000,500,500,100,20);
//			  slalom_trapezoid_accel_lturn(300,10000,100,460,80,90);
//			  trapezoid_accel_forward(2000,500,500,100,20);//90む台形加速の関数(2000,500,500,100,20);


		}else if(hip_adjustment==2){
//		  hip_drop();
		}
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
		if(hip_adjustment==0){//条件に合わない時普通に
			  trapezoid_accel_forward(2000,500,500,100,70);//90む台形加速の関数
			  motor_pwm_off();
			  motor_pwm_on();
			  trapezoid_accel_lturn(2000,100,400,80,180);//左180°曲がる
			  motor_pwm_off();
			  motor_pwm_on();
			  trapezoid_accel_forward(2000,100,500,500,90);//90む台形加速の関数
		}else if(hip_adjustment==3){
		  hip_drop();
		}
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


//左手法
void left_hand_method(void){
	  motor_excitation_on();//励磁スタート
	  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
	  //初期化
	  x=0;
	  y=0;
	  z=0;
	  column[0]=0b1000000000000000;
	  row[0]=0b0000000000000000;

	  x=x;
	  y=y+1;
	  z=z;//　最初(0,1)から出発、(この時、(0,0.5)あたりにいるが一旦(0,0)にいたとみなす)北向き
	  //
	  //座標(いたマスの座標→センサーの捉える壁の座標)と方角更新→壁情報更新(if文で方角によって座標→センサーの手筈を決める)
	  //動作(動作直前の条件分布は何も見ないこと)によって座標だけ更新1マスしか進まない//動作によって方角と座標が1つ変わる(直進なら変わらないが)
	  while(1){
//ここにゴール座標入れる！！に来たら90前進して停止
		if(x==goal_x&&y==goal_y){
			trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
			HAL_Delay(1000);//1秒経ってから励磁解除
			motor_excitation_on();//励磁ストップ
		 break;//無限ループから脱出
		 }
	  if((float)g_sensor[1][0]<WALLREAD_L){//[1][0]は、左壁, [2][0]は、右壁 //ここは左壁無し
	  //90 mm直進,90°左旋回,90 mm直進,停止
	  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
	  trapezoid_accel_lturn(2000,100,400,80,90);//左90°曲がる
	  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
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
	  if(z==3){//西に対して、、
		  x=x;
		  y=y-1;
		  if(z==-1){
			  z=3;
		  }
	  }
	  //向き更新
		  z=z-1;
	  }else if(g_sensor[0][0]<WALLREAD_FL&&g_sensor[3][0]<WALLREAD_FR){//前壁なし
	  //180 mm直進,停止
	  trapezoid_accel_forward(2000,100,500,100,180);
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
	  if(z==3){//西に対して、、
		  x=x-1;
		  y=y;
		  if(z==-1){
			  z=3;
		  }
	  }
	  //向き更新
		  z=z;
	  }else if((float)g_sensor[2][0]<WALLREAD_R){//右壁がない
//					g_WallControlStatus=g_WallControlStatus|(1<<1);//2bit目を1にする修正済
	  //90 mm直進,90°右旋回,90 mm直進,停止
	  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
	  trapezoid_accel_rturn(2000,100,400,80,90);//左90°曲がる
	  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
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
	  if(z==3){//西に対して、、
		  x=x;
		  y=y+1;
		  if(z==-1){
			  z=3;
		  }
	  }
	  //向き更新
		  z=z+1;

	  }else{
//					g_WallControlStatus=g_WallControlStatus&~(1<<1);//2bit目を0にする
	  //90前進,180°左旋回,90前進,停止
	  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
	  trapezoid_accel_lturn(2000,100,400,80,180);//左180°曲がる
	  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
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

	  //方角によってどう変わるか//壁情報はこれから入るマスで(x,y)今までいた座標によって決まる//z=0,1,2,3
	  if(z==0){//(x,y),北に対して、、
		  //左のセンサー
		  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
			  if(x>0){
			  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする//0bit目から数えることとする　要は1番目と呼んでいるのを0bit目として判断
			  }
		  }else{//左あり
			  if(x>0){
			  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
			  }
		  }
		  //前のセンサー
		  if(g_sensor[0][0]<WALLREAD_FL&&g_sensor[3][0]<WALLREAD_FR){//前なし
			  row[y]=row[y]&~(1<<(x));//xビット目を0にする
		  }else{//前あり
			  row[y]=row[y]|(1<<(x));//xを1にする
		  }
		  //右のセンサー
		  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
			  column[x]=column[x]&~(1<<(y));//yビット目を0にする
		  }else{//右あり
			  column[x]=column[x]|(1<<(y));//yビット目を1にする
		  }

		  if(z==4){
			  z=0;
		  }
	  }else if(z==1){//(x,y),東に対して、、
		  //左のセンサー
		  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
			  row[y]=row[y]&~(1<<(x));//xビット目を0にする
		  }else{//左あり
			  row[y]=row[y]|(1<<(x));//xを1にする
		  }
		  //前のセンサー
		  if(g_sensor[0][0]<WALLREAD_FL&&g_sensor[3][0]<WALLREAD_FR){//右なし

			  column[x]=column[x]&~(1<<(y));//yビット目を0にする
		  }else{//右あり
			  column[x]=column[x]|(1<<(y));//yビット目を1にする
		  }
		  //右のセンサー
		  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
			  if(y>0){
			  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
			  }
		  }else{//右あり
			  if(y>0){
			  row[y-1]=row[y-1]|(1<<(x));//xを1にする
			  }
		  }

	  }else if(z==2){//(x,y),南に対して、、
		  //左のセンサー
		  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
			  column[x]=column[x]&~(1<<(y));//yビット目を0にする
		  }else{//左あり
			  column[x]=column[x]|(1<<(y));//yビット目を1にする
		  }
		  //前のセンサー
		  if(g_sensor[0][0]<WALLREAD_FL&&g_sensor[3][0]<WALLREAD_FR){//前なし
			  if(y>0){
			  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
			  }
		  }else{//前あり
			  if(y>0){
			  row[y-1]=row[y-1]|(1<<(x));//xを1にする
			  }
		  }
		  //右のセンサー
		  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
			  if(x>0){
			  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
			  }
		  }else{//右あり
			  if(x>0){
			  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
			  }
		  }
	  }else if(z==3||z==-1){//(x,y),西に対して、、
		  //左のセンサー
		  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
			  if(y>0){
			  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
			  }
		  }else{//左あり
			  if(y>0){
			  row[y-1]=row[y-1]|(1<<(x));//xを1にする
			  }
		  }
		  //前のセンサー
		  if(g_sensor[0][0]<WALLREAD_FL&&g_sensor[3][0]<WALLREAD_FR){//右なし
			  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
		  }else{//右あり
			  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
		  }
		  //右のセンサー
		  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
			  row[y]=row[y]&~(1<<(x));//xビット目を0にする
		  }else{//右あり
			  row[y]=row[y]|(1<<(x));//xを1にする
		  }
		  if(z==-1){
			  z=3;
		  }
	  }

	  }
	  step_number();
	  printf("歩数マップ作成時x=%d,y=%d,z=%d\n\r",x,y,z);
	  Print_Wall_2();
	  }
}



//左手法に壁情報
//左手法
void left_hand_method_2(void){
      motor_excitation_on();//励磁スタート
	  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
	  //初期化
	  x=0;
	  y=0;
	  z=0;
	  //一回全部壁をつけてみる
	  column[0]=0b1000000000000000;
	  row[0]=0b0000000000000000;

	  x=x;
	  y=y+1;
	  z=z;//　最初(0,1)から出発、(この時、(0,0.5)あたりにいるが一旦(0,0)にいたとみなす)北向き
	  //
	  //座標(いたマスの座標→センサーの捉える壁の座標)と方角更新→壁情報更新(if文で方角によって座標→センサーの手筈を決める)
	  //動作(動作直前の条件分布は何も見ないこと)によって座標だけ更新1マスしか進まない//動作によって方角と座標が1つ変わる(直進なら変わらないが)
	  while(1){
//ここにゴール座標入れる！！に来たら90前進して停止
		if(x==goal_x&&y==goal_y){
			trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
			HAL_Delay(1000);//1秒経ってから励磁解除
			motor_excitation_off();//励磁ストップ
		 break;//無限ループから脱出
		 }

		 //①一番外側いる時
				if((x==0)){//①-1最左壁一列
					//北側を見ている  左センサーを除く
					 if(z==0){//(x,y),北に対して、、
						  //前のセンサー
						  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//前なし
							  row[y]=row[y]&~(1<<(x));//xビット目を0にする
						  }else{//前あり
							  row[y]=row[y]|(1<<(x));//xを1にする
						  }
						  //右のセンサー
						  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
							  column[x]=column[x]&~(1<<(y));//yビット目を0にする
						  }else{//右あり
							  column[x]=column[x]|(1<<(y));//yビット目を1にする
						  }

						  if(z==4){
							  z=0;
						  }
					 }
					//東側を見ている　全センサーを見てよし
						else if(z==1){
							//左のセンサー
						  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
							  row[y]=row[y]&~(1<<(x));//xビット目を0にする
						  }else{//左あり
							  row[y]=row[y]|(1<<(x));//xを1にする
						  }
						  //前のセンサー
						  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//右なし

							  column[x]=column[x]&~(1<<(y));//yビット目を0にする
						  }else{//右あり
							  column[x]=column[x]|(1<<(y));//yビット目を1にする
						  }
						  //右のセンサー
						  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
							  if(y>0){
							  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
							  }
						  }else{//右あり
							  if(y>0){
							  row[y-1]=row[y-1]|(1<<(x));//xを1にする
							  }
						  }
						}
					//南側を見ている  右センサーを除く
						else if(z==2){//(x,y),南に対して、、
						  //左のセンサー
						  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
							  column[x]=column[x]&~(1<<(y));//yビット目を0にする
						  }else{//左あり
							  column[x]=column[x]|(1<<(y));//yビット目を1にする
						  }
						  //前のセンサー
						  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//前なし
							  if(y>0){
							  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
							  }
						  }else{//前あり
							  if(y>0){
							  row[y-1]=row[y-1]|(1<<(x));//xを1にする
							  }
						  }
						}
					 //西側を見ている  前センサーを除く
					   else if(z==3){//(x,y),西に対して、、
						  //}else if((z==3)||(z==-1)){//(x,y),西に対して、、
						  //左のセンサー
						  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
							  if(y>0){
							  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
							  }
						  }else{//左あり
							  if(y>0){
							  row[y-1]=row[y-1]|(1<<(x));//xを1にする
							  }
						  }
						  //右のセンサー
						  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
							  row[y]=row[y]&~(1<<(x));//xビット目を0にする
						  }else{//右あり
							  row[y]=row[y]|(1<<(x));//xを1にする
						  }
						  if(z==-1){
							  z=3;
						  }
						}

				}else if(y==15){//①-2最前壁一列
					if(z==0){
					//北側を見ている　前センサーを除く
					      //左のセンサー
						  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
							  if(x>0){
							  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする//0bit目から数えることとする　要は1番目と呼んでいるのを0bit目として判断
							  }
						  }else{//左あり
							  if(x>0){
							  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
							  }
						  }
						  //右のセンサー
						  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
							  column[x]=column[x]&~(1<<(y));//yビット目を0にする
						  }else{//右あり
							  column[x]=column[x]|(1<<(y));//yビット目を1にする
						  }

						  if(z==4){
							  z=0;
						  }
					}
					//東側を見ている　左センサーを除く
					else if(z==1){
						  //前のセンサー
						  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//右なし

							  column[x]=column[x]&~(1<<(y));//yビット目を0にする
						  }else{//右あり
							  column[x]=column[x]|(1<<(y));//yビット目を1にする
						  }
						  //右のセンサー
						  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
							  if(y>0){
							  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
							  }
						  }else{//右あり
							  if(y>0){
							  row[y-1]=row[y-1]|(1<<(x));//xを1にする
							  }
						  }
						}
					//南側を見ている　全センサーOK
					else if(z==2){//(x,y),南に対して、、
						  //左のセンサー
						  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
							  column[x]=column[x]&~(1<<(y));//yビット目を0にする
						  }else{//左あり
							  column[x]=column[x]|(1<<(y));//yビット目を1にする
						  }
						  //前のセンサー
						  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//前なし
							  if(y>0){
							  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
							  }
						  }else{//前あり
							  if(y>0){
							  row[y-1]=row[y-1]|(1<<(x));//xを1にする
							  }
						  }
						  //右のセンサー
						  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
							  if(x>0){
							  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
							  }
						  }else{//右あり
							  if(x>0){
							  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
							  }
						  }
					  }
					//西側を見ている　右除く
					else if(z==3){//(x,y),西に対して、、
						  //}else if((z==3)||(z==-1)){//(x,y),西に対して、、
						  //左のセンサー
						  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
							  if(y>0){
							  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
							  }
						  }else{//左あり
							  if(y>0){
							  row[y-1]=row[y-1]|(1<<(x));//xを1にする
							  }
						  }
						  //前のセンサー
						  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//右なし
							  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
						  }else{//右あり
							  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
						  }
						  if(z==-1){
							  z=3;
						  }
						}
				    }else if(x==15){//①-3最右壁一列
					//北側を見ている　右センサーを除く
					if(z==0){
					      //左のセンサー
						  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
							  if(x>0){
							  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする//0bit目から数えることとする　要は1番目と呼んでいるのを0bit目として判断
							  }
						  }else{//左あり
							  if(x>0){
							  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
							  }
						  }
						  //前のセンサー
						  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//前なし
							  row[y]=row[y]&~(1<<(x));//xビット目を0にする
						  }else{//前あり
							  row[y]=row[y]|(1<<(x));//xを1にする
						  }
						  if(z==4){
							  z=0;
						  }
					}

					//東側を見ている　前センサーを除く
					else if(z==1){//(x,y),東に対して、、
						  //左のセンサー
						  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
							  row[y]=row[y]&~(1<<(x));//xビット目を0にする
						  }else{//左あり
							  row[y]=row[y]|(1<<(x));//xを1にする
						  }
						  //右のセンサー
						  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
							  if(y>0){
							  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
							  }
						  }else{//右あり
							  if(y>0){
							  row[y-1]=row[y-1]|(1<<(x));//xを1にする
							  }
						  }
					  }
					//南側を見ている　左センサーを除く
					else if(z==2){//(x,y),南に対して、、
						  //前のセンサー
						  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//前なし
							  if(y>0){
							  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
							  }
						  }else{//前あり
							  if(y>0){
							  row[y-1]=row[y-1]|(1<<(x));//xを1にする
							  }
						  }
						  //右のセンサー
						  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
							  if(x>0){
							  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
							  }
						  }else{//右あり
							  if(x>0){
							  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
							  }
						  }
					  }
					//西側を見ている　全センサーを見てよし
					else if(z==3){//(x,y),西に対して、、
						  //}else if((z==3)||(z==-1)){//(x,y),西に対して、、
						  //左のセンサー
						  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
							  if(y>0){
							  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
							  }
						  }else{//左あり
							  if(y>0){
							  row[y-1]=row[y-1]|(1<<(x));//xを1にする
							  }
						  }
						  //前のセンサー
						  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//右なし
							  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
						  }else{//右あり
							  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
						  }
						  //右のセンサー
						  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
							  row[y]=row[y]&~(1<<(x));//xビット目を0にする
						  }else{//右あり
							  row[y]=row[y]|(1<<(x));//xを1にする
						  }
						  if(z==-1){
							  z=3;
						  }
						}
				}else if(y==0){//①-4最後壁一列
					//北側を見ている　全よし
			       if(z==0){
					      //左のセンサー
						  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
							  if(x>0){
							  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする//0bit目から数えることとする　要は1番目と呼んでいるのを0bit目として判断
							  }
						  }else{//左あり
							  if(x>0){
							  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
							  }
						  }
						  //前のセンサー
						  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//前なし
							  row[y]=row[y]&~(1<<(x));//xビット目を0にする
						  }else{//前あり
							  row[y]=row[y]|(1<<(x));//xを1にする
						  }
						  //右のセンサー
						  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
							  column[x]=column[x]&~(1<<(y));//yビット目を0にする
						  }else{//右あり
							  column[x]=column[x]|(1<<(y));//yビット目を1にする
						  }

						  if(z==4){
							  z=0;
						  }
					//東側を見ている　右イラン
				}else if(z==1){//(x,y),東に対して、、
						  //左のセンサー
						  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
							  row[y]=row[y]&~(1<<(x));//xビット目を0にする
						  }else{//左あり
							  row[y]=row[y]|(1<<(x));//xを1にする
						  }
						  //前のセンサー
						  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//なし

							  column[x]=column[x]&~(1<<(y));//yビット目を0にする
						  }else{//あり
							  column[x]=column[x]|(1<<(y));//yビット目を1にする
						  }
					  }
					//南側を見ている　前なし
				else if(z==2){//(x,y),南に対して、、
						  //左のセンサー
						  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
							  column[x]=column[x]&~(1<<(y));//yビット目を0にする
						  }else{//左あり
							  column[x]=column[x]|(1<<(y));//yビット目を1にする
						  }
						  //右のセンサー
						  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
							  if(x>0){
							  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
							  }
						  }else{//右あり
							  if(x>0){
							  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
							  }
						  }
					  }
					//西側を見ている　左なし
				else if(z==3){//(x,y),西に対して、、
						  //}else if((z==3)||(z==-1)){//(x,y),西に対して、、
						  //前のセンサー
						  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//右なし
							  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
						  }else{//右あり
							  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
						  }
						  //右のセンサー
						  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
							  row[y]=row[y]&~(1<<(x));//xビット目を0にする
						  }else{//右あり
							  row[y]=row[y]|(1<<(x));//xを1にする
						  }
						  if(z==-1){
							  z=3;
						  }
						}
				}

				//②一番外側にいないとき//以下ではx>0とかの条件はいらんと思うけど、、まあ必要あれば除くって感じで
				else{

			    if(z==0){//(x,y),北に対して、、

			      //左のセンサー
				  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
					  if(x>0){
					  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする//0bit目から数えることとする　要は1番目と呼んでいるのを0bit目として判断
					  }
				  }else{//左あり
					  if(x>0){
					  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
					  }
				  }
				  //前のセンサー
				  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//前なし
					  row[y]=row[y]&~(1<<(x));//xビット目を0にする
				  }else{//前あり
					  row[y]=row[y]|(1<<(x));//xを1にする
				  }
				  //右のセンサー
				  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
					  column[x]=column[x]&~(1<<(y));//yビット目を0にする
				  }else{//右あり
					  column[x]=column[x]|(1<<(y));//yビット目を1にする
				  }

				  if(z==4){
					  z=0;
				  }
			  }else if(z==1){//(x,y),東に対して、、
				  //左のセンサー
				  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
					  row[y]=row[y]&~(1<<(x));//xビット目を0にする
				  }else{//左あり
					  row[y]=row[y]|(1<<(x));//xを1にする
				  }
				  //前のセンサー
				  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//右なし

					  column[x]=column[x]&~(1<<(y));//yビット目を0にする
				  }else{//右あり
					  column[x]=column[x]|(1<<(y));//yビット目を1にする
				  }
				  //右のセンサー
				  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
					  if(y>0){
					  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
					  }
				  }else{//右あり
					  if(y>0){
					  row[y-1]=row[y-1]|(1<<(x));//xを1にする
					  }
				  }

			  }else if(z==2){//(x,y),南に対して、、
				  //左のセンサー
				  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
					  column[x]=column[x]&~(1<<(y));//yビット目を0にする
				  }else{//左あり
					  column[x]=column[x]|(1<<(y));//yビット目を1にする
				  }
				  //前のセンサー
				  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//前なし
					  if(y>0){
					  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
					  }
				  }else{//前あり
					  if(y>0){
					  row[y-1]=row[y-1]|(1<<(x));//xを1にする
					  }
				  }
				  //右のセンサー
				  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
					  if(x>0){
					  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
					  }
				  }else{//右あり
					  if(x>0){
					  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
					  }
				  }
			  }else if(z==3){//(x,y),西に対して、、
				  //}else if((z==3)||(z==-1)){//(x,y),西に対して、、
				  //左のセンサー
				  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
					  if(y>0){
					  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
					  }
				  }else{//左あり
					  if(y>0){
					  row[y-1]=row[y-1]|(1<<(x));//xを1にする
					  }
				  }
				  //前のセンサー
				  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//右なし
					  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
				  }else{//右あり
					  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
				  }
				  //右のセンサー
				  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
					  row[y]=row[y]&~(1<<(x));//xビット目を0にする
				  }else{//右あり
					  row[y]=row[y]|(1<<(x));//xを1にする
				  }
				  if(z==-1){
					  z=3;
				  }
				}
				}




	  if((float)g_sensor[1][0]<WALLREAD_L){//[1][0]は、左壁, [2][0]は、右壁 //ここは左壁無し
	  //90 mm直進,90°左旋回,90 mm直進,停止
	  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
	  trapezoid_accel_lturn(2000,100,400,80,90);//左90°曲がる
	  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
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
	  if(z==3){//西に対して、、
		  x=x;
		  y=y-1;
		  if(z==-1){
			  z=3;
		  }
	  }
	  //向き更新
		  z=z-1;
	  }else if(g_sensor[0][0]<WALLREAD_FL&&g_sensor[3][0]<WALLREAD_FR){//前壁なし
	  //180 mm直進,停止
	  trapezoid_accel_forward(2000,100,500,100,180);
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
	  if(z==3){//西に対して、、
		  x=x-1;
		  y=y;
		  if(z==-1){
			  z=3;
		  }
	  }
	  //向き更新
		  z=z;
	  }else if((float)g_sensor[2][0]<WALLREAD_R){//右壁がない
//					g_WallControlStatus=g_WallControlStatus|(1<<1);//2bit目を1にする修正済
	  //90 mm直進,90°右旋回,90 mm直進,停止
	  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
	  trapezoid_accel_rturn(2000,100,400,80,90);//左90°曲がる
	  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
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
	  if(z==3){//西に対して、、
		  x=x;
		  y=y+1;
		  if(z==-1){
			  z=3;
		  }
	  }
	  //向き更新
		  z=z+1;

	  }else{
//					g_WallControlStatus=g_WallControlStatus&~(1<<1);//2bit目を0にする
	  //90前進,180°左旋回,90前進,停止
	  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
	  trapezoid_accel_lturn(2000,100,400,80,180);//左180°曲がる
	  trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
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

	  //方角によってどう変わるか//壁情報はこれから入るマスで(x,y)今までいた座標によって決まる//z=0,1,2,3
	  if(z==0){//(x,y),北に対して、、
		  //左のセンサー
		  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
			  if(x>0){
			  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする//0bit目から数えることとする　要は1番目と呼んでいるのを0bit目として判断
			  }
		  }else{//左あり
			  if(x>0){
			  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
			  }
		  }
		  //前のセンサー
		  if(g_sensor[0][0]<WALLREAD_FL&&g_sensor[3][0]<WALLREAD_FR){//前なし
			  row[y]=row[y]&~(1<<(x));//xビット目を0にする
		  }else{//前あり
			  row[y]=row[y]|(1<<(x));//xを1にする
		  }
		  //右のセンサー
		  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
			  column[x]=column[x]&~(1<<(y));//yビット目を0にする
		  }else{//右あり
			  column[x]=column[x]|(1<<(y));//yビット目を1にする
		  }

		  if(z==4){
			  z=0;
		  }
	  }else if(z==1){//(x,y),東に対して、、
		  //左のセンサー
		  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
			  row[y]=row[y]&~(1<<(x));//xビット目を0にする
		  }else{//左あり
			  row[y]=row[y]|(1<<(x));//xを1にする
		  }
		  //前のセンサー
		  if(g_sensor[0][0]<WALLREAD_FL&&g_sensor[3][0]<WALLREAD_FR){//右なし

			  column[x]=column[x]&~(1<<(y));//yビット目を0にする
		  }else{//右あり
			  column[x]=column[x]|(1<<(y));//yビット目を1にする
		  }
		  //右のセンサー
		  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
			  if(y>0){
			  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
			  }
		  }else{//右あり
			  if(y>0){
			  row[y-1]=row[y-1]|(1<<(x));//xを1にする
			  }
		  }

	  }else if(z==2){//(x,y),南に対して、、
		  //左のセンサー
		  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
			  column[x]=column[x]&~(1<<(y));//yビット目を0にする
		  }else{//左あり
			  column[x]=column[x]|(1<<(y));//yビット目を1にする
		  }
		  //前のセンサー
		  if(g_sensor[0][0]<WALLREAD_FL&&g_sensor[3][0]<WALLREAD_FR){//前なし
			  if(y>0){
			  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
			  }
		  }else{//前あり
			  if(y>0){
			  row[y-1]=row[y-1]|(1<<(x));//xを1にする
			  }
		  }
		  //右のセンサー
		  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
			  if(x>0){
			  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
			  }
		  }else{//右あり
			  if(x>0){
			  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
			  }
		  }
	  }else if(z==3||z==-1){//(x,y),西に対して、、
		  //左のセンサー
		  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
			  if(y>0){
			  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
			  }
		  }else{//左あり
			  if(y>0){
			  row[y-1]=row[y-1]|(1<<(x));//xを1にする
			  }
		  }
		  //前のセンサー
		  if(g_sensor[0][0]<WALLREAD_FL&&g_sensor[3][0]<WALLREAD_FR){//右なし
			  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
		  }else{//右あり
			  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
		  }
		  //右のセンサー
		  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
			  row[y]=row[y]&~(1<<(x));//xビット目を0にする
		  }else{//右あり
			  row[y]=row[y]|(1<<(x));//xを1にする
		  }
		  if(z==-1){
			  z=3;
		  }
	  }

	  }
	  step_number();
	  printf("歩数マップ作成時x=%d,y=%d,z=%d\n\r",x,y,z);
	  Print_Wall_2();
	  }
}

//関数化した一番外側にいるときの壁情報の処理, 加えて, 区画の境界にいるときに壁情報の取得諸々
void wall_sensor(){//最短走行用で探索済の壁を記録するっていうコードも入れる
	 //①一番外側いる時//歩数マップ作成時に壁がない方と同じ歩数になりがちなので、外壁沿いで壁があったらその奥を255にする操作を入れてみる
			if((x==0)){//①-1最左壁一列
				//北側を見ている  左センサーを除く
				 if(z==0){//(x,y),北に対して、、
					  //前のセンサー
					  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//前なし
						  row[y]=row[y]&~(1<<(x));//xビット目を0にする
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました
					  }else{//前あり
						  row[y]=row[y]|(1<<(x));//xを1にする
						  Dist_map[x][y+1]=255;//壁のある奥を255歩にする
						  Front_wall=255;
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました
					  }
					  //右のセンサー
					  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
						  column[x]=column[x]&~(1<<(y));//yビット目を0にする
						  //最短走行用
						  short_column[x]=short_column[x]|(1<<(y));//探索しました
					  }else{//右あり
						  column[x]=column[x]|(1<<(y));//yビット目を1にする
						  Dist_map[x+1][y]=255;//壁のある奥を255歩にする
						  Right_wall=255;
						  //最短走行用
						  short_column[x]=short_column[x]|(1<<(y));//探索しました用
					  }

					  if(z==4){
						  z=0;
					  }
				 }
				//東側を見ている　全センサーを見てよし
					else if(z==1){
						//左のセンサー
					  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
						  row[y]=row[y]&~(1<<(x));//xビット目を0にする
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました

					  }else{//左あり
						  row[y]=row[y]|(1<<(x));//xを1にする
						  Dist_map[x][y+1]=255;
						  Left_wall=255;
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました
					  }
					  //前のセンサー
					  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//右なし

						  column[x]=column[x]&~(1<<(y));//yビット目を0にする
						  //最短走行用
						  short_column[x]=short_column[x]|(1<<(y));//探索しました用
					  }else{//前あり
						  column[x]=column[x]|(1<<(y));//yビット目を1にする
						  Dist_map[x+1][y]=255;//壁のある奥を255歩にする
						  Front_wall=255;
						  //最短走行用
						  short_column[x]=short_column[x]|(1<<(y));//探索しました用
					  }
					  //右のセンサー
					  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
						  if(y>0){
						  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
						  //最短走行用
						  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
						  }
					  }else{//右あり
						  if(y>0){
						  row[y-1]=row[y-1]|(1<<(x));//xを1にする
						  Dist_map[x][y-1]=255;
						  Right_wall=255;
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました
						  }
					  }
					}
				//南側を見ている  右センサーを除く
					else if(z==2){//(x,y),南に対して、、
					  //左のセンサー
					  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
						  column[x]=column[x]&~(1<<(y));//yビット目を0にする
						  //最短走行用
						  short_column[x]=short_column[x]|(1<<(y));//探索しました用
					  }else{//左あり
						  column[x]=column[x]|(1<<(y));//yビット目を1にする
						  Dist_map[x+1][y]=255;
						  Left_wall=255;
						  //最短走行用
						  short_column[x]=short_column[x]|(1<<(y));//探索しました用
					  }
					  //前のセンサー
					  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//前なし
						  if(y>0){
						  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
						  //最短走行用
						  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
						  }
					  }else{//前あり
						  if(y>0){
						  row[y-1]=row[y-1]|(1<<(x));//xを1にする
						  Dist_map[x][y-1]=255;
						  Front_wall=255;
						  //最短走行用
						  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
						  }
					  }
					}
				 //西側を見ている  前センサーを除く
				   else if(z==3){//(x,y),西に対して、、
					  //}else if((z==3)||(z==-1)){//(x,y),西に対して、、
					  //左のセンサー
					  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
						  if(y>0){
						  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
						  //最短走行用
						  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
						  }
					  }else{//左あり
						  if(y>0){
						  row[y-1]=row[y-1]|(1<<(x));//xを1にする
						  Dist_map[x][y-1]=255;
						  Left_wall=255;
						  //最短走行用
						  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
						  }
					  }
					  //右のセンサー
					  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
						  row[y]=row[y]&~(1<<(x));//xビット目を0にする
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました
					  }else{//右あり
						  row[y]=row[y]|(1<<(x));//xを1にする
						  Dist_map[x][y+1]=255;
						  Right_wall=255;
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました
					  }
					  if(z==-1){
						  z=3;
					  }
					}

			}else if(y==15){//①-2最前壁一列
				if(z==0){
				//北側を見ている　前センサーを除く
				      //左のセンサー
					  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
						  if(x>0){
						  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする//0bit目から数えることとする　要は1番目と呼んでいるのを0bit目として判断
						  //最短走行用
						  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
						  }
					  }else{//左あり
						  if(x>0){
						  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
						  Dist_map[x-1][y]=255;
						  Left_wall=255;
						  //最短走行用
						  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
						  }
					  }
					  //右のセンサー
					  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
						  column[x]=column[x]&~(1<<(y));//yビット目を0にする
						  //最短走行用
						  short_column[x]=short_column[x]|(1<<(y));//探索しました用
					  }else{//右あり
						  column[x]=column[x]|(1<<(y));//yビット目を1にする
						  Dist_map[x+1][y]=255;
						  Right_wall=255;
						  //最短走行用
						  short_column[x]=short_column[x]|(1<<(y));//探索しました用
					  }

					  if(z==4){
						  z=0;
					  }
				}
				//東側を見ている　左センサーを除く
				else if(z==1){
					  //前のセンサー
					  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//右なし

						  column[x]=column[x]&~(1<<(y));//yビット目を0にする
						  //最短走行用
						  short_column[x]=short_column[x]|(1<<(y));//探索しました用
					  }else{//前あり
						  column[x]=column[x]|(1<<(y));//yビット目を1にする
						  Dist_map[x+1][y]=255;
						  Front_wall=255;
						  //最短走行用
						  short_column[x]=short_column[x]|(1<<(y));//探索しました用

					  }
					  //右のセンサー
					  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
						  if(y>0){
						  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
						  //最短走行用
						  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
						  }
					  }else{//右あり
						  if(y>0){
						  row[y-1]=row[y-1]|(1<<(x));//xを1にする
						  Dist_map[x][y-1]=255;
						  Right_wall=255;
						  //最短走行用
						  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
						  }
					  }
					}
				//南側を見ている　全センサーOK
				else if(z==2){//(x,y),南に対して、、
					  //左のセンサー
					  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
						  column[x]=column[x]&~(1<<(y));//yビット目を0にする
						  //最短走行用
						  short_column[x]=short_column[x]|(1<<(y));//探索しました用
					  }else{//左あり
						  column[x]=column[x]|(1<<(y));//yビット目を1にする
						  Dist_map[x+1][y]=255;
						  Left_wall=255;
						  //最短走行用
						  short_column[x]=short_column[x]|(1<<(y));//探索しました用
					  }
					  //前のセンサー
					  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//前なし
						  if(y>0){
						  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
						  //最短走行用
						  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
						  }
					  }else{//前あり
						  if(y>0){
						  row[y-1]=row[y-1]|(1<<(x));//xを1にする
						  Dist_map[x][y-1]=255;
						  Front_wall=255;
						  //最短走行用
						  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
						  }
					  }
					  //右のセンサー
					  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
						  if(x>0){
						  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
						  //最短走行用
						  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
						  }
					  }else{//右あり
						  if(x>0){
						  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
						  Dist_map[x-1][y]=255;
						  Right_wall=255;
						  //最短走行用
						  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
						  }
					  }
				  }
				//西側を見ている　右除く
				else if(z==3){//(x,y),西に対して、、
					  //}else if((z==3)||(z==-1)){//(x,y),西に対して、、
					  //左のセンサー
					  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
						  if(y>0){
						  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
						  //最短走行用
						  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
						  }
					  }else{//左あり
						  if(y>0){
						  row[y-1]=row[y-1]|(1<<(x));//xを1にする
						  Dist_map[x][y-1]=255;
						  Left_wall=255;
						  //最短走行用
						  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
						  }
					  }
					  //前のセンサー
					  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//右なし
						  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
						  //最短走行用
						  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
					  }else{//前あり
						  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
						  Dist_map[x-1][y]=255;
						  Front_wall=255;
						  //最短走行用
						  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
					  }
					  if(z==-1){
						  z=3;
					  }
					}
			    }else if(x==15){//①-3最右壁一列
				//北側を見ている　右センサーを除く
				if(z==0){
				      //左のセンサー
					  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
						  if(x>0){
						  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする//0bit目から数えることとする　要は1番目と呼んでいるのを0bit目として判断
						  //最短走行用
						  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
						  }
					  }else{//左あり
						  if(x>0){
						  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
						  Dist_map[x-1][y]=255;
						  Left_wall=255;
						  //最短走行用
						  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
						  }
					  }
					  //前のセンサー
					  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//前なし
						  row[y]=row[y]&~(1<<(x));//xビット目を0にする
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました
					  }else{//前あり
						  row[y]=row[y]|(1<<(x));//xを1にする
						  Dist_map[x][y+1]=255;
						  Front_wall=255;
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました
					  }
					  if(z==4){
						  z=0;
					  }
				}

				//東側を見ている　前センサーを除く
				else if(z==1){//(x,y),東に対して、、
					  //左のセンサー
					  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
						  row[y]=row[y]&~(1<<(x));//xビット目を0にする
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました
					  }else{//左あり
						  row[y]=row[y]|(1<<(x));//xを1にする
						  Dist_map[x][y+1]=255;
						  Left_wall=255;
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました
					  }
					  //右のセンサー
					  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
						  if(y>0){
						  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
						  //最短走行用
						  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
						  }
					  }else{//右あり
						  if(y>0){
						  row[y-1]=row[y-1]|(1<<(x));//xを1にする
						  Dist_map[x][y-1]=255;
						  Right_wall=255;
						  //最短走行用
						  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
						  }
					  }
				  }
				//南側を見ている　左センサーを除く
				else if(z==2){//(x,y),南に対して、、
					  //前のセンサー
					  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//前なし
						  if(y>0){
						  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
						  //最短走行用
						  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
						  }
					  }else{//前あり
						  if(y>0){
						  row[y-1]=row[y-1]|(1<<(x));//xを1にする
						  Dist_map[x][y-1]=255;
						  Front_wall=255;
						  //最短走行用
						  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
						  }
					  }
					  //右のセンサー
					  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
						  if(x>0){
						  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
						  //最短走行用
						  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
						  }
					  }else{//右あり
						  if(x>0){
						  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
						  Dist_map[x-1][y]=255;
						  Right_wall=255;
						  //最短走行用
						  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
						  }
					  }
				  }
				//西側を見ている　全センサーを見てよし
				else if(z==3){//(x,y),西に対して、、
					  //}else if((z==3)||(z==-1)){//(x,y),西に対して、、
					  //左のセンサー
					  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
						  if(y>0){
						  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
						  //最短走行用
						  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
						  }
					  }else{//左あり
						  if(y>0){
						  row[y-1]=row[y-1]|(1<<(x));//xを1にする
						  Dist_map[x][y-1]=255;
						  Left_wall=255;
						  //最短走行用
						  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
						  }
					  }
					  //前のセンサー
					  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//右なし
						  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
						  //最短走行用
						  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
					  }else{//前あり
						  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
						  Dist_map[x-1][y]=255;
						  Front_wall=255;
						  //最短走行用
						  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
					  }
					  //右のセンサー
					  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
						  row[y]=row[y]&~(1<<(x));//xビット目を0にする
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました
					  }else{//右あり
						  row[y]=row[y]|(1<<(x));//xを1にする
						  Dist_map[x][y+1]=255;
						  Right_wall=255;
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました
					  }
					  if(z==-1){
						  z=3;
					  }
					}
			}else if(y==0){//①-4最後壁一列
				//北側を見ている　全よし
		       if(z==0){
				      //左のセンサー
					  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
						  if(x>0){
						  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする//0bit目から数えることとする　要は1番目と呼んでいるのを0bit目として判断
						  //最短走行用
						  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
						  }
					  }else{//左あり
						  if(x>0){
						  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
						  Dist_map[x-1][y]=255;
						  Left_wall=255;
						  //最短走行用
						  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
						  }
					  }
					  //前のセンサー
					  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//前なし
						  row[y]=row[y]&~(1<<(x));//xビット目を0にする
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました
					  }else{//前あり
						  row[y]=row[y]|(1<<(x));//xを1にする
						  Dist_map[x][y+1]=255;
						  Front_wall=255;
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました
					  }
					  //右のセンサー
					  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
						  column[x]=column[x]&~(1<<(y));//yビット目を0にする
						  //最短走行用
						  short_column[x]=short_column[x]|(1<<(y));//探索しました用
					  }else{//右あり
						  column[x]=column[x]|(1<<(y));//yビット目を1にする
						  Dist_map[x+1][y]=255;
						  Right_wall=255;
						  //最短走行用
						  short_column[x]=short_column[x]|(1<<(y));//探索しました用
					  }

					  if(z==4){
						  z=0;
					  }
				//東側を見ている　右イラン
			}else if(z==1){//(x,y),東に対して、、
					  //左のセンサー
					  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
						  row[y]=row[y]&~(1<<(x));//xビット目を0にする
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました
					  }else{//左あり
						  row[y]=row[y]|(1<<(x));//xを1にする
						  Dist_map[x][y+1]=255;
						  Left_wall=255;
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました
					  }
					  //前のセンサー
					  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//なし

						  column[x]=column[x]&~(1<<(y));//yビット目を0にする
						  //最短走行用
						  short_column[x]=short_column[x]|(1<<(y));//探索しました用
					  }else{//あり
						  column[x]=column[x]|(1<<(y));//yビット目を1にする
						  Dist_map[x+1][y]=255;
						  Front_wall=255;
						  //最短走行用
						  short_column[x]=short_column[x]|(1<<(y));//探索しました用
					  }
				  }
				//南側を見ている　前なし
			else if(z==2){//(x,y),南に対して、、
					  //左のセンサー
					  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
						  column[x]=column[x]&~(1<<(y));//yビット目を0にする
						  //最短走行用
						  short_column[x]=short_column[x]|(1<<(y));//探索しました用
					  }else{//左あり
						  column[x]=column[x]|(1<<(y));//yビット目を1にする
						  Dist_map[x+1][y]=255;
						  Left_wall=255;
						  //最短走行用
						  short_column[x]=short_column[x]|(1<<(y));//探索しました用
					  }
					  //右のセンサー
					  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
						  if(x>0){
						  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
						  //最短走行用
						  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
						  }
					  }else{//右あり
						  if(x>0){
						  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
						  Dist_map[x-1][y]=255;
						  Right_wall=255;
						  //最短走行用
						  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
						  }
					  }
				  }
				//西側を見ている　左なし
			else if(z==3){//(x,y),西に対して、、
					  //}else if((z==3)||(z==-1)){//(x,y),西に対して、、
					  //前のセンサー
					  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//右なし
						  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
						  //最短走行用
						  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
					  }else{//前あり
						  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
						  Dist_map[x+1][y]=255;
						  Front_wall=255;
						  //最短走行用
						  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
					  }
					  //右のセンサー
					  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
						  row[y]=row[y]&~(1<<(x));//xビット目を0にする
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました
					  }else{//右あり
						  row[y]=row[y]|(1<<(x));//xを1にする
						  Dist_map[x][y+1]=255;
						  Right_wall=255;
						  //最短走行用
						  short_row[y]=short_row[y]|(1<<(x));//探索しました
					  }
					  if(z==-1){
						  z=3;
					  }
					}
			}

			//②一番外側にいないとき//以下ではx>0とかの条件はいらんと思うけど、、まあ必要あれば除くって感じで
			else{

		    if(z==0){//(x,y),北に対して、、

		      //左のセンサー
			  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
				  if(x>0){
				  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする//0bit目から数えることとする　要は1番目と呼んでいるのを0bit目として判断
				  //最短走行用
				  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
				  }
			  }else{//左あり
				  if(x>0){
				  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
				  Left_wall=255;
				  //最短走行用
				  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
				  }
			  }
			  //前のセンサー
			  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//前なし
				  row[y]=row[y]&~(1<<(x));//xビット目を0にする
				  //最短走行用
				  short_row[y]=short_row[y]|(1<<(x));//探索しました
			  }else{//前あり
				  row[y]=row[y]|(1<<(x));//xを1にする
				  Front_wall=255;
				  //最短走行用
				  short_row[y]=short_row[y]|(1<<(x));//探索しました
			  }
			  //右のセンサー
			  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
				  column[x]=column[x]&~(1<<(y));//yビット目を0にする
				  //最短走行用
				  short_column[x]=short_column[x]|(1<<(y));//探索しました用
			  }else{//右あり
				  column[x]=column[x]|(1<<(y));//yビット目を1にする
				  Right_wall=255;
				  //最短走行用
				  short_column[x]=short_column[x]|(1<<(y));//探索しました用
			  }

			  if(z==4){
				  z=0;
			  }
		  }else if(z==1){//(x,y),東に対して、、
			  //左のセンサー
			  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
				  row[y]=row[y]&~(1<<(x));//xビット目を0にする
				  //最短走行用
				  short_row[y]=short_row[y]|(1<<(x));//探索しました
			  }else{//左あり
				  row[y]=row[y]|(1<<(x));//xを1にする
				  Left_wall=255;
				  //最短走行用
				  short_row[y]=short_row[y]|(1<<(x));//探索しました
			  }
			  //前のセンサー
			  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//右なし

				  column[x]=column[x]&~(1<<(y));//yビット目を0にする
				  //最短走行用
				  short_column[x]=short_column[x]|(1<<(y));//探索しました用
			  }else{//前あり
				  column[x]=column[x]|(1<<(y));//yビット目を1にする
				  Front_wall=255;
				  //最短走行用
				  short_column[x]=short_column[x]|(1<<(y));//探索しました用
			  }
			  //右のセンサー
			  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
				  if(y>0){
				  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
				  //最短走行用
				  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
				  }
			  }else{//右あり
				  if(y>0){
				  row[y-1]=row[y-1]|(1<<(x));//xを1にする
				  Right_wall=255;
				  //最短走行用
				  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
				  }
			  }

		  }else if(z==2){//(x,y),南に対して、、
			  //左のセンサー
			  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
				  column[x]=column[x]&~(1<<(y));//yビット目を0にする
				  //最短走行用
				  short_column[x]=short_column[x]|(1<<(y));//探索しました用
			  }else{//左あり
				  column[x]=column[x]|(1<<(y));//yビット目を1にする
				  Left_wall=255;
				  //最短走行用
				  short_column[x]=short_column[x]|(1<<(y));//探索しました用
			  }
			  //前のセンサー
			  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//前なし
				  if(y>0){
				  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
				  //最短走行用
				  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
				  }
			  }else{//前あり
				  if(y>0){
				  row[y-1]=row[y-1]|(1<<(x));//xを1にする
				  Front_wall=255;
				  //最短走行用
				  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
				  }
			  }
			  //右のセンサー
			  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
				  if(x>0){
				  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
				  //最短走行用
				  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
				  }
			  }else{//右あり
				  if(x>0){
				  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
				  Right_wall=255;
				  //最短走行用
				  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
				  }
			  }
		  }else if(z==3){//(x,y),西に対して、、
			  //}else if((z==3)||(z==-1)){//(x,y),西に対して、、
			  //左のセンサー
			  if((float)g_sensor[1][0]<WALLREAD_L){//左なし
				  if(y>0){
				  row[y-1]=row[y-1]&~(1<<(x));//xビット目を0にする
				  //最短走行用
				  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
				  }
			  }else{//左あり
				  if(y>0){
				  row[y-1]=row[y-1]|(1<<(x));//xを1にする
				  Left_wall=255;
				  //最短走行用
				  short_row[y-1]=short_row[y-1]|(1<<(x));//探索しました
				  }
			  }
			  //前のセンサー
			  if((g_sensor[0][0]<WALLREAD_FL)&&(g_sensor[3][0]<WALLREAD_FR)){//前なし
				  column[x-1]=column[x-1]&~(1<<(y));//yビット目を0にする
				  //最短走行用
				  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
			  }else{//前あり
				  column[x-1]=column[x-1]|(1<<(y));//yビット目を1にする
				  Front_wall=255;
				  //最短走行用
				  short_column[x-1]=short_column[x-1]|(1<<(y));//探索しました用
			  }
			  //右のセンサー
			  if((float)g_sensor[2][0]<WALLREAD_R){//右がない
				  row[y]=row[y]&~(1<<(x));//xビット目を0にする
				  //最短走行用
				  short_row[y]=short_row[y]|(1<<(x));//探索しました
			  }else{//右あり
				  row[y]=row[y]|(1<<(x));//xを1にする
				  Right_wall=255;
				  //最短走行用
				  short_row[y]=short_row[y]|(1<<(x));//探索しました
			  }
			  if(z==-1){
				  z=3;
			  }
			}
			}

}


//最短走行前の仮想上の歩数マップを作るときに壁がある場合に255を入れる 探索で探索済か否かを入れればいいってことを把握してね
void short_ver_wall_sensor(){//最短走行用で探索済の壁を記録するっていうコードも入れる
	 //①一番外側いる時//歩数マップ作成時に壁がない方と同じ歩数になりがちなので、外壁沿いで壁があったらその奥を255にする操作を入れてみる
			if((x==0)){//①-1最左壁一列
				//北側を見ている  左センサーを除く
				 if(z==0){//(x,y),北に対して、、
					  //前のセンサー ((column[x]&(1<<(y)))!=(1<<(y)))       ((row[y]&(1<<(x)))!=(1<<(x)))
					  if((row[y]&(1<<(x)))!=(1<<(x))){//前なし
					  }else{//前あり
						  Dist_map[x][y+1]=255;//壁のある奥を255歩にする
						  Front_wall=255;
					  }
					  //右のセンサー
					  if((column[x]&(1<<(y)))!=(1<<(y))){//右がない
					  }else{//右あり
						  Dist_map[x+1][y]=255;//壁のある奥を255歩にする
						  Right_wall=255;
					  }

					  if(z==4){
						  z=0;
					  }
				 }
				//東側を見ている　全センサーを見てよし
					else if(z==1){
						//左のセンサー
					  if((row[y]&(1<<(x)))!=(1<<(x))){//左なし

					  }else{//左あり
						  Dist_map[x][y+1]=255;
						  Left_wall=255;
					  }
					  //前のセンサー
					  if((column[x]&(1<<(y)))!=(1<<(y))){//右なし
					  }else{//前あり
						  Dist_map[x+1][y]=255;//壁のある奥を255歩にする
						  Front_wall=255;
					  }
					  //右のセンサー
					  if(y>0){
						  if((row[y-1]&(1<<(x)))!=(1<<(x))){//右がない
						  }else{//右あり
							  row[y-1]=row[y-1]|(1<<(x));//xを1にする
							  Dist_map[x][y-1]=255;
							  Right_wall=255;
					  }

					  }
					}
				//南側を見ている  右センサーを除く
					else if(z==2){//(x,y),南に対して、、
					  //左のセンサー
					  if((column[x]&(1<<(y)))!=(1<<(y))){//左なし
					  }else{//左あり
						  Dist_map[x+1][y]=255;
						  Left_wall=255;
					  }
					  //前のセンサー
					  if(y>0){
					  if((row[y-1]&(1<<(x)))!=(1<<(x))){//前なし
					  }else{//前あり
						  if(y>0){
							  Dist_map[x][y-1]=255;
							  Front_wall=255;
						  }
						  }
					  }
					}
				 //西側を見ている  前センサーを除く
				   else if(z==3){//(x,y),西に対して、、
					  //}else if((z==3)||(z==-1)){//(x,y),西に対して、、
					  //左のセンサー
					  if(y>0){
						  if((row[y-1]&(1<<(x)))!=(1<<(x))){//左なし
						  }
					  else{//左あり
						  if(y>0){
							  Dist_map[x][y-1]=255;
							  Left_wall=255;
						  }
					  }
					  }
					  //右のセンサー
					  if((row[y]&(1<<(x)))!=(1<<(x))){//右がない
					  }else{//右あり
						  Dist_map[x][y+1]=255;
						  Right_wall=255;
					  }
					  if(z==-1){
						  z=3;
					  }
					}

			}else if(y==15){//①-2最前壁一列/////////
			    if(z==0){//(x,y),北に対して、、前なし

			      //左のセンサー側
			      if(x>0){
			    	  if((column[x-1]&(1<<(y)))!=(1<<(y))){//左なし
					  }else{//左あり
						  if(x>0){
							  Left_wall=255;
							  }
				  }
				  }
				  //右のセンサー
				  if((column[x]&(1<<(y)))!=(1<<(y))){//右がない
				  }else{//右あり
					  Dist_map[x+1][y]=255;//壁のある奥を255歩にする
					  Right_wall=255;
				  }
				  if(z==4){
					  z=0;
				  }

			  }//東側を見ている 左なし
				else if(z==1){
				  //前のセンサー
				  if((column[x]&(1<<(y)))!=(1<<(y))){//右なし
				  }else{//前あり
					  Dist_map[x+1][y]=255;//壁のある奥を255歩にする
					  Front_wall=255;
				  }
				  //右のセンサー
				  if(y>0){
					  if((row[y-1]&(1<<(x)))!=(1<<(x))){//右がない
					  }else{//右あり
						  Dist_map[x][y-1]=255;
						  Right_wall=255;
				  }

				  }
				}//南側を見ている 全部おk
				else if(z==2){//(x,y),南に対して、、
				  //左のセンサー
				  if((column[x]&(1<<(y)))!=(1<<(y))){//左なし
				  }else{//左あり
					  Dist_map[x+1][y]=255;
					  Left_wall=255;
				  }
				  //前のセンサー
				  if(y>0){
				  if((row[y-1]&(1<<(x)))!=(1<<(x))){//前なし
				  }else{//前あり
					  if(y>0){
						  Dist_map[x][y-1]=255;
						  Front_wall=255;
					  }
					  }
				  }
				  //右のセンサー
				  if(x>0){
					  if((column[x-1]&(1<<(y)))!=(1<<(y))){//右がない
					  }else{//右あり
						  if(x>0){
							  Dist_map[x-1][y]=255;
							  Right_wall=255;
						  }
					  }
				  }
			  }else if(z==3){//(x,y),西に対して、、 右なし
				  //}else if((z==3)||(z==-1)){//(x,y),西に対して、、
				  //左のセンサー
				  if(y>0){
					  if((row[y-1]&(1<<(x)))!=(1<<(x))){//左なし
					  }
				  else{//左あり
					  if(y>0){
						  Dist_map[x][y-1]=255;
						  Left_wall=255;
					  }
				  }
				  }
				  //前のセンサー
				  if((column[x-1]&(1<<(y)))!=(1<<(y))){//前なし
				  }else{//前あり
					  Dist_map[x-1][y]=255;
					  Front_wall=255;
				  }
				  if(z==-1){
					  z=3;
				  }
				}

			}else if(x==15){//①-3最右壁一列
				    if(z==0){//(x,y),北に対して、、右なし

				      //左のセンサー側
				      if(x>0){
				    	  if((column[x-1]&(1<<(y)))!=(1<<(y))){//左なし
						  }else{//左あり
							  if(x>0){
								  Left_wall=255;
								  }
					  }
					  }
				      //前のセンサー ((column[x]&(1<<(y)))!=(1<<(y)))       ((row[y]&(1<<(x)))!=(1<<(x)))
					  if((row[y]&(1<<(x)))!=(1<<(x))){//前なし
					  }else{//前あり
						  Dist_map[x][y+1]=255;//壁のある奥を255歩にする
						  Front_wall=255;
					  }

				  }//東側を見ている 前なし
					else if(z==1){
						//左のセンサー
					  if((row[y]&(1<<(x)))!=(1<<(x))){//左なし

					  }else{//左あり
						  Dist_map[x][y+1]=255;
						  Left_wall=255;
					  }
					  //右のセンサー
					  if(y>0){
						  if((row[y-1]&(1<<(x)))!=(1<<(x))){//右がない
						  }else{//右あり
							  Dist_map[x][y-1]=255;
							  Right_wall=255;
					  }

					  }
					}//南側を見ている  左なし
					else if(z==2){//(x,y),南に対して、、
					  //前のセンサー
					  if(y>0){
					  if((row[y-1]&(1<<(x)))!=(1<<(x))){//前なし
					  }else{//前あり
						  if(y>0){
							  Dist_map[x][y-1]=255;
							  Front_wall=255;
						  }
						  }
					  }
					  //右のセンサー
					  if(x>0){
						  if((column[x-1]&(1<<(y)))!=(1<<(y))){//右がない
						  }else{//右あり
							  if(x>0){
								  Dist_map[x-1][y]=255;
								  Right_wall=255;
							  }
						  }
					  }
				  }else if(z==3){//(x,y),西に対して、、 全部おk
					  //}else if((z==3)||(z==-1)){//(x,y),西に対して、、
					  //左のセンサー
					  if(y>0){
						  if((row[y-1]&(1<<(x)))!=(1<<(x))){//左なし
						  }
					  else{//左あり
						  if(y>0){
							  Dist_map[x][y-1]=255;
							  Left_wall=255;
						  }
					  }
					  }
					  //前のセンサー
					  if((column[x-1]&(1<<(y)))!=(1<<(y))){//前なし
					  }else{//前あり
						  Dist_map[x-1][y]=255;
						  Front_wall=255;
					  }
					  //右のセンサー
					  if((row[y]&(1<<(x)))!=(1<<(x))){//右がない
					  }else{//右あり
						  Dist_map[x][y+1]=255;
						  Right_wall=255;
					  }
					  if(z==-1){
						  z=3;
					  }
					}
			}else if(y==0){//①-4最後壁一列
				if(z==0){//(x,y),北に対して、、 全部よし

						      //左のセンサー側
						      if(x>0){
						    	  if((column[x-1]&(1<<(y)))!=(1<<(y))){//左なし
								  }else{//左あり
									  if(x>0){
										  Left_wall=255;
										  }
							  }
							  }
						      //前のセンサー ((column[x]&(1<<(y)))!=(1<<(y)))       ((row[y]&(1<<(x)))!=(1<<(x)))
							  if((row[y]&(1<<(x)))!=(1<<(x))){//前なし
							  }else{//前あり
								  Dist_map[x][y+1]=255;//壁のある奥を255歩にする
								  Front_wall=255;
							  }
							  //右のセンサー
							  if((column[x]&(1<<(y)))!=(1<<(y))){//右がない
							  }else{//右あり
								  Dist_map[x+1][y]=255;//壁のある奥を255歩にする
								  Right_wall=255;
							  }
							  if(z==4){
								  z=0;
							  }

						  }
				//東側を見ている　右イラン
			else if(z==1){
				//左のセンサー
			  if((row[y]&(1<<(x)))!=(1<<(x))){//左なし

			  }else{//左あり
				  Dist_map[x][y+1]=255;
				  Left_wall=255;
			  }
			  //前のセンサー
			  if((column[x]&(1<<(y)))!=(1<<(y))){//右なし
			  }else{//前あり
				  Dist_map[x+1][y]=255;//壁のある奥を255歩にする
				  Front_wall=255;
			  }
			}
				//南側を見ている　前なし

			else if(z==2){//(x,y),南に対して、、
						  //左のセンサー
						  if((column[x]&(1<<(y)))!=(1<<(y))){//左なし
						  }else{//左あり
							  Dist_map[x+1][y]=255;
							  Left_wall=255;
						  }
						  //右のセンサー
						  if(x>0){
							  if((column[x-1]&(1<<(y)))!=(1<<(y))){//右がない
							  }else{//右あり
								  if(x>0){
									  Dist_map[x-1][y]=255;
									  Right_wall=255;
								  }
							  }
						  }
					  }
				//西側を見ている　左なし
			else if(z==3){//(x,y),西に対して、、
						  //}else if((z==3)||(z==-1)){//(x,y),西に対して、、
						  //前のセンサー
						  if((column[x-1]&(1<<(y)))!=(1<<(y))){//前なし
						  }else{//前あり
							  Dist_map[x-1][y]=255;
							  Front_wall=255;
						  }
						  //右のセンサー
						  if((row[y]&(1<<(x)))!=(1<<(x))){//右がない
						  }else{//右あり
							  Dist_map[x][y+1]=255;
							  Right_wall=255;
						  }
						  if(z==-1){
							  z=3;
						  }
						}
			}

			////////////////
			//②一番外側にいないとき//以下ではx>0とかの条件はいらんと思うけど、、まあ必要あれば除くって感じで
			else{

			///////////
				//北側を見ている  左センサーを除く
								 if(z==0){//(x,y),北に対して、、
									  //前のセンサー ((column[x]&(1<<(y)))!=(1<<(y)))       ((row[y]&(1<<(x)))!=(1<<(x)))
									  if((row[y]&(1<<(x)))!=(1<<(x))){//前なし
									  }else{//前あり
										  Dist_map[x][y+1]=255;//壁のある奥を255歩にする
										  Front_wall=255;
									  }
									  //右のセンサー
									  if((column[x]&(1<<(y)))!=(1<<(y))){//右がない
									  }else{//右あり
										  Dist_map[x+1][y]=255;//壁のある奥を255歩にする
										  Right_wall=255;
									  }

									  if(z==4){
										  z=0;
									  }
								 }
								//東側を見ている　全センサーを見てよし
									else if(z==1){
										//左のセンサー
									  if((row[y]&(1<<(x)))!=(1<<(x))){//左なし

									  }else{//左あり
										  Dist_map[x][y+1]=255;
										  Left_wall=255;
									  }
									  //前のセンサー
									  if((column[x]&(1<<(y)))!=(1<<(y))){//右なし
									  }else{//前あり
										  Dist_map[x+1][y]=255;//壁のある奥を255歩にする
										  Front_wall=255;
									  }
									  //右のセンサー
									  if(y>0){
										  if((row[y-1]&(1<<(x)))!=(1<<(x))){//右がない
										  }else{//右あり
											  row[y-1]=row[y-1]|(1<<(x));//xを1にする
											  Dist_map[x][y-1]=255;
											  Right_wall=255;
									  }

									  }
									}
								//南側を見ている  右センサーを除く
									else if(z==2){//(x,y),南に対して、、
									  //左のセンサー
									  if((column[x]&(1<<(y)))!=(1<<(y))){//左なし
									  }else{//左あり
										  Dist_map[x+1][y]=255;
										  Left_wall=255;
									  }
									  //前のセンサー
									  if(y>0){
									  if((row[y-1]&(1<<(x)))!=(1<<(x))){//前なし
									  }else{//前あり
										  if(y>0){
											  Dist_map[x][y-1]=255;
											  Front_wall=255;
										  }
										  }
									  }
									}
								 //西側を見ている  前センサーを除く
								   else if(z==3){//(x,y),西に対して、、
									  //}else if((z==3)||(z==-1)){//(x,y),西に対して、、
									  //左のセンサー
									  if(y>0){
										  if((row[y-1]&(1<<(x)))!=(1<<(x))){//左なし
										  }
									  else{//左あり
										  if(y>0){
											  Dist_map[x][y-1]=255;
											  Left_wall=255;
										  }
									  }
									  }
									  //右のセンサー
									  if((row[y]&(1<<(x)))!=(1<<(x))){//右がない
									  }else{//右あり
										  Dist_map[x][y+1]=255;
										  Right_wall=255;
									  }
									  if(z==-1){
										  z=3;
									  }
									}

			///////////

		    if(z==0){//(x,y),北に対して、、

		      //左のセンサー側
		      if(x>0){
		    	  if((column[x-1]&(1<<(y)))!=(1<<(y))){//左なし
				  }else{//左あり
					  if(x>0){
						  Left_wall=255;
						  }
			  }
			  }
		      //前のセンサー ((column[x]&(1<<(y)))!=(1<<(y)))       ((row[y]&(1<<(x)))!=(1<<(x)))
			  if((row[y]&(1<<(x)))!=(1<<(x))){//前なし
			  }else{//前あり
				  Dist_map[x][y+1]=255;//壁のある奥を255歩にする
				  Front_wall=255;
			  }
			  //右のセンサー
			  if((column[x]&(1<<(y)))!=(1<<(y))){//右がない
			  }else{//右あり
				  Dist_map[x+1][y]=255;//壁のある奥を255歩にする
				  Right_wall=255;
			  }
			  if(z==4){
				  z=0;
			  }

		  }//東側を見ている
			else if(z==1){
				//左のセンサー
			  if((row[y]&(1<<(x)))!=(1<<(x))){//左なし

			  }else{//左あり
				  Dist_map[x][y+1]=255;
				  Left_wall=255;
			  }
			  //前のセンサー
			  if((column[x]&(1<<(y)))!=(1<<(y))){//右なし
			  }else{//前あり
				  Dist_map[x+1][y]=255;//壁のある奥を255歩にする
				  Front_wall=255;
			  }
			  //右のセンサー
			  if(y>0){
				  if((row[y-1]&(1<<(x)))!=(1<<(x))){//右がない
				  }else{//右あり
					  Dist_map[x][y-1]=255;
					  Right_wall=255;
			  }

			  }
			}//南側を見ている
			else if(z==2){//(x,y),南に対して、、
			  //左のセンサー
			  if((column[x]&(1<<(y)))!=(1<<(y))){//左なし
			  }else{//左あり
				  Dist_map[x+1][y]=255;
				  Left_wall=255;
			  }
			  //前のセンサー
			  if(y>0){
			  if((row[y-1]&(1<<(x)))!=(1<<(x))){//前なし
			  }else{//前あり
				  if(y>0){
					  Dist_map[x][y-1]=255;
					  Front_wall=255;
				  }
				  }
			  }
			  //右のセンサー
			  if(x>0){
				  if((column[x-1]&(1<<(y)))!=(1<<(y))){//右がない
				  }else{//右あり
					  if(x>0){
						  Dist_map[x-1][y]=255;
						  Right_wall=255;
					  }
				  }
			  }
		  }else if(z==3){//(x,y),西に対して、、
			  //}else if((z==3)||(z==-1)){//(x,y),西に対して、、
			  //左のセンサー
			  if(y>0){
				  if((row[y-1]&(1<<(x)))!=(1<<(x))){//左なし
				  }
			  else{//左あり
				  if(y>0){
					  Dist_map[x][y-1]=255;
					  Left_wall=255;
				  }
			  }
			  }
			  //前のセンサー
			  if((column[x-1]&(1<<(y)))!=(1<<(y))){//前なし
			  }else{//前あり
				  Dist_map[x-1][y]=255;
				  Front_wall=255;
			  }
			  //右のセンサー
			  if((row[y]&(1<<(x)))!=(1<<(x))){//右がない
			  }else{//右あり
				  Dist_map[x][y+1]=255;
				  Right_wall=255;
			  }
			  if(z==-1){
				  z=3;
			  }
			}
			}

}



//足立法
void adachi_method(void){
	//
//	//歩数マップの初期化これは足立法の繰り返しでは入れてはいけないきがす、、
//	for(x=0;x<=15;x++){
//		for(y=0;y<=15;y++){
//			Dist_map[x][y]=255;
//		}
//	}

	  //初期化 最初初期化の次に歩数マップの初期化をしていたために最初のx,y,zが16,16,0担ってたこれは終わらんわ
	  x=0;
	  y=0;
	  z=0;
	  column[0]=0b1000000000000000;
	  row[0]=0b0000000000000000;

      wall_information_initialize();//壁情報の初期化
	  //初期動作
      motor_excitation_on();//励磁スタート
	  trapezoid_accel_forward(2000,100,500,100,90);//90進む台形加速の関数

	  x=x;
	  y=y+1;
	  z=z;//　最初(0,1)から出発、(この時、(0,0.5)あたりにいるが一旦(0,1)にいたとみなす)北向き

//	  //
//	  	//歩数マップの初期化これは足立法の繰り返しでは入れてはいけないきがす、、
//	  	for(x=0;x<=15;x++){
//	  		for(y=0;y<=15;y++){
//	  			Dist_map[x][y]=255;
//	  		}
//	  	}

	  	//wall_information_initialize();
		while(1){
			printf("プログラムの初めx=%d,y=%d,z=%d\n\r",x,y,z);
			//終了判定
			if((x==goal_x)&&(y==goal_y)){//ゴールに来たら終了
					trapezoid_accel_forward(2000,100,500,100,90);//90む台形加速の関数
					HAL_Delay(1000);//1秒経ってから励磁解除
					motor_excitation_off();//励磁ストップ
					break;//無限ループから脱出
			}


		   //<壁情報更新>1223のプログラムでは例外処理をそれぞれ書いていたが、ここでは一旦、各方角の外壁の例外処理をしている
		     //今(x,y,z)



		     //方角によってどう変わるか//z=0,1,2,3
			  //例外処理(位置によって場合分け)

			wall_sensor();
//			Print_Wall_2();
			//歩数マップ作成
			step_number();
//			step_number_revised();
//			printf("歩数マップ作成時x=%d,y=%d,z=%d\n\r",x,y,z);
//			Print_Wall_2();

			//最小歩数判定//行動決定
			least_step_judgement_and_action_decision();
//			printf("最初歩数判定、行動決定時x=%d,y=%d,z=%d\n\r",x,y,z);

			//行動&座標更新
			action_based_on_direction_decision_and_coordinate_update();
//			printf("行動後、座標更新時x=%d,y=%d,z=%d\n\r",x,y,z);
		}
}

//連続足立法
void continual_adachi_method(void){
	//
//	//歩数マップの初期化これは足立法の繰り返しでは入れてはいけないきがす、、
//	for(x=0;x<=15;x++){
//		for(y=0;y<=15;y++){
//			Dist_map[x][y]=255;
//		}
//	}

	  //初期化 最初初期化の次に歩数マップの初期化をしていたために最初のx,y,zが16,16,0担ってたこれは終わらんわ
	  x=0;
	  y=0;
	  z=0;
	  column[0]=0b1000000000000000;
	  row[0]=0b0000000000000000;


      wall_information_initialize();//壁情報の初期化
	  //初期動作
      motor_excitation_on();
	  motor_pwm_on();
	  trapezoid_accel_backward(600,100,200,100,80);//90back台形加速の関数遅くね
	  HAL_Delay(500);//
	  motor_pwm_off();
	  motor_pwm_on();
	  trapezoid_accel_forward(2000,100,500,500,120);//120む台形加速の関数

      /////
	  x=x;
	  y=y+1;
	  z=z;//　最初(0,1)から出発、(この時、(0,0.5)あたりにいるが一旦(0,1)にいたとみなす)北向き

//	  //
//	  	//歩数マップの初期化これは足立法の繰り返しでは入れてはいけないきがす、、
//	  	for(x=0;x<=15;x++){
//	  		for(y=0;y<=15;y++){
//	  			Dist_map[x][y]=255;
//	  		}
//	  	}

	  	//wall_information_initialize();
		while(1){
//			printf("プログラムの初めx=%d,y=%d,z=%d\n\r",x,y,z);

			//片方のボタンを押したら終了
			if ((HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0)||(HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0)){
				ring_interrupt();
				//物理的に止まる関数
				motor_pwm_off();
				HAL_Delay(1000);//1秒経ってから励磁解除
				motor_excitation_off();//励磁ストップ;
				break;//止まった後、while分抜けられるかな 抜けたらそのままadachimethodo終わるはず
			}
			//終了判定
			if((x==goal_x)&&(y==goal_y)){//ゴールに来たら終了
					trapezoid_accel_forward(2000,500,500,100,90);//90む台形加速の関数
					motor_pwm_off();
					HAL_Delay(1000);//1秒経ってから励磁解除
					motor_excitation_off();//励磁ストップ
					break;//無限ループから脱出
			}


		   //<壁情報更新>1223のプログラムでは例外処理をそれぞれ書いていたが、ここでは一旦、各方角の外壁の例外処理をしている
		     //今(x,y,z)



		     //方角によってどう変わるか//z=0,1,2,3
			  //例外処理(位置によって場合分け)

			wall_sensor();

//			Print_Wall_2();
			//20mm進んでいる間に歩数マップ作成する関数
			before_step_number_revised();
			step_ver_trapezoid_accel_forward(2000,500,500,500,20);
//			step_number_revised();
			after_step_number_revised();
//			printf("歩数マップ作成時x=%d,y=%d,z=%d\n\r",x,y,z);
//			Print_Wall_2();

			//最小歩数判定//行動決定
			least_step_judgement_and_action_decision();
//			printf("最初歩数判定、行動決定時x=%d,y=%d,z=%d\n\r",x,y,z);

			//行動&座標更新
			continual_ver_action_based_on_direction_decision_and_coordinate_update();
//			printf("行動後、座標更新時x=%d,y=%d,z=%d\n\r",x,y,z);
			if (HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0){
							break;
						}
		}
}



//スラローム走行ver.

void slalom_continual_adachi_method(void){
	//
//	//歩数マップの初期化これは足立法の繰り返しでは入れてはいけないきがす、、
//	for(x=0;x<=15;x++){
//		for(y=0;y<=15;y++){
//			Dist_map[x][y]=255;
//		}
//	}

	  //初期化 最初初期化の次に歩数マップの初期化をしていたために最初のx,y,zが16,16,0担ってたこれは終わらんわ
	  x=0;
	  y=0;
	  z=0;
	  column[0]=0b1000000000000000;
	  row[0]=0b0000000000000000;
//	  column[15]=0b1111111111111111;
//	  row[15]=0b1111111111111111;

      wall_information_initialize();//壁情報の初期化
	  //初期動作
      motor_excitation_on();
	  motor_pwm_on();
	  trapezoid_accel_backward(600,100,200,100,80);//90back台形加速の関数遅くね
	  HAL_Delay(500);//
	  motor_pwm_off();
	  motor_pwm_on();
	  trapezoid_accel_forward(2000,100,500,500,120);//120む台形加速の関数


//      motor_excitation_on();//励磁スタート
//      motor_pwm_on();
//      trapezoid_accel_backward(1000,100,300,100,80);//90back台形加速の関数遅くね
//	  motor_pwm_off();
//	  motor_pwm_on();
//      trapezoid_accel_forward(2000,100,500,500,120);//120む台形加速の関数
//	  trapezoid_accel_forward(2000,100,500,500,90);//90進む台形加速の関数
	  x=x;
	  y=y+1;
	  z=z;//　最初(0,1)から出発、(この時、(0,0.5)あたりにいるが一旦(0,1)にいたとみなす)北向き

//	  //
//	  	//歩数マップの初期化これは足立法の繰り返しでは入れてはいけないきがす、、
//	  	for(x=0;x<=15;x++){
//	  		for(y=0;y<=15;y++){
//	  			Dist_map[x][y]=255;
//	  		}
//	  	}

	  	//wall_information_initialize();

	  //片方のボタンを押したら終了
		while(1){



			if ((HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0)||(HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0)){
				//物理的に止まる関数
				ring_interrupt();
				motor_pwm_off();
				HAL_Delay(1000);//1秒経ってから励磁解除
				motor_excitation_off();//励磁ストップ;
				break;
			}else{
//			}else{
//			printf("プログラムの初めx=%d,y=%d,z=%d\n\r",x,y,z);
			//終了判定
			if((x==goal_x)&&(y==goal_y)){//ゴールに来たら終了
					trapezoid_accel_forward(2000,500,500,100,90);//90む台形加速の関数
					motor_pwm_off();
					HAL_Delay(1000);//1秒経ってから励磁解除
					motor_excitation_off();//励磁ストップ
					break;//無限ループから脱出
			}


		   //<壁情報更新>1223のプログラムでは例外処理をそれぞれ書いていたが、ここでは一旦、各方角の外壁の例外処理をしている
		     //今(x,y,z)



		     //方角によってどう変わるか//z=0,1,2,3
			  //例外処理(位置によって場合分け)

	 //関数化した一番外側にいるときの壁情報の処理, 加えて, 区画の境界にいるときに壁情報の取得諸々
			wall_sensor();

//			Print_Wall_2();
			//20mm進んでいる間に歩数マップ作成する関数
			before_step_number_revised();
			step_ver_trapezoid_accel_forward(2000,500,500,500,20);
//			step_number_revised();
			after_step_number_revised();
//			printf("歩数マップ作成時x=%d,y=%d,z=%d\n\r",x,y,z);
//			Print_Wall_2();

			//最小歩数判定//行動決定
			least_step_judgement_and_action_decision();
//			printf("最初歩数判定、行動決定時x=%d,y=%d,z=%d\n\r",x,y,z);

			//行動&座標更新
			slalom_continual_ver_action_based_on_direction_decision_and_coordinate_update();
//			printf("行動後、座標更新時x=%d,y=%d,z=%d\n\r",x,y,z);
//			}
		}
		}
}

void side_added_slalom_continual_adachi_method(void){
	//
//	//歩数マップの初期化これは足立法の繰り返しでは入れてはいけないきがす、、
//	for(x=0;x<=15;x++){
//		for(y=0;y<=15;y++){
//			Dist_map[x][y]=255;
//		}
//	}

	  //初期化 最初初期化の次に歩数マップの初期化をしていたために最初のx,y,zが16,16,0担ってたこれは終わらんわ
	  x=0;
	  y=0;
	  z=0;
	  column[0]=0b1000000000000000;
	  row[0]=0b0000000000000000;
//	  column[15]=0b1111111111111111;
//	  row[15]=0b1111111111111111;

      wall_information_initialize();//壁情報の初期化
	  //初期動作
      motor_excitation_on();
	  motor_pwm_on();
	  trapezoid_accel_backward(600,100,200,100,80);//90back台形加速の関数遅くね
	  HAL_Delay(500);//
	  motor_pwm_off();
	  motor_pwm_on();
	  trapezoid_accel_forward(2000,100,500,500,120);//120む台形加速の関数


//      motor_excitation_on();//励磁スタート
//      motor_pwm_on();
//      trapezoid_accel_backward(1000,100,300,100,80);//90back台形加速の関数遅くね
//	  motor_pwm_off();
//	  motor_pwm_on();
//      trapezoid_accel_forward(2000,100,500,500,120);//120む台形加速の関数
//	  trapezoid_accel_forward(2000,100,500,500,90);//90進む台形加速の関数
	  x=x;
	  y=y+1;
	  z=z;//　最初(0,1)から出発、(この時、(0,0.5)あたりにいるが一旦(0,1)にいたとみなす)北向き

//	  //
//	  	//歩数マップの初期化これは足立法の繰り返しでは入れてはいけないきがす、、
//	  	for(x=0;x<=15;x++){
//	  		for(y=0;y<=15;y++){
//	  			Dist_map[x][y]=255;
//	  		}
//	  	}

	  	//wall_information_initialize();

	  //片方のボタンを押したら終了
		while(1){



			if ((HAL_GPIO_ReadPin(SWITCH_1_GPIO_Port,SWITCH_1_Pin)==0)||(HAL_GPIO_ReadPin(SWITCH_2_GPIO_Port,SWITCH_2_Pin)==0)){
				//物理的に止まる関数
				ring_interrupt();
				motor_pwm_off();
				HAL_Delay(1000);//1秒経ってから励磁解除
				motor_excitation_off();//励磁ストップ;
				break;
			}else{
//			}else{
//			printf("プログラムの初めx=%d,y=%d,z=%d\n\r",x,y,z);
			//終了判定
			if((x==goal_x)&&(y==goal_y)){//ゴールに来たら終了
					trapezoid_accel_forward(2000,500,500,100,90);//90む台形加速の関数
					motor_pwm_off();
					HAL_Delay(1000);//1秒経ってから励磁解除
					motor_excitation_off();//励磁ストップ
					break;//無限ループから脱出
			}


		   //<壁情報更新>1223のプログラムでは例外処理をそれぞれ書いていたが、ここでは一旦、各方角の外壁の例外処理をしている
		     //今(x,y,z)



		     //方角によってどう変わるか//z=0,1,2,3
			  //例外処理(位置によって場合分け)

	 //関数化した一番外側にいるときの壁情報の処理, 加えて, 区画の境界にいるときに壁情報の取得諸々
			wall_sensor();

//			Print_Wall_2();
			//20mm進んでいる間に歩数マップ作成する関数
			before_step_number_revised();
			step_ver_trapezoid_accel_forward(2000,500,500,500,20);
//			step_number_revised();
			after_step_number_revised();
//			printf("歩数マップ作成時x=%d,y=%d,z=%d\n\r",x,y,z);
//			Print_Wall_2();

			//最小歩数判定//行動決定
			least_step_judgement_and_action_decision();
//			printf("最初歩数判定、行動決定時x=%d,y=%d,z=%d\n\r",x,y,z);

			//行動&座標更新
			side_add_slalom_continual_ver_action_based_on_direction_decision_and_coordinate_update();
//			printf("行動後、座標更新時x=%d,y=%d,z=%d\n\r",x,y,z);
//			}
		}
		}
}
