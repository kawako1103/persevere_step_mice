/*
 * wall_control.c
 *
 *  Created on: Dec 16, 2022
 *      Author: kawaguchitakahito
 */

#include "trapezoid_acc_model.h"
#include "PL_sensor.h"
#include "trapezoid_acc_model.h"
#include <PL_sensor.h>

float PID_wall;
float SENSOR_GAIN=0.2;//仮
int wall_control_flg;//flgの宣言
float v_l;
float v_r;
float CENTER_L=520;//機体が中心にいる時の左側のセンサ値(仮)
float CENTER_R=280;//機体が中心にいる時の右側のセンサ値（仮）
float THERESHOULD_L=260;//左壁判定の閾値（仮実測値からよろ）//壁がないとき69~71なので 70 少し大きめで100
float THERESHOULD_R=180;//右壁判定の閾値（仮実測値からよろ）//壁がないとき35~40なので38 少し大きめて70
//float THERESHOULD_FL=260;//左壁判定の閾値（仮実測値からよろ）//ありで240/壁がないとき98なので 170少し小さめで140
//float THERESHOULD_FR=180;//右壁判定の閾値（仮実測値からよろ）//ありで344/壁がないとき175 259.5少し小さめで220
float THERESHOULD_DIFF_L=400;//左壁の壁切れ判定の閾値(仮同上) 520-70=450 400にしておこう揺らぎを少なくしたかったらもう少し大きくして
float THERESHOULD_DIFF_R=500;//右壁の壁切れ判定の閾値(仮同上) 280-38=242 200にしておこう同上
int g_WallControlStatus;
int j;


//壁制御の概要1の時は、制御量をただ求めてただけ
//void calWallControl(void){
//	if (wall_control_flg==1){
//	PID_wall = SENSOR_GAIN*(((float)g_sensor[1][0]-CENTER_L)  //left_wall_sensor_valueが左壁(g_sensor[1][0]のはず)のセンサ値
//			-((float)g_sensor[2][0]-CENTER_R));//right_wall_sensor_valueが右壁(g_sensor[2][0]のはず)のセンサ値
//	 //PID_wall=センサーゲイン×偏差で制御量(出力)を表す　ex)値が正なら右のセンサ値<左のセンサ値/負なら左のセンサ値<右のセンサ値
//	}else{
//		PID_wall = 0;
//	}
//	}

//壁制御の概要2の時　概要1の制御量に加え、壁がないとそこに吸い込まれてしまうので、それを防ぐために閾値について展開している//閾値以下の場合は壁制御を切る
//void calWallControl(void){//bit演算を利用、合っている？
//	if (wall_control_flg==1){
//	if((float)g_sensor[1][0]>THERESHOULD_L){//逆じゃね センサ値が小さいと壁ないってわけなので
//		g_WallControlStatus=g_WallControlStatus|(1<<0);//1bit目を1にする修正済
//
//	}else{
//		g_WallControlStatus=g_WallControlStatus&~(1<<0);//1bit目を０にする
//
//	}
//	if((float)g_sensor[1][0]>THERESHOULD_R){
//		g_WallControlStatus=g_WallControlStatus|(1<<1);//2bit目を1にする修正済
//
//	}else{
//		g_WallControlStatus=g_WallControlStatus&~(1<<1);//2bit目を0にする
//
//		}
////	if((g_WallControlStatus&(1<<(n-1)))==(1<<(n-1)))//これ理解できてません、、n bit目が1か確認(n=) 使わなくても行けるか？→安定化で使うのね
//	if(g_WallControlStatus==0b00){
//		PID_wall = 0;
//	}//2進数は0bを数字の接頭辞につければいけるとか？
//	else if(g_WallControlStatus==0b01){
//		PID_wall = SENSOR_GAIN*(2*(float)(g_sensor[1][0]-CENTER_L));
//	}
//	else if(g_WallControlStatus==0b10){
//		PID_wall = SENSOR_GAIN*(-2*(float)(g_sensor[2][0]-CENTER_R));
//	}
//	else if(g_WallControlStatus==0b11){
//		PID_wall = SENSOR_GAIN*(((float)g_sensor[1][0]-CENTER_L)-((float)g_sensor[2][0]-CENTER_R));
//	}
//	}else{
//		PID_wall = 0;
//	}
//}

//壁制御の概要3の時　概要2に加え、センサー値の変化量から安定化を図る急激な変化量を感知する解釈ですこの時点では。
void calWallControl(void){//bit演算を利用、合っている？
if (wall_control_flg==1){
	if((float)g_sensor[1][0]>THERESHOULD_L){//[1][0]は、左壁, [2][0]は、右壁
			g_WallControlStatus=g_WallControlStatus|(1<<0);//1bit目を1にする修正済

		}else{
			g_WallControlStatus=g_WallControlStatus&~(1<<0);//1bit目を０にする

		}
		if((float)g_sensor[2][0]>THERESHOULD_R){//g_sensor[1][0]になってたが、修正
			g_WallControlStatus=g_WallControlStatus|(1<<1);//2bit目を1にする修正済

		}else{
			g_WallControlStatus=g_WallControlStatus&~(1<<1);//2bit目を0にする

			}

		//前回左壁があり、、、、前回ということは、ここから概要3が始まる？！
			if((g_WallControlStatus&(1<<(1-1)))==(1<<(1-1))){//n bit目が1か確認(n=1)
				if((float)g_sensor[1][0]<THERESHOULD_L||(float)g_sensor[1][1]-(float)g_sensor[1][0]>THERESHOULD_DIFF_L){//ちょうど列が一つ増えた的な感じで使えないかな
					g_WallControlStatus=g_WallControlStatus&~(1<<0);//(1回でも起きたらを表現できてるか？)1bit目を０にする
				}else{

				}

			}
			else{
				if((float)g_sensor[1][0]>THERESHOULD_L && (float)g_sensor[1][1]-(float)g_sensor[1][0]<THERESHOULD_DIFF_L){
					for (j = 0; j <= 500; j++) {
					}
					if((float)g_sensor[1][0]>THERESHOULD_L && (float)g_sensor[1][1]-(float)g_sensor[1][0]<THERESHOULD_DIFF_L){
						g_WallControlStatus=g_WallControlStatus&~(0<<0);//([一定以上続いたら]を[一定時間たってもこのままであれば]とした。1bit目を1にする
					}else{
					}
				}else{

				}
			}
		//前回右壁がありor not
			if((g_WallControlStatus&(1<<(2-1)))==(1<<(2-1))){//n bit目が1か確認(n=2)
				if((float)g_sensor[2][0]<THERESHOULD_L||(float)g_sensor[2][1]-(float)g_sensor[2][0]>THERESHOULD_DIFF_R){//ちょうど列が一つ増えた的な感じで使えないかな
					g_WallControlStatus=g_WallControlStatus&~(1<<1);//(1回でも起きたらを表現できてるか？)2bit目を０にする
				}else{
				}

			}
			else{
				if((float)g_sensor[2][0]>THERESHOULD_L && (float)g_sensor[2][1]-(float)g_sensor[2][0]<THERESHOULD_DIFF_R){
					for (j = 0; j <= 500; j++) {
					}
					if((float)g_sensor[2][0]>THERESHOULD_R && (float)g_sensor[2][1]-(float)g_sensor[2][0]<THERESHOULD_DIFF_R){
						g_WallControlStatus=g_WallControlStatus|(1<<1);//([一定以上続いたら]を[一定時間たってもこのままであれば]とした。2bit目を1にする
					}else{
					}
				}else{

				}
			}

//	if((g_WallControlStatus&(1<<(n-1)))==(1<<(n-1)))//これ理解できてません、、n bit目が1か確認(n=) 使わなくても行けるか？
	if(g_WallControlStatus==0b00){
		PID_wall = 0;
	}//2進数は0bを数字の接頭辞につければいけるとか？
	else if(g_WallControlStatus==0b01){
		PID_wall = SENSOR_GAIN*(2*(float)(g_sensor[1][0]-CENTER_L));
	}
	else if(g_WallControlStatus==0b10){
		PID_wall = SENSOR_GAIN*(-2*(float)(g_sensor[2][0]-CENTER_R));
	}
	else if(g_WallControlStatus==0b11){
		PID_wall = SENSOR_GAIN*(((float)g_sensor[1][0]-CENTER_L)-((float)g_sensor[2][0]-CENTER_R));
	}
}else{
	PID_wall = 0;
	}
}
//
////前回左壁があり、、、、前回ということは、ここから概要3が始まる？！
//	if((g_WallControlStatus&(1<<(1-1)))==(1<<(1-1))){//n bit目が1か確認(n=1)
//		if((float)g_sensor[1][0]<THERESHOULD_L||(float)g_sensor[1][1]-(float)g_sensor[1][0]>THERESHOULD_DIFF_L){//ちょうど列が一つ増えた的な感じで使えないかな
//			g_WallControlStatus=g_WallControlStatus&~(1<<0);//(1回でも起きたらを表現できてるか？)1bit目を０にする
//		}else{
//
//		}
//
//	}
//	else{
//		if((float)g_sensor[1][0]>THERESHOULD_L && (float)g_sensor[1][1]-(float)g_sensor[1][0]<THERESHOULD_DIFF_L){
//			for (j = 0; j <= 500; j++) {
//			}
//			if((float)g_sensor[1][0]>THERESHOULD_L && (float)g_sensor[1][1]-(float)g_sensor[1][0]<THERESHOULD_DIFF_L){
//				g_WallControlStatus=g_WallControlStatus&~(0<<0);//([一定以上続いたら]を[一定時間たってもこのままであれば]とした。1bit目を1にする
//			}else{
//			}
//		}else{
//
//		}
//	}
////前回右壁がありor not
//	if((g_WallControlStatus&(1<<(2-1)))==(1<<(2-1))){//n bit目が1か確認(n=2)
//		if((float)g_sensor[2][0]<THERESHOULD_L||(float)g_sensor[2][1]-(float)g_sensor[2][0]>THERESHOULD_DIFF_R){//ちょうど列が一つ増えた的な感じで使えないかな
//			g_WallControlStatus=g_WallControlStatus&~(1<<1);//(1回でも起きたらを表現できてるか？)2bit目を０にする
//		}else{
//		}
//
//	}
//	else{
//		if((float)g_sensor[2][0]>THERESHOULD_L && (float)g_sensor[2][1]-(float)g_sensor[2][0]<THERESHOULD_DIFF_R){
//			for (j = 0; j <= 500; j++) {
//			}
//			if((float)g_sensor[2][0]>THERESHOULD_R && (float)g_sensor[2][1]-(float)g_sensor[2][0]<THERESHOULD_DIFF_R){
//				g_WallControlStatus=g_WallControlStatus|(1<<1);//([一定以上続いたら]を[一定時間たってもこのままであれば]とした。2bit目を1にする
//			}else{
//			}
//		}else{
//
//		}
//	}
//	//最後に制御量を出す
//	if(g_WallControlStatus==0b00){
//			PID_wall = 0;
//		}//2進数は0bを数字の接頭辞につければいけるとか？
//		else if(g_WallControlStatus==0b01){
//			PID_wall = SENSOR_GAIN*(2*(float)(g_sensor[1][0]-CENTER_L));
//		}
//		else if(g_WallControlStatus==0b10){
//			PID_wall = SENSOR_GAIN*(-2*(float)(g_sensor[2][0]-CENTER_R));
//		}
//		else if(g_WallControlStatus==0b11){
//			PID_wall = SENSOR_GAIN*(((float)g_sensor[1][0]-CENTER_L)-((float)g_sensor[2][0]-CENTER_R));
//		}
//	}
//	if(g_WallControlStatus==0b00){
//			PID_wall = 0;
//		}//2進数は0bを数字の接頭辞につければいけるとか？
//		else if(g_WallControlStatus==0b01){
//			PID_wall = SENSOR_GAIN*(2*(float)(g_sensor[1][0]-CENTER_L));
//		}
//		else if(g_WallControlStatus==0b10){
//			PID_wall = SENSOR_GAIN*(-2*(float)(g_sensor[2][0]-CENTER_R));
//		}
//		else if(g_WallControlStatus==0b11){
//			PID_wall = SENSOR_GAIN*(((float)g_sensor[1][0]-CENTRAL_L)-((float)g_sensor[2][0]-CENTRAL_R));
//		}

//	PID_wall = SENSOR_GAIN*(((float)g_sensor[1][0]-CENTRAL_L)-((float)g_sensor[2][0]-CENTRAL_R));
//	//left_wall_sensor_valueが左壁(g_sensor[1][0]のはず)のセンサ値
//	//right_wall_sensor_valueが右壁(g_sensor[2][0]のはず)のセンサ値
//
//	return PID_wall; //PID_wall=センサーゲイン×偏差で制御量(出力)を表す　ex)値が正なら右のセンサ値<左のセンサ値/負なら左のセンサ値<右のセンサ値
//}



//以下いらない(使っても動かない)
//void wall_control_interupt(void){
//
//		if (wall_control_flg==1){
////			  PID_wall = calWallControl();
//			  vel += acc * dt;
//			  if (wall_control_flg==1){
////			  		  PID _wall=calWallControl();//壁制御のPIDの量の計算
//			  		  v_l += PID_wall;
//			  		  v_r -= PID_wall;
//			  }else{
//			  }
//			  leftCWCCW(v_l);
//			  rightCWCCW(v_r);
//			  g_motorCount_l = calPWMCount(v_l);
//			  g_motorCount_r = calPWMCount(v_r);
//
//		  }else{
//		  }
//		  }
