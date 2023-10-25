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
int wall_cut_control_flg;
int wall_cut_control_start_flg;
int near_wall_cutting_flg;
float v_l;
float v_r;
float CENTER_L=550;//機体が中心にいる時の左側のセンサ値(仮) 1228以前　520/280  1228 550/315  2/17 以前550  2/18　以前545 584  565(合宿時)//2023/10/23 550
float CENTER_R=454;//機体が中心にいる時の右側のセンサ値（仮	  315							  2/17 以前315  2/18　以前323.5　(合宿時)403 //2023/10/23 454
//

////
////2023/2/18以前
//左右壁あり壁から逆側の壁にくっつけた時//L=315 R=166
//  壁なし             //L=120  R=33　→L平均217.5  R平均100くらい） 103/32

////2023/2/19　大会時
//左右壁あり壁から逆側の壁にくっつけた時//L=292 R=211
//  壁なし             //L=124  R=52　→L平均208  R平均131.5くらい） 103/32

////2023/10/23
//左右壁あり壁から逆側の壁にくっつけた時//L=297 R=232
//  壁なし             //L=78  R=32　→L平均188  R平均132くらい） 103/32

//だいたい見ればわかるが、センサー値の調整は、
//1.左右壁あり状態で中央にマウスをおいて、左右のセンサ値(標準機ならセンサー2と３が横壁を見るセンサー(センサー1と4が前を見ていることに注意))の大体の平均を見て、CENTER_L, Rに入れる
//2.マウスを左右壁あり状態のところにおいて、左のセンサー値を見るときは右の壁にくっつけて、読む　逆も然り。　次に壁なしの時の左と右どちらの値も読む
//3.2でとれた左右それぞれ二つずつの値の平均をとって、WALLREADL,Rに入れる。機体の走行の様子を見て、ちょっと値を上げたりして調整して
//4.THERESHOULDに関しては、走行の様子だけを見て判断するしかないかも。

float WALLREAD_L=188;//217.5
float WALLREAD_R=132;//100 ///131.5壁を読むのであげた 151.5///大会用迷路では大丈夫と信じる

float THERESHOULD_L=260;//壁制御用で使う左壁判定の閾値（仮実測値からよろ）//壁がないとき69~71なので 70 少し大きめで100  260→217
float THERESHOULD_R=180;//右壁判定の閾値（仮実測値からよろ）//壁がないとき35~40なので38 少し大きめて70    180→100

///////////
///12/18//
//前壁あり壁から180m遠い時//FL=245 FR=235  //1228/FL:203 FR:162
//  壁なし             //FL=111  FR=98　 //1228/FL:120 FR:102
                      //→FL平均178  FR平均166  平均かそれより低いくらいがいいそう 1228→FL平均161.5  FR平均132
//////////

///////////
///2/19//
//前壁あり壁から180m遠い時//FL=324 FR=295
//  壁なし             //FL=139  FR=152
                      //→FL平均231.5  FR平均223.5  平均かそれより低いくらいがいいそう 1228→FL平均161.5  FR平均132
//////////

//前壁あり壁から壁の境目(いつも通り測定するときの距離)遠い時
					  //FL=245 FR=235  //1228/FL:295 FR:275
//  壁なし             //FL=111  FR=98　 //1228/FL:120 FR:102
                      //→FL平均178  FR平均166  平均かそれより低いくらいがいいそう 1228→FL平均207.5  FR平均188.5

//壁情報取得用
float WALLREAD_FL=231.5; //207.5 センサーの傾き

float WALLREAD_FR=223.5; //188.5

//壁情報 前壁だけの時、干渉しているであるときの横センサー値//
float FrontWallInterferenceFL=710;
float FrontWallInterferenceFR=720;

//壁制御用つまり、壁制御用で以下二行はいらない？
float THERESHOULD_FL=260;//左前壁判定の閾値（仮実測値からよろ）//ありで240/壁がないとき98なので 170少し小さめで140 260にしてた 240→200
float THERESHOULD_FR=180;//右前壁判定の閾値（仮実測値からよろ）//ありで344/壁がないとき175 259.5少し小さめで220   180にしてた
float THERESHOULD_DIFF_L=415;//左壁の壁切れ判定の閾値(仮同上) 520-70=450 400にしておこう揺らぎを少なくしたかったらもう少し大きくして 2022/2/18以前400 大会時、壁制御が吸い込まれてたから揺らぎを少なくしたい　閾値をあげた 430で改善した 430では小さすぎる局面が出てきたので415にしてみる
float THERESHOULD_DIFF_R=530;//右壁の壁切れ判定の閾値(仮同上) 280-38=242 200にしておこう同上　左の記述よくわからないがとりあえず上げてみる　2022/2/18以前500 もう少しあげる(530(まあ良い)→545(だめ)→(535)にした)　わんちゃん　両方のバランスが大事かも(545の時もう少し早めに右を検出)
int g_WallControlStatus;
int j;

///////
//前壁制御用
///////
float FrontWallR = 425;//322が区画にいるとき400 ちょっと早すぎるのであげます
float FrontWallL = 292;//190が区画にいるとき267  317

///////
///////スラロームの時、前壁に近過ぎて横のセンサーが機能しなくなった時に壁制御を切る用
float nearest_FrontWallR = 930;
float nearest_FrontWallL = 880;//横センサーの光が壁と壁の間の角にきた時


///////
///////横センサ値を距離に直してオフセット調整云々  右に曲がる-左のセンサー値で  左に曲がる-右のセンサー値 スラロームで適用
///L 0(左に37ずれ):901/30:608/60:363://74(右に37ずれ):289
///右に曲がるとき, 左センサ(センサ値x)に対しては左向きを正として y = ((37-0)/(901-595))*(x-595)+0 　その後, yだけ左によっているので, yだけオフセットを増やす 両符号
///R 0:599/30:485/60:275//74:178
///左に曲がるとき, 右センサ(センサ値x)に対しては右向きを正として y = ((37-0)/(599-388.5))*(x-388.5)+0

///
///L 0(左に37ずれ):901///74(右に37ずれ):289
///R 0(左に37ずれ):599///74(右に37ずれ):178


//0 (52)   と74 は妥当  194 機体104 機体半分52　　178 89

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
		if(((float)g_sensor[0][0]>FrontWallInterferenceFL)||((float)g_sensor[3][0]>FrontWallInterferenceFR)){
			PID_wall = 0;
		}else{
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
		}else{
		PID_wall = 0;
		}
}
}
}

void wall_cut_control(void){//壁切れ補正をするときしか使わない
	if (wall_cut_control_flg==1){
	if(((float)g_sensor[0][0]>FrontWallInterferenceFR)||((float)g_sensor[3][0]>FrontWallInterferenceFL)){
//		PID_wall = 0;
	}else{
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
//	if(g_WallControlStatus==0b00){
//		PID_wall = 0;
//	}//2進数は0bを数字の接頭辞につければいけるとか？
	if(g_WallControlStatus==0b01){
		//進む距離を調整する(160mm進んでいる時に起きたら　起きた瞬間に台形加速の進むべき距離を調整　実測値を元に調整)
		wall_cut_control_start_flg=1;
//		PID_wall = SENSOR_GAIN*(2*(float)(g_sensor[1][0]-CENTER_L));

	}
	else if(g_WallControlStatus==0b10){
		//進む距離を調整する(160mm進んでいる時に起きたら　起きた瞬間に台形加速の進むべき距離を調整　実測値を元に調整)
		wall_cut_control_start_flg=1;
//		PID_wall = SENSOR_GAIN*(-2*(float)(g_sensor[2][0]-CENTER_R));
	}
	else if(g_WallControlStatus==0b11){
		wall_cut_control_start_flg=1;
//		PID_wall = SENSOR_GAIN*(((float)g_sensor[1][0]-CENTER_L)-((float)g_sensor[2][0]-CENTER_R));
	}
	else{
//	PID_wall = 0;
	}
}
}
}

void near_wall_cutting(void){
	if(near_wall_cutting_flg==1){
		if(((float)g_sensor[0][0]>nearest_FrontWallL)||((float)g_sensor[3][0]>nearest_FrontWallR)){
			wall_control_flg=0;
		}else{

		}
	}
}

//void front_wall_control(void){
//	if(((float)g_sensor[1][0]>FrontWallR)||((float)g_sensor[2][0]>FrontWallL)){
//		break;//前壁の値が一定値以下になったら20 mm前進を終わらせ、スラロームにいけるようにしているつもり
//	//printf("%f\n\r",vel);
////			printf("vel=%f,left_dis=%f,x_dec=%f,v_end=%f,accm=%f\n\r",vel,left_dis,x_dec,v_end,accm);
//}
//}

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

//左手法の
