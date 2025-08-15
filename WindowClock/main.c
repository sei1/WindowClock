/*
 * ATtiny1616
 * F_CPU=250000UL
 * 1MHz内部クロック(16MHz 16分周)
 *
 * Created: 2025/05/18
 * Author : SEIICHIRO SASAKI

――――――――――――――――――――――――――――――――――――
□ Fuses設定
――――――――――――――――――――――――――――――――――――
ACTIVE = DIS
LVL = BODLEVEL0
SAMPFREQ = 1KHZ
SLEEP = DIS
FREQSEL = 20MHZ
OSCLOCK = [ ]
CRCSRC = NOCRC
EESAVE = [ ]
RSTPINCFG = UPDI
SUT = 64MS
CMPA = [ ]
CMPAEN = [ ]
CMPB = [ ]
CMPBEN = [ ]
CMPC = [ ]
CMPCEN = [ ]
CMPD = [ ]
CMPDEN = [ ]
PERIOD = OFF
WINDOW = OFF

APPEND = 0x00 (valid)
BODCFG = 0x00 (valid)
BOOTEND = 0x00 (valid)
OSCCFG = 0x02 (valid)
SYSCFG0 = 0xF6 (valid)
SYSCFG1 = 0x07 (valid)
TCD0CFG = 0x00 (valid)
WDTCFG = 0x00 (valid)


――――――――――――――――――――――――――――――――――――
□ I/Oピン設定
――――――――――――――――――――――――――――――――――――

PA0 … RESET/UPDI
PA1 … 7seg A A/D1
PA2 … 7seg A B/D2
PA3 … 7seg A C/D3
PA4 … 7seg A D
PA5 … 7seg A E
PA6 … 7seg A F
PA7 … 7seg A G

PB0 … IR Sensor In
PB1 … Switch In
PB2 … Clock TOSC2
PB3 … Clock TOSC1
PB4 … 7seg C4
PB5 … 7seg C5

PC0 … 7seg A DP
PC1 … 7seg C1
PC2 … 7seg C2
PC3 … 7seg C3

                            ┏━━━       ━━━┓
                       VDD ━┃ 1°       20 ┃━ GND
              7seg A D PA4 ━┃ 2        19 ┃━ PA3 7seg A C/D3
              7seg A E PA5 ━┃ 3        18 ┃━ PA2 7seg A B/D2
              7seg A F PA6 ━┃ 4        17 ┃━ PA1 7seg A A/D1
              7seg A G PA7 ━┃ 5        16 ┃━ PA0 RESET/UPDI
               7seg C5 PB5 ━┃ 6        15 ┃━ PC3 7seg C3
               7seg C4 PB4 ━┃ 7        14 ┃━ PC2 7seg C2
           Clock TOSC1 PB3 ━┃ 8        13 ┃━ PC1 7seg C1
           Clock TOSC2 PB2 ━┃ 9        12 ┃━ PC0 7seg A DP
             Switch In PB1 ━┃ 10       11 ┃━ PB0 IR Sensor In
                            ┗━━━━━━━━━━━━━┛


***9時間の長時間スリープを実現するための手法***
表示する時刻(算出時刻)と実際に計測し続ける時刻(保存時刻)を別ける

起動時と時計合わせ時に保存時刻を設定する
保存時刻をRTC割り込みで9時間毎に進める
表示する時間はRTC_CNTを実時間に足し合わせた時間。これを算出時刻と呼ぶ
赤外線センサーで起きた時に算出時刻を作り、TCA割り込みでCNTを監視し増加したら表示する時間を増やす
起きる毎に算出時刻は都度生成し、スリープをまたいで保存したりはしない
長時間スリープの電源節約以外のメリットとしてはプログラムが単純になることと
仮に割り込みに不具合が生じ算出時刻が少しずれたとしても保存時刻は正確に9時間毎に加算されるため誤差の蓄積がないことが挙げられる

*/

//---------------------------------
// 定義とインクルード
//---------------------------------

#define   F_CPU 1000000UL //16MHzを16分周

#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

//7セグ表示モード
#define MODE_CLOCK 1
#define MODE_HOUR_SET 2
#define MODE_MIN_SET 3

//キャパシタ保護のため放電動作を行うキャパシタ電圧
#define MAX_SUPPLY_V 5.2

//低電圧リセットをかける電圧
#define MIN_SUPPLY_V 1.70


//---------------------------------
// グローバル変数の宣言
//---------------------------------

//7セグLEDの0～9を点灯させるビットの配列
uint8_t seg[10] = {
	0b01111110, 0b00001100, 0b10110110, 0b10011110, 0b11001100,
	0b11011010, 0b11111010, 0b01001110, 0b11111110, 0b11011110
};

//保存時刻
uint8_t memory_min  = 0;
uint8_t memory_hour = 0;

//算出時刻 保存RTC_CNTを足し合わせた時刻
uint8_t calc_min  = 0;
uint8_t calc_hour = 0;

//起き上がったら1以上の値を入れてタイマーでデクリメントしていく変数。0になったらスリープする
uint16_t wakeup = 0;

//表示モード
uint8_t mode = MODE_CLOCK;

//ボタン 長押しチェック用変数
volatile uint16_t long_push = 0;

//長押しでモードを切り替えた後、ボタンを離した時に1回ボタンを入力したことになってしまうのを防ぐため
//モード切替後最初のボタン入力を1回無効にするフラグ
uint8_t change_mode_after = 0;

//現在の電源電圧と太陽電池電圧
float supply_v = 0.0;
float  solar_v = 0.0;

//MODE_CLOCKでボタンを押した時(非長押し)、一時的に電圧を表示するフラグ。wakeupと同様にタイマーでデクリメントしていき0になったら表示を戻す
uint8_t display_v = 0;

//起きたのにまだ電圧測定してないの？的なフラグ
//寝る前に1をセットして起きたら電圧を測定し0に戻す。0なら以後電圧を測定しない。この動作で起きた時に1回だけ電圧測定する動作になる
uint8_t yet_v = 1;

//7セグの明るさレベル 6段階 16.67% 25% 30% 50% 75% 100%
uint8_t brightness = 6;

//キャパシタ保護用放電動作中フラグ
volatile uint8_t discharge = 0;

//RTC_CNTを120で割った余りが0の時、算出時刻を1回だけ更新するためのフラグ
//起動時や時計変更時のRTC_CNTが0でもこの条件を満たし、算出時刻を進めてしまうため初期値は既に更新したことを表す1とする
static uint8_t calc_updated = 1;

//12時間表記をするか。1ならする。0なら24時間表記。デフォルトは7セグの消費電力を削減するために12時間表記
uint8_t system12 = 1;

//3回連続で時刻合わせが行われたら24時間表記にする そのためのカウンター
uint8_t s24count = 0;

//起動後まだ時刻を設定していないフラグ
uint8_t unset = 1;

//7セグに電圧を表示用するために使う変数
uint8_t v_dig1 = 0;
uint8_t v_dig2 = 0;
uint8_t v_dig4 = 0;
uint8_t v_dig5 = 0;

//
uint8_t old_calc_min = 255;
uint8_t old_calc_hour = 255;
uint8_t old_dig1 = 0;
uint8_t old_dig2 = 0;
uint8_t old_dig4 = 0;
uint8_t old_dig5 = 0;

//---------------------------------
// プログラム本文
//---------------------------------


//キャパシタに蓄えられた電源電圧と太陽電池の発電電圧を取得する関数
void get_v (void) {
	
	uint16_t x = 0;
	uint16_t y = 0;

	//PB1をタクトスイッチ入力から一時的に太陽電池電圧の測定ピンに切り替える
	PORTB_PIN1CTRL = 0b00000000; //プルアップ無効 エッジを検出しない
	VPORTB_DIR    |= 0b00000010; //出力モードに変更
	VPORTB_OUT    &= 0b11111101; //出力Low

	//この状態でPB1の電圧を抜いている間に基準電圧1.1V / 電源電圧を測定しておく

	// 基準電圧1.1V / 電源電圧 を10bit(1024段階)で測定
	ADC0_MUXPOS = 0b00011101; //基準電圧

	ADC0_COMMAND = 1;//AD変換開始
	while(ADC0_COMMAND);
	ADC0_COMMAND = 0;//AD変換終了

	y = ADC0_RES;
	
	//電源電圧を算出
	supply_v = 1023 * 1.1 / y;


	//PB1を入力モードに戻す
	VPORTB_DIR    &= 0b11111101; 

	//ADC入力ピンの選択
	// PB1ピンの電圧 / 電源電圧 を10bit(1024段階)で測定
	ADC0_MUXPOS = 0b00001010; //AIN10 = PB1

	ADC0_COMMAND = 1;//AD変換開始
	while(ADC0_COMMAND);
	ADC0_COMMAND = 0;//AD変換終了

	x = ADC0_RES;

	//PB1のプルアップを有効に戻す
	PORTB_PIN1CTRL = 0b00001001; //プルアップ有効 両方のエッジを検出する

	//太陽電池電圧を算出
	solar_v = x * 1.1 / y;



	//7セグの明るさ設定
	//太陽電池の電圧によって周囲の明るさを判定し7セグの明るさを変化させる
	if(solar_v > 1.8 || discharge) {
		brightness = 6;
	}else if(solar_v > 1.2) {
		brightness = 5;
	}else if(solar_v > 0.8) {
		brightness = 4;
	}else if(solar_v > 0.5) {
		brightness = 3;
	}else if(solar_v > 0.3) {
		brightness = 2;
	}else{
		brightness = 1;
	}

	//電圧を7セグに表示する準備 ここで行っておくことで計算が1回で済みTCA割り込みの動作が軽快になる
	uint8_t spv = supply_v * 10;
	uint8_t slv =  solar_v * 10;
	v_dig1  = seg[spv % 10];
	v_dig2  = seg[(spv / 10) % 10];
	v_dig4  = seg[slv % 10];
	v_dig5  = seg[(slv / 10) % 10];
}

//ボタン操作を受け付けながら待機する関数
void sens_delay_ms (uint16_t num) {

	for (unsigned int i = 0; i < num; i++){
		if(!(VPORTB_IN & PIN1_bm)) {

			//ここにタクトスイッチが押された時の動作を記述
			wakeup = 5000;
		
			switch (mode) {
				case MODE_CLOCK:
					while(!(VPORTB_IN & PIN1_bm));
					if(change_mode_after) {
						change_mode_after = 0;
					}else{
						//電圧の取得
						get_v();
						display_v = 255;
						wakeup = 1000; //電圧表示だけなら長く表示する必要ないのでwakeup値上書き
						_delay_ms(100);
					}
				break;

				case MODE_HOUR_SET:
					while(!(VPORTB_IN & PIN1_bm));
					if(change_mode_after) {
						change_mode_after = 0;
					}else{
						if(++memory_hour >= 24) memory_hour = 0;
						calc_hour = memory_hour;
						_delay_ms(100);
					}
				break;

				case MODE_MIN_SET:
					while(!(VPORTB_IN & PIN1_bm));
					if(change_mode_after) {
						change_mode_after = 0;
					}else{
						if(++memory_min >= 60) {
							memory_min = 0;
							if(++memory_hour >= 24) memory_hour = 0;
						}
						calc_hour = memory_hour;
						calc_min = memory_min;
						_delay_ms(100);
					}
				break;
			}
		}
		_delay_ms(1);
	}
}

//7セグをすべて消灯する関数
void seg_all_off (void) {

	//他のセグメントを一瞬でも光らせないようPA1~7までとPC0を一度全て消灯
	VPORTA_OUT &= 0b00000001;
	VPORTC_OUT &= 0b11110000; //PC1～3はダイナミック点灯用トランジスタ。これらもOFF
	//ダイナミック点灯用トランジスタも全てOFF
	VPORTB_OUT &= 0b11001111;
}

//モードを切り替える関数
//引数に0を指定した場合は次の定数のモードへ 定数を指定した場合はそのモードへ
void change_mode (uint8_t cmode) {
	if(cmode) {
		mode = cmode;
	}else if(mode == MODE_MIN_SET) {
		mode = MODE_CLOCK;
	}else{
		mode++;
	}
}

//保存時刻を初期化
void init_memory_clock (void) {
	if(unset) { //未設定なら00:00で初期化
		memory_hour = calc_hour = 0;
		memory_min = calc_min = 0;
	}else{
		memory_hour = calc_hour;
		memory_min = calc_min;
	}
	RTC_CNT = 0;
	calc_updated = 1;
}

//TCA割り込み
ISR (TCA0_CMP0_vect) {

	TCA0_SINGLE_CNT = 0;//カウントリセット
	TCA0_SINGLE_INTFLAGS |= 0b00010000; //割り込み要求フラグを解除

	//wakeupが0ならセグをすべて消灯してそれ以外を実行しない
	//メインループのseg_all_off関数とsleep_mode関数の間にこの割り込みが入り中途半端に7セグが点灯した状態でスリープするのを防ぐ記述
	if(!wakeup) {
		seg_all_off();
		return;
	}

	static uint8_t out_dig = 0;//発光させる桁
	static uint8_t colon = 0;//点滅コロン
	uint8_t dig1, dig2, dig3, dig4, dig5 = 0;
	uint8_t dig2c, dig5c;
	dig2c = dig5c = 0;

	//現状このドットは使用していないのでコメントアウト 使うときは有効化してください
	//uint8_t dig1c, dig4c,;
	//dig1c = di45c = 0;

	//太陽電池の発電電圧とキャパシタの電圧を表示
	if(display_v && mode == MODE_CLOCK) {
		dig1  = v_dig1;
		dig2  = v_dig2;
		dig3  = 0b00000000;
		dig4  = v_dig4;
		dig5  = v_dig5;
		dig2c = dig5c = 0b00000001;//ドット(小数点)

	}else if(unset && mode == MODE_CLOCK) {//時計未設定なら全てハイフンで上書き
		dig1 = dig2 = dig4 = dig5 = 0b10000000;
		dig3 = 0b00000110;

	}else{//時刻を表示

		//時刻が更新されていれば表示を刷新するための計算を行う。更新されていなければ前回表示したものをそのまま表示
		if(old_calc_min != calc_min || old_calc_hour != calc_hour) {
			//これまでの値を古い値を格納する変数に移し替える
			old_calc_min = calc_min;
			old_calc_hour = calc_hour;

			//12時間表記設定と24時間表記設定で表示を切り替える
			uint8_t display_hour = 0;
			if(system12) {
				if(!calc_hour) display_hour = 12; //0時を12時と表記
				else if (calc_hour > 12) display_hour = calc_hour - 12; //13時以降を1時、2時…と表す
				else display_hour = calc_hour;
			}else{
				display_hour = calc_hour;
			}

			old_dig1 = dig1  = seg[calc_min % 10];
			old_dig2 = dig2  = seg[(calc_min / 10) % 10];
			old_dig4 = dig4  = seg[display_hour % 10];

			//dig5のみ0なら不点灯にする(ゼロサプレス)
			uint8_t zerocheck = (display_hour / 10) % 10;
			if(zerocheck == 0) {
				old_dig5 = dig5 = 0b00000000;
			}else{
				old_dig5 = dig5   = seg[zerocheck];
			}

		}else{
			dig1 = old_dig1;
			dig2 = old_dig2;
			dig4 = old_dig4;
			dig5 = old_dig5;
		}

		dig3  = colon;

	}

	//時刻設定時の点滅演出
	//点滅カウンター
	static uint16_t wink = 0;
	if(mode == MODE_HOUR_SET) {
		if(++wink < 512) dig4 = dig5 = 0b00000000;
		else if (wink > 1023) wink = 0;
	}else if(mode == MODE_MIN_SET) {
		if(++wink < 512) dig1 = dig2 = 0b00000000;
		else if (wink > 1023) wink = 0;
	}else{
		wink = 0;
	}


	//他のセグメントを一瞬でも光らせないようPA1~7までとPC0を一度全て消灯
	seg_all_off();

	//明るさ設定値に応じた点灯判定
	uint8_t seg_on = 0; //1を入れるとセグがONになる
	static uint8_t bn_pwm_count = 1; //7セグを間欠で点灯させるために0～12までを繰り返し数えるカウンター

	if(++bn_pwm_count > 24) bn_pwm_count = 1;

	//間欠点灯で明るさを調整
	switch (brightness) {

		case 1: //12.5% bn_pwm_countが8,16,24のタイミングで点灯
			if(
				bn_pwm_count == 8  ||
				bn_pwm_count == 16 ||
				bn_pwm_count == 24
			) {
				seg_on = 1;
			}
		break;

		case 2: //16.67% bn_pwm_countが6,12,18,24のタイミングで点灯
			if(
				bn_pwm_count == 6  ||
				bn_pwm_count == 12 ||
				bn_pwm_count == 18 ||
				bn_pwm_count == 24
			) {
				seg_on = 1;
			}
		break;

		case 3: //25% bn_pwm_countが4,8,12,16,20,24のタイミングで点灯
			if(bn_pwm_count % 4) {
				seg_on = 1;
			}
		break;

		case 4: //33.33% bn_pwm_countが3,6,9,12,15,18,21,24のタイミングで点灯
			if(bn_pwm_count % 3) {
				seg_on = 1;
			}
		break;

		case 5: //50% bn_pwm_countが2,4,6,8,10,12,14,16,18,20,22,24のタイミングで点灯
			if(bn_pwm_count % 2) {
				seg_on = 1;
			}
		break;

		case 6: //100% bn_pwm_countがいくつでも全てのタイミングで点灯
			seg_on = 1;
		break;
	}

	//点灯実行
	if(seg_on) {
		switch (out_dig) {

			case 0:
			VPORTB_OUT |= 0b00010000;
			VPORTA_OUT = dig1;
			//現状このドットは使用していないのでコメントアウト 使うときは有効化してください
			//VPORTC_OUT = (dig1c  & 0b00000001) | (VPORTC_OUT & 0b11111110);//PC1～7に影響を与えないようマスク処理をしてPC0に値を代入
			break;

			case 1:
			VPORTC_OUT |= 0b00001000;
			VPORTA_OUT = dig2;
			if(display_v) VPORTC_OUT = (dig2c  & 0b00000001) | (VPORTC_OUT & 0b11111110);//PC1～7に影響を与えないようマスク処理をしてPC0に値を代入
			break;

			case 2:
			VPORTB_OUT |= 0b00100000;
			VPORTA_OUT = dig3;
			break;

			case 3:
			VPORTC_OUT |= 0b00000100;
			VPORTA_OUT = dig4;
			//現状このドットは使用していないのでコメントアウト 使うときは有効化してください
			//VPORTC_OUT = (dig4c  & 0b00000001) | (VPORTC_OUT & 0b11111110);//PC1～7に影響を与えないようマスク処理をしてPC0に値を代入
			break;

			case 4:
			VPORTC_OUT |= 0b00000010;
			VPORTA_OUT = dig5;
			if(display_v) VPORTC_OUT = (dig5c  & 0b00000001) | (VPORTC_OUT & 0b11111110);//PC1～7に影響を与えないようマスク処理をしてPC0に値を代入
			break;

		}
	}


	//5回に1回やること
	if (++out_dig == 5) {
		//out_digの0~5トグル動作
		out_dig = 0;

		//コロンの点滅動作
		if(!(RTC_CNTL & 0b00000001) || mode != MODE_CLOCK) { //コロンの点滅
			colon = 0b00000110;
		}else{
			colon = 0b00000000;
		}

		//算出時刻を進める
		//old_rtc_cntにカウント値を保存して最新のカウント値と比較し、0.5秒に1回だけ剰余演算で判定する
		static uint16_t old_rtc_cnt = 0;
		if(old_rtc_cnt != RTC_CNT) {
			old_rtc_cnt = RTC_CNT;
			if(!(RTC_CNT % 120)) { 
				if(calc_updated == 0) {
					calc_updated = 1;
					if(++calc_min >= 60) {
						calc_hour++;
						calc_min -= 60;
					}
					if(calc_hour >= 24) calc_hour -= 24;
				}
			}else{
				calc_updated = 0;
			}
		}

		//スリープへ向けwakeupを減らす
		if(wakeup) wakeup--;

		//電圧表示時間を減らす
		if(display_v) display_v--;

		//タクトスイッチが長押しされている場合、長押しカウントを加算
		if(VPORTB_IN & PIN1_bm) {
			long_push = 0;
		}else{
			if(++long_push > 1200) {
				long_push = 0;
				//時刻設定をした後、算出時刻を保存時刻に代入しカウントリセット
				init_memory_clock();
				change_mode(0);
				change_mode_after = 1;

				unset = 0; //時刻未設定フラグを折る
				//スリープを挟まず3回連続で時刻合わせを行った場合は24時間表記に切り替える
				if(mode == MODE_HOUR_SET) {
					s24count++;
					if(s24count >= 3) {
						s24count = 0;
						system12 = 0;
					}else{
						system12 = 1;
					}
				}
			}
		}
	}

}

//外部割り込み PB0が変化したら 両方のエッジを検出する(片方エッジにしたいがそうするとなぜかスタンバイから復帰しない)
ISR(PORTB_PORT_vect) {
	
	PORTB_INTFLAGS |= 0b00000010; //割り込み要求フラグ解除

	//赤外線センサー PB0がLowに切り替わったら何もせず返す 両方のエッジを検出するようにしているので立ち下がりエッジ割り込みはここで無効にする
	if(!(VPORTB_IN & PIN0_bm)) {
		return;
	}

	//赤外線センサー PB0がHighだったら一定時間起き上がらせる
	if(VPORTB_IN & PIN0_bm) {

		//まず電圧測定する
		if(yet_v) {
			yet_v = 0; //電圧測定をしたことをyet_vに記録
			get_v();
		}

		if(!wakeup) {

			//現在時刻を算出
			
			//眠っている間に計測した秒数を分に換算
			uint16_t slept_min = RTC_CNT / 120;

			calc_hour = memory_hour + slept_min / 60;
			calc_min  = memory_min  + slept_min % 60;

			if(calc_min >= 60) {
				calc_hour++;
				calc_min -= 60;
			}
			
			if(calc_hour >= 24) calc_hour -= 24;
			calc_updated = 1;
		}

		//一定時間起き上がらせる
		if(wakeup < 1000) wakeup = 1000;
		return;
	}

	return;
}

//リアルタイムクロック 比較一致割り込み
ISR(RTC_CNT_vect) {
	RTC_CNT = 0;
	RTC_INTFLAGS |= 0b00000010;

	//時計を9時間進める
	if (mode == MODE_CLOCK) {
		memory_hour += 9;
		if(memory_hour >= 24) memory_hour -= 24;
	}
	
	//日常点検作業
	if(!wakeup){
		get_v();
		//低電圧再起動処理 起きてる時にやると一瞬7セグがちらつくので起きてない時(厳密には起きてるカウントがされていない時)にやる
		if(supply_v <= MIN_SUPPLY_V) {
			//停止処理
			//ウォッチドッグタイマを0.008秒で起動
			wdt_enable(0b00000001);//0.008秒の場合、右4桁をデータシート上の8CLKのレジスタ設定値にする
			//待機(しているあいだにウォッチドッグリセットがかかる)
			_delay_ms(100);
		}
		//高電圧放電処理
		if(supply_v >= MAX_SUPPLY_V) {
			wakeup = 7000;
			discharge = 1;
		}
	}

	return;
}

int main(void) {

	//■□■――――――――――――――――――――――――――――――――――――■□■
	//		□ レジスタの設定
	//■□■――――――――――――――――――――――――――――――――――――■□■
	CPU_CCP = 0xD8;//保護されたI/Oレジスタの変更を許可する
	CLKCTRL_MCLKCTRLA = 0b00000000; //16/20 MHz内部オシレータ
	CPU_CCP = 0xD8;//保護されたI/Oレジスタの変更を許可する
	CLKCTRL_MCLKCTRLB = 0b00000111; //16分周

	//入出力モード設定
	VPORTA_DIR = 0b11111111; //ポートA 
	VPORTB_DIR = 0b11111100; //ポートB 
	VPORTC_DIR = 0b11111111; //ポートC

	//出力の初期化
	VPORTA_OUT = 0b00000000; //ポートA
	VPORTB_OUT = 0b00000000; //ポートB
	VPORTC_OUT = 0b00000000; //ポートC

	//焦電型赤外線センサー入力
	PORTB_PIN0CTRL = 0b00000001; //プルアップ無効 両方のエッジを検出する
	//タクトスイッチ入力
	PORTB_PIN1CTRL = 0b00001000; //プルアップ有効

	CPU_CCP = 0xD8;//保護されたI/Oレジスタの変更を許可する
	CLKCTRL_XOSC32KCTRLA = 0b00000001; //外付け水晶振動子 イネーブル
	
	//RTC設定
	RTC_INTCTRL = 0b00000010; //比較一致割り込み許可
	RTC_CLKSEL  = 0b00000010; //クロック選択 XOSC32Kからの32.768 kHz
	//STATUS.CTRLABUSYフラグが1の間待機
	while((RTC_STATUS & 0b00000001));
	RTC_CTRLA   = 0b11110001; //ｽﾀﾝﾊﾞｲ休止動作でもRTC許可 16384分周=0.5秒カウント RTC許可

	//割り込みたい間隔の秒数をx2-1して代入
	RTC_CMP = (uint16_t) 32400 * 2 - 1; //32400秒=540分=9時間 16ビット最大値以内で表せる極力長い時間を代入するとこの値になる
	RTC_CNT = 0;

	//タイマーA
	TCA0_SINGLE_CTRLA = 0b00001101; //1024分周 動作許可
	TCA0_SINGLE_CTRLB = 0b00000000; //
	TCA0_SINGLE_CMP0 = 1; // カウントがこの値に達したら割り込み(TCA0_CMP1_vect)が発生
	TCA0_SINGLE_INTCTRL = 0b00010000; //CMP0割り込み許可

	//ADC設定
	ADC0_CTRLA = 0b00000001; //ADC Enable
	ADC0_CTRLB = 0b00000000; //累積なし
	ADC0_CTRLC = 0b01010101; //VREF = VDD, プリスケーラ1/64
	VREF_CTRLA = 0b00010000; //内部基準電圧 1.1V

	set_sleep_mode(SLEEP_MODE_STANDBY); //スリープモードを設定

	//少し待機
	_delay_ms(5);

	sei(); //割り込み許可
	
	while (1) {

		//放電中ループ 起き続け3秒毎に電圧を計り下がっていたらブレイク
		while (discharge) {
			get_v();
			if(supply_v < (MAX_SUPPLY_V - 0.1)) {
				discharge = 0;
				wakeup = 0;
				break;
			}
			sens_delay_ms(3000);
			wakeup = 7000;
		}

		if(!wakeup) {
			//寝る準備
			seg_all_off();
			change_mode(MODE_CLOCK);
			display_v = 0;
			yet_v = 1;
			s24count = 0;
			old_calc_min = old_calc_hour = 255;
			//寝る
			sleep_mode();
		}
		
		sens_delay_ms(5);

	}
}

