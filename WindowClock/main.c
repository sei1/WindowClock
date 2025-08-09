/*
 * ATtiny1616
 * F_CPU=250000UL
 * 250kHz内部クロック(16MHz 64分周)
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

*/

//---------------------------------
// 定義とインクルード
//---------------------------------

#define F_CPU 250000UL
//#define F_CPU 32000UL


#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>

//7セグ表示モード
#define MODE_CLOCK 1
#define MODE_HOUR_SET 2
#define MODE_MIN_SET 3


//---------------------------------
// グローバル変数の宣言
//---------------------------------

//7セグLEDの0～9を点灯させるビットの配列
uint8_t seg[10] = {
	0b01111110, 0b00001100, 0b10110110, 0b10011110, 0b11001100,
	0b11011010, 0b11111010, 0b01001110, 0b11111110, 0b11011110
};

//7セグLEDに表示させる4桁の数字
uint8_t min  = 0;
uint8_t hour = 0;

//起き上がったら1以上の値を入れてタイマーでデクリメントしていく変数。0になったらスリープする
uint16_t wakeup = 0;

//表示モード
uint8_t mode = MODE_CLOCK;

//ボタン 長押しチェック用変数
volatile uint16_t long_push = 0;

//リクエスト変数
uint8_t request_increment_hour = 0;
uint8_t request_increment_min  = 0;

//現在の電源電圧と太陽電池電圧
float supply_v = 3.0;
float  solar_v = 3.0;


//---------------------------------
// プログラム本文
//---------------------------------

//ボタン操作を受け付けながら待機する関数
void sens_delay_ms (uint16_t num) {

	for (unsigned int i = 0; i < num; i++){
		if(!(VPORTB_IN & PIN1_bm)) {
			//ここにタクトスイッチが押された時の動作を記述
			VPORTA_OUT = VPORTA_OUT & 0b11110111;
			_delay_ms(300);
			VPORTA_OUT = VPORTA_OUT | 0b00001000;
		}
		_delay_ms(1);
	}
}

//キャパシタに蓄えられた電源電圧を取得する関数
float get_supply_v (void) {
	
	// 基準電圧1.1V / 電源電圧 を10bit(1024段階)で測定
	ADC0_MUXPOS = 0b00011101; //基準電圧

	ADC0_COMMAND = 1;//AD変換開始
	while(ADC0_COMMAND);
	ADC0_COMMAND = 0;//AD変換終了
	
	//電源電圧を算出して返す
	return 1023 * 1.1 / ADC0_RES;
}

	uint16_t x = 0;
	uint16_t y = 0;

//太陽電池の発電電圧を取得する関数
float get_solar_v (void) {


	
	// 基準電圧1.1V / 電源電圧 を10bit(1024段階)で測定
	ADC0_MUXPOS = 0b00011101; //基準電圧

	ADC0_COMMAND = 1;//AD変換開始
	while(ADC0_COMMAND);
	ADC0_COMMAND = 0;//AD変換終了

	y = ADC0_RES;

	//PB1をタクトスイッチ入力から一時的に太陽電池電圧の測定ピンに切り替える
	PORTB_PIN1CTRL = 0b00000000; //プルアップ無効 エッジを検出しない
	VPORTB_DIR    |= 0b00000010; //ポートB 
	VPORTB_OUT    &= 0b11111101; //ポートB
	_delay_us(500);
	VPORTB_DIR    &= 0b11111101; //ポートB

	//ADC入力ピンの選択
	// PB1ピンの電圧 / 電源電圧 を10bit(1024段階)で測定
	ADC0_MUXPOS = 0b00001010; //AIN10 = PB1

	ADC0_COMMAND = 1;//AD変換開始
	while(ADC0_COMMAND);
	ADC0_COMMAND = 0;//AD変換終了

	x = ADC0_RES;

	//PB1のプルアップを有効に戻す
	PORTB_PIN1CTRL = 0b00001001; //プルアップ有効 両方のエッジを検出する

	//太陽電池電圧を算出して返す
	return x * 1.1 / y;
}

//7セグをすべて消灯する関数
void seg_all_off (void) {

	//他のセルの消灯ドットを一瞬でも光らせないようPA1~7までとPC0を一度全て消灯
	VPORTA_OUT = VPORTA_OUT & 0b00000001;
	VPORTC_OUT = VPORTC_OUT & 0b11111110;
	//ダイナミック点灯用トランジスタも全てOFF
	VPORTB_OUT = VPORTB_OUT & 0b11001111;
	VPORTC_OUT = VPORTC_OUT & 0b11110001;
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

//TCA割り込み
ISR (TCA0_CMP0_vect) {

	//wakeupが0ならセグをすべて消灯してそれ以外を実行しない
	//メインループのseg_all_off関数とsleep_mode関数の間にこの割り込みが入り中途半端に7セグが点灯した状態でスリープするのを防ぐ記述
	if(!wakeup) {
		seg_all_off();
		return;
	}

	TCA0_SINGLE_CNT = 0;//カウントリセット
	TCA0_SINGLE_INTFLAGS = 0b00010000; //割り込み要求フラグを解除

	static uint8_t sel = 0;
	uint8_t dig1, dig2, dig3, dig4, dig5;

	dig1   = seg[min % 10];
	dig2   = seg[(min / 10) % 10];
	dig3   = 0b00000110;
	dig4   = seg[hour % 10];

	//dig5のみ0なら不点灯にする(ゼロサプレス)
	uint8_t zerocheck = (hour / 10) % 10;
	if(zerocheck == 0) {
		dig5 = 0b00000000;
	}else{
		dig5   = seg[zerocheck];
	}

	//時刻設定時の点滅演出
	//点滅カウンター
	static uint8_t wink = 0;
	if(mode == MODE_HOUR_SET) {
		if(++wink < 128) dig4 = dig5 = 0b00000000;
	}else if(mode == MODE_MIN_SET) {
		if(++wink < 128) dig1 = dig2 = 0b00000000;
	}else{
		wink = 0;
	}


	//他のセルの消灯ドットを一瞬でも光らせないようPA1~7までとPC0を一度全て消灯
	seg_all_off();

	switch ( sel ) {

		case 0:
		VPORTB_OUT = VPORTB_OUT | 0b00010000;
		//VPORTA_OUT = (dig1  & 0b01111111) | (PORTD & 0b10000000);//PD7に影響を与えないようマスク処理をしてPD0～6に値を代入
		VPORTA_OUT = dig1;
		break;

		case 1:
		VPORTC_OUT = VPORTC_OUT | 0b00001000;
		VPORTA_OUT = dig2;
		break;

		case 2:
		VPORTB_OUT = VPORTB_OUT | 0b00100000;
		VPORTA_OUT = dig3;
		break;

		case 3:
		VPORTC_OUT = VPORTC_OUT | 0b00000100;
		VPORTA_OUT = dig4;
		break;

		case 4:
		VPORTC_OUT = VPORTC_OUT | 0b00000010;
		VPORTA_OUT = dig5;
		break;

	}

	//5回に1回やること
	if ( ++sel == 5 ) {
		//selの0~5トグル動作
		sel = 0;

		//スリープへ向けwakeupを減らす
		if(wakeup) wakeup--;

		//タクトスイッチが長押しされている場合、長押しカウントを加算
		if(VPORTB_IN & PIN1_bm) {
			long_push = 0;
		}else{
			if(++long_push > 300) {
				long_push = 0;
				RTC_CNT = 0; //時刻設定をした後、秒数が0から始まるようにする
				change_mode(0);
			}
		}
	}



}

//外部割り込み PB0が変化したら 両方のエッジを検出する(片方エッジにしたいがそうするとなぜかスタンバイから復帰しない)
ISR(PORTB_PORT_vect) {
	
	PORTB_INTFLAGS = PORTB_INTFLAGS | 0b00000010; //割り込み要求フラグ解除

	//タクトスイッチが押されたら(PB1がLowだったら)
	if(!(VPORTB_IN & PIN1_bm)) {
		
		wakeup = 1600;
		
		switch (mode) {
			case MODE_CLOCK:
				
			break;

			case MODE_HOUR_SET:
				request_increment_hour = 1;
			break;

			case MODE_MIN_SET:
				request_increment_min = 1;
			break;
		}
		
		return;
	}

	//赤外線センサー PB0がLowに切り替わったら何もせず返す 両方のエッジを検出するようにしているので立ち下がりエッジ割り込みはここで無効にする
	if(!(VPORTB_IN & PIN0_bm)) {
		return;
	}

	//赤外線センサー PB0がHighだったら一定時間起き上がらせる
	if(VPORTB_IN & PIN0_bm) {
		wakeup = 800;
		return;
	}

	return;
}

//リアルタイムクロック 比較一致割り込み
ISR(RTC_CNT_vect) {
	RTC_CNT = 0;
	RTC_INTFLAGS = RTC_INTFLAGS | 0b00000010;

	//時計を進める
	if (mode == MODE_CLOCK && ++min >= 60) {
		min = 0;
		if(++hour >= 24) hour = 0;
	}

	//電圧測定(スリープ中にやる)
	if(!wakeup) {
		//電源電圧の取得
		supply_v = get_supply_v();

		//太陽電池電圧の取得
		solar_v = get_solar_v();

		if(solar_v >3)  _delay_ms(1);
	}

	
	return;
}

// ISR(RTC_PIT_vect) {
// 	RTC_PITINTFLAGS = 0b00000001; //周期割り込み要求フラグ解除

// 	//10秒に1回LEDを光らせる
// 	if(++count == 10) {
// 		count = 0;

// 		VPORTA_OUT = VPORTA_OUT | 0b00001000;
// 		_delay_ms(1);
// 		VPORTA_OUT = VPORTA_OUT & 0b11110111;
// 	}
	
// 	return;
// }


int main(void) {

	//■□■――――――――――――――――――――――――――――――――――――■□■
	//		□ レジスタの設定
	//■□■――――――――――――――――――――――――――――――――――――■□■
	CPU_CCP = 0xD8;//保護されたI/Oレジスタの変更を許可する
	CLKCTRL_MCLKCTRLA = 0b00000000; //16/20 MHz内部オシレータ
	//CLKCTRL_MCLKCTRLA = 0b00000001; //32 KHz内部超低消費電力オシレータ
	CPU_CCP = 0xD8;//保護されたI/Oレジスタの変更を許可する
	CLKCTRL_MCLKCTRLB = 0b00001011; //64分周

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
	PORTB_PIN1CTRL = 0b00001001; //プルアップ有効 両方のエッジを検出する


	
	CPU_CCP = 0xD8;//保護されたI/Oレジスタの変更を許可する
	CLKCTRL_XOSC32KCTRLA = 0b00000001; //外付け水晶振動子 イネーブル
	
	//RTC設定
	RTC_INTCTRL = 0b00000010; //比較一致割り込み許可
	RTC_CLKSEL  = 0b00000010; //クロック選択 XOSC32Kからの32.768 kHz
	//STATUS.CTRLABUSYフラグが1の間待機
	while((RTC_STATUS & 0b00000001));
	RTC_CTRLA   = 0b11111001; //ｽﾀﾝﾊﾞｲ休止動作でもRTC許可 32768分周 RTC許可

	//割り込みたい間隔の秒数-1
	RTC_CMP = 0;
	
	// //RTC PIT 周期割り込み設定
	// RTC_PITCTRLA = 0b01110001; //16384分周 周期割り込み計時器許可
	// RTC_PITINTCTRL = 0b00000001; //周期割り込み許可

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

		if(!wakeup) {
			seg_all_off();
			change_mode(MODE_CLOCK);
			sleep_mode();
		}

		//リクエスト処理
		if(request_increment_hour) {
			request_increment_hour = 0;
			while(!(VPORTB_IN & PIN1_bm));
			if(mode == MODE_HOUR_SET) {
				if(++hour >= 24) hour = 0;
				_delay_ms(100);
				request_increment_hour = 0;
			}
		}

		if(request_increment_min) {
			request_increment_min = 0;
			while(!(VPORTB_IN & PIN1_bm));
			if(mode == MODE_MIN_SET) {
				if(++min >= 60) {
					min = 0;
					if(++hour >= 24) hour = 0;
				}
				_delay_ms(100);
				request_increment_min = 0;
			}
		}

		_delay_ms(1);
		

	}
}

