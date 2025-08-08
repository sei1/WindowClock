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



//---------------------------------
// グローバル変数の宣言
//---------------------------------

//7セグLEDの0～9を点灯させるビットの配列
uint8_t seg[10] = {
	0b01111110, 0b00001100, 0b10110110, 0b10011110, 0b11001100,
	0b11011010, 0b11111010, 0b01001110, 0b11111110, 0b11011110
};

//7セグLEDに表示させる4桁の数字
volatile uint8_t min  = 0;
volatile uint8_t hour = 0;

uint16_t wakeup = 0;



//ボタン操作を受け付けながら待機する関数
void sens_delay_ms(uint16_t num) {

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

//7セグをすべて消灯する関数
void seg_all_off(void) {

	//他のセルの消灯ドットを一瞬でも光らせないようPA1~7までとPC0を一度全て消灯
	VPORTA_OUT = VPORTA_OUT & 0b00000001;
	VPORTC_OUT = VPORTC_OUT & 0b11111110;
	//ダイナミック点灯用トランジスタも全てOFF
	VPORTB_OUT = VPORTB_OUT & 0b11001111;
	VPORTC_OUT = VPORTC_OUT & 0b11110001;
}


//TCA割り込み
ISR (TCA0_CMP1_vect) {

	//wakeupが0ならセグをすべて消灯してそれ以外を実行しない
	//メインループのseg_all_off関数とsleep_mode関数の間にこの割り込みが入り中途半端に7セグが点灯した状態でスリープするのを防ぐ記述
	if(!wakeup) {
		seg_all_off();
		return;
	}

	TCA0_SINGLE_CNT = 0;//カウントリセット
	TCA0_SINGLE_INTFLAGS = 0b00100000; //割り込み要求フラグを解除

	static uint8_t sel = 0;
	uint8_t dig1, dig2, dig3, dig4, dig5;

	dig1   = seg[min % 10];
	dig2   = seg[(min / 10) % 10];
	dig3   = 0b00000110;
	dig4   = seg[hour % 10];

	uint8_t zerocheck = (hour / 10) % 10;
	if(zerocheck == 0) {
		dig5 = 0b00000000;
	}else{
		dig5   = seg[zerocheck];
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

	//selの0~5トグル
	if ( ++sel == 5 ) {
		sel = 0;
		if(wakeup) wakeup--;
	}

}

//外部割り込み PB0が変化したら 両方のエッジを検出する(片方エッジにしたいがそうするとなぜかスタンバイから復帰しない)
ISR(PORTB_PORT_vect) {
	PORTB_INTFLAGS = PORTB_INTFLAGS | 0b00000010; //割り込み要求フラグ解除

	//PB0がLowだったら何もせず返す 両方のエッジを検出するようにしているので立ち下がりエッジ割り込みはここで無効にする
	if(!(VPORTB_IN & PIN0_bm)) {
		return;
	}

	wakeup = 800;

	//VPORTA_OUT = VPORTA_OUT | 0b00001000;
	//sens_delay_ms(5000);
	//VPORTA_OUT = VPORTA_OUT & 0b11110111;
	return;
}

//リアルタイムクロック 比較一致割り込み
ISR(RTC_CNT_vect) {
	RTC_CNT = 0;
	RTC_INTFLAGS = RTC_INTFLAGS | 0b00000010;

	if (++min >= 60) {
		min = 0;
		if(++hour >= 24) hour = 0;
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
	PORTB_PIN1CTRL = 0b00001000; //プルアップ有効


	
	CPU_CCP = 0xD8;//保護されたI/Oレジスタの変更を許可する
	CLKCTRL_XOSC32KCTRLA = 0b00000001; //外付け水晶振動子 イネーブル
	
	//RTC設定
	RTC_INTCTRL = 0b00000010; //比較一致割り込み許可
	RTC_CLKSEL  = 0b00000010; //クロック選択 XOSC32Kからの32.768 kHz
	//STATUS.CTRLABUSYフラグが1の間待機
	while((RTC_STATUS & 0b00000001));
	RTC_CTRLA   = 0b11111001; //ｽﾀﾝﾊﾞｲ休止動作でもRTC許可 16384分周 RTC許可

	//割り込みたい間隔の秒数-1
	RTC_CMP = 0;
	
	// //RTC PIT 周期割り込み設定
	// RTC_PITCTRLA = 0b01110001; //16384分周 周期割り込み計時器許可
	// RTC_PITINTCTRL = 0b00000001; //周期割り込み許可

	//タイマーA
	TCA0_SINGLE_CTRLA = 0b00001101; //1024分周 動作許可
	TCA0_SINGLE_CTRLB = 0b00000000; //
	TCA0_SINGLE_CMP1 = 1; // カウントがこの値に達したら割り込み(TCA0_CMP1_vect)が発生
	TCA0_SINGLE_INTCTRL = 0b00100000; //TRIGA割り込み許可

	set_sleep_mode(SLEEP_MODE_STANDBY); //スリープモードを設定

	//少し待機
	_delay_ms(5);

	sei(); //割り込み許可
	
	while (1) {

		if(!wakeup) {
			seg_all_off();
			sleep_mode();
		}

		_delay_ms(1);
		

	}
}

