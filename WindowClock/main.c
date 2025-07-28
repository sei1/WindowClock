/*
 * ATtiny1616
 * F_CPU=3333333
 * 3.3MHz内部クロック
 *
 * Created: 2025/05/18
 * Author : SEIICHIRO SASAKI

――――――――――――――――――――――――――――――――――――
□ Fuses設定
――――――――――――――――――――――――――――――――――――
ACTIVE = ENABLED
LVL = BODLEVEL7
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
BODCFG = 0xE4 (valid)
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
PA1 … LCD RS
PA2 … LCD E
PA3 … UV LED OUT
PA4 … Y/0 タッチセンサー UP
PA5 … Y/1 タッチセンサー DOWN
PA6 … Y/2 タッチセンサー SET
PA7 … Y/3 タッチセンサー RIGHT

PB0 … ステッピングモーター Dir(High or Lowで回転方向の指定)
PB1 … ステッピングモーター Step(1パルス送信すると1分解能分回転させられる)
PB2 … ステッピングモーター x-Enable(Lowで有効化、命令を受け付け保持トルクも発生する。Highでモーターを無力化)
PB3 … ステッピングモーター y-Enable(Lowで有効化、命令を受け付け保持トルクも発生する。Highでモーターを無力化)
PB4 … マイクロスイッチ x-Limit
PB5 … マイクロスイッチ y-Limit

PC0 … LCDデータ送信用
PC1 … LCDデータ送信用
PC2 … LCDデータ送信用
PC3 … LCDデータ送信用

                            ┏━━━       ━━━┓
                       VDD ━┃ 1°       20 ┃━ GND
      Touch Sensor Y/0 PA4 ━┃ 2        19 ┃━ PA3 UV LED OUT
      Touch Sensor Y/1 PA5 ━┃ 3        18 ┃━ PA2 LCD E
      Touch Sensor Y/2 PA6 ━┃ 4        17 ┃━ PA1 LCD RS
      Touch Sensor Y/3 PA7 ━┃ 5        16 ┃━ PA0 RESET/UPDI
               y-Limit PB5 ━┃ 6        15 ┃━ PC3 LCD data
               x-Limit PB4 ━┃ 7        14 ┃━ PC2 LCD data
           Clock TOSC1 PB3 ━┃ 8        13 ┃━ PC1 LCD data
           Clock TOSC2 PB2 ━┃ 9        12 ┃━ PC0 LCD data
                  Step PB1 ━┃ 10       11 ┃━ PB0 Enable
                            ┗━━━━━━━━━━━━━┛

*/

//---------------------------------
// 定義とインクルード
//---------------------------------

#define F_CPU 16000000UL
//#define F_CPU 32000UL


#include <stdio.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>



//---------------------------------
// グローバル変数の宣言
//---------------------------------

uint8_t count = 9;


//回転させる関数
void rotate_p (void) {

	//モーターを有効化
	VPORTB_OUT = VPORTB_OUT & 0b11111110;

	//3200stepで1回転のため2560で4/5回転。4/5回転で1mm移動
	for (uint16_t i = 0; i < 3200; i++) {
		_delay_us(600);
		VPORTB_OUT = VPORTB_OUT | 0b00000010;
		_delay_us(10);
		VPORTB_OUT = VPORTB_OUT & 0b11111101;
	}

	//モーターを無力化
	VPORTB_OUT = VPORTB_OUT | 0b00000001;
}

// ISR(RTC_CNT_vect) {
// 	RTC_CNT = 0;
// 	RTC_INTFLAGS = RTC_INTFLAGS | 0b00000010;
	
	
// 	// VPORTA_OUT = VPORTA_OUT | 0b00001000;
// 	// _delay_ms(5);
// 	// VPORTA_OUT = VPORTA_OUT & 0b11110111;
// 	return;
// }

ISR(RTC_PIT_vect) {
	RTC_PITINTFLAGS = 0b00000001; //周期割り込み要求フラグ解除

	//10秒に1回LEDを光らせる
	if(++count == 10) {
		count = 0;

		VPORTA_OUT = VPORTA_OUT | 0b00001000;
		_delay_ms(1);
		VPORTA_OUT = VPORTA_OUT & 0b11110111;
	}
	
	return;
}


int main(void) {

	//■□■――――――――――――――――――――――――――――――――――――■□■
	//		□ レジスタの設定
	//■□■――――――――――――――――――――――――――――――――――――■□■
	CPU_CCP = 0xD8;//保護されたI/Oレジスタの変更を許可する
	CLKCTRL_MCLKCTRLA = 0b00000000; //16/20 MHz内部オシレータ
	//CLKCTRL_MCLKCTRLA = 0b00000001; //32 KHz内部超低消費電力オシレータ
	CPU_CCP = 0xD8;//保護されたI/Oレジスタの変更を許可する
	CLKCTRL_MCLKCTRLB = 0b00000000; //分周なし

	//入出力モード設定
	VPORTA_DIR = 0b00001110; //ポートA 
	VPORTB_DIR = 0b00000011; //ポートB 
	VPORTC_DIR = 0b00001111; //ポートC

	//出力の初期化
	VPORTA_OUT = 0b00000000; //ポートA
	VPORTB_OUT = 0b00000000; //ポートB
	VPORTC_OUT = 0b00000000; //ポートC

	//プルアップの有効化 各ピン毎に PINnCTRLで設定 ビット3に1を書くことでプルアップ有効 詳細はデータシートで
	PORTA_PIN4CTRL = 0b00001000;
	PORTA_PIN5CTRL = 0b00001000;
	PORTA_PIN6CTRL = 0b00001000;
	PORTA_PIN7CTRL = 0b00001000;
	
	CPU_CCP = 0xD8;//保護されたI/Oレジスタの変更を許可する
	CLKCTRL_XOSC32KCTRLA = 0b00000001; //外付け水晶振動子 イネーブル
	
	//RTC設定
	RTC_INTCTRL = 0b00000000; //比較一致割り込み禁止
	RTC_CLKSEL  = 0b00000010; //クロック選択 XOSC32Kからの32.768 kHz
	//STATUS.CTRLABUSYフラグが1の間待機
	while((RTC_STATUS & 0b00000001));
	RTC_CTRLA   = 0b11110001; //ｽﾀﾝﾊﾞｲ休止動作でもRTC許可 16384分周 RTC許可
	
	//RTC PIT 周期割り込み設定
	RTC_PITCTRLA = 0b01110001; //16384分周 周期割り込み計時器許可
	RTC_PITINTCTRL = 0b00000001; //周期割り込み許可

	RTC_CMP = 1;


	//少し待機
	_delay_ms(5);

	sei(); //割り込み許可
	
	set_sleep_mode(SLEEP_MODE_PWR_DOWN); //スリープモードを設定

	while (1) {

		sleep_mode();

	}
}

