
#include "AlpaCar.h"
#include "AlpacaLS.h"
#include "PIDController.h"
#include "constdef.h"
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_PWMServoDriver.h>
#include "PCInterrupt.h"

#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)

// PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);
// 距離センサ
VL53L0X vl53_r;
VL53L0X vl53_l;
const uint8_t vl53_r_addr = 0x30;
const uint8_t vl53_l_addr = 0x31;

// ラインセンサ
AlpacaLS line;

// ライントレースPID
PIController<double> line_pid(0.5, 0.0, 20.0);

// 現在地推定用
PosEstimator<double> pos_est(MM_BET_TIRES / MM_PER_CNT);


#define TURNING_SPD 30
#define TRACE_SPD   15
#define CTRL_SPD    50

unsigned long wt;
unsigned long intvl;
int flag = 0;
void func() {
	Serial.println("hoge");
	if ( digitalRead(wt) == LOW ) {
		unsigned long buf = millis();
		intvl = wt - buf;
		wt = buf;
		flag = 1;
	}
}

void setup() {
	// 念のため安定するまで待つ
	delay(100);

	// シリアル通信設定
	Serial.begin(115200);
	// (一応)I2C初期化
	Wire.begin();
	Wire.setClock(400000L);

	// PCA9685初期化(PWM:60Hz)
	pwm.begin();
	pwm.setPWMFreq(60);
	// とりあえずすべてOFF
	for ( int i=0; i<16; ++i ) { pwm.setPWM(i, 0, 0); }

	// 距離センサ(右)初期化
	pwm.setPWM(1, 0, 4095); // 左はOFF
	delay(50);
	vl53_r.init();
	vl53_r.setAddress(vl53_r_addr);
	vl53_r.setTimeout(100);
	vl53_r.startContinuous();
	// 距離センサ(左)初期化
	pwm.setPWM(1, 0, 0);
	delay(50);
	vl53_l.init();
	vl53_l.setAddress(vl53_l_addr);
	vl53_l.setTimeout(100);
	vl53_l.startContinuous();

	// PID設定
	AlpaCar.set_PID_params(2.0, 1.0, 150);
	// 車体設定
	pos_est.set_scale(MM_PER_CNT);

	// スイッチ押して離されるまで待機
	wait_switch_push_release();

	// ちょっと待つ
	delay(500);

	// 割り込み/モータ制御開始
	// AlpaCar.start(T2_INTR_MS);
	// AlpaCar.t2_updated = 0;
}

void loop() {
	Serial.print(vl53_r.readRangeSingleMillimeters());
	Serial.print("  ");
	Serial.print(vl53_l.readRangeSingleMillimeters());
	if (vl53_r.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
	Serial.println();
	// T字までライントレース
	// if (AlpaCar.t2_updated) {
	// 	// とりあえずフラグ折る
	// 	--AlpaCar.t2_updated;
	// 	round_trip_linetrace();
	// }
}

void round_trip_linetrace() {
	// 前回のライン読み取り値
	static double prev_linepos = 0.0;
	// とりあえず受信
	line.communicate();

	if ( line.state == STATE_NOLINE ) {
		// ラインがない場合
		// 停止
		delay(150);
		AlpaCar.set_target_count(0, 0);
		delay(300);
		
		// 前回ライン位置から、近い方に旋回
		if ( prev_linepos > 0 ) {
			AlpaCar.set_target_count(TURNING_SPD, -TURNING_SPD);
		} else {
			AlpaCar.set_target_count(-TURNING_SPD, TURNING_SPD);
		}

		// ライン読み取るまで待機
		while ( 1 ) {
			line.communicate();
			if ( line.state == STATE_INLINE ) { break; }
		}

		// 停止して少し待つ
		AlpaCar.set_target_count(0, 0);
		delay(300);
	}

	// ライントレース
	pid_linetrace();
	// ライン値の保存
	prev_linepos = line.position;
}


void printrl() {
	Serial.print(AlpaCar.spd_L);
	Serial.print('\t');
	Serial.println(AlpaCar.spd_R);
}

void printxy() {
	Serial.print("x: ");
	Serial.print(pos_est.get_true_x());
	Serial.print(", y: ");
	Serial.print(pos_est.get_true_y());
	Serial.print(", ang: ");
	Serial.print(pos_est.get_angle_deg());
	Serial.print(", RT: ");
	Serial.println(pos_est.get_true_route());
}


// PID制御で行くやつ
void pid_linetrace() {
	// ラインの場所取得
	double l_pos = line.position;

	// PI制御
	double ctrl = line_pid.control(l_pos, 0.0);
	int r_trg = ( TRACE_SPD + ctrl );
	int l_trg = ( TRACE_SPD - ctrl );

	// 適用
	AlpaCar.set_target_count(r_trg, l_trg);
}


void setLEDs(uint8_t led_mask) {
	int led6_val = (led_mask & 0x01) ? 4095 : 0;
	int led7_val = (led_mask & 0x02) ? 4095 : 0;
	int led8_val = (led_mask & 0x04) ? 4095 : 0;
	int led9_val = (led_mask & 0x08) ? 4095 : 0;
	pwm.setPWM(6, 0, led6_val);
	pwm.setPWM(7, 0, led7_val);
	pwm.setPWM(8, 0, led8_val);
	pwm.setPWM(9, 0, led9_val);
}

void wait_switch_push_release() {
	unsigned long watch = 0;
	int led_val = LOW;
	unsigned long buf = millis();

	// SW1/2 どちらかが LOW になるまで待機 (LED点滅)
	while ( AlpaCar.read_switch1() == HIGH && AlpaCar.read_switch2() == HIGH) {
		buf = millis();
		if ( buf - watch > 500 ) {
			watch = buf;
			if ( led_val == HIGH ) {
				led_val = LOW;
				setLEDs(0b0101);
			} else {
				led_val = HIGH;
				setLEDs(0b1010);
			}
		}
	}

	// チャタリング対策
	delay(100);

	// SW1/2 が両方 HIGH になるまで待機 (LED点滅)
	while ( AlpaCar.read_switch1() == LOW || AlpaCar.read_switch2() == LOW) {
		buf = millis();
		if ( buf - watch > 500 ) {
			watch = buf;
			if ( led_val == HIGH ) {
				led_val = LOW;
				setLEDs(0b0101);
			} else {
				led_val = HIGH;
				setLEDs(0b1010);
			}
		}
	}

	// 消灯
	setLEDs(0b0000);
}
