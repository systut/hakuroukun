
//st:steering ac:accel pm:potentiometer

#define SPEED_ST 255    //stモーター速度 (0-255)
#define SPEED_AC 255    //acモーター速度
#define PM_ST_N 555     //直進時のst pm値 //565 250
#define PM_ST_LIMR 200  //PM_ST_Nと右操舵限度のpm値の差
#define PM_ST_LIML 290  //PM_ST_Nと左操舵限度のpm値の差
#define PM_AC_N 290     //停止時のac pm値290
#define PM_AC_LIMU 390  //PM_AC_Nとアクセル踏込限度のpm値の差
#define PM_AC_LIMD 20   //PM_AC_Nとアクセル戻し限度のpm値の差

const int BUTTON_ST_R = 30;  //右操舵ボタン
const int BUTTON_ST_L = 31;  //左操舵ボタン
const int BUTTON_AC_U = 32;  //加速ボタン
const int BUTTON_AC_D = 33;  //減速ボタン

//モータードライバ接続ピン DIR:回転方向 PWM:回転速度
const int MD_ST_DIR = 6;
const int MD_ST_PWM = 7;
const int MD_AC_DIR = 11;
const int MD_AC_PWM = 12;

//操作限度を示すLED接続ピン
const int LED_ST = 40;
const int LED_AC = 41;

const int RELAY_ALARM = 50;
const int RELAY_MOTOR = 51;

////テスト用（Uno）
//const int BUTTON_ST_R = 32; //右操舵ボタン
//const int BUTTON_ST_L = 33; //左操舵ボタン
//const int BUTTON_AC_U = 2; //加速ボタン
//const int BUTTON_AC_D = 3; //減速ボタン
//
////モータードライバ接続ピン DIR:回転方向 PWM:回転速度
//const int MD_ST_DIR = 5;
//const int MD_ST_PWM = 6;
//const int MD_AC_DIR = 10;
//const int MD_AC_PWM = 11;
//
////操作限度を示すLED接続ピン
//const int LED_ST = 12;
//const int LED_AC = 13;

int pm_st = 0, pm_ac = 0;                                        //pm読み値
int push_st_r = 0, push_st_l = 0, push_ac_u = 0, push_ac_d = 0;  //操作ボタンの状態
int sig_st_r = 0, sig_st_l = 0, sig_ac_u = 0, sig_ac_d = 0;      //モーター操作信号
int light_st = 0, light_ac = 0;                                  //LED操作信号

void setup() {
  Serial.begin(9600);

  pinMode(BUTTON_ST_R, INPUT);
  pinMode(BUTTON_ST_L, INPUT);
  pinMode(BUTTON_AC_U, INPUT);
  pinMode(BUTTON_AC_D, INPUT);

  pinMode(MD_ST_DIR, OUTPUT);
  pinMode(MD_ST_PWM, OUTPUT);
  pinMode(MD_AC_DIR, OUTPUT);
  pinMode(MD_AC_PWM, OUTPUT);

  pinMode(LED_ST, OUTPUT);
  pinMode(LED_AC, OUTPUT);

  pinMode(RELAY_ALARM, OUTPUT);
  pinMode(RELAY_MOTOR, OUTPUT);

  digitalWrite(RELAY_ALARM, LOW);
  digitalWrite(RELAY_MOTOR, LOW);
}

void loop() {
  //各ポテンショメータの値を参照
  pm_st = analogRead(0);
  pm_ac = analogRead(1);

  //各プッシュボタンの状態を保存
  push_st_r = digitalRead(BUTTON_ST_R);
  push_st_l = digitalRead(BUTTON_ST_L);
  push_ac_u = digitalRead(BUTTON_AC_U);
  push_ac_d = digitalRead(BUTTON_AC_D);

  //操作信号初期化
  light_st = 0;
  light_ac = 0;
  sig_st_r = 0;
  sig_st_l = 0;
  sig_ac_u = 0;
  sig_ac_d = 0;

  //プッシュボタン:操舵右の処理
  //操作限度の場合LEDを点灯し操作しない
  //操作可能な場合モーターを駆動
  if (push_st_r == HIGH) {
    if (pm_st < PM_ST_N - PM_ST_LIMR) {
      light_st = 1;
    } else {
      sig_st_r = 1;
    }
  }

  //プッシュボタン:操舵左の処理
  else if (push_st_l == HIGH) {
    if (pm_st > PM_ST_N + PM_ST_LIML) {
      light_st = 1;
    } else {
      sig_st_l = 1;
    }
  }

  //プッシュボタン:アクセル加速の処理
  else if (push_ac_u == HIGH) {
    if (pm_ac > PM_AC_N + PM_AC_LIMU) {
      light_ac = 1;
    } else {
      sig_ac_u = 1;
    }
  }

  //プッシュボタン:アクセル減速の処理
  else if (push_ac_d == HIGH) {
    if (pm_ac < PM_AC_N - PM_AC_LIMD) {
      light_ac = 1;
    } else {
      sig_ac_d = 1;
    }
  }

  //信号に従いSTモーターを操作
  if (sig_st_r == HIGH) {
    digitalWrite(MD_ST_DIR, LOW);
    analogWrite(MD_ST_PWM, SPEED_ST);
    Serial.println("右操舵");
  }

  else if (sig_st_l == HIGH) {
    digitalWrite(MD_ST_DIR, HIGH);
    analogWrite(MD_ST_PWM, SPEED_ST);
    Serial.println("左操舵");
  }

  else {
    analogWrite(MD_ST_PWM, 0);
    digitalWrite(MD_ST_DIR, LOW);
  }

  //信号に従いACモーターを操作
  if (sig_ac_u == HIGH) {
    digitalWrite(MD_AC_DIR, LOW);
    analogWrite(MD_AC_PWM, SPEED_AC);
    Serial.println("アクセル加速");
  }

  else if (sig_ac_d == HIGH) {
    digitalWrite(MD_AC_DIR, HIGH);
    analogWrite(MD_AC_PWM, SPEED_AC);
    Serial.println("アクセル減速");
  }

  else {
    analogWrite(MD_AC_PWM, 0);
    digitalWrite(MD_AC_DIR, LOW);
  }

  //信号に従いLEDを点灯
  if (light_st == HIGH) {
    digitalWrite(LED_ST, HIGH);
    Serial.println("stlim");
  } else {
    digitalWrite(LED_ST, LOW);
  }

  if (light_ac == HIGH) {
    digitalWrite(LED_AC, HIGH);
    Serial.println("aclim");
  } else {
    digitalWrite(LED_AC, LOW);
  }

  Serial.print(pm_st);
  Serial.print(",");
  Serial.println(pm_ac);

  delay(50);
}
