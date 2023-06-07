//st:steering ac:accel pm:potentiometer

#define SPEED_ST 255 //stモーター速度 (0-255)
#define SPEED_AC 100 //acモーター速度
#define PM_ST_N 550 //直進時のst pm値
#define PM_AC_N 310 //停止時のac pm値
#define TIME1 4000 //直進時間[ms]
#define TIME2 4000 //旋回時間[ms]

//操作ボタン接続ピン
const int BUTTON = 30;

//モータードライバ接続ピン DIR:回転方向 PWM:回転速度
const int MD_ST_DIR = 6;
const int MD_ST_PWM = 7;
const int MD_AC_DIR = 11;
const int MD_AC_PWM = 12;

int pm_st, pm_ac; //pm読み値

int state = 0; //スタートボタンの状態

unsigned long ref_time; //時刻の保存

void setup()
{
  Serial.begin(9600);

  pinMode(BUTTON, INPUT);

  pinMode(MD_ST_DIR, OUTPUT);
  pinMode(MD_ST_PWM, OUTPUT);
  pinMode(MD_AC_DIR, OUTPUT);
  pinMode(MD_AC_PWM, OUTPUT);

  digitalWrite(MD_ST_DIR, LOW);
  digitalWrite(MD_ST_PWM, LOW);
  digitalWrite(MD_AC_DIR, LOW);
  digitalWrite(MD_AC_PWM, LOW);
}


void loop()
{
  //スタートボタンが押されるまで待機
  while (state == LOW)
  {
    state = digitalRead(BUTTON);
  }

  wait(5000); //一定時間待機

  motor_st(PM_ST_N); //stモーター駆動

  motor_ac(750); //acモーター駆動

  wait(TIME1); //一定時間直進

  motor_ac(PM_AC_N);

  motor_st(500);

  motor_ac(750);

  wait(TIME2); //一定時間旋回

  motor_ac(PM_AC_N);
}


//一定時間待機する関数
void wait(unsigned long wait_time)
{
  ref_time = millis();
  while ((millis() - ref_time) < wait_time);
}


//stモーターを駆動させる関数
void motor_st(int pm_st_ref)
{
  pm_st = analogRead(0);

  if (pm_st > pm_st_ref)
  {
    while (pm_st > pm_st_ref)
    {
      digitalWrite(MD_ST_DIR, LOW);
      analogWrite(MD_ST_PWM, SPEED_ST);

      pm_st = analogRead(0);
    }
  }

  else if (pm_st < pm_st_ref)
  {
    while (pm_st < pm_st_ref)
    {
      digitalWrite(MD_ST_DIR, HIGH);
      analogWrite(MD_ST_PWM, SPEED_ST);

      pm_st = analogRead(0);
    }
  }

  analogWrite(MD_ST_PWM, 0);
  digitalWrite(MD_ST_DIR, LOW);
}


//acモーターを駆動させる関数
void motor_ac(int pm_ac_ref)
{
  pm_ac = analogRead(1);

  if (pm_ac < pm_ac_ref)
  {
    while (pm_ac < pm_ac_ref)
    {
      digitalWrite(MD_AC_DIR, LOW);
      analogWrite(MD_AC_PWM, SPEED_AC);

      pm_ac = analogRead(1);
    }
  }

  else if (pm_ac > pm_ac_ref)
  {
    while (pm_ac > pm_ac_ref)
    {
      digitalWrite(MD_AC_DIR, HIGH);
      analogWrite(MD_AC_PWM, SPEED_AC);

      pm_ac = analogRead(1);
    }
  }

  analogWrite(MD_AC_PWM, 0);
  digitalWrite(MD_AC_DIR, LOW);
}
