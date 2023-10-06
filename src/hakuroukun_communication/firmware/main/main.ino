#define SPEED_st 255
#define SPEED_ac 100
#define PM_st_N 565 //565 250
#define PM_st_LIMR 195
#define PM_st_LIML 195
#define PM_ac_N 290 //290
#define PM_ac_LIMU 390
#define PM_ac_LIMD 20
#define TIME 1000

const int BUTTON_stR = 30;
const int BUTTON_stL = 31;
const int BUTTON_acU = 32;
const int BUTTON_acD = 33;

const int MD_st_DIR = 6;
const int MD_st_PWM = 7;
const int MD_ac_DIR = 11;
const int MD_ac_PWM = 12;

const int LED_st = 40;
const int LED_ac = 41;

int PM_st, PM_ac;
int PUSH_stR = 0, PUSH_stL = 0, PUSH_acU = 0, PUSH_acD = 0;
int pre_st = 0, pre_ac = 0;

unsigned long time_st = 0, time_ac = 0;

String com, com1, com2;
int com_st = PM_st_N;
int com_ac = PM_ac_N;


void setup()
{
  Serial.begin(115200);
  Serial.setTimeout(50);

  pinMode(BUTTON_stR, INPUT);
  pinMode(BUTTON_stL, INPUT);
  pinMode(BUTTON_acU, INPUT);
  pinMode(BUTTON_acD, INPUT);

  pinMode(MD_st_DIR, OUTPUT);
  pinMode(MD_st_PWM, OUTPUT);
  pinMode(MD_ac_DIR, OUTPUT);
  pinMode(MD_ac_PWM, OUTPUT);

  pinMode(LED_st, OUTPUT);
  pinMode(LED_ac, OUTPUT);

  digitalWrite(MD_st_DIR, LOW);
  digitalWrite(MD_st_PWM, LOW);
  digitalWrite(MD_ac_DIR, LOW);
  digitalWrite(MD_ac_PWM, LOW);

  digitalWrite(LED_st, LOW);
  digitalWrite(LED_ac, LOW);

  while (!Serial.available());
}

void loop()
{
  if (Serial.available())
  {
    com = Serial.readStringUntil('\n');
    if (com.length() == 7 && com.charAt(3) == ',')
    {
      com1 = com.substring(0, 3);
      com2 = com.substring(4, 7);
      com_st = com1.toInt();
      com_ac = com2.toInt();
      Serial.print(com_st);
      Serial.print(',');
      Serial.print(com_ac);
    }
    else if (com == "stop")
    {
      robot_stop();
    }
    else
    {
      Serial.print("error");
    }
    Serial.write(10);
  }

  if ((com_st > PM_st_N + PM_st_LIML || com_st < PM_st_N - PM_st_LIMR) || (com_ac > PM_ac_N + PM_ac_LIMU || com_ac < PM_ac_N - PM_ac_LIMD))
  {
    com_st = PM_st_N;
    com_ac = PM_ac_N;
  }

  motor_st(com_st);
  motor_ac(com_ac);
}


void motor_st(int PM_st_REF)
{
  if (PM_st_REF != pre_st || millis() - time_st > TIME)
  {
    PM_st = analogRead(0);

    if (PM_st > PM_st_REF)
    {
      while (PM_st > PM_st_REF)
      {
        digitalWrite(MD_st_DIR, LOW);
        analogWrite(MD_st_PWM, SPEED_st);
        PM_st = analogRead(0);
      }
    }
    else if (PM_st < PM_st_REF)
    {
      while (PM_st < PM_st_REF)
      {
        digitalWrite(MD_st_DIR, HIGH);
        analogWrite(MD_st_PWM, SPEED_st);
        PM_st = analogRead(0);
      }
    }

    analogWrite(MD_st_PWM, 0);
    digitalWrite(MD_st_DIR, LOW);
    time_st = millis();
  }
  pre_st = PM_st_REF;
}


void motor_ac(int PM_ac_REF)
{
  if (PM_ac_REF != pre_ac || millis() - time_ac > TIME)
  {
    PM_ac = analogRead(1);

    if (PM_ac < PM_ac_REF)
    {
      while (PM_ac < PM_ac_REF)
      {
        digitalWrite(MD_ac_DIR, LOW);
        analogWrite(MD_ac_PWM, SPEED_ac);
        PM_ac = analogRead(1);
      }
    }
    else if (PM_ac > PM_ac_REF)
    {
      while (PM_ac > PM_ac_REF)
      {
        digitalWrite(MD_ac_DIR, HIGH);
        analogWrite(MD_ac_PWM, SPEED_ac);
        PM_ac = analogRead(1);
      }
    }

    analogWrite(MD_ac_PWM, 0);
    digitalWrite(MD_ac_DIR, LOW);
    time_ac = millis();
  }
  pre_ac = PM_ac_REF;
}


void robot_stop()
{
  motor_ac(PM_ac_N);
  motor_st(PM_st_N);
  Serial.print("stopped");
  Serial.write(10);
  while (1);
}
