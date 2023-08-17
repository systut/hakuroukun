#define SPEED_st 255
#define SPEED_ac 100
#define PM_st_N 550
#define PM_st_LIM_R 195
#define PM_st_LIM_L 195
#define PM_ac_N 290
#define PM_ac_LIM_U 390
#define PM_ac_LIM_D 20
#define TIME 1000
#define TEST_TIME 10000

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

const int RELAY_ALARM = 50;
const int RELAY_MOTOR = 51;

int PM_st, PM_ac;
int PUSH_st_R = 0, PUSH_st_L = 0, PUSH_ac_U = 0, PUSH_ac_D = 0;
int pre_st = 0, pre_ac = 0;

unsigned long time_st = 0, time_ac = 0;
unsigned long start_time;

String com, com1, com2;
int com_st = PM_st_N;
int com_ac = PM_ac_N;

String command = "";
String acceleration = "290";
String steering = "550";
String control_status = "0";
String direction = "0";
String direction_mode = "0"; // 0 for forward, 1 for backward
bool stop_signal = false;

void setup() {
  Serial.begin(115200);

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

  pinMode(RELAY_ALARM, OUTPUT);
  pinMode(RELAY_MOTOR, OUTPUT);

  digitalWrite(MD_st_DIR, LOW);
  digitalWrite(MD_st_PWM, LOW);
  digitalWrite(MD_ac_DIR, LOW);
  digitalWrite(MD_ac_PWM, LOW);

  digitalWrite(LED_st, LOW);
  digitalWrite(LED_ac, LOW);

  digitalWrite(RELAY_ALARM, LOW);
  digitalWrite(RELAY_MOTOR, LOW);
}


void loop() {
  
  if (Serial.available()) {

    command = Serial.readStringUntil("\r\n");

    // 00 000 000
    if (command.length() != 10) {
      control_status = "1";

    }

    else {

      control_status = "0";

      direction = command.substring(1, 2);

      steering = command.substring(2, 5);

      acceleration = command.substring(5, 8);

      com_st = steering.toInt();

      com_ac = acceleration.toInt();

    }

    String response = control_status + direction + steering + acceleration;

    Serial.println(response);

    Serial.flush();
  }

  if (direction == "1") {
    switch_backward();

  } else {
    switch_forward();

  }

  if ((com_st > PM_st_N + PM_st_LIM_L || com_st < PM_st_N - PM_st_LIM_R) || (com_ac > PM_ac_N + PM_ac_LIM_U || com_ac < PM_ac_N - PM_ac_LIM_D))
  {
    com_st = PM_st_N;
    com_ac = PM_ac_N;
  }

  motor_st(com_st);
  motor_ac(com_ac);
  
}

void switch_backward() {
  if (direction_mode == "1") return;

  digitalWrite(RELAY_ALARM, HIGH);
  digitalWrite(RELAY_MOTOR, HIGH);

  direction_mode = "1";
}

void switch_forward() {
  if (direction_mode == "0") return;

  digitalWrite(RELAY_ALARM, LOW);
  digitalWrite(RELAY_MOTOR, LOW);

  direction_mode = "0";
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
  com_ac = 290;
  motor_ac(PM_ac_N);
  motor_st(PM_st_N);
  Serial.print("stopped");
  Serial.write(10);
  while (1);
}
