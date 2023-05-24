//com4
String com,com1,com2;
int com_st,com_ac;

void setup()
{
  Serial.begin(9600);
  Serial.setTimeout(50);
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
      Serial.print("stopped");
      Serial.write(10);
      while(1);
    }
    else
    {
      Serial.print("error");
    }
    Serial.write(10);
  }
}
