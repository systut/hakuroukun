int sequence_id = 0
String command = "";
String response = "";
String control_status = ""

void setup() {
  Serial.begin(115200);
}

void loop() {
  if (!Serial.available()) {  
    command = Serial.readStringUntil("\r\n");

    // 00000 00 000 000 
    if (command.length() != 14) {
      String response = "0000002000000";

      Serial.println(response);

      Serial.flush();

      return;
    }

    String sequence_id = command.substring(0, 5); 

    String control_status = command.substring(5, 6);

    String acceleration = command.substring(6, 9);

    String steering = command.substring(9, 12);

    response = sequence_id + control_mode + control_status + acceleration + steering;

    Serial.println(response);

    Serial.flush();

    return;
  }
}
