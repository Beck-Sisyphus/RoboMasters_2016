String msg;

// Serial 3 for:
// connected tx of trapezoid board to rx3 of arduino mega
//Serial for: serial monitor window to show data 
void setup() {
 Serial.begin(115200);
 Serial1.begin(115200);
}

// reads input as strings
// need char manipulation to cut away
// extra, wrong bits in message
void loop() {
  if(Serial.available() > 0) {
    msg = Serial.readString();
    for(int i = 0; i < msg.length(); i++) {
      msg.setCharAt(i, msg.charAt(i) & 127);
    }
  Serial.println(msg);
  }
    if(Serial1.available() > 0) {
    msg = Serial1.readString();
    for(int i = 0; i < msg.length(); i++) {
      msg.setCharAt(i, msg.charAt(i) & 127);
    }
    Serial.println(msg);
  }
}

// this serial interrupt is not supported in arduino mini
// so use polling method in loop()
/*
void serialEvent3() {
  if(Serial3.available() > 0) {
    msg = Serial3.readString();
    for(int i = 0; i < msg.length(); i++) {
      msg.setCharAt(i, msg.charAt(i) & 127);
    }
    Serial.println(msg);
  }
}
*/

