// for mySerial, which also works instead of using Serial3
#include <SoftwareSerial.h>

int i = 0;
//SoftwareSerial mySerial(10, 11);  // mySerial(rx, tx);

// Serial 3 for:
// connected rx of trapezoid board to tx3 of arduino mega
// Serial for: serial monitor window to show data 
// when write to 
void setup() {
//  Serial.begin(115200);
  Serial3.begin(115200);
//  mySerial.begin(115200);
  
}

// write different 18 byte messages every second
void loop() {

  if(i%2 == 0) {
    Serial3.write('a');
//    Serial3.write("ABCDEFGHIJKLMNOPQR");
//    mySerial.write("ABCDEFGHIJKLMNOPQR");
  } else {
    Serial3.write('b');

//    Serial3.write("0123456789!@#$%^&*");
//    mySerial.write("0123456789!@#$%^&*");

  } 
  i++;
  delay(1000);
  
}
