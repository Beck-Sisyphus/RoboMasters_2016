// To be run in conjunction with SerialPytArd_Test
// Python file

// for mySerial, which also works instead of using Serial3
#include <SoftwareSerial.h>

//String msg = "ABCDEFGHIJKLMNOP";
char tx[16];
char rx[16];

int i = 0;

//SoftwareSerial mySerial(10, 11);  // mySerial(rx, tx);

// Serial 3 for: trapezoid board
// Serial for: back to tx1 pySerial
void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  
}

// write different 18 byte messages every second
void loop() {

  if(Serial.available() > 0) {

    Serial.readBytes((char *) rx, 16);
    for(int i = 0; i < 16; i++) {
      rx[i] = (rx[i] & 255); 
    }
    
    Serial3.write((uint8_t*) rx, 16);
    Serial.write((uint8_t*) rx, 16);
  }       
}
