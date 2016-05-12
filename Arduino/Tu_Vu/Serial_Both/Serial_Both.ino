// for mySerial, which also works instead of using Serial3
#include <SoftwareSerial.h>

//String msg = "ABCDEFGHIJKLMNOP";
char tx[16];
char rx[16];

int i = 0;
float a = -3.14159;
float b = 123.456;
float c = 3.14159;

int16_t ay = a * 100;
int16_t bee = b * 100;
int16_t cee = c * 100;

char a1 = ay >> 8; 
char a2 = ay & 255;

char b1 = bee >> 8; 
char b2 = bee & 255;

char c1 = cee >> 8; 
char c2 = cee & 255;

//SoftwareSerial mySerial(10, 11);  // mySerial(rx, tx);

// Serial 3 for:
// connected rx of trapezoid board to tx3 of arduino mega
// Serial for: serial monitor window to show data 
// when write to 
void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  Serial2.begin(115200);
  
}

// write different 18 byte messages every second
void loop() {

  for(int i = 0; i < 16; i++) {
    rx[i] = ' '; 
  }
  if(i%3 == 0) {
//        msg.setCharAt(4, a1);
//        msg.setCharAt(5, a2);
//        Serial3.print(msg);
    tx[14] = a1;
    tx[15] = a2;
    Serial3.write((uint8_t*) tx, 16);
  } else if(i % 3 == 1) {
//        msg.setCharAt(4, b1);
//        msg.setCharAt(5, b2);
//        Serial3.print(msg); 
    tx[14] = b1;
    tx[15] = b2;
    Serial3.write((uint8_t*) tx, 16);
  } else {
    tx[14] = c1;
    tx[15] = c2;
    Serial3.write((uint8_t*) tx, 16);
  }
  i++;
//  if(Serial2.available() > 0) {

    Serial2.readBytes((char*) rx, 16);
    for(int i = 0; i < 16; i++) {
      rx[i] = (rx[i] & 127); 
    }
    Serial.println(rx);

//  }

    delay(1000);        
}
