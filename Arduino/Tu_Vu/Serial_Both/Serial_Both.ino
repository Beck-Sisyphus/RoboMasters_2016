// for mySerial, which also works instead of using Serial3
#include <SoftwareSerial.h>

//String msg = "ABCDEFGHIJKLMNOP";
char tx[16];
char rx[16];

int i = 0;
float a = -3.14159;
float b = 123.456;

int16_t ay = a * 100;
int16_t bee = b * 100;
char a1 = ay >> 8; 
char a2 = ay & 255;

char b1 = bee >> 8; 
char b2 = bee & 255;

//SoftwareSerial mySerial(10, 11);  // mySerial(rx, tx);

// Serial 3 for:
// connected rx of trapezoid board to tx3 of arduino mega
// Serial for: serial monitor window to show data 
// when write to 
void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  Serial2.begin(115200);
//  mySerial.begin(115200);
//  if(a < 0.0) {
//    a1 = ~(ay >> 8);  
//  } else {
//    a1 = ay >> 8; 
//  }
//  
//  if(b < 0.0) {
//    b1 = ~(bee >> 8);  
//  } else {
//    b1 = bee >> 8; 
//  }
  
}

// write different 18 byte messages every second
void loop() {

        for(int i = 0; i < 16; i++) {
          rx[i] = ' '; 
        }
      if(i%2 == 0) {
//        msg.setCharAt(4, a1);
//        msg.setCharAt(5, a2);
//        Serial3.print(msg);
        tx[4] = a1;
        tx[5] = a2;
        Serial3.write(tx, 16);
      } else {
//        msg.setCharAt(4, b1);
//        msg.setCharAt(5, b2);
//        Serial3.print(msg); 
        tx[4] = b1;
        tx[5] = b2;
        Serial3.write(tx, 16);
      } 
      i++;
//      if(Serial2.available() > 0) {

        Serial2.readBytes(rx, 16);
        for(int i = 0; i < 16; i++) {
          rx[i] = (rx[i] & 127); 
        }
        Serial.println((uint8_t)-58, BIN);

//      }

    delay(1000);        
}
