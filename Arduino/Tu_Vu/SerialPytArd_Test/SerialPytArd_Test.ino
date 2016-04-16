// To be run in conjunction with SerialPytArd_Test
// Python file

// for mySerial, which also works instead of using Serial3
#include <SoftwareSerial.h>

//String msg = "ABCDEFGHIJKLMNOP";
char tx[16];
char rx[16];

int i = 0;
float a = -3.14159;
float b = 123.456;
float c = 3.14159;
float d = 2.0191;

int16_t ay = a * 100;
int16_t bee = b * 100;
int16_t cee = c * 100;
int16_t dee = d * 100;

char a1 = ay >> 8; 
char a2 = ay & 255;

char b1 = bee >> 8; 
char b2 = bee & 255;

char c1 = cee >> 8; 
char c2 = cee & 255;

char d1 = dee >> 8; 
char d2 = dee & 255;

//SoftwareSerial mySerial(10, 11);  // mySerial(rx, tx);

// Serial 3 for:
// connected rx of trapezoid board to tx3 of arduino mega
// Serial for: serial monitor window to show data 
// when write to 
void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);
  
}

// write different 18 byte messages every second
void loop() {

  for(int i = 0; i < 16; i++) {
    rx[i] = ' '; 
  }

  if(Serial.available() > 0) {

    Serial.readBytes(rx, 16);
    for(int i = 0; i < 16; i++) {
      rx[i] = (rx[i] & 255); 
    }
    
    Serial3.write(rx, 16);
    
//    if(i%3 == 0) {
//      tx[4] = a1;
//      tx[5] = a2;
//      Serial.write(tx, 16);
//    } else if(i%3 == 1) {
//      tx[4] = b1;
//      tx[5] = b2;
//      Serial.write(tx, 16);
//    } else {
//      tx[4] = c1;
//      tx[5] = c2;
//      Serial.write(tx, 16);
//    }
    i++;

//
  }

//    delay(1000);        
}
