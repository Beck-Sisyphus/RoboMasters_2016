/*
uw robomasters
file: debug_forwarder.ino

forwards packets from trapezoid board as received
RX is digital pin 10 (connect to TX of other device)
TX is digital pin 11 (connect to RX of other device)

*/

#include <SoftwareSerial.h>

SoftwareSerial s(10, 11); // RX, TX

char read_char;

void setup() {
    Serial.begin(115200);
    s.begin(115200);
}

void loop() {
    if (s.available()) {
        delay(4); // works slightly better with delay (?)
        read_char = s.read() & 127;
        Serial.print(read_char);
    }
    // if (Serial.available()) {
    //     s.write(Serial.read());
    // }
}
