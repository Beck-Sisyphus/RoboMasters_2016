#include <Servo.h> 

#define LED_PIN (13)
#define SERVO_PIN (6)

Servo servo;

void setup() {
    Serial.begin(9600);
    pinMode(LED_PIN, OUTPUT);
    servo.attach(SERVO_PIN);
}

void loop() {
    if (Serial.available()) {
        char servo_state = Serial.read();
        if (servo_state) {
            digitalWrite(LED_PIN, HIGH);
            servo.write(180);
        } else {
            digitalWrite(LED_PIN, LOW);
            servo.write(0);
        }
    }
}