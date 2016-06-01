// For controling servo and friction wheels
// Servo range [0, 180]
#include <Servo.h>
Servo myServo;

int8_t front_left_pin = 10; 
int8_t front_right_pin = 9;
int8_t rear_left_pin = 8;
int8_t rear_right_pin = 7;

// for initialize 1 at a time
int i = 0;

void setup() {
  myServo.attach(11);
  pinMode(front_left_pin, OUTPUT);
  pinMode(front_right_pin, OUTPUT);
  pinMode(rear_left_pin, OUTPUT);
  pinMode(rear_right_pin, OUTPUT);
  analogWrite(front_left_pin, 255 * 0.4);
  analogWrite(front_right_pin, 255 * 0.4);
  analogWrite(rear_left_pin, 255 * 0.4);
  analogWrite(rear_right_pin, 255 * 0.4);
  delay(500);
}

void loop() {
  // initializes friction wheels 
  // one at a time
  // 40% duty cycle
  // equivalent to setting PWMx = 1000 in trapezoid board
  if(i == 0) {
    analogWrite(front_left_pin, 255 * 0.4);
    delay(500);
    i++;
  } else if(i == 1) {
    analogWrite(front_right_pin, 255 * 0.4);
    delay(500);
    i++;
  } else if(i == 2) {
    analogWrite(rear_left_pin, 255 * 0.4);
    delay(500);
    i++;
  } else if(i == 3) {
    analogWrite(rear_right_pin, 255 * 0.4);
    delay(500);
    i++;
  } else {
      // 90% duty cycle
      analogWrite(front_left_pin, 255 * 0.99);
      analogWrite(front_right_pin, 255 * 0.99);
      analogWrite(rear_left_pin, 255 * 0.99);
      analogWrite(rear_right_pin, 255 * 0.99);
      
      // spins servo
      myServo.write(50);
      delay(750);
      myServo.write(150);
      delay(750);
  }
}

