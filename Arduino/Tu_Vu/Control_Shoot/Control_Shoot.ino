// For controling servo, stepper, and friction wheels
// with remote controller
// Servo range [0, 180]
#include <Servo.h>
char trigger[1];

// friction wheels pwm
int8_t front_left_pin = 10; 
int8_t front_right_pin = 9;
int8_t rear_left_pin = 8;
int8_t rear_right_pin = 7;

// golf ball feeder stepper motor
// j counter to avoid using delay and screwing communication
int8_t direct = 2;
int8_t spin = 3;
int8_t j = 0;

// servo motor
// i counter to avoid using delay and screwing communication
// sw for direction change
Servo myServo;
int8_t serv = 11;
int i = 0;
bool sw = false;

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);

  // Servo
  myServo.attach(serv);

  // PWM
  pinMode(front_left_pin, OUTPUT);
  pinMode(front_right_pin, OUTPUT);
  pinMode(rear_left_pin, OUTPUT);
  pinMode(rear_right_pin, OUTPUT);
  analogWrite(front_left_pin, 255 * 0.4);
//  delay(500);
  analogWrite(front_right_pin, 255 * 0.4);
//  delay(500);
  analogWrite(rear_left_pin, 255 * 0.4);
//  delay(500);
  analogWrite(rear_right_pin, 255 * 0.4);
//  delay(500);

  // stepper 
  pinMode(direct, OUTPUT);     
  pinMode(spin, OUTPUT);
  digitalWrite(direct, LOW);
  digitalWrite(spin, LOW);
}

void loop() {
  // initializes friction wheels 
  // one at a time
  // 40% duty cycle
  // equivalent to setting PWMx = 1000 in trapezoid board
  if(Serial3.available() > 0) {
    Serial3.readBytes((char*) trigger, 1);
    trigger[0] = trigger[0] & 255;
    if(trigger[0] == 48 || trigger[0] == 6) {
      noShoot();
    } else {
      shoot();
    }
    Serial.println(trigger[0] & 255);
  }
//  shoot();
//  if(i < 7500 && !sw) {
//    myServo.write(50);    
//  } else if(i < 7500 && sw) {
//    myServo.write(150);
//  } else if(i > 7500) {
//    sw = !sw;
//    i = 0;
//  }
//  i++;
}


void shoot() {
  // 90% duty cycle
  analogWrite(front_left_pin, 255 * 0.99);
  analogWrite(front_right_pin, 255 * 0.99);
  analogWrite(rear_left_pin, 255 * 0.99);
  analogWrite(rear_right_pin, 255 * 0.99);

  // servo
  if(i < 1000 && !sw) {
    myServo.write(50);    
  } else if(i < 1000 && sw) {
    myServo.write(150);
  } else if(i > 1000) {
    sw = !sw;
    i = 0;
  }

  // stepper
  // approximately delay(1) between writes
  if(j < 10) {
    digitalWrite(spin, HIGH);
  } else {
    digitalWrite(spin, LOW); 
    j = 0;
  }
  i++;
  j++;
  
//  myServo.write(50);
//  // spins servo
//  myServo.write(50);
//  myServo.write(150);

}

void noShoot() {
  analogWrite(front_left_pin, 255 * 0.4);
  analogWrite(front_right_pin, 255 * 0.4);
  analogWrite(rear_left_pin, 255 * 0.4);
  analogWrite(rear_right_pin, 255 * 0.4);
//  myServo.write(150);
}


