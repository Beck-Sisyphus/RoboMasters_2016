char input = 0;
#include <Servo.h>

#include <avr/io.h>
#include <avr/interrupt.h>


// friction wheels pwm
int8_t front_left_pin = 6; 
int8_t front_right_pin = 12;
int8_t rear_left_pin = 8;
int8_t rear_right_pin = 7;

// golf ball feeder stepper motor
// j counter to avoid using delay and screwing communication
int8_t direct = 2;
int8_t spin = 3;

// servo motor
// i counter to avoid using delay and screwing communication
// sw for direction change
Servo myServo;
int8_t serv = 46;

 
void setup() {
  Serial.begin(115200);
  // servo setup
  myServo.attach(serv);
  // stepper setup
  pinMode(direct, OUTPUT);     
  pinMode(spin, OUTPUT);
  digitalWrite(direct, LOW);
  digitalWrite(spin, LOW);
 
//    // initialize Timer2
//    cli();          // disable global interrupts
    TCCR2A = 0;     // set entire TCCR1A register to 0
    TCCR2B = 0;     // same for TCCR1B
 
    // set compare match register to desired timer count:
    OCR2A = 3;
    // turn on CTC mode:
    TCCR2B |= (1 << WGM12);
    // Set CS10 and CS12 bits for 1024 prescaler:
    TCCR2B |= (1 << CS10);
    TCCR2B |= (1 << CS12);
    // enable timer compare interrupt:
    TIMSK2 |= (1 << OCIE1A);
    // enable global interrupts:
//    sei();

  pinMode(front_left_pin, OUTPUT);
  pinMode(front_right_pin, OUTPUT);
  pinMode(rear_left_pin, OUTPUT);
  pinMode(rear_right_pin, OUTPUT);
  analogWrite(front_left_pin, 255 * 0.4);
  analogWrite(front_right_pin, 255 * 0.4);
  analogWrite(rear_left_pin, 255 * 0.4);
  analogWrite(rear_right_pin, 255 * 0.4);

}
 
void loop()
{
  if(Serial.available() > 0) {
  
    input = Serial.read();
    for(int i = 0; i < 16; i++) {
      input = (input & 255);
    }
    

    
    if(input == '1') {
      
      analogWrite(front_left_pin, 255 * 0.99);
      analogWrite(front_right_pin, 255 * 0.99);
      analogWrite(rear_left_pin, 255 * 0.99);
      analogWrite(rear_right_pin, 255 * 0.99);
      myServo.write(30);
      delay(500);
      myServo.write(150);
      delay(500);
    } else {
      analogWrite(front_left_pin, 255 * 0.4);
      analogWrite(front_right_pin, 255 * 0.4);
      analogWrite(rear_left_pin, 255 * 0.4);
      analogWrite(rear_right_pin, 255 * 0.4);
    }
  }
}
 
ISR(TIMER2_COMPA_vect) {
    if(input == 1) {
      digitalWrite(spin, HIGH);
      delay(1);
    
      digitalWrite(spin, LOW);
      delay(1);
    }
}
