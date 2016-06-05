// Arduino timer CTC interrupt example
// www.engblaze.com
 
#include <Servo.h>

#include <avr/io.h>
#include <avr/interrupt.h>


// friction wheels pwm
int8_t front_left_pin = 13; 
int8_t front_right_pin = 12;
int8_t rear_left_pin = 8;
int8_t rear_right_pin = 7;

// golf ball feeder stepper motor
// j counter to avoid using delay and screwing communication
int8_t direct = 2;
int8_t spin = 3;
int j = 0;

// servo motor
// i counter to avoid using delay and screwing communication
// sw for direction change
Servo myServo;
int8_t serv = A0;
int i = 0;
bool sw = false;


/*  For Communication between TX1, Arduino, Trapezoid */
char txTrap[16];
char txTX1[16];
char rxTX1[16];
int16_t header;
int8_t feeder_motor_state;
int8_t friction_motor_state;
int16_t pitch_req;
int16_t yaw_req;
int16_t feeder_motor_pwm;
int16_t friction_motor_pwm;

// estimate of kalAngles at int to tx
int16_t kalIntX;
int16_t kalIntY;
int16_t kalIntZ;

float kalAngleX = 0;
float kalAngleY = 0;
float kalAngleZ = 0;

// Constants to get more decimal places of float data
// will divide by same amount in TX1 and Trapezoid
int kalConstX = 100;
int kalConstY = 100;
int kalConstZ = 100;

 
void setup()
{
  Serial.begin(115200);
  //Serial1.begin(115200);
  // servo setup
  myServo.attach(serv);
  // stepper setup
  pinMode(direct, OUTPUT);     
  pinMode(spin, OUTPUT);
  digitalWrite(direct, LOW);
  digitalWrite(spin, LOW);
 
//    // initialize Timer1
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
//  TCCR2B = 0x00;        //Disbale Timer2 while we set it up
//
//  TCNT2  = 130;         //Reset Timer Count to 130 out of 255
//
//  TIFR2  = 0x00;        //Timer2 INT Flag Reg: Clear Timer Overflow Flag
//
//  TIMSK2 = 0x01;        //Timer2 INT Reg: Timer2 Overflow Interrupt Enable
//
//  TCCR2A = 0x00;        //Timer2 Control Reg A: Normal port operation, Wave Gen Mode normal
//
//  TCCR2B = 3;        //Timer2 Control Reg B: Timer Prescaler set to 128

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
  
    Serial.readBytes((char*) rxTX1, 16);
    for(int i = 0; i < 16; i++) {
      rxTX1[i] = (rxTX1[i] & 255);
      Serial1.println("hello");
    }
    
  
  
    friction_motor_state = rxTX1[3] & 255;
    if(friction_motor_state == 1) {
      
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
 
ISR(TIMER2_COMPA_vect)
{
    if(friction_motor_state == 1) {
      digitalWrite(spin, HIGH);
      delay(1);
    
      digitalWrite(spin, LOW);
      delay(1);
    }
    
//      TCNT2 = 130;           //Reset Timer to 130 out of 255
//
//  TIFR2 = 0x00;          //Timer2 INT Flag Reg: Clear Timer Overflow Flag
}
