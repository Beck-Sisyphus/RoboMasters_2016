// For controling servo, stepper, and friction wheels
// with ROS from TX1

// for mySerial, which also works instead of using Serial3
//#include <SoftwareSerial.h>
#include <Servo.h>

// friction wheels pwm
int8_t front_left_pin = 10; 
int8_t front_right_pin = 9;
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
int8_t serv = 11;
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
  /******************************************************/

void setup() {

  /*  For Communication between TX1, Arduino, Trapezoid */
  // rx from TX1
  // header, (load, trigger), pitch, yaw, PWM
  // tx to TX1
  // header, kalAngleX, kalAngleY, kalAngleZ
  Serial.begin(115200);
  // tx to trapezoid
  // header, kalAngleZ, pitch, yaw, PWM
  Serial1.begin(115200);
  /******************************************************/
  // pwm initialize
  pinMode(front_left_pin, OUTPUT);
  pinMode(front_right_pin, OUTPUT);
  pinMode(rear_left_pin, OUTPUT);
  pinMode(rear_right_pin, OUTPUT);
  analogWrite(front_left_pin, 255 * 0.4);
  analogWrite(front_right_pin, 255 * 0.4);
  analogWrite(rear_left_pin, 255 * 0.4);
  analogWrite(rear_right_pin, 255 * 0.4);
  //delay(500);

  // servo setup
  myServo.attach(serv);
  
  // stepper setup
  pinMode(direct, OUTPUT);     
  pinMode(spin, OUTPUT);
  digitalWrite(direct, LOW);
  digitalWrite(spin, LOW);


}

// write different 18 byte messages every second
void loop() {

  /******************************************************/
  /*  For Communication between TX1, Arduino, Trapezoid */
  // runs everytime TX1 sends information to Arduino
  if(Serial.available() > 0) {
    
    Serial.readBytes((char*) rxTX1, 16);
    for(int i = 0; i < 16; i++) {
      rxTX1[i] = (rxTX1[i] & 255);
    }

    // receive info from rx buffer
    header = ((int16_t) rxTX1[0] << 8) | (rxTX1[1] & 255);
    feeder_motor_state = rxTX1[2] & 255;
    friction_motor_state = rxTX1[3] & 255;
    pitch_req = ((int16_t) rxTX1[4] << 8) | (rxTX1[5] & 255);
    yaw_req = ((int16_t) rxTX1[6] << 8) | (rxTX1[7] & 255);
    feeder_motor_pwm = ((int16_t) rxTX1[8] << 8) | (rxTX1[9]  & 255);
    friction_motor_pwm = ((int16_t) rxTX1[10] << 8) | (rxTX1[11]  & 255);

    if(friction_motor_state == 1) {
      shoot();  
    } else {
      noShoot();  
    }
    
    Serial.println(feeder_motor_state);

//    // change multiplied number later
//    kalIntX = kalAngleX * kalConstX;
//    kalIntY = kalAngleY * kalConstY;
//    kalIntZ = kalAngleZ * kalConstZ;
//    
//    // tx to TX1
//    txTX1[0] = (header >> 8) & 255;
//    txTX1[1] = header & 255;
//    txTX1[2] = (kalIntX >> 8) & 255;
//    txTX1[3] = kalIntX & 255;
//    txTX1[4] = (kalIntY >> 8) & 255;
//    txTX1[5] = kalIntY & 255;
//    txTX1[6] = (kalIntZ >> 8) & 255;
//    txTX1[7] = kalIntZ & 255;
//    Serial.write((uint8_t*) txTX1, 16);
//    
//
//    // tx to trapezoid board
//    txTrap[0] = (header >> 8) & 255;
//    txTrap[1] = header & 255;
//    txTrap[2] = feeder_motor_state;
//    txTrap[3] = friction_motor_state;
//    txTrap[4] = (pitch_req >> 8) & 255;
//    txTrap[5] = pitch_req & 255;
//    txTrap[6] = (yaw_req >> 8) & 255;
//    txTrap[7] = yaw_req & 255;
//    txTrap[8] = (feeder_motor_pwm >> 8) & 255;
//    txTrap[9] = feeder_motor_pwm & 255;
//    txTrap[10] = (friction_motor_pwm >> 8) & 255;
//    txTrap[11] = friction_motor_pwm & 255;
//    Serial1.write((uint8_t*) txTrap, 16);
  }
  
  /******************************************************/
}

void shoot() {
  analogWrite(front_left_pin, 255 * 0.99);
  analogWrite(front_right_pin, 255 * 0.99);
  analogWrite(rear_left_pin, 255 * 0.99);
  analogWrite(rear_right_pin, 255 * 0.99);

  // servo
  // approximately delay(500) between writes
  if(i < 7500 && !sw) {
    myServo.write(50);    
  } else if(i < 7500 && sw) {
    myServo.write(150);
  } else if(i > 7500) {
    sw = !sw;
    i = 0;
  }

  // stepper
  // approximately delay(1) between writes
  if(j < 15) {
    digitalWrite(spin, HIGH);
  } else {
    digitalWrite(spin, LOW); 
    j = 0;
  }
  
  i++;
  j++;
}

void noShoot() {
  analogWrite(front_left_pin, 255 * 0.4);
  analogWrite(front_right_pin, 255 * 0.4);
  analogWrite(rear_left_pin, 255 * 0.4);
  analogWrite(rear_right_pin, 255 * 0.4);
}


// this serial interrupt is not supported in arduino mini
// so use polling method in loop()

//void serialEvent3() {
////  if(Serial3.available() > 0) {
////    msg = Serial3.readString();
////    for(int i = 0; i < msg.length(); i++) {
////      msg.setCharAt(i, msg.charAt(i) & 127);
////    }
////    Serial.println(msg);
////  }
//}


