// ROS TX1 communicate to Arduino UNO
// to send information to Trapezoid board
#include <SoftwareSerial.h>


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
SoftwareSerial mySerial(10, 11); // RX, TX
void setup() {

  /*  For Communication between TX1, Arduino, Trapezoid */
  // tx to trapezoid
  Serial.begin(115200);
  mySerial.begin(115200);
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

//    // receive info from rx buffer
    header = ((int16_t) rxTX1[0] << 8) | (rxTX1[1] & 255);
    feeder_motor_state = rxTX1[2] & 255;
    friction_motor_state = rxTX1[3] & 255;
    pitch_req = ((int16_t) rxTX1[4] << 8) | (rxTX1[5] & 255);
    yaw_req = ((int16_t) rxTX1[6] << 8) | (rxTX1[7] & 255);
    feeder_motor_pwm = ((int16_t) rxTX1[8] << 8) | (rxTX1[9]  & 255);
    friction_motor_pwm = ((int16_t) rxTX1[10] << 8) | (rxTX1[11]  & 255);

    // tx to trapezoid board
    txTrap[0] = (header >> 8) & 255;
    txTrap[1] = header & 255;
    txTrap[2] = feeder_motor_state;
    txTrap[3] = friction_motor_state;
    txTrap[4] = (pitch_req >> 8) & 255;
    txTrap[5] = pitch_req & 255;
    txTrap[6] = (yaw_req >> 8) & 255;
    txTrap[7] = yaw_req & 255;
    txTrap[8] = (feeder_motor_pwm >> 8) & 255;
    txTrap[9] = feeder_motor_pwm & 255;
    txTrap[10] = (friction_motor_pwm >> 8) & 255;
    txTrap[11] = friction_motor_pwm & 255;
    mySerial.write((uint8_t*) txTrap, 16);
  }
  
  /******************************************************/
}


