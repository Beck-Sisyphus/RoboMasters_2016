/*
 * Shiyu Xia, Robomasters turret
 * Motor control
 * Input: 1/0 from arduino mega digital pin
 * Output: I1, I2 to L293D
 * 
 */

// L293D
#define MOTOR_A (5)
#define MOTOR_B (7)
#define TRIGGER (10)

// upper and lower limits
#define LOWER_LIMIT (400)
#define UPPER_LIMIT (860)

// voltage measurement
#define ANALOG_0 (0) // read voltage from A pin0
#define ANALOG_1 (1) // read voltage from A pin1


void setup() {
  Serial.begin(9600);
  pinMode(MOTOR_A, OUTPUT);
  pinMode(MOTOR_B, OUTPUT);
  pinMode(TRIGGER, INPUT);
}

// direction
int dir = 1;
// state machine
int ps = 0;
int ns = 0;
// timer
unsigned long pastMillis = 0;
unsigned long currentMillis = 0;
// voltage measurement
int v0 = 0;
int v1 = 0;

void loop() {
  int isOn = digitalRead(TRIGGER);
  switch (isOn){
    case HIGH:
      // start counting time
      currentMillis = millis();
      // read voltage. Voltage drops when stalling
      v0 = analogRead(ANALOG_0);
      v1 = analogRead(ANALOG_1);
      Serial.print(v0);
      Serial.print(" ");
      Serial.println(v1);
      // Spin
      Serial.println(dir);
    
      Serial.print(ps);
      Serial.print(" ");
      Serial.println(ns);
      Serial.println(currentMillis - pastMillis);
      spin(dir);
      
      if(currentMillis - pastMillis > 50) {
        if (isDangerous(v0, v1)) {
          ns = 1;
        } else {
          ns = 0;
        }
        if (ns != ps && ps == 0) {
          dir = -dir;
        } else {
          ns = 0;
        }
        ps = ns;
        pastMillis = currentMillis;
      }
    case LOW:
      digitalWrite(MOTOR_A, LOW);
      digitalWrite(MOTOR_B, LOW);
  }
}

// detect dangerous situation
// 
boolean isDangerous(int v0, int v1){
  if (v0 > LOWER_LIMIT && v0 < UPPER_LIMIT){
    return true;
  }
  if (v1 > LOWER_LIMIT && v1 < UPPER_LIMIT) {
    return true;
  }
  return false;
}

void spin(int dir) {
   if(dir > 0){
    digitalWrite(MOTOR_A, HIGH);
    digitalWrite(MOTOR_B, LOW);
   } else {
    digitalWrite(MOTOR_A, LOW);
    digitalWrite(MOTOR_B, HIGH);
   }
}

