/*
 * Shiyu Xia, Robomasters turret
 * Motor control
 * Input: 1/0 from arduino mega digital pin
 * Output: I1, I2 to L293D
 * 
 */

// voltage measurement
int aPin0 = 0; // read voltage from A pin0
int aPin1 = 1; // read voltage from A pin1
int v0 = 0;
int v1 = 0;

// dangerous threshold:
int threshold = 700;

// L293D
int A = 5;
int B = 11;


void setup() {
  Serial.begin(9600);
  pinMode(A, OUTPUT);
  pinMode(B, OUTPUT);
}

int dir = 1;
int ps = 0;
int ns = 0;
long pastMillis = 0;
void loop() {
  // read voltage. Voltage drops when stalling
  v0 = analogRead(aPin0);
  v1 = analogRead(aPin1);
  Serial.print(v0);
  Serial.print(" ");
  Serial.println(v1);
  // Spin
  Serial.println(dir);

  Serial.print(ps);
  Serial.print(" ");
  Serial.println(ns);
  unsigned long currentMillis = millis();

  spin(dir);
  
  if(currentMillis - pastMillis > 300) {
    if (isDangerous(v0, v1)) {
      ns = 1;
    } else {
      ns = 0;
    }
    if (ns != ps && ps == 0) {
      
      dir = -dir;
      pastMillis = currentMillis;
    } else {
      ns = 0;
    }
    ps = ns;
  }
}

// detect dangerous situation
// 
boolean isDangerous(int v0, int v1){
  if (v0 > 600 && v0 < 690){
    return true;
  }
  if (v1 > 600 && v1 < 690) {
    return true;
  }
  return false;
  /*
  if (dir == 1){
    if (v1 < threshold) {
      return true;
    }
  } else {
    if (v0 < threshold){
      return true;
    }
  }
  return false;
*/
}

void spin(int dir) {
   if(dir > 0){
    digitalWrite(A, HIGH);
    digitalWrite(B, LOW);
   } else {
    digitalWrite(A, LOW);
    digitalWrite(B, HIGH);
   }
}

