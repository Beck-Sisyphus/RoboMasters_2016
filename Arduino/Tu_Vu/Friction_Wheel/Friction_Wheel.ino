int8_t front_left_pin = 10; 
int8_t front_right_pin = 9;
int8_t rear_left_pin = 8;
int8_t rear_right_pin = 7;

void setup() {
  pinMode(front_left_pin, OUTPUT);
  pinMode(front_right_pin, OUTPUT);
  pinMode(rear_left_pin, OUTPUT);
  pinMode(rear_right_pin, OUTPUT);


  // initializes friction wheels 
  // 40% duty cycle
  // equivalent to setting PWMx = 1000 in trapezoid board
  analogWrite(front_left_pin, 255 * 0.4);
  analogWrite(front_right_pin, 255 * 0.4);
  analogWrite(rear_left_pin, 255 * 0.4);
  analogWrite(rear_right_pin, 255 * 0.4);

  delay(500);
}

void loop() {
  // equivalent to setting PWMx = 1500 in trapezoid board
  analogWrite(front_left_pin, 255 * 0.6);
  analogWrite(front_right_pin, 255 * 0.6);
  analogWrite(rear_left_pin, 255 * 0.6);
  analogWrite(rear_right_pin, 255 * 0.6);

}

