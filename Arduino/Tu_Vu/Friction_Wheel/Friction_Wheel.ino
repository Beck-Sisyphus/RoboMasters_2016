int8_t pin = 9;

void setup() {
  pinMode(pin, OUTPUT);

  // initializes friction wheels 
  // 40% duty cycle
  // equivalent to setting PWMx = 1000 in trapezoid board
  analogWrite(pin, 255 * 0.4);
  delay(500);
}

void loop() {
  // equivalent to setting PWMx = 1500 in trapezoid board
  analogWrite(pin, 255 * 0.6);
}
