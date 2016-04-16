char str[16];

void setup() {
//  Serial3.begin(115200);
  Serial.begin(115200);
}

void loop() {
  for(int i = 0; i < 16; i++) {
    str[i] = i + 'a';
  }

//  Serial3.write(str, 16);
  Serial.write(str, 16);
  delay(1000);

}
