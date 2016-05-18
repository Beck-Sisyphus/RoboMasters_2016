int Distance = 0;  // Record the number of steps we've taken

void setup() {                
  pinMode(5, OUTPUT);     
  pinMode(6, OUTPUT);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
}

void loop() {
  digitalWrite(6, HIGH);
  delayMicroseconds(100);          
  digitalWrite(6, LOW); 
  delayMicroseconds(100);
  Distance = Distance + 1;   // record this step
  
  // Check to see if we are at the end of our move
  if (Distance == 3600)
  {
    // We are! Reverse direction (invert DIR signal)
    if (digitalRead(5) == LOW)
    {
      digitalWrite(5, HIGH);
    }
    else
    {
      digitalWrite(5, LOW);
    }
    // Reset our distance back to zero since we're
    // starting a new move
    Distance = 0;
    // Now pause for half a second
    delay(500);
  }
}
