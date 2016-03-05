void setup()
{
pinMode(2,OUTPUT);
pinMode(3,OUTPUT);
}

void loop()
{
  //digitalWrite(3,!(digitalRead(3)));
  for (int I=0;I<5000;I++) // As the easy stepper driver is in microstepping mode
                           // We need to increase the step count.
  {
   digitalWrite(2,HIGH);
   digitalWrite(2,LOW) ;
   delayMicroseconds(160);
  }
}
