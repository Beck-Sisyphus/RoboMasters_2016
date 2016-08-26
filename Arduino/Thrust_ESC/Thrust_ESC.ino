#include <Servo.h>
 
Servo esc;
int escPin = 37;
int throttlePin = 0;
 
void setup()
{
    esc.attach(escPin);
}
 
void loop()
{
    // int throttle = analogRead(throttlePin);
    // throttle = map(throttle, 0, 1023, 0, 179);
    esc.write(0);
}
