#include <Servo.h>

Servo myservo;
int ServoDeg; // servo rotation in degrees
int Servo_us; // servo pwm command in micro seconds

void setup()
{
  Serial.begin(115200);
  myservo.attach(9);
}

void loop()
{
   Serial.print("Specify rotation angle (-90 to 90 in degrees) \t: ");
   while (Serial.available()==0){ }
    ServoDeg = Serial.parseInt();
    Serial.print(ServoDeg);
    Servo_us = 1500+(ServoDeg*10);
    //Serial.print("\t pwm command (microseconds) \t: ");
    //Serial.print(Servo_us);
    Serial.println(" ");
   myservo.writeMicroseconds (Servo_us);

}
