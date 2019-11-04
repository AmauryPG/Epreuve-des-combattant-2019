#include "functionsMove.h"

void setup()
{
  BoardInit();

  PinceOpen();

  pinMode(pinCapteurMilieu,OUTPUT);
  pinMode(pinCapteurDroit,OUTPUT);
  pinMode(pinCapteurGauche,OUTPUT);

  Serial.begin(9600);  
}

void loop()
{ 
  PIDAcceleration(0.0, 0.3,30);
  MOTOR_SetSpeed(0,0);
  MOTOR_SetSpeed(1,0);
  delay(5000);
  delay(10);
}
