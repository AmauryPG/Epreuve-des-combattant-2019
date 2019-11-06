#include "functionsMove.h"

void setup()
{
  BoardInit();

  PinceOpen();

  ENCODER_Reset(moteurGauche);
  ENCODER_Reset(moteurDroit);

  pinMode(pinCapteurMilieu,OUTPUT);
  pinMode(pinCapteurDroit,OUTPUT);
  pinMode(pinCapteurGauche,OUTPUT);

  SERVO_Enable(pince); 

  Serial.begin(9600);  
}

void loop()
{   
  TournerSurPlace(180,gauche,0.2);
  MOTOR_SetSpeed(moteurDroit,0);
  MOTOR_SetSpeed(moteurGauche,0);
  delay(500);
}
