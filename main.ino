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
  //porterBalle(0);

  pinceLente(170);
  delay(1000);
}
