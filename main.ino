#include "functionsMove.h"

void setup()
{
  BoardInit();

  ENCODER_Reset(moteurGauche);
  ENCODER_Reset(moteurDroit);

  pinMode(pinCapteurMilieu, OUTPUT);
  pinMode(pinCapteurDroit, OUTPUT);
  pinMode(pinCapteurGauche, OUTPUT);

  SERVO_Enable(pince);
}

void loop()
{
  /*
  PinceOpen();
//premier virage
  ENCODER_Reset(moteurGauche);
  ENCODER_Reset(moteurDroit);

  TournerSurPlace(45, gauche, 0.3);
//premier trajet
  ENCODER_Reset(moteurGauche);
  ENCODER_Reset(moteurDroit);
 
  PIDAvancer(0, 0.3, 50, 10);
//deuxieme virage
  ENCODER_Reset(moteurGauche);
  ENCODER_Reset(moteurDroit);

  TournerSurPlace(90, droit, 0.3);
//deuxieme trajet
  ENCODER_Reset(moteurGauche);
  ENCODER_Reset(moteurDroit);

  PIDAvancer(0, 0.3, 17, 15);
//virage 3
  ENCODER_Reset(moteurGauche);
  ENCODER_Reset(moteurDroit);

  TournerSurPlace(97, gauche, 0.3);
//troisieme trajet
  ENCODER_Reset(moteurGauche);
  ENCODER_Reset(moteurDroit);

  PIDAvancer(0, 0.3, 73, 15);

  MOTOR_SetSpeed(moteurGauche, 0);
  MOTOR_SetSpeed(moteurDroit, 0);

  PinceClose();

  delay(400);
//retour
  TournerSurPlace(180, gauche, 0.3);

  ENCODER_Reset(moteurGauche);
  ENCODER_Reset(moteurDroit);

  PIDAvancer(0, 0.3, 104, 15);

  MOTOR_SetSpeed(moteurGauche, 0);
  MOTOR_SetSpeed(moteurDroit, 0);

  TournerSurPlace(135, gauche, 0.3);

  PinceOpen();
  for (int i = 0; i<500;i++)
  {
    MOTOR_SetSpeed(moteurDroit, -0.3);
    MOTOR_SetSpeed(moteurGauche, -0.3);
  }

  MOTOR_SetSpeed(moteurGauche, 0);
  MOTOR_SetSpeed(moteurDroit, 0);*/
  if (ROBUS_IsBumper(3) == 1)
  {
 
    porterBalle(0);
    delay(1000000); 
  }
}
