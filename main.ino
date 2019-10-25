#include "functions.h"

void setup()
{
  BoardInit();

  pinMode(22, INPUT);
  pinMode(24, INPUT);
  pinMode(26, INPUT); 

  pinMode(pinCapteurDroit, INPUT);
  pinMode(pinCapteurMilieu, INPUT);
  pinMode(pinCapteurGauche, INPUT);

  Serial.begin(9600);

}

void loop()
{

/*
  PID(0.2); 
  MOTOR_SetSpeed(moteurDroit,0);
  MOTOR_SetSpeed(moteurGauche,0);
  delay(5000);
 
  Serial.println(getDistanceInfrarouge(ROBUS_ReadIR(3)));
  delay(500);*/
  SuiveurLigne();
}
