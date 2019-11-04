#include "functionsMove.h"

void setup()
{
  BoardInit();

  PinceOpen();

  Serial.begin(9600);  
}

void loop()
{ 
  PID(0.5,ENCODER_Read(moteurGauche),ENCODER_Read(moteurDroit));
  /*
  if(ROBUS_IsBumper(3))
  {
    ChercherBalle(1);
  }
  delay(100000);*/
}
