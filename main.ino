#include "functionsMove.h"

void setup()
{
  BoardInit();

  PinceOpen();

  Serial.begin(9600);  
}

void loop()
{ 
  
  ChercherBalle(1);
  delay(10);
}
