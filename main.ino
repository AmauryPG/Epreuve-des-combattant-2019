#include <LibRobus.h>   

#include "functions.h"

#define moteurGauche 0
#define moteurDroit  1

void setup()
{
  BoardInit(); 
}

void loop()
{ 

  /*
  Serial.println(ROBUS_ReadIR(0));
  delay(100);
  
  SERVO_Enable(0);
  for(int i = 0; i <= 180; i+=10){
    SERVO_SetAngle(0,i);
    Serial.println(i);
    delay(1000);
  }*/
}