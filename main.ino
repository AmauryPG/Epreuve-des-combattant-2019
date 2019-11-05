#include "functionsMove.h"

void setup()
{
  BoardInit();

  Serial.begin(9600);  
}

void loop()
{ 
  /*PIDAcceleration(0.0, 0.3,30);
  MOTOR_SetSpeed(0,0);
  MOTOR_SetSpeed(1,0);
  delay(5000);
  delay(10);*/

  SuiveurLigne();
  delay(50);
}
