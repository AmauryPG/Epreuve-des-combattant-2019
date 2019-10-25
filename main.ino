#include "functions.h"

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup()
{
  BoardInit();

  Serial.begin(9600); 

  /*
  if (tcs.begin())
  {
    Serial.println("Found sensor");
  }
  else
  {
    Serial.println("No TCS34725 found ... check your connections");
    while (1)
    {
    }
  }*/
}

void loop()
{ 
  PIDacceleration(0,0.5,10);
  delay(10);
}
