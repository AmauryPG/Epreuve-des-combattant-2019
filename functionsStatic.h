#include <LibRobus.h>
#include <Arduino.h>
#include <Adafruit_TCS34725.h>

#define moteurGauche 0
#define moteurDroit 1

#define pinCapteurGauche 1
#define pinCapteurMilieu 1
#define pinCapteurDroit 1

int getDistanceEncodeur(float distanceEnCM);
int getAngleEncodeur(float angleEnDegre);

int isTolerance(float input, float limite, float tolerance);
int isTolerance(float input, float limite, float tolerancePositive, float toleranceNegative);
int isZone(int rouge, int bleu, int vert, int rougeRef, int bleuRef, int vertRef, int tolerance);

uint16_t getDistanceInfrarouge(uint16_t rawInfoInfrarouge);
