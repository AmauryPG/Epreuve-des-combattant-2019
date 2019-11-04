#include <LibRobus.h>
#include <Arduino.h> 

#define moteurGauche 0
#define moteurDroit 1

#define pinCapteurGauche 24
#define pinCapteurMilieu 22
#define pinCapteurDroit 26

#define vitesseNormale -0.3
#define vitesseLente -0.2
#define vitesseLentePlus -0.1


int getDistanceEncodeur(float distanceEnCM);
int getAngleEncodeur(float angleEnDegre);
int Conv_DigitalAnalog();
