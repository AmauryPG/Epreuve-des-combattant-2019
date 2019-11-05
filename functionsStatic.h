#include <LibRobus.h>
#include <Arduino.h> 

#define moteurGauche 0
#define moteurDroit 1

#define gauche 0
#define droit 1

#define pinCapteurGauche 1
#define pinCapteurMilieu 1
#define pinCapteurDroit 1

int getDistanceEncodeur(float distanceEnCM);
int getAngleEncodeur(float angleEnDegre); 
