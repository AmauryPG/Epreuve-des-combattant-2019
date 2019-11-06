#include <LibRobus.h>
#include <Arduino.h> 

#define moteurGauche 0
#define moteurDroit 1

#define pinCapteurGauche 24
#define pinCapteurMilieu 22
#define pinCapteurDroit 26

#define gauche 0
#define droit 1

#define pince 0

int getDistanceEncodeur(float distanceEnCM);
int getAngleEncodeur(float angleEnDegre); 
