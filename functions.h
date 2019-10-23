#include <LibRobus.h>   
#include <Arduino.h>  

#define moteurGauche 0
#define moteurDroit  1

#define pinCapteurGauche 1
#define pinCapteurMilieu 1
#define pinCapteurDroit 1

void EtapeUnCombattant();
void TournerSurPlace(float angleEnDegre, float vitesseSurUn); 
void Avancer(float distance, float vitesse);
void PID(float vitesse);

int getDistanceEncodeur(float distanceEnCM);
int getAngleEncodeur(float angleEnDegre); 
