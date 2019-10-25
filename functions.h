#include <LibRobus.h>   
#include <Arduino.h>  
#include <Adafruit_TCS34725.h>

#define moteurGauche 0
#define moteurDroit  1

#define pinCapteurGauche 24
#define pinCapteurMilieu 22
#define pinCapteurDroit 26

void EtapeUnCombattant();
void TournerSurPlace(float angleEnDegre, float vitesseSurUn); 
void Avancer(float distance, float vitesse);
void PID(float vitesse);
void SuiveurLigne();

int getDistanceEncodeur(float distanceEnCM);
int getAngleEncodeur(float angleEnDegre);
int Conv_DigitalAnalog();

uint16_t getDistanceInfrarouge(uint16_t rawInfoInfrarouge);
