#include "functionsStatic.h"

void EtapeUnCombattant();
void TournerSurPlace(float angleEnDegre, float vitesseSurUn);
void Avancer(float distance, float vitesse);
void PIDMotor(double vitesse);
void PIDSuiveurLigne(float vitesse);
void PIDacceleration(float vitesseInitial, float vitesseFinale, float distance);