#include "functionsStatic.h"

void PinceClose();
void PinceOpen();

void PID(double vitesse, double setPoint, double variable);
void PIDAvancer(float vitesseInitial, float vitesseFinale, float distanceCM, float distanceAcceleration);
void PIDAcceleration(float vitesseInitial, float vitesseFinale, float distanceCM);

void TournerSurPlace(float angleEnDegreCercle, float vitesse);