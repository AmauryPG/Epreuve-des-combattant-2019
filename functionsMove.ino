#include "functionsMove.h"

const int distributionZone[][2] = {{0, 1},
                                   {2, 3}};

//zone0, zone1, zone 2, zone 3
const int bleuZone[] = {0, 0, 0, 0};
const int vertZone[] = {0, 0, 0, 0};
const int rougeZone[] = {0, 0, 0, 0};

///////////////////////////////fonction action////////////////////////////////////

void ChercherZoneComplexe(Adafruit_TCS34725 tcs)
{
    AlignerLigne();

    do
    {
        PIDSuiveurLigne(0.5);
    } while (isZone(rougeZone[0], bleuZone[0], vertZone[0], tcs, 10) || isZone(rougeZone[1], bleuZone[1], vertZone[1], tcs, 10) || isZone(rougeZone[2], bleuZone[2], vertZone[2], tcs, 10) || isZone(rougeZone[3], bleuZone[3], vertZone[3], tcs, 10));

    //avancer un peu pour etre au milieu de la zone

    if (isZone(rougeZone[bonneZone], bleuZone[bonneZone], vertZone[bonneZone], tcs, 10))
    {
        //trouver la bonne zone
        //AttraperBalle();
    }
    else
    {
        TournerSurPlace(180, 0.3);
        TournerSurPlace(45, 0.3);
        TournerSurPlace(90, 0.3);
    }
}

//aligner le robot avec la ligne
void AlignerLigne()
{
    //moitie du robot
    // PID(0.5, getDistanceEncodeur(10));
    //tourner pour etre parallele a la ligne
    TournerSurPlace(90, 0.3);
}

//trouve la ligne selon ses capteurs infrarouges
////////////////////////////////////ERREUR POSSIBLE : les limites de l'infrarouge
void TrouverLigne()
{
    TournerSurPlace(ChercherMur(), 0.5);
    PIDLigne(-0.5);
}

//avance jusqu'a trouver la ligne
void PIDLigne(float vitesse)
{
    int detecteurCouleur;
    while (digitalRead(pinCapteurGauche) || digitalRead(pinCapteurMilieu) || digitalRead(pinCapteurDroit))
    {
        //PID();
    }
    MOTOR_SetSpeed(moteurGauche, 0);
    MOTOR_SetSpeed(moteurDroit, 0);
}

//tourne sur place avec un angle et une vitesse
void TournerSurPlace(float angleEnDegre, float vitesseSurUn)
{
    int moteur1 = moteurDroit;
    int moteur2 = moteurGauche;
    int direction = 1;

    if (angleEnDegre < 0)
    {
        moteur1 = moteurGauche;
        moteur2 = moteurDroit;
        direction = -1;
    }

    //set distance en encodeur
    int angleEncodeur = getAngleEncodeur(angleEnDegre);

    //avance les deux moteurs
    while (direction * angleEncodeur >= ENCODER_Read(moteur1) && direction * -angleEncodeur <= ENCODER_Read(moteur2))
    {
        MOTOR_SetSpeed(moteur1, vitesseSurUn);
        MOTOR_SetSpeed(moteur2, -vitesseSurUn);
    }
    //reset et arrete les moteurs avec un delay
    MOTOR_SetSpeed(moteur1, 0);
    MOTOR_SetSpeed(moteur2, 0);
}

//cherche le mur plus proche et retourne l'angle de ce mur
int ChercherMur()
{
    int petiteDistance;
    int grandeDistance;
    int grandAngle;
    int petitAngle;

    //initialise la valeur initial de direction
    if (ROBUS_ReadIR(3) > ROBUS_ReadIR(2))
    {
        petiteDistance = ROBUS_ReadIR(2);
        grandeDistance = ROBUS_ReadIR(3);
    }
    else
    {
        petiteDistance = ROBUS_ReadIR(3);
        grandeDistance = ROBUS_ReadIR(2);
    }

    for (int angle = 10; angle <= 180; angle += 10)
    {
        //tourne sur place
        TournerSurPlace(angle, 0.1);
        delay(10);

        //compare les different resultat et retourne le plus petit
        if (ROBUS_ReadIR(3) > ROBUS_ReadIR(2))
        {
            if (petiteDistance > ROBUS_ReadIR(2))
            {
                petiteDistance = ROBUS_ReadIR(2);
                petitAngle = angle;
            }

            if (grandeDistance < ROBUS_ReadIR(2))
            {
                grandeDistance = ROBUS_ReadIR(2);
                grandAngle = angle;
            }
        }
        else
        {
            if (petiteDistance > ROBUS_ReadIR(3))
            {
                petiteDistance = ROBUS_ReadIR(3);
                petitAngle = angle;
            }

            if (grandeDistance < ROBUS_ReadIR(3))
            {
                grandeDistance = ROBUS_ReadIR(3);
                grandAngle = angle;
            }
        }
    }
    //le plus 180 est pour commence ou le zero est
    //============================================distance entre mur et ligne============================================
    if (petiteDistance < getDistanceInfrarouge(46))
    {
        return petitAngle + 180;
    }
    else
    {
        return grandAngle + 180;
    }
}

double kp = 0.010;
double ki = 0.0000001;
double kd = 0.018;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double TotalError, rateError;

void PIDMotor(double vitesse)
{
    currentTime = millis();                             //get current time
    elapsedTime = (double)(currentTime - previousTime); //compute time elapsed from previous computation

    error = ENCODER_Read(moteurGauche) - ENCODER_Read(moteurDroit); // determine error
    TotalError += error * elapsedTime;                              // compute integral
    rateError = (error - lastError) / elapsedTime;                  // compute derivative

    double out = kp * error + ki * TotalError + kd * rateError; //PID output

    lastError = error;          //remember current error
    previousTime = currentTime; //remember current time

    MOTOR_SetSpeed(moteurDroit, out);
    MOTOR_SetSpeed(moteurGauche, vitesse);
    delay(5); //delay for the motors
}

double Kp = 0.010;
double Ki = 0.0000001;
double Kd = 0.018;

unsigned long currentTime2, previousTime2;
double elapsedTime2;
double error2;
double lastError2;
double TotalError2, rateError2;

void PIDSuiveurLigne(float vitesse)
{
    currentTime2 = millis();                               //get current time
    elapsedTime2 = (double)(currentTime2 - previousTime2); //compute time elapsed from previous computation

    error2 = digitalRead(pinCapteurMilieu) - 2 * digitalRead(pinCapteurGauche) - 4 * digitalRead(pinCapteurDroit); // determine error
    TotalError2 += error2 * elapsedTime2;                                                                          // compute integral
    rateError2 = (error2 - lastError2) / elapsedTime2;                                                             // compute derivative

    double out = Kp * error2 + Ki * TotalError2 + Kd * rateError2; //PID output

    lastError2 = error2;          //remember current error
    previousTime2 = currentTime2; //remember current time

    MOTOR_SetSpeed(moteurDroit, out);
    MOTOR_SetSpeed(moteurGauche, vitesse);
    delay(5); //delay for the motors
}

void PIDacceleration(float vitesseInitial, float vitesseFinale, float distanceCM)
{
    float acceleration = (pow(vitesseFinale, 2) - pow(vitesseInitial, 2)) / (2 * getDistanceEncodeur(distanceCM));
    int temps = 1;
    float vitesseModifier = vitesseInitial + acceleration * temps;

    while (vitesseModifier <= vitesseFinale)
    {
        PIDMotor(vitesseModifier);
        temps++;
        vitesseModifier = vitesseInitial + acceleration * temps;
    } 
}
