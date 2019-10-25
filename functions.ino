#include "functions.h"

const int distributionZone[][2] = {{0, 1},
                                   {2, 3}};

//zone0, zone1, zone 2, zone 3
const int bleuZone[] = {0, 0, 0, 0};
const int vertZone[] = {0, 0, 0, 0};
const int rougeZone[] = {0, 0, 0, 0};

//-----------------------main fonction de l'etape 1---------------------------
void EtapeUnCombattant()
{
    //Chercher le mur le plus proche et tourne vers la direction
}

//---------------------------fonction secondaire-------------------------------

///////////////////////////////fonction get////////////////////////////////////

//retourne la distance en encodeur
int getDistanceEncodeur(float distanceEnCM)
{
    //formule pour transfomer CM pour encodeur (ratio ; 7,8cm : 1tour ; 3200pulse : 1tour)
    return (3200 / (PI * 7.8) * distanceEnCM) + 20;
}

//modifier legerement la variable de rotation vers 19, pas trop
//retourne l'angle(degre) en encodeur d'arc
int getAngleEncodeur(float angleEnDegre)
{
    //calcule pour avoir l'arc et ensuite le transfomer en language encodeur
    return getDistanceEncodeur((19.4) * PI * angleEnDegre / 360);
}

//retourne la distance en cm
uint16_t getDistanceInfrarouge(uint16_t rawInfoInfrarouge)
{
    if (rawInfoInfrarouge < 10)
    {
        rawInfoInfrarouge = 10;
    }
    return (6762 / (rawInfoInfrarouge - 9)) - 4;
}

//retourne 1 si la variable est dans la tolerence
int isTolerance(float input, float limite, float tolerance)
{
    return (input <= (limite + tolerance) && input >= (limite + tolerance));
}

int isTolerance(float input, float limite, float tolerancePositive, float toleranceNegative)
{
    return (input <= (limite + tolerancePositive) && input >= (limite - toleranceNegative));
}

//retourne 1 si le capteur est dans la bonne couleur
int isZone(int rouge, int bleu, int vert, Adafruit_TCS34725 tcs, int tolerance)
{
    uint16_t clear, rougeRef, vertRef, bleuRef;

    tcs.setInterrupt(false); // turn on LED

    delay(60); // takes 50ms to read

    tcs.getRawData(&rougeRef, &vertRef, &bleuRef, &clear);

    return (isTolerance(rouge, rougeRef, tolerance) && isTolerance(bleu, bleuRef, tolerance) && isTolerance(vert, vertRef, tolerance));
}

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

/*
float kp = 0.00967;
float ki = 0.00000004;
float kd = 0.00496;

float errorTr;
float lastErrorR;
float proportionR;
float integralR;
float derivativeR;

float circ = 7.8;

float speedR;

float integralActiveZone = 100;

void PID(float vitesse, int distance)
{
    int quit = 1;

    while (quit && distance < ENCODER_Read(moteurGauche))
    {
        int m;
        if (Serial.available() > 0)
        {
            m = Serial.read();
            if (m == 'q')
            {
                kp += 0.00001;
            }
            else if (m == 'Q')
            {
                kp -= 0.00001;
            }
            if (m == 'w')
            {
                ki += 0.00000001;
            }
            else if (m == 'W')
            {
                ki -= 0.00000001;
            }
            if (m == 'e')
            {
                kd += 0.00001;
            }
            else if (m == 'E')
            {
                kd -= 0.00001;
            }
            if (m == 'p')
            {
                quit = 0;
            }
        }

        float errorR = ENCODER_Read(moteurGauche) - ENCODER_Read(moteurDroit);

        if (errorR < integralActiveZone && errorR != 0)
        {
            errorTr += errorR;
        }
        else
        {
            errorTr = 0;
        }

        if (errorTr > 70 / ki)
        {
            errorTr = 70 / ki;
        }

        if (errorR == 0)
        {
            derivativeR = 0;
        }

        proportionR = errorR * kp;

        integralR = errorTr * ki;

        derivativeR = (errorR - lastErrorR) * kd;

        lastErrorR = errorR;

        //motor
        speedR = proportionR + integralR + derivativeR;

        Serial.println(errorR);

        MOTOR_SetSpeed(moteurGauche, vitesse);
        MOTOR_SetSpeed(moteurDroit, vitesse + speedR);

        delay(10);
    }
    Serial.println(kp, 5);
    Serial.println(ki, 8);
    Serial.println(kd, 5);
}
*/

double kp = 0.010;
double ki = 0.0000001;
double kd = 0.018;

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
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

void PIDSuiveurLigne(float vitesse)
{
    currentTime = millis();                             //get current time
    elapsedTime = (double)(currentTime - previousTime); //compute time elapsed from previous computation

    error = digitalRead(pinCapteurMilieu) - 2 * digitalRead(pinCapteurGauche) - 4 * digitalRead(pinCapteurDroit); // determine error
    TotalError += error * elapsedTime;                                                                            // compute integral
    rateError = (error - lastError) / elapsedTime;                                                                // compute derivative

    double out = Kp * error + Ki * TotalError + Kd * rateError; //PID output

    lastError = error;          //remember current error
    previousTime = currentTime; //remember current time

    MOTOR_SetSpeed(moteurDroit, out);
    MOTOR_SetSpeed(moteurGauche, vitesse);
    delay(5); //delay for the motors
}