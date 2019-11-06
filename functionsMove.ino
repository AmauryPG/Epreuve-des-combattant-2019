#include "functionsMove.h"

//               pour le robot B
double kp = 0.010;
double ki = 0.0000004;
double kd = 0.018;

float vitesse = 0.5;

///////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////fonction action////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

int distance_virage(float angle_en_degre, int virage)
{ 
    if (virage == gauche)
    { 
        return getDistanceEncodeur(((angle_en_degre * PI * 18.5) / 360.0)-2.3009*pow(10,-7)*pow(angle_en_degre,3)+0.000143121*pow(angle_en_degre,2)-0.01121*angle_en_degre+0.222065);
    }
    else if (virage == droit)
    {
        { 
            return getDistanceEncodeur(((angle_en_degre * PI * 18.5) / 360.0) + 3);
        }
    } 
}

//tourne sur place avec un angle et une vitesse
void TournerSurPlace(float angle_en_degre, int virage, float vitesse)
{
    float pulse;
    ENCODER_ReadReset(moteurGauche);
    ENCODER_ReadReset(moteurDroit);

    if (virage == gauche)
    {
        pulse = distance_virage(angle_en_degre, gauche);

        while (abs(ENCODER_Read(moteurDroit)) < pulse && abs(ENCODER_Read(moteurGauche)) < pulse)
        {
            MOTOR_SetSpeed(moteurGauche, -vitesse);
            MOTOR_SetSpeed(moteurDroit, vitesse);
        }

        MOTOR_SetSpeed(moteurGauche, 0);
        MOTOR_SetSpeed(moteurDroit, 0);
    }
    else if (virage == droit)
    {
        pulse = distance_virage(angle_en_degre, droit);
        
        while (abs(ENCODER_Read(moteurDroit)) < pulse && abs(ENCODER_Read(moteurGauche)) < pulse)
        {
            MOTOR_SetSpeed(moteurGauche, vitesse);
            MOTOR_SetSpeed(moteurDroit, -vitesse);
        }

        MOTOR_SetSpeed(moteurGauche, 0);
        MOTOR_SetSpeed(moteurDroit, 0);
    }
}

int Conv_DigitalAnalog() // Conversion Digital -> Analog
{
    return 2 * digitalRead(pinCapteurGauche) + 4 * digitalRead(pinCapteurMilieu) + 8 * digitalRead(pinCapteurDroit);
}

void SuiveurLigne() // Prototype #1 du suiveur de ligne
{
    switch (Conv_DigitalAnalog())
    {
    case 2:
        MOTOR_SetSpeed(moteurDroit, -0.6);
        MOTOR_SetSpeed(moteurGauche, -0.3);
        break;
    case 4:
        MOTOR_SetSpeed(moteurDroit, -0.6);
        MOTOR_SetSpeed(moteurGauche, -0.6);
        break;
    case 6:
        MOTOR_SetSpeed(moteurDroit, -0.6);
        MOTOR_SetSpeed(moteurGauche, -0.4);
        break;
    case 8:
        MOTOR_SetSpeed(moteurGauche, -0.6);
        MOTOR_SetSpeed(moteurDroit, -0.3);
        break;
    case 12:
        MOTOR_SetSpeed(moteurGauche, -0.6);
        MOTOR_SetSpeed(moteurDroit, -0.4);
        break;
    case 14:
        MOTOR_SetSpeed(moteurDroit, 0);
        MOTOR_SetSpeed(moteurGauche, 0);

        break;

    default:
        MOTOR_SetSpeed(moteurDroit, -0.2);
        MOTOR_SetSpeed(moteurGauche, -0.2);
        break;
    }
}

//*********************************************************************************************************
//                                             PID general
//                                            *Pas toucher*
//*********************************************************************************************************

unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double TotalError, rateError;

void PID(double vitesse, double setPoint, double variable)
{
    currentTime = millis();                             //get current time
    elapsedTime = (double)(currentTime - previousTime); //compute time elapsed from previous computation

    error = setPoint - variable;                   // determine error
    TotalError += error * elapsedTime;             // compute integral
    rateError = (error - lastError) / elapsedTime; // compute derivative

    double out = kp * error + ki * TotalError + kd * rateError; //PID output

    lastError = error;          //remember current error
    previousTime = currentTime; //remember current time

    MOTOR_SetSpeed(moteurDroit, out);
    MOTOR_SetSpeed(moteurGauche, vitesse);
    delay(5); //delay for the motors
}

void PIDAcceleration(float vitesseInitial, float vitesseFinale, float distanceCM)
{
    /****
     * x   | y
     * 100 | 109
     * 95  | 53
     * 90  | 52
     * 70  | 50
     * 50  | 50
     * 25  | 48
    */
    float constante = ((-24.5407) / (distanceCM - 100.408)) + 48.8973;
    float acceleration = constante * ((pow(vitesseFinale, 2) - pow(vitesseInitial, 2)) / (2 * getDistanceEncodeur(distanceCM)));
    int temps = 1;
    float vitesseModifier = vitesseInitial + acceleration;

    while (vitesseModifier <= vitesseFinale)
    {
        //moteur droit est slave et gauche est master
        PID(vitesseModifier, ENCODER_Read(moteurGauche), ENCODER_Read(moteurDroit));

        temps++;
        vitesseModifier = vitesseInitial + acceleration * temps;
    }

    ENCODER_Reset(moteurGauche);
    ENCODER_Reset(moteurDroit);
}

void PIDAvancer(float vitesseInitial, float vitesseFinale, float distanceCM, float distanceAcceleration)
{
    PIDAcceleration(vitesseInitial, vitesseFinale, distanceAcceleration);
    while (ENCODER_Read(moteurGauche) < getDistanceEncodeur(distanceCM))
    {
        PID(vitesseFinale, ENCODER_Read(moteurGauche), ENCODER_Read(moteurDroit));
    }
}

void PinceOpen()
{
    SERVO_SetAngle(pince, 170);
}

void PinceClose()
{
    SERVO_SetAngle(pince, 90);
}

void PinceAttraper()
{
    PinceOpen();
    delay(700);
    PinceClose();
    delay(700);
    PinceOpen();
}

void PinceLaisseBalle()
{
    for (int i = 0; i < 80; i++)
    {
        SERVO_SetAngle(pince, 140);
        delay(80);
        SERVO_SetAngle(pince, 170);
        delay(80);
    }
}

/****************************************************************************************
 *                              fonctions principaux
****************************************************************************************/
void porterBalle(int zone)
{
    //distribution des zones
    //0         1
    //   robot
    //2         3

    /**************************************Aller chercher la balle au centre**************************************/
    //aller assez lentement pour pas pousser la balle
    PIDAvancer(0, 0.2, 10, 5);

    //attraper la balle
    PinceClose();

    /********************************************Aller porter la balle********************************************/
    switch (zone)
    {
    case 0:
        //avancer assez pour eviter la zone noir
        PIDAvancer(0, vitesse, 10, 5);

        //tourner a gauche
        TournerSurPlace(90, gauche, vitesse);

        //avancer jusqu'a la ligne
        while (!digitalRead(pinCapteurDroit) && !digitalRead(pinCapteurDroit) && !digitalRead(pinCapteurDroit))
        {
            PID(vitesse, ENCODER_Read(moteurGauche), ENCODER_Read(moteurDroit));
        }

        //suiveur de ligne
        /***************************************************/

        //foncer dans le mur
        /**************verifier la distance*****************/
        PIDAvancer(vitesse, vitesse, 30, 0);

        //relacher la balle
        PinceOpen();

        //reculer de la zone
        PIDAvancer(0, -vitesse, 10, 0);
        break;
    case 1:
        //avancer assez pour eviter la zone noir
        PIDAvancer(0, vitesse, 10, 5);

        //tourner a droit
        TournerSurPlace(90, droit, vitesse);

        //avancer jusqu'a la ligne
        while (!digitalRead(pinCapteurDroit) && !digitalRead(pinCapteurDroit) && !digitalRead(pinCapteurDroit))
        {
            PID(vitesse, ENCODER_Read(moteurGauche), ENCODER_Read(moteurDroit));
        }

        //suiveur de ligne
        /***************************************************/

        //foncer dans le mur
        /**************verifier la distance*****************/
        PIDAvancer(vitesse, vitesse, 30, 0);

        //relacher la balle
        PinceOpen();

        //reculer de la zone
        PIDAvancer(0, -vitesse, 10, 0);
        break;
    case 2:
        //avancer assez pour eviter la zone noir
        PIDAvancer(0, vitesse, 5, 3);

        //tourner a gauche
        TournerSurPlace(90, gauche, vitesse);

        //avancer jusqu'a la ligne
        while (!digitalRead(pinCapteurDroit) && !digitalRead(pinCapteurDroit) && !digitalRead(pinCapteurDroit))
        {
            PID(vitesse, ENCODER_Read(moteurGauche), ENCODER_Read(moteurDroit));
        }

        //tourner a gauche
        TournerSurPlace(90, gauche, vitesse);

        //avancer jusqu'a la ligne
        while (!digitalRead(pinCapteurDroit) && !digitalRead(pinCapteurDroit) && !digitalRead(pinCapteurDroit))
        {
            PID(vitesse, ENCODER_Read(moteurGauche), ENCODER_Read(moteurDroit));
        }

        //suiveur de ligne
        /***************************************************/

        //foncer dans le mur
        /**************verifier la distance*****************/
        PIDAvancer(vitesse, vitesse, 30, 0);

        //relacher la balle
        PinceOpen();

        //reculer de la zone
        PIDAvancer(0, -vitesse, 10, 0);
        break;
    case 3:
        //avancer assez pour eviter la zone noir
        PIDAvancer(0, vitesse, 5, 3);

        //tourner a droit
        TournerSurPlace(90, droit, vitesse);

        //avancer jusqu'a la ligne
        while (!digitalRead(pinCapteurDroit) && !digitalRead(pinCapteurDroit) && !digitalRead(pinCapteurDroit))
        {
            PID(vitesse, ENCODER_Read(moteurGauche), ENCODER_Read(moteurDroit));
        }

        //tourner a droit
        TournerSurPlace(90, droit, vitesse);

        //avancer jusqu'a la ligne
        while (!digitalRead(pinCapteurDroit) && !digitalRead(pinCapteurDroit) && !digitalRead(pinCapteurDroit))
        {
            PID(vitesse, ENCODER_Read(moteurGauche), ENCODER_Read(moteurDroit));
        }

        //suiveur de ligne
        /***************************************************/

        //foncer dans le mur
        /**************verifier la distance*****************/
        PIDAvancer(vitesse, vitesse, 30, 0);

        //relacher la balle
        PinceOpen();

        //reculer de la zone
        PIDAvancer(0, -vitesse, 10, 0);
        break;
    }
}