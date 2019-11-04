#include "functionsStatic.h"

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

// Conversion Digital -> Analog
// Sert à la fonction suive
int Conv_DigitalAnalog()
{
    int Value_CapteurG, Value_CapteurD, Value_CapteurM, Value_SumCapteur;

    if (digitalRead(pinCapteurGauche) == 1)
    {
        Value_CapteurG = 2;
    }
    else
    {
        Value_CapteurG = 0;
    }
    if (digitalRead(pinCapteurMilieu) == 1)
    {
        Value_CapteurM = 4;
    }
    else
    {
        Value_CapteurM = 0;
    }
    if (digitalRead(pinCapteurDroit) == 1)
    {
        Value_CapteurD = 8;
    }
    else
    {
        Value_CapteurD = 0;
    }
    Value_SumCapteur = Value_CapteurG + Value_CapteurM + Value_CapteurD;
    return Value_SumCapteur;
}
