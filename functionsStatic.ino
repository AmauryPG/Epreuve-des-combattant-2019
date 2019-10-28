#include "functionsStatic.h"

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

