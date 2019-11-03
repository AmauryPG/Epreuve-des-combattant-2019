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
