#include "functions.h"

//-----------------------main fonction de l'etape 1---------------------------
void EtapeUnCombattant()
{
    //Chercher le mur le plus proche et tourne vers la direction
    TournerSurPlace(ChercherLignePlusProche(), 0.4);
    ENCODER_Reset(moteurDroit);
    ENCODER_Reset(moteurGauche);
}
//kjkjkjkj
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

///////////////////////////////fonction action////////////////////////////////////

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
int ChercherLignePlusProche()
{
    int petiteDistance;
    int grandeDistance;

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
            }

            if (grandeDistance < ROBUS_ReadIR(2))
            {
                grandeDistance = ROBUS_ReadIR(2);
            }
        }
        else
        {
            if (petiteDistance > ROBUS_ReadIR(3))
            {
                petiteDistance = ROBUS_ReadIR(3);
            }

            if (grandeDistance < ROBUS_ReadIR(3))
            {
                grandeDistance = ROBUS_ReadIR(3);
            }
        }
    }
    //le plus 180 est pour commence ou le zero est
    if (petiteDistance < 45.72)
    {
        return petiteDistance + 180;
    }
    else
    {
        return grandeDistance + 180;
    } 
}