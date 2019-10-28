/*
Projet: Robot cuisinier
Equipe: p12
Auteurs: Ludo et JORDAN JEROME
Description: Breve description du script
Date: Derniere date de modification
*/

/* ****************************************************************************
Inclure les librairies de functions que vous voulez utiliser
**************************************************************************** */

#include <LibRobus.h> // Essentielle pour utiliser RobUS
#include <stdio.h>
#include <math.h>


/* ****************************************************************************
Variables globales et defines
**************************************************************************** */
#define pi 3.141592653589793238
#define moteurGauche 0
#define moteurDroit 1
#define gauche 1
#define droit -1

/* ****************************************************************************
Vos propres fonctions sont creees icifgfhfhfhf
**************************************************************************** */

int CM_to_pulse(float distance_en_cm)
{
  int pulse = ((3200*distance_en_cm)/(7.8*pi))+30;
  return (pulse);
}

int distance_virage(float angle_en_degre, int virage)
{
  int pulse;
  if (virage == gauche)
  {
    float arc = ((angle_en_degre*pi*18.0)/360.0)+2.3;
    pulse = CM_to_pulse(arc);
  }
  else if (virage == droit)
  {
    {
      float arc = ((angle_en_degre*pi*18.0)/360.0)+3;
      pulse = CM_to_pulse(arc);
    }
  }
  return pulse;
}


int calc_erreur(int reset)
{
  int erreur = 0;
  static int derniere_lecture_gauche = 0;
  static int derniere_lecture_droit = 0;
  if (reset == 1)
  {
    erreur =0;
    derniere_lecture_gauche = 0;
    derniere_lecture_droit = 0;
    return 0;
  }
  erreur = (abs(ENCODER_Read(moteurGauche)) - derniere_lecture_gauche) - abs((ENCODER_Read(moteurDroit)) - derniere_lecture_droit);
  derniere_lecture_gauche = abs((ENCODER_Read(moteurGauche)));
  derniere_lecture_droit = abs((ENCODER_Read(moteurDroit)));
  return erreur;
}

int calc_erreur_accumulee(int reset)
{
  int erreur = 0;
  static int erreur_accumulee = 0;
  if (reset == 1)
  {
    erreur_accumulee = 0;
    return 0;
  }
  erreur = calc_erreur(0);
  erreur_accumulee += erreur;
  return erreur_accumulee;
}

float pid(/*float vitesse_slave*/)
{
  int erreur = calc_erreur(0);
  int erreur_accumulee = calc_erreur_accumulee(0);
  float ajout = 0;
  float kp = 0.011;
  float ki = 0.000016;
  ajout = (kp*erreur) + (ki*erreur_accumulee);
  return ajout;
//  vitesse_slave += ajout;
//  return vitesse_slave;
}


void accelere(float vitesse_max, int bonds)
{
  int erreur = 0;
  float vitesse_presente = 0.15;
  int i = 1;
  float vitesse_slave = vitesse_presente;
  int erreur;
  MOTOR_SetSpeed(moteurGauche,vitesse_presente);
	MOTOR_SetSpeed(moteurDroit,vitesse_slave);
  while(vitesse_presente <= vitesse_max)
  {
    MOTOR_SetSpeed(moteurGauche, vitesse_presente);
    MOTOR_SetSpeed(moteurDroit, vitesse_slave);
    erreur = calc_erreur(0);
    while(-50 > erreur && 50 > erreur)
    {
      vitesse_slave += pid();
      MOTOR_SetSpeed(moteurDroit, vitesse_slave);
    }
    vitesse_presente += i/bonds*vitesse_max;
    vitesse_slave = vitesse_presente;
    i++;
  }
}

void avance(float distance, float vitesse)
{
  delay(10);
  int pulse = CM_to_pulse(distance);
  float vitesse_slave = vitesse;
  float temps_mesure = 5;
  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);
  
  MOTOR_SetSpeed(moteurDroit, vitesse_slave);
	MOTOR_SetSpeed(moteurGauche, vitesse);
  while (ENCODER_Read(moteurGauche) < pulse) //divise par deux si accel est presente
  {
    vitesse_slave += pid(/*vitesse_slave*/);
    MOTOR_SetSpeed(moteurDroit, vitesse_slave);
    delay(5);
  }
  MOTOR_SetSpeed(moteurDroit, 0);
	MOTOR_SetSpeed(moteurGauche, 0);
  calc_erreur_accumulee(1);
	calc_erreur(1);
}

void tourne(float angle_en_degre, int virage, float vitesse)
{
  float pulse;
  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);
  if (virage == gauche)
  {
    pulse = distance_virage(angle_en_degre, gauche);
    while (abs(ENCODER_Read(1)) < pulse && abs(ENCODER_Read(0)) < pulse)
    {
      MOTOR_SetSpeed(0, -vitesse);
      MOTOR_SetSpeed(1, vitesse);
//      vitesse_slave += pid();
    }
    MOTOR_SetSpeed(1, 0);
    MOTOR_SetSpeed(0,0);
  }
  else if (virage == droit)
  {
    pulse = distance_virage(angle_en_degre, droit);
    while (abs(ENCODER_Read(0)) < pulse && ENCODER_Read(1) < pulse)
    {
      MOTOR_SetSpeed(0, vitesse);
      MOTOR_SetSpeed(1, -vitesse);
    }
    MOTOR_SetSpeed(1, 0);
    MOTOR_SetSpeed(0,0);
  }
}

/*
void tourne(float angle_en_degre, int virage, float vitesse)
{
  float pulse;
  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);
  if (virage == gauche)
  {
    pulse = distance_virage(angle_en_degre, gauche);
    while (abs(ENCODER_Read(1)) < pulse) //&& ENCODER_Read(0) < pulse )
    {
      MOTOR_SetSpeed(0, 0); //-vitesse);
      MOTOR_SetSpeed(1, vitesse);
    }
    MOTOR_SetSpeed(1, 0);
//    MOTOR_SetSpeed(0,0);
  }
    else if (virage == droit)
    {
      pulse = distance_virage(angle_en_degre, droit);
      while (abs(ENCODER_Read(0)) < pulse )//&& ENCODER_Read(1) < pulse)
    {
      MOTOR_SetSpeed(0, vitesse);
      MOTOR_SetSpeed(1, 0);//-vitesse);
    }
//    MOTOR_SetSpeed(1, 0);
    MOTOR_SetSpeed(0,0);
  }
//  delay(500);
}
*/

/*void avancer(float distance, float vitesse)
{
  float erreur,kp, ajout;
  int temps_mesure;
  int pulse = CM_to_pulse(distance);
  Serial.println("Pulse is");
  Serial.println(pulse);
  ajout = 0;
  ENCODER_ReadReset(0);
  ENCODER_ReadReset(1);
  temps_mesure = 5;
  while ((ENCODER_Read(0) < pulse))
  {
    MOTOR_SetSpeed(0, vitesse);
    MOTOR_SetSpeed(1, vitesse + ajout);
    delay(temps_mesure);
    kp = 0.001;
    erreur = ENCODER_Read(0)- ENCODER_Read(1);
    erreur /= temps_mesure;
    ajout = erreur*kp;
  }
  MOTOR_SetSpeed(0, 0);
  MOTOR_SetSpeed(1, 0);
  delay(500);
}
*/

/* ****************************************************************************
Fonctions d'initialisation (setup)
**************************************************************************** */
// -> Se fait appeler au debut du programme
// -> Se fait appeler seulement un fois
// -> Generalement on y initilise les varibbles globales

void setup(){
  BoardInit();
  
}


/* ****************************************************************************
Fonctions de boucle infini (loop())
**************************************************************************** */
// -> Se fait appeler perpetuellement suite au "setup"

void loop()
{
  accelere(0.3, 100);
//  avance(50, 0.3);
//  tourne (180, 1, 0.2);
delay(2000);
// SOFT_TIMER_Update(); // A decommenter pour utiliser des compteurs logiciels
  delay(10);// Delais pour décharger le CPU
}

