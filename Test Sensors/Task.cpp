#include "Arduino.h" 
#include "parameters.h"
#include "Motor.h"
#include "MyServo.h"
#include "Commandes.h"

// on définit ici toutes les variables auxquelles doivent accéder les taches
void init_arduino();
extern Motor FRmotor;
extern Motor FLmotor;
extern Motor RRmotor;
extern Motor RLmotor;
extern long int volatile CoderCount[4] ; 
extern bool obst;
extern MyServo frontservo;

int value[4] = {0,0,0,0};  // 'signed PWM' values between -255 and 255
int setpoint[4] = {0,0,0,0}; // PWM setpoints between -255 and 255
unsigned char deltaMax = 10;  // maximum gap between 2 successive values

//g
volatile unsigned long pulseStart = 0;   
volatile unsigned long pulseDuration = 0;
int distanceUS = 80;
int usObstacleCounter = 0;
// gestion du multitache. définition de la période de chaque tache
// definition du nombre de taches maximales
#define numtask 4
int deltask[numtask]={0,1000,500,100} ;
int timtask[numtask] ;
bool taskon[numtask]={true,false,true,false} ;
void task0(); // tache permanente de lecture de la liaison série 
void task1(); // tache du chien de garde
void task2(); // tache de transition douce pour les moteurs
void task3();

void (*NameTask[numtask])() = {  task0,task1,task2,task3  };
int convertMeasureIR(float volts) {
  if (volts < 1.0) {
    return (int)(28.0 / volts);
  } else {
    volts -= 0.28;
    if (volts <= 0) return 80;
    return (int)(20.2 / volts);
  }
}
///////////////////////////////////////////////////////////////////////////
//   mise en route et arret des taches périodiques
/////////////////////////////////////////////////////////////////////////
void TaskOn(int task)
{ taskon[task]=true;  
  timtask[task] = (int)millis()+deltask[task]; 
}

void TaskOff(int task)
{ taskon[task]=false;  
}

void TaskOffAll() {
  for (int k=1; k<numtask; k++)
    taskon[k]=false ;
}

bool TaskState(int task) {
  return(taskon[task]);
}

  // lancement des differentes taches périodiques
void launch_tasks() {
  for (int k=0;k<numtask;k++)
    if (taskon[k]) 
      if ((((int)millis()-timtask[k])>0) || (deltask[k]==0))   // si on a dépassé le temps du prochain lancement ou si tache permanente
      {
        NameTask[k]() ;                   // on lance la tache k
        timtask[k]=timtask[k]+deltask[k] ;
      }
}

//g
void timeEcho() {
  if (digitalRead(URM_ECHO) == LOW)      // Front descendant : début d'impulsion[cite: 4]
    pulseStart = micros();              //[cite: 4]
  else {                                 // Front montant : fin d'impulsion[cite: 4]
    pulseDuration = micros() - pulseStart; //[cite: 4]
  }
}
// Initialisation de l'URM37 (à appeler dans setup / init_arduino)[cite: 4]
void URMsetup() {
  pinMode(URM_ECHO, INPUT_PULLUP);      //[cite: 4]
  attachInterrupt(digitalPinToInterrupt(URM_ECHO), timeEcho, CHANGE); //[cite: 4]
}


////////////////////////////////////////////////////////////////////
//
// LES TACHES PERIODIQUES
//
////////////////////////////////////////////////////////////////////

extern bool ConnOn ;

// tache permanente de lecture de la liaison série  
void task0() {
  char CharIn;
  if (Serial.available() > 0) {
    // on lit le premier caractère qui arrive
    CharIn = Serial.read();
    if ((CharIn=='A') || (ConnOn)) decod_serial(CharIn);  // on ne fait un decodage de commande que si on est connecté
  }

}

// tache du chien de garde  
void task1() {
  // A COMPLETER
}

void task2() {
  for (int i=0; i<4; i++) {
    if (value[i] < setpoint[i])
      if (value[i] > (setpoint[i]-deltaMax) )
        value[i] = setpoint[i];
      else
        value[i] += deltaMax;
    else
      if (value[i] < (setpoint[i]+deltaMax) )
        value[i] = setpoint[i];
      else
        value[i] -= deltaMax;
  }
  
  FRmotor.setvalue(value[0]); // after updating the value to improve reactivity to new setpoints 
  FLmotor.setvalue(value[1]);
  RRmotor.setvalue(value[2]);
  RLmotor.setvalue(value[3]);
}
void task3() {
  float volts = analogRead(IR_pin) * 5.0 / 1024.0;
  distanceInCm = convertMeasureIR(volts); // Calcul effectué UNIQUE dans la tâche

  // Condition de détection : distance valide et inférieure à 15 cm
  if (distanceInCm < 15 && distanceInCm != 0) {
    irObstacleCounter++; // Incrémente si la mesure est suspecte/proche
  } else {
    irObstacleCounter = 0; // Réinitialise le compteur si la voie est libre
  }

  // Déclenchement de la sécurité seulement après 3 détections consécutives
  if (irObstacleCounter >= 3) {
    obst = true;
    // Coupure d'urgence des consignes moteurs
    setpoint[0] = 0;
    setpoint[1] = 0;
    setpoint[2] = 0;
    setpoint[3] = 0;
  } else if (irObstacleCounter == 0) {
    obst = false;
  }
}

// Tâche #3 : Protection Ultrasons URM37
/*void task3() {
  // Calcul unique de la distance en cm (50us par cm)[cite: 4]
  if (pulseDuration > 0) {
    distanceUS = (int)(pulseDuration / 50); //[cite: 4]
  }

  // Ignorer les lectures invalides (ex: > 900 cm ou 0)[cite: 4]
  if (distanceUS > 900) { //[cite: 4]
    distanceUS = 80;
  }

  // Filtrage anti-bruit : 3 détections consécutives < 15 cm
  if (distanceUS < 15 && distanceUS > 0) {
    usObstacleCounter++;
  } else {
    usObstacleCounter = 0; // Réinitialise si la voie est claire
  }

  // Arrêt d'urgence si 3 mesures consécutives valides
  if (usObstacleCounter >= 3) {
    obst = true;
    setpoint[0] = 0;
    setpoint[1] = 0;
    setpoint[2] = 0;
    setpoint[3] = 0;
  } else if (usObstacleCounter == 0) {
    obst = false;
  }
}
*/

