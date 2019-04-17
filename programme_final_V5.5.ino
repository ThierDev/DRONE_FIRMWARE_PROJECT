#include<Wire.h>

////////////////////////////////////////////////////////////////////////////////////////
//Déclaration des variables 
////////////////////////////////////////////////////////////////////////////////////////

#include <Servo.h>
                        // qdqpter les coefficients du pid (voire protocole)
Servo myservoAV;
Servo myservoAR;
Servo myservoG;
Servo myservoD;
// valeurs volatile pour la commande intterompre. 
volatile int altitude_in,roll_in,pitch_in,yaw_in; // roll
volatile byte mem_ch1, mem_ch2, mem_ch3, mem_ch4;
volatile float temps_1,temps_2,temps_3,temps_4; 


int pinch1 = 52;
int pinch2 = 51;
int pinch3 = 53;
int pinch4 = 50;


int start;
int throttle;  

//################################# Variable pour  moteurs ###################################

int mot_AV, mot_AR, mot_G, mot_D;
unsigned long loop_timer, esc_loop_timer, temps_actuel;
unsigned long  timer_channel_1,timer_channel_2,timer_channel_3, timer_channel_4;


//################################# Variable du gyro ###################################
const int MPU_addr=0x68;  // I2C address of the MPU-6050
byte highByte, lowByte;
int cal_int;
int16_t gyro_roll,gyro_pitch,gyro_yaw,gyro_roll_ds,gyro_pitch_ds,gyro_yaw_ds;
int16_t gyro_roll_cal,gyro_pitch_cal,gyro_yaw_cal;


//################################# gains du PID #######################################

//gains pour le roulis:
float P_gain_roll = 2.0;
float I_gain_roll = 0.01;
float D_gain_roll = 5.0;
int max_roll = 400;

//gains pour le tangage:
float P_gain_pitch = 2.0;
float I_gain_pitch = 0.01;     //Pour l'étalonnage de ces valeurs, voir dossier word
float D_gain_pitch = 5.0;
int max_pitch = 400;

//gains pour le lacet:
float P_gain_yaw = 3.5;
float I_gain_yaw = 0.02;
float D_gain_yaw = 0.0;
int max_yaw = 400;


//################################# variables du PID #######################################

//valeurs du PID pour le roulis:
float val_P_roll;
float val_I_roll;
float val_D_roll;
float val_I_roll_pcdt = 0.0;
float roll_erreur_pcdt = 0.0;
float val_correction_roll;

//valeurs du PID pour le tangage:
float val_P_pitch;
float val_I_pitch;
float val_D_pitch;
float val_I_pitch_pcdt = 0.0;
float pitch_erreur_pcdt = 0.0;
float val_correction_pitch;

//valeurs du PID pour le lacet:
float val_P_yaw;
float val_I_yaw;
float val_D_yaw;
float val_I_yaw_pcdt = 0.0;
float yaw_erreur_pcdt = 0.0;
float val_correction_yaw;

//################################# Variables pour la télécommande ###################################



//variables pour le PID (pulsation convertie en dps):
float tel_roll_dps;
float tel_pitch_dps;
float tel_yaw_dps;

//erreur entre le gyro et la télécommande:
float roll_erreur;
float pitch_erreur;
float yaw_erreur;

int pin_servoAV = 6;
int pin_servoAR = 8;
int pin_servoG = 9;
int pin_servoD = 7;


void setup()
{
  DDRH |= B01111000;

  pinMode(13,OUTPUT);
  Serial.begin(9600);
  Wire.begin();  

  digitalWrite(13,HIGH);                                       //Allume la led 
  delay(3000); 

  
  Wire.beginTransmission(105);                                 //Demarre la communication avec le gyroscope 
  Wire.write(0x20);                                            //On veut ecrire sur le registre 1 (20 hex).
  Wire.write(0x0F);                                            //configure le bit registre a 00001111 ( cela reveille le gyroscope ) 
  Wire.endTransmission();                                      //Termine la communication avec le gyroscope. 

  Wire.beginTransmission(105);                                 //Demarre la communication avec le gyroscope 
  Wire.write(0x23);                                            //On veut ecrire sur le registre 4 (23 hex).
  Wire.write(0x90);                                            //Configure le bit registre a 10010000 (cela permet du mettre a jour le cache en permanence et configuration 500degres par secondes )
  Wire.endTransmission();                                      //Termine la communication avec le gyroscope.
    

  delay(250);   
  
    //Calibration avant vol gyro 
    for (cal_int = 0; cal_int < 2000 ; cal_int ++){              //Accumule 2000 valeurs pour la calibration
      if(cal_int % 15 == 0)digitalWrite(13, !digitalRead(13));  //Fait clignoter la LED pour indiquer la calibration.
      gyro_signal();                                           //Appelle la fonction gyroscope 
      gyro_roll_cal +=gyro_roll;                               // Fait la somme sur 2000 valeurs sur tout les axes 
      gyro_pitch_cal +=gyro_pitch;                              
      gyro_yaw_cal += gyro_yaw;                                  
      //pour éviter une décalibration des ESC on envoi encore une pulsation de 1000us 
      PORTH |= B01111000;
      delayMicroseconds(1000);
      PORTH &= B10000111;
      delay(3);
                                                      
    }
    //Maintenant on a optenu 2000 valeurs on peut faire la moyenne 
  gyro_roll_cal /= 2000;                                       //Divise le total par 2000 
  gyro_pitch_cal /= 2000;                                      
  gyro_yaw_cal /= 2000;                                        
  
  PCICR |= (1 << PCIE0);                                       //Pour que le PinChangeInterruptRoutine sur les port  PCIE0 
  PCMSK0 |= (1 << PCINT0);                                     //Le pin PCINT0 (digital input 53) va crée l'intérruption au changement d'état   
  PCMSK0 |= (1 << PCINT1);                                     //le pin PCINT1 (digital input 52) va crée l'intérruption au changement d'état  
  PCMSK0 |= (1 << PCINT2);                                     //le pin PCINT2 (digital input 51) va crée l'intérruption au changement d'état  
  PCMSK0 |= (1 << PCINT3);                                     //le pin PCINT3 (digital input 50) va crée l'intérruption au changement d'état  

  //Pour éviter la décalibration systèmatique des ESC on envoi une pulsation de 1000us jusqua recevoir le signal. 
  while(altitude_in <990 || altitude_in > 1020 || yaw_in <1800){
    start++;
    PORTH |= B01111000;
    delayMicroseconds(1000);
    PORTH &= B10000111;
    delay(3);  
    if (start==125){
      digitalWrite(13, !digitalRead(13));
      start=0; 
    }
    
  }
  myservoAV.attach(pin_servoAV); 
  myservoAR.attach(pin_servoAR);
  myservoG.attach(pin_servoG);
  myservoD.attach(pin_servoD);
  start=0;
  digitalWrite(13,LOW); 
  
}

void loop()
{ //Serial.print("Altitude in   "); Serial.print(altitude_in);Serial.print("  Yaw in:");Serial.print(yaw_in);Serial.print("  pitch"); Serial.print(pitch_in);Serial.print("   Tel roulis: "); Serial.println(roll_in);
  //Serial.print(" Tel tangage:"); Serial.print(gyro_pitch);Serial.print(" Tel lacet:"); Serial.print(gyro_yaw);Serial.print(" Tel roulis:"); Serial.println(gyro_roll);
  gyro_signal();
  //Serial.print(" Tel tangage: "); Serial.print(gyro_pitch_ds);Serial.print("   Tel lacet:"); Serial.print(gyro_yaw_ds);Serial.print("    Tel roulis:"); Serial.println(gyro_roll_ds);
  gyro_roll_ds = (gyro_roll_ds * 0.8) + ((gyro_roll / 57.14286) * 0.2);            //Conversion en degres par seconde 
  gyro_pitch_ds = (gyro_pitch_ds * 0.8) + ((gyro_pitch / 57.14286) * 0.2);         .
  gyro_yaw_ds = (gyro_yaw_ds * 0.8) + ((gyro_yaw / 57.14286) * 0.2);               
   //Pour démmarrer les moteurs: throttle low and yaw left (etape n0 1).
  if(altitude_in < 1050 && yaw_in < 1050)start = 1;
  //quand le stick de la telecommande reviens au centre on démmarre les moteurs (etapes 2).
  if(start == 1 && altitude_in < 1050 && yaw_in > 1490){
    start = 2;
    roll_erreur = 0;
    roll_erreur_pcdt = 0;
    pitch_erreur=0;
    pitch_erreur_pcdt =0;
    yaw_erreur = 0;
    yaw_erreur_pcdt= 0;
    
  }
  //On arrete les moteurs :  throttle low and yaw right.
  if(start == 2 && altitude_in< 1050 && yaw_in > 1950)start = 0;

  // On veut convertir la pulsation de la télécommande en degrés par seconde pour le roulis:
  tel_pitch_dps = tel_yaw_dps =tel_roll_dps = 0;
  if(roll_in > 1508)    //on prend une zone morte entre 1492 et 1508 pour éviter des perturbations inutiles (si la valeur de la télécommande n'est pas parfaitement calibré)
  {
    tel_roll_dps = (roll_in - 1508)/3.0;                                   // cela correspond à notre angle, partie positive (drone se décale à droite): 
  }                                                                         // (2000-1508)/2,73 = 180 d/s (la valeur maximale)
  else if(roll_in < 1492)
  {
    tel_roll_dps = (roll_in - 1492)/3.0;                                   // pour notre angle négatif (décalage à gauche)
  } 

  // On veut convertir la pulsation de la télécommande en degrés par seconde pour le tangage:
  if(pitch_in > 1508)    //on prend une zone morte entre 1492 et 1508 pour éviter des perturbations inutiles (si la valeur de la télécommande n'est pas parfaitement calibré)
  {
    tel_pitch_dps = (pitch_in - 1508)/3.0;                                 // cela correspond à notre angle, partie positive (drone avance): 
  }                                                                         // (2000-1508)/2,73 = 180 d/s (la valeur maximale)
  else if(pitch_in < 1492)
  {
    tel_pitch_dps = (pitch_in - 1492)/3.0;                                 // pour notre angle négatif (drone recule)
  }

  // On veut convertir la pulsation de la télécommande en degrés par seconde pour le lacet:
  if(yaw_in > 1508)    //on prend une zone morte entre 1492 et 1508 pour éviter des perturbations inutiles (si la valeur de la télécommande n'est pas parfaitement calibré)
  {
    tel_yaw_dps = (yaw_in - 1508)/3.0;                                     // cela correspond à notre angle, partie positive (drone pivote à droite): 
  }                                                                         // (2000-1508)/2,73 = 180 d/s (la valeur maximale)
  else if(yaw_in < 1492)    
  {
    tel_yaw_dps = (yaw_in - 1492)/3.0;                                     // pour notre angle négatif (drone pivote à gauche)
  }

  // execution du calcul du PID:
  PID();
  throttle = altitude_in;

  if (start==2){
    if(throttle>1800)throttle=1800;
    mot_AV = throttle - val_correction_pitch + val_correction_yaw;
    mot_D = throttle - val_correction_roll - val_correction_yaw;
    mot_AR = throttle + val_correction_pitch + val_correction_yaw;
    mot_G = throttle + val_correction_roll - val_correction_yaw;
    
    if (mot_AV < 1100) mot_AV = 1100;                                         //Garde les moteurs en route 
    if (mot_D < 1100) mot_D = 1100;                                         
    if (mot_AR < 1100) mot_AR = 1100;                                         
    if (mot_G < 1100) mot_G = 1100;                                         
      
    if(mot_AV > 2000) mot_AV = 2000;                                           //Limite la pulsation a 2000us.
    if(mot_D > 2000) mot_D = 2000;                                           
    if(mot_AR > 2000) mot_AR = 2000;                                           
    if(mot_G > 2000) mot_G = 2000;                                           
    // On peut enfin donner des valeurs au moteur:
  }
  
  else{
    mot_AV = 1000;                                                           //sinon on garde la pulsation a 1000us
    mot_D = 1000;                                                           
    mot_AR = 1000;                                                           
    mot_G = 1000;                                                           
  }

  myservoAV.writeMicroseconds(mot_AV);
  myservoD.writeMicroseconds(mot_D);
  myservoAR.writeMicroseconds(mot_AR);
  myservoG.writeMicroseconds(mot_G);
  
  //Serial.print("  Moteur 1:");Serial.print(mot_AV);Serial.print("  Moteur 2:");Serial.print(mot_D);Serial.print("  Moteur 3:");Serial.print(mot_AR);Serial.print("  Moteur 4:");Serial.println(mot_G);
  //Serial.print("  PID tangage:");Serial.print(val_correction_pitch);Serial.print("  PID roulis:");Serial.print(val_correction_roll);Serial.print("  PID lacet:");Serial.println(val_correction_yaw);
}


//##################### controle du PID ################################################################
void PID()
{
  //calcul des différentes erreurs:
  roll_erreur = gyro_roll_ds - tel_roll_dps;
  pitch_erreur = gyro_pitch_ds - tel_pitch_dps;
  yaw_erreur = gyro_yaw_ds - tel_yaw_dps;

  //calcul de la correction à apporter au roulis:
  val_P_roll = roll_erreur*P_gain_roll;
  val_I_roll = val_I_roll_pcdt + roll_erreur*I_gain_roll;
  val_D_roll = (roll_erreur - roll_erreur_pcdt)*D_gain_roll;

  if(val_I_roll > max_roll)
  {
    val_I_roll = max_roll;
  }
  else if(val_I_roll < max_roll * -1)
  {
    val_I_roll = max_roll * -1;
  }

  val_correction_roll = val_P_roll + val_I_roll + val_D_roll;
  if (val_correction_roll > max_roll)
  {
    val_correction_roll = max_roll;
  }
  else if(val_correction_roll < -1*max_roll)
  {
    val_correction_roll = -1*max_roll;
  }

  val_I_roll_pcdt = val_I_roll;
  roll_erreur_pcdt = roll_erreur;

  //calcul de la correction à apporter au tangage:
  val_P_pitch = pitch_erreur*P_gain_pitch;
  val_I_pitch = val_I_pitch_pcdt + pitch_erreur*I_gain_pitch;
  val_D_pitch = (pitch_erreur - pitch_erreur_pcdt)*D_gain_pitch;

  if(val_I_pitch > max_pitch)
  {
    val_I_pitch = max_pitch;
  }
  else if(val_I_pitch < max_pitch * -1)
  {
    val_I_pitch = max_pitch * -1;
  }

  val_correction_pitch = val_P_pitch + val_I_pitch + val_D_pitch;
  if (val_correction_pitch > max_pitch)
  {
    val_correction_pitch = max_pitch;
  }
  else if(val_correction_pitch < -1*max_pitch)
  {
    val_correction_pitch = -1*max_pitch;
  }
  
  val_I_pitch_pcdt = val_I_pitch;
  pitch_erreur_pcdt = pitch_erreur;
  
  //calcul de la correction à apporter au lacet:
  val_P_yaw = yaw_erreur*P_gain_yaw;
  val_I_yaw = val_I_yaw_pcdt + yaw_erreur*I_gain_yaw;
  val_D_yaw = (yaw_erreur - yaw_erreur_pcdt)*D_gain_yaw;

  if(val_I_yaw > max_yaw)
  {
    val_I_yaw = max_yaw;
  }
  else if(val_I_yaw < max_yaw * -1)
  {
    val_I_yaw = max_yaw * -1;
  }

  val_correction_yaw = val_P_yaw + val_I_yaw + val_D_yaw;
  if (val_correction_yaw > max_yaw)
  {
    val_correction_yaw = max_yaw;
  }
  else if(val_correction_yaw < -1*max_yaw)
  {
    val_correction_yaw = -1*max_yaw;
  }

  val_I_yaw_pcdt = val_I_yaw;
  yaw_erreur_pcdt = yaw_erreur;
}


//##################### lecture des pulsation de la telecommande chaque canal a sa fonction pour chaque pin d'intéruption########
// c'est compliquer comme code lisez bien les commentaires. 

ISR(PCINT0_vect){
  
  temps_actuel=micros();
  
  /////////// Pour le throttle (poussée)////////////
  if(PINB & B00000001){ //`1==High , 0==Low si le booléen mem_ch est LOW 
    if (mem_ch3==0){
     mem_ch3=1;//et que le signal est HIGH on est sur la partie montante de la pulsation , on dit alors a mem_ch qu'on est HIGH 
     temps_3= temps_actuel;// on prend ce temps de depart de high en considération 
    }
  }
  else if(mem_ch3 == 1){ // maintenant si le booléen est Haut
    mem_ch3=0;//on sait que on est sur la partie descendante de la pulsation. On dit donc ceci au booléen 
    altitude_in=(temps_actuel - temps_3); //On a donc parcouru toute la pulsation et donc on peut calculer sa durée. 
  }

  ///////// Pour le roulis /////////////
  if(PINB & B00000010 ){ //`1==High , 0==Low
    if (mem_ch1==0){
      mem_ch1=1; 
      temps_1=temps_actuel;
    }
  }
  else if(mem_ch1 == 1){
    mem_ch1=0;
    roll_in=(temps_actuel-temps_1);
  }
  ///////////Pour le tanguage /////////
  if(PINB & B00000100){ //`1==High , 0==Low
    if (mem_ch2 == 0){
      mem_ch2=1; 
      temps_2=temps_actuel;
    }
  }
  else if(mem_ch2 == 1){
    mem_ch2=0;
    pitch_in=(temps_actuel-temps_2); 
  }
  /////////////  Lacet ////////////////
  if(PINB & B00001000){ //`1==High , 0==Low
    if(mem_ch4 == 0){
      mem_ch4=1; 
      temps_4=temps_actuel;
    }
  }
  else if(mem_ch4 == 1){
    mem_ch4=0;
    yaw_in=(temps_actuel - temps_4);
  }
}
//############################################ ROUTINE POUR LE GYROSCOPE ####################################

void gyro_signal(){
  
  Wire.beginTransmission(105);                                 //Commence la communication avec le  gyroscope 
  Wire.write(168);                                             //On commmence la lecture au registre 28h, celui de la position du tanguage 
  Wire.endTransmission();                                      //On termine la communication 
  Wire.requestFrom(105, 6);                                    //On fait la requete des 6 octets du gyro (deux pour chaque axe) 
  while(Wire.available()< 6);                                 //On attend que les octets arrivent 

  
  lowByte = Wire.read();                                       
  highByte = Wire.read();
  gyro_pitch=((highByte<<8)|lowByte); 
   gyro_pitch*=-1 ;
  if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal; 

  lowByte = Wire.read();                                       
  highByte = Wire.read();
  gyro_roll=((highByte<<8)|lowByte); 
  //gyro_roll*=-1 ;
  if(cal_int == 2000)gyro_roll -= gyro_roll_cal; 
  
  lowByte = Wire.read();                                       
  highByte = Wire.read();
  gyro_yaw=((highByte<<8)|lowByte);  
  gyro_yaw*=-1 ;
  if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal; 
//  Attention il faut tester pour inverser les axes on fait gyro_yaw*=-1 

}


