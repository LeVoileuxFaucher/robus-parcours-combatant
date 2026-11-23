/*
Projet: Parcours du combattant
Equipe: P18
Auteurs: Tristan Faucher
Description: Robot ROBUS — suivi de ligne, detection de couleur et parcours d'obstacles
Date: 2025-10-02
*/

/*
Inclure les librairies de functions que vous voulez utiliser
*/
#include <LibRobus.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"


#define BLANC 400
#define NOIR 850
#define PLAGE (NOIR - BLANC)

#define VITESSE_BASE 0.3
#define VITESSE_MAX 0.33
#define VITESSE_MIN -0.33

#define Kp 0.3

#define DIST90G 1900
#define DIST90D 1900
#define DIST180G 3800
#define DIST180D 3800
#define FREQMIN 743
#define FREQMAX 800
#define PID_IMPACT 500

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

float MoteurspeedDroit = 0;  // Dernière vitesse du moteur droit (utile pour savoir si le moteur tourne encore)
float MoteurspeedGauche = 0; // Dernière vitesse du moteur gauche
int Micropin = A0;           // Pin analogique
int SuiveurGauchepin = A5; 
int SuiveurCentrepin = A6; 
int SuiveurDroitpin = A7; 
float vitesse = 0.3;        // Vitesse standard de déplacement
uint16_t Freq = 0;
 int derniereDirection = 0;
int couleur;
  
/*
Prototypes des fonctions personnalisées
 -> Sert à annoncer leur existence avant le code
*/
void Sui_Lah_Ling(void);
void AvanceXcm(uint16_t Distance_cm);
void Gauche90(void);
void Droit90(void);
void Gauche180(void);
void Droit180(void);
void arret(void);
void avance(void);
void recule(void);
void tourneDroit(void);
void tourneGauche(void);
void beep(int count);



void Sui_Lah_Ling()
{
  int16_t TabLectures[3] = {0,0,0};
  TabLectures[LEFT] = analogRead(SuiveurGauchepin);
  TabLectures[FRONT] = analogRead(SuiveurCentrepin);
  TabLectures[RIGHT] = analogRead(SuiveurDroitpin);

  // --- Normalisation entre 0 (blanc) et 1 (noir) ---
  float normG = constrain((float)(TabLectures[LEFT] - BLANC) / PLAGE, 0, 1);
  float normC = constrain((float)(TabLectures[FRONT] - BLANC) / PLAGE, 0, 1);
  float normD = constrain((float)(TabLectures[RIGHT] - BLANC) / PLAGE, 0, 1);

  // --- Calcul de la position de la ligne ---
  // pondération : -1 (gauche), 0 (centre), +1 (droite)
  float somme = normG + normC + normD;
  float position = 0; // 0 = centré

  if (somme > 0)
    position = (-1 * normG + 0 * normC + 1 * normD) / somme;

  // position varie entre -1 (ligne à gauche) et +1 (ligne à droite)

  // --- Calcul de la correction proportionnelle ---
  float correction = Kp * position;

  // --- Calcul des vitesses des moteurs ---
  float vitesseGauche = VITESSE_BASE - correction;
  float vitesseDroite = VITESSE_BASE + correction;

  // --- Gestion de la perte de ligne ---
  if (somme < 0.1) // presque tout blanc : ligne perdue
  {
    if (derniereDirection < 0)
    {
      vitesseGauche = -0.2;
      vitesseDroite = 0.2;
    }
    else if (derniereDirection > 0)
    {
      vitesseGauche = 0.2;
      vitesseDroite = -0.2;
    }
    else
    {
      vitesseGauche = 0.15;
      vitesseDroite = 0.15;
    }
  }
  else
  {
    // Sauvegarde de la dernière direction significative
    if (position < -0.2) derniereDirection = -1;
    else if (position > 0.2) derniereDirection = 1;
    else derniereDirection = 0;
  }

  // --- Limitation des vitesses ---
  vitesseGauche = constrain(vitesseGauche, VITESSE_MIN, VITESSE_MAX);
  vitesseDroite = constrain(vitesseDroite, VITESSE_MIN, VITESSE_MAX);

  // --- Appliquer les vitesses ---
  MOTOR_SetSpeed(LEFT, vitesseGauche);
  MOTOR_SetSpeed(RIGHT, vitesseDroite);
  // --- Debug ---
  Serial.print("G:");
  Serial.print(TabLectures[LEFT]);
  Serial.print(" C:");
  Serial.print(TabLectures[FRONT]);
  Serial.print(" D:");
  Serial.print(TabLectures[RIGHT]);
  Serial.print(" | Pos:");
  Serial.print(position, 2);
  Serial.print(" | VL:");
  Serial.print(vitesseGauche, 2);
  Serial.print(" VR:");
  Serial.print(vitesseDroite, 2);
  Serial.print(" | AP:");
  Serial.println(derniereDirection);
  
}

/*
Fait émettre un certain nombre de bips au buzzer du robot
 -> Utile comme feedback sonore (diagnostic ou orientation)
*/
void beep(int count)
{
  for (int i = 0; i < count; i++)
  {
    AX_BuzzerON(); // Active le buzzer
    delay(100);
    AX_BuzzerOFF(); // Désactive le buzzer
    delay(100);
  }
  delay(400); // Petite pause avant la suite
}



/*
Arrête complètement les moteurs
 -> Le robot s’immobilise
*/

void arret(void)
{
  MOTOR_SetSpeed(RIGHT, 0);
  MOTOR_SetSpeed(LEFT, 0);
  MoteurspeedDroit = 0;
  MoteurspeedGauche = 0;
}

/*
Fait avancer le robot tout droit
 -> Les deux moteurs tournent à la même vitesse positive
*/
void avance(void)
{
  MOTOR_SetSpeed(RIGHT, vitesse);
  MOTOR_SetSpeed(LEFT, vitesse);
  MoteurspeedDroit = vitesse;
  MoteurspeedGauche = vitesse;
}

/*
Fait reculer le robot
 -> Les deux moteurs tournent en sens inverse
*/
void recule(void)
{
  MOTOR_SetSpeed(RIGHT, -vitesse);
  MOTOR_SetSpeed(LEFT, -vitesse);
  MoteurspeedDroit = -vitesse;
  MoteurspeedGauche = -vitesse;
}

/*
Rotation sur place vers la droite
 -> Le moteur gauche avance, le droit recule
*/
void tourneDroit(void)
{
  MOTOR_SetSpeed(RIGHT, -vitesse);
  MOTOR_SetSpeed(LEFT, vitesse);
  MoteurspeedDroit = -vitesse;
  MoteurspeedGauche = vitesse;
}

/*
Rotation sur place vers la gauche
 -> Le moteur droit avance, le gauche recule
*/
void tourneGauche(void)
{
  MOTOR_SetSpeed(RIGHT, vitesse);
  MOTOR_SetSpeed(LEFT, -vitesse);
  MoteurspeedDroit = vitesse;
  MoteurspeedGauche = -vitesse;
}

/*
Tourne le robot de 180° vers la droite
 -> Utilise les codeurs pour mesurer l’angle parcouru
*/
void Droit180(void)
{
  tourneDroit();
  while (!((MoteurspeedDroit == 0) && (MoteurspeedGauche == 0)))
  {
    if (DIST180D <= ENCODER_Read(LEFT)) // Codeur gauche atteint la distance équivalente à 180°
    {
      MOTOR_SetSpeed(LEFT, 0);
      MoteurspeedGauche = 0;
    }
    if (-DIST180D >= ENCODER_Read(RIGHT)) // Codeur droit aussi
    {
      MOTOR_SetSpeed(RIGHT, 0);
      MoteurspeedDroit = 0;
    }
  }
  ENCODER_ReadReset(RIGHT); // On réinitialise pour la prochaine mesure
  ENCODER_ReadReset(LEFT);
}

/*
Tourne le robot de 90° vers la droite
*/
void Droit90(void)
{
  tourneDroit();
  while (!((MoteurspeedDroit == 0) && (MoteurspeedGauche == 0)))
  {
    if (DIST90D <= ENCODER_Read(LEFT)) // 90° atteint côté gauche
    {
      MOTOR_SetSpeed(LEFT, 0);
      MoteurspeedGauche = 0;
    }
    if (-DIST90D >= ENCODER_Read(RIGHT)) // 90° atteint côté droit
    {
      MOTOR_SetSpeed(RIGHT, 0);
      MoteurspeedDroit = 0;
    }
  }
  ENCODER_ReadReset(RIGHT);
  ENCODER_ReadReset(LEFT);
}

/*
Tourne le robot de 180° vers la gauche
*/
void Gauche180(void)
{
  tourneGauche();
  while (!((MoteurspeedDroit == 0) && (MoteurspeedGauche == 0)))
  {
    if (-DIST180G >= ENCODER_Read(LEFT)) // Codeur gauche atteint la valeur de rotation
    {
      MOTOR_SetSpeed(LEFT, 0);
      MoteurspeedGauche = 0;
    }
    if (DIST180G <= ENCODER_Read(RIGHT)) // Codeur droit aussi
    {
      MOTOR_SetSpeed(RIGHT, 0);
      MoteurspeedDroit = 0;
    }
  }
  ENCODER_ReadReset(RIGHT);
  ENCODER_ReadReset(LEFT);
}

/*
Tourne le robot de 90° vers la gauche
*/
void Gauche90(void)
{
  tourneGauche();
  while (!((MoteurspeedDroit == 0) && (MoteurspeedGauche == 0)))
  {
    if (-DIST90G >= ENCODER_Read(LEFT)) // Codeur gauche atteint 90°
    {
      MOTOR_SetSpeed(LEFT, 0);
      MoteurspeedGauche = 0;
    }
    if (DIST90G <= ENCODER_Read(RIGHT)) // Codeur droit atteint 90°
    {
      MOTOR_SetSpeed(RIGHT, 0);
      MoteurspeedDroit = 0;
    }
  }
  ENCODER_ReadReset(RIGHT);
  ENCODER_ReadReset(LEFT);
}



/*
Avance le robot de 50 cm ou jusqu’à un obstacle
 -> Vérifie en continu les capteurs
 -> S’arrête si distance atteinte ou mur détecté
*/
void AvanceXcm(uint16_t Distance_cm)
{
  uint16_t Diff;
  float ratio = 3200.0 / 23.938936;
  uint16_t Distance_Encodeur = (uint16_t)(Distance_cm * ratio);

  avance();
   
  while (!((MoteurspeedDroit == 0) && (MoteurspeedGauche == 0)))
  {
    if (Distance_Encodeur <= ENCODER_Read(RIGHT)) // ≈ X cm roue droite
    {
      MOTOR_SetSpeed(RIGHT, 0);
      MoteurspeedDroit = 0;
    }
    if ((Distance_Encodeur <= ENCODER_Read(LEFT))) // ≈ X cm roue gauche
    {
      MOTOR_SetSpeed(LEFT, 0);
      MoteurspeedGauche = 0;
    }
    if (ENCODER_Read(RIGHT) > ENCODER_Read(LEFT))
    {
      Diff = ENCODER_Read(RIGHT) - ENCODER_Read(LEFT);
      MOTOR_SetSpeed(RIGHT, vitesse + -(Diff / PID_IMPACT));
      MOTOR_SetSpeed(LEFT, vitesse + (Diff / PID_IMPACT));
    }
    else if (ENCODER_Read(RIGHT) < ENCODER_Read(LEFT))
    {
      Diff = ENCODER_Read(LEFT) - ENCODER_Read(RIGHT);
      MOTOR_SetSpeed(RIGHT, vitesse + (Diff / PID_IMPACT));
      MOTOR_SetSpeed(LEFT, vitesse + -(Diff / PID_IMPACT));
    } 
  }
    arret();
  // Reset des codeurs pour prochaine mesure
  ENCODER_ReadReset(RIGHT);
  ENCODER_ReadReset(LEFT);
}

bool inRange(float value, float target, float tol) {
  return (value >= target - tol) && (value <= target + tol);
}

uint8_t getColorCode(float r, float g, float b) {
  if (inRange(r, 130, 20) && inRange(g, 70, 10) && inRange(b, 45, 10)) return 4; // Jaune
  if (inRange(r, 90, 10)  && inRange(g, 80, 10) && inRange(b, 80, 10)) return 5; // Rouge
  if (inRange(r, 80, 10)  && inRange(g, 80, 10) && inRange(b, 90, 10)) return 6; // Bleu
  if (inRange(r, 90, 10)  && inRange(g, 70, 10) && inRange(b, 90, 10)) return 1; // Magenta

  return 0; // Inconnue
}

/*
Fonctions d'initialisation (setup)
 -> S’exécute une seule fois au démarrage
 -> Configure les capteurs, moteurs et afficheurs
*/

void setup()
{
  
  BoardInit();        // Initialise la carte
  Serial.begin(9600); // Active la communication série
  Wire.begin();
  
 
 if (tcs.begin()) {
    Serial.println("Capteur TCS34725 détecté !");
  } else {
    Serial.println("Erreur : Capteur non détecté.");
    while (1); // Bloque le programme si le capteur n'est pas trouvé
  }

  // Active la LED intégrée
  tcs.setInterrupt(false); // LED allumée


  
  pinMode(Micropin, INPUT_PULLUP);
  pinMode(SuiveurGauchepin, INPUT);
  pinMode(SuiveurCentrepin, INPUT);
  pinMode(SuiveurDroitpin, INPUT);
  ENCODER_ReadReset(RIGHT);
  ENCODER_ReadReset(LEFT);
  arret();
  beep(1); // Bipe au démarrage
}

/*
Boucle principale (loop)
 -> Se répète indéfiniment
 -> Décide la direction et avance
*/
void loop()
{
  /*// Attente jusqu’à ce que le robot détecte un bumper pressé
  while ((!ROBUS_IsBumper(3)) && !((FREQMAX >= Freq) && (FREQMIN <= Freq)))
  {
    //Freq = analogRead(Micropin);
    //delay(10);
    Serial.println("Capteur prêt !");
  }
  
  // Boucle infinie de navigation
  while (1)
  {
    //Sui_Lah_Ling();
    colorSensor.readRGB(&r, &g, &b);
  
  Serial.print("R: "); Serial.print(r);
  Serial.print(" | G: "); Serial.print(g);
  Serial.print(" | B: "); Serial.print(b);
  Serial.print("  --> Couleur: ");
    
     if (r > g && r > b) {
    Serial.println("Rouge");
  } else if (g > r && g > b) {
    Serial.println("Vert");
  } else if (b > r && b > g) {
    Serial.println("Bleu");
  } else if (r > 150 && g > 150 && b > 150) {
    Serial.println("Blanc");
  } else if (r < 50 && g < 50 && b < 50) {
    Serial.println("Noir");
  } else {
    Serial.println("Inconnue / Mixte");
  }
    
    delay(100);
  }
    */
  uint16_t rRaw, gRaw, bRaw, cRaw;
  float r, g, b;

  tcs.getRawData(&rRaw, &gRaw, &bRaw, &cRaw);

  // Normalisation 0-255
  r = (float)rRaw / cRaw * 255.0;
  g = (float)gRaw / cRaw * 255.0;
  b = (float)bRaw / cRaw * 255.0;

  // Affichage des valeurs RGB
  Serial.print("R: "); Serial.print((int)r);
  Serial.print(" | G: "); Serial.print((int)g);
  Serial.print(" | B: "); Serial.print((int)b);

  // Détection couleur
  uint8_t colorCode = getColorCode(r, g, b);

  Serial.print(" | Couleur détectée: ");
  switch (colorCode) {
    case 1: Serial.println("Magenta"); break;
    case 2: Serial.println("Vert"); break;
    case 3: Serial.println("Bleu"); break;
    case 4: Serial.println("Jaune"); break;
    default: Serial.println("Inconnue"); break;
  }

  Serial.println("----------------------------");
  delay(500);

}
