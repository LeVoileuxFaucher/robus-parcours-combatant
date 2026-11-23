# RobUS — Parcours du combattant / Obstacle Course Robot

> Robot mobile autonome : suivi de ligne, détection de couleurs et déplacements odométriques sur un parcours d'obstacles.
>
> *Autonomous mobile robot: line following, colour detection and odometry-based moves on an obstacle course.*

![PlatformIO](https://img.shields.io/badge/PlatformIO-Arduino_Mega_2560-orange)
![Capteur](https://img.shields.io/badge/capteur-TCS34725-informational)
![Statut](https://img.shields.io/badge/statut-prototype-yellow)

---

## Aperçu / Overview

**FR** — Projet de robotique (équipe P18) sur plateforme **RobUS**. Le robot combine trois briques : un **suivi de ligne** à trois capteurs analogiques avec correction proportionnelle, un **capteur de couleur RGB TCS34725** pour identifier les zones du parcours, et des **déplacements calibrés en encodeurs** pour avancer d'une distance précise.

**EN** — Robotics project (team P18) on the **RobUS** platform. The robot combines three building blocks: **three-sensor line following** with proportional correction, a **TCS34725 RGB colour sensor** to identify course zones, and **encoder-calibrated moves** for precise distances.

> ⚠️ **État du projet** : les briques de navigation et de détection sont implémentées ; la boucle principale est actuellement en **mode calibration du capteur de couleur** (affichage RGB + classification sur la liaison série).

## Fonctionnement / How it works

### Suivi de ligne / Line following

```cpp
position = (-1·G + 0·C + 1·D) / (G + C + D)   // G, C, D normalisés 0..1
correction = Kp · position                     // Kp = 0.3
vitesseGauche = VITESSE_BASE - correction
vitesseDroite = VITESSE_BASE + correction
```

- Les lectures analogiques (A5, A6, A7) sont normalisées entre **blanc (400)** et **noir (850)**.
- La position pondérée donne un écart continu entre -1 (ligne à gauche) et +1 (ligne à droite).
- Si la ligne est perdue (`somme < 0.1`), le robot pivote du côté de la **dernière direction** connue.

### Détection de couleur / Colour detection

Le TCS34725 renvoie des valeurs RGB brutes, normalisées sur 0–255 :

```cpp
r = rRaw / cRaw * 255;  // idem pour g et b
```

Classification par plages de tolérance (`getColorCode`) : jaune, rouge, bleu, magenta.

### Déplacements / Moves

`AvanceXcm(distance)` convertit les centimètres en impulsions d'encodeur (`3200 / 23.94` impulsions/cm) et corrige l'écart entre les roues pendant l'avance.

## Matériel / Hardware

| Élément | Rôle |
|---|---|
| Carte ArduinoX + Arduino Mega 2560 | Contrôleur |
| 2 moteurs CC avec encodeurs | Propulsion + odométrie |
| 3 capteurs de ligne (A5, A6, A7) | Suivi de trajectoire |
| Capteur TCS34725 (I²C) | Identification des couleurs |
| Buzzer | Retours sonores |
| Microphone (A0) | Départ par tonalité (743–800 Hz) |

## Architecture du code / Code layout

| Fichier | Contenu |
|---|---|
| `src/main.cpp` | Suivi de ligne, détection couleur, odométrie, virages |
| `platformio.ini` | Cible `megaatmega2560`, framework Arduino |

Fonctions clés :

| Fonction | Rôle |
|---|---|
| `Sui_Lah_Ling()` | Suivi de ligne proportionnel avec récupération |
| `getColorCode(r, g, b)` | Classification couleur par plages |
| `AvanceXcm(cm)` | Avance odométrique corrigée |
| `Droit90()`, `Gauche90()`, `Droit180()`, `Gauche180()` | Rotations calibrées |
| `beep(n)` | Retour sonore |

## Compilation / Build

```bash
pip install platformio
git clone https://github.com/UdeS-GRO/LibRobUS.git lib/LibRobUS

pio run
pio run --target upload
```

> La bibliothèque `LibRobUS` et le pilote `Adafruit_TCS34725` sont fournis par le cours et ne sont pas redistribués ici. Le dossier `lib/` est ignoré par Git.

## Résultats et limites / Results & limitations

- Suivi de ligne fluide grâce à la position pondérée et à la récupération de ligne.
- Classification couleur fiable après calibration sous l'éclairage du local.
- Les seuils RGB sont **empiriques** : ils doivent être recalibrés si l'éclairage change.
- La séquence complète du parcours reste à finaliser (la boucle principale est en mode test).

## Ce que j'ai appris / What I learned

- Normalisation de capteurs analogiques et asservissement proportionnel.
- Calibration d'un capteur couleur et gestion des conditions d'éclairage.
- Conversion d'une distance physique en impulsions d'encodeur.
- Structuration d'un programme embarqué en modules réutilisables.

## Crédits / Credits

- **Auteur** : Tristan Faucher — équipe P18, Université de Sherbrooke.
- **Bibliothèque RobUS** : [UdeS-GRO/LibRobUS](https://github.com/UdeS-GRO/LibRobUS).
- **Capteur** : [Adafruit TCS34725](https://github.com/adafruit/Adafruit_TCS34725).
- Projet de cours publié à des fins de portfolio.

## Licence / License

© Tristan Faucher. Tous droits réservés. Code publié à titre de démonstration.
