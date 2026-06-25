/* * Fichier : Reception_LoRa_QAI.ino
 * Auteur : Maël Lefebvre
 * Date : Juin 2026
 * Description : Programme de test pour la réception des trames LoRa 
 * envoyées par la station QAI. Affiche les données environnementales 
 * reçues directement sur le moniteur série.
 */

#include <SPI.h>
#include <LoRa.h>

// --- CONFIGURATION DES BROCHES LORA ---
const int csPin = 10;
const int resetPin = 9;
const int irqPin = 2; // Broche G0 ou DIO0

void setup() {
  // Initialisation de la communication avec le PC
  Serial.begin(9600); // Vitesse de communication avec le moniteur série
  while (!Serial);
  
  Serial.println("--- RÉCEPTEUR LORA EN ÉCOUTE ---");
  
  // Assignation des broches au module LoRa
  LoRa.setPins(csPin, resetPin, irqPin);

  // Démarrage du module (868 MHz)
  if (!LoRa.begin(868E6)) { // Doit être sur la même fréquence que l'émetteur
    Serial.println("Échec du démarrage du module LoRa.");
    while (1); // Verrouille le programme en cas d'erreur matérielle
  }
  
  // Configuration radio (Paramètres obligatoirement identiques à la station)
  LoRa.setSyncWord(0xF3);        // Canal privé de communication
  LoRa.setSpreadingFactor(9);    // Facteur d'étalement (adapté pour traverser les murs)
  LoRa.setTxPower(20);           // Puissance radio
  
  Serial.println("[OK] En attente des donnees de la station...");
}

void loop() {
  // Écoute continue du réseau LoRa
  int packetSize = LoRa.parsePacket();
  
  // Si un paquet est détecté
  if (packetSize) {
    String message = "";
    
    // Reconstruction de la trame CSV caractère par caractère
    while (LoRa.available()) {
      message += (char)LoRa.read();
    }
    
    // Affichage brut des mesures à l'écran
    Serial.println(message); 
  }
}