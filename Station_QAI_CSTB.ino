/* * Fichier : Station_QAI_CSTB.ino
 * Auteur : Maël Lefebvre
 * Date : Juin 2026
 * Description : Programme principal du prototype nomade de la station QAI.
 * Gère l'acquisition des données de 7 capteurs environnementaux, 
 * l'affichage OLED, l'enregistrement de secours sur carte microSD, 
 * la transmission des données (LoRa/Wi-Fi) et l'optimisation énergétique.
 */

#include <Wire.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <SensirionI2cScd30.h>
#include <SensirionI2CSgp40.h>
#include <SensirionI2cSps30.h>
#include <VOCGasIndexAlgorithm.h>
#include <DFRobot_OzoneSensor.h>
#include <RTCZero.h>
#include <WiFiNINA.h>
#include <Fonts/Org_01.h>
#include <ArduinoLowPower.h>
#include <DFRobot_MultiGasSensor.h>
#include <math.h>
#include <LoRa.h>
#include "DFRobot_SFA40.h"

// ======================================================
// CONFIGURATION DEBUG ET TESTS
// ======================================================
#define DEBUG_ACTIF 0  // Passer à 1 pour activer les retours sur le moniteur série
#define MODE_TEST 0   // Paaser à 1 pour desactiver l'affichege de la batterie

// Macros de débogage pour désactiver facilement les Serial.print en production
#if DEBUG_ACTIF
#define DEBUG_BEGIN(...) Serial.begin(__VA_ARGS__)
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_BEGIN(...)
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

// ======================================================
// CONFIGURATION GENERALE & BROCHES
// ======================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// Broches d'interface utilisateur
const int pinBoutonWiFi = 3;
const int pinBoutonSuivant = 2;

// --- CONFIGURATION BATTERIE ---
const int batteriePin = A2;
const float tensionMax = 4.2;
const float tensionMin = 3.2;
const float tensionShutdown = 3.2;
const float tensionAlerte = 3.4;
bool alerteBatterie = false;
int batteriePct = 0;

// --- CONFIGURATION LORA ---
const int csLoRa = 9;
const int resetLoRa = 7;
const int irqLoRa = 8;

// --- CONFIGURATION CARTE SD (SPI Logiciel) ---
const int chipSelect = 10;
const uint8_t SOFT_MISO_PIN = 4;
const uint8_t SOFT_MOSI_PIN = 5;
const uint8_t SOFT_SCK_PIN = 6;

SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;
SdFat sd;

// --- CONFIGURATION WIFI ---
char ssid[] = "QAI";
char pass[] = "38463389";
int status = WL_IDLE_STATUS;

// --- TIMERS ET DELAIS (en millisecondes) ---
const long TEMPS_VEILLE = 30000;       // Veille de l'écran après 30s
const long DELAI_WIFI = 180000;        // Coupure du Wi-Fi après 3min
const long TEMPS_CYCLE = 600000;       // Cycle de mesure complet (10 min)
const long INTERVALLE_SGP40 = 1000;    // Lecture COV toutes les 1s
const long DELAI_CHAUFFE = 30000;      // Temps de préchauffage du SPS30
const int GMT = 0;

// ======================================================
// STRUCTURE DES DONNEES
// ======================================================
// Structure permettant de regrouper et centraliser toutes les variables environnementales
struct AirData {
  float temp = 0;
  float hum = 0;
  float press = 0;
  float co2 = 0;
  int vocIndex = 0;
  float no2 = 0;
  float o3 = 0;
  float hcho = 0;
  float pm1 = 0;
  float pm25 = 0;
  float pm10 = 0;
};
AirData mesures;

// ======================================================
// VARIABLES GLOBALES (Drapeaux d'état et Chronos)
// ======================================================
bool ecranActif = true;
bool modeWifi = false;
bool sdOk = false;
bool loraOk = false;
bool sps30_en_chauffe = false;
bool sfa40Ok = false;  // Drapeau de sécurité pour le capteur HCHO

int pageMenu = 0;

// Variables pour l'anti-rebond des boutons poussoirs
bool etatBoutonPrecedent = HIGH;
bool etatBoutonWifiPrecedent = HIGH;
unsigned long tempsAntiRebondBouton = 0;
unsigned long tempsAntiRebondWifi = 0;

// Chronomètres non-bloquants (remplacement de la fonction delay)
unsigned long chronoSGP40 = 0;
unsigned long chronoMesure = 0;
unsigned long dernierAppui = 0;
unsigned long chronoWifi = 0;
unsigned long chronoBatterie = 0;

// ======================================================
// INSTANCIATION DES OBJETS MATERIELS
// ======================================================
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
Adafruit_BME680 bme;
SensirionI2cScd30 scd30;
SensirionI2CSgp40 sgp40;
SensirionI2cSps30 sps30;
VOCGasIndexAlgorithm voc_algorithm;
DFRobot_OzoneSensor ozone;
DFRobot_GAS_I2C no2_i2c(&Wire, 0x74);
DFRobot_SFA40 sfa40(&Wire);
RTCZero rtc;
WiFiServer server(80);

// ======================================================
// PROTOTYPES DES FONCTIONS
// ======================================================
void allumerAlimWiFi();
void couperAlimWiFi();
void miseEnSecurite();

// --- FONCTIONS SYSTÈME ET UTILITAIRES ---

// Callback pour horodater les fichiers créés sur la carte SD
void dateTime(uint16_t* date, uint16_t* time, uint8_t* ms10) {
  *date = FAT_DATE(rtc.getYear() + 2000, rtc.getMonth(), rtc.getDay());
  *time = FAT_TIME(rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
  if (ms10) *ms10 = 0;
}

// Fonction de diagnostic pour scanner les adresses des modules sur le bus I2C
void scannerI2C() {
  DEBUG_PRINTLN(F("Scan I2C..."));
  byte count = 0;
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      DEBUG_PRINT(F("I2C trouve a 0x"));
      if (address < 16) DEBUG_PRINT("0");
      DEBUG_PRINTLN(address, HEX);
      count++;
    }
  }
}

// Génère le nom du fichier du jour (Format: AAMMJJ.CSV)
String getDailyFilename() {
  char filename[13];
  sprintf(filename, "%02d%02d%02d.CSV", rtc.getYear(), rtc.getMonth(), rtc.getDay());
  return String(filename);
}

// Ajoute l'en-tête CSV (noms des colonnes) si le fichier vient d'être créé
void ensureDailyFileHeader(const String& filename) {
  FsFile dataFile = sd.open(filename.c_str(), FILE_WRITE);
  if (dataFile) {
    if (dataFile.size() == 0) {
      dataFile.println("Date;Heure;Temp(C);Hum(%);Pres(hPa);CO2(ppm);VOC(idx);NO2(ppm);O3(ppb);HCHO(ppb);PM1.0(ug/m3);PM2.5(ug/m3);PM10(ug/m3)");
    }
    dataFile.close();
  } else {
    DEBUG_PRINT(F("Impossible de creer le fichier : "));
    DEBUG_PRINTLN(filename);
  }
}

// --- INITIALISATION DES MODULES ---

// Récupère l'heure exacte via un serveur NTP lors de l'allumage
void initialiserHeureParWiFi() {
  allumerAlimWiFi();
  DEBUG_PRINT(F("Connexion au reseau: "));
  DEBUG_PRINTLN(ssid);

  int wifiTries = 0;
  while (status != WL_CONNECTED && wifiTries < 5) {
    status = WiFi.begin(ssid, pass);
    wifiTries++;
    delay(5000);
  }

  if (status == WL_CONNECTED) {
    DEBUG_PRINTLN(F("WiFi OK !"));
    rtc.begin();
    unsigned long epoch = 0;
    int numberOfTries = 0;

    do {
      epoch = WiFi.getTime();
      numberOfTries++;
      delay(1000);
    } while ((epoch == 0) && (numberOfTries < 50));

    if (epoch == 0) {
      // Sécurité si connexion OK mais NTP inaccessible
      rtc.setHours(12); rtc.setMinutes(0); rtc.setSeconds(0);
      rtc.setDay(17); rtc.setMonth(6); rtc.setYear(26);
    } else {
      rtc.setEpoch(epoch + (GMT * 3600));
    }
    couperAlimWiFi();
  } else {
    // Mode hors-ligne : Utilisation de l'heure en dur
    DEBUG_PRINTLN(F("Echec WiFi. Utilisation de la date de secours."));
    rtc.begin();
    rtc.setHours(10);   
    rtc.setMinutes(11); 
    rtc.setSeconds(0);  
    rtc.setDay(17);     
    rtc.setMonth(6);    
    rtc.setYear(26);    
    couperAlimWiFi();
  }
}

// Séquence d'allumage et de configuration de chaque capteur I2C
void initialiserCapteurs() {
  delay(500); 

  bme.begin();
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);

  scd30.begin(Wire, SCD30_I2C_ADDR_61);
  scd30.stopPeriodicMeasurement();
  delay(500);
  scd30.setMeasurementInterval(60);
  delay(100);
  scd30.startPeriodicMeasurement(0);

  sgp40.begin(Wire);

  sps30.begin(Wire, 0x69);
  delay(100);
  sps30.deviceReset();
  delay(1500);

  ozone.begin(0x73);
  ozone.setModes(MEASURE_MODE_PASSIVE);

  no2_i2c.begin();
  no2_i2c.changeAcquireMode(no2_i2c.PASSIVITY);
}

void initialiserCarteSD() {
  if (sd.begin(SdSpiConfig(chipSelect, DEDICATED_SPI, SD_SCK_MHZ(4), &softSpi))) {
    sdOk = true;
    String filename = getDailyFilename();
    ensureDailyFileHeader(filename);
    DEBUG_PRINTLN(F("Carte SD OK !"));
  } else {
    sdOk = false;
    DEBUG_PRINTLN(F("Erreur carte SD"));
  }
}

String obtenirTimestamp() {
  char buf[25];
  sprintf(buf, "%02d/%02d/%02d %02d:%02d:%02d",
          rtc.getDay(), rtc.getMonth(), rtc.getYear(),
          rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
  return String(buf);
}

// --- GESTION DU SERVEUR WEB LOCAL ---

// Active le point d'accès Wi-Fi pour la récupération des données
void lancerModeWifi() {
  if (WiFi.beginAP("Station_QAI_test") == WL_AP_LISTENING) {
    server.begin();
    modeWifi = true;
    display.clearDisplay();
    display.setFont(&Org_01);
    display.setCursor(0, 12);
    display.setTextSize(2);
    display.println("WIFI ACTIF");
    display.setCursor(0, 30);
    display.setTextSize(3);
    display.println("IP: ");
    display.println("192.168.4.1");
    display.setFont();
    display.display();
  }
}

// Intercepte les requêtes du navigateur et propose le téléchargement du fichier CSV
void gererServeurWeb() {
  WiFiClient client = server.available();
  if (!client) return;
  String request = client.readStringUntil('\r');
  client.flush();

  String currentFile = getDailyFilename();

  if (request.indexOf("/download") != -1) {
    FsFile dataFile = sd.open(currentFile.c_str(), O_READ);
    if (dataFile) {
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/csv");
      client.print("Content-Disposition: attachment; filename=\"");
      client.print(currentFile);
      client.println("\"");
      client.println("Connection: close");
      client.println();
      while (dataFile.available()) {
        client.write(dataFile.read());
      }
      dataFile.close();
    }
  } else {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html; charset=utf-8");
    client.println("Connection: close");
    client.println();
    client.println("<!DOCTYPE html><html><body><h1>Station QAI</h1></body></html>");
  }
  client.stop();
}

// --- GESTION DE L'ALIMENTATION ---

void lireBatterie() {
  float somme = 0;
  int nbLectures = 10;
  // Moyennage matériel pour lisser les variations de tension
  for (int i = 0; i < nbLectures; i++) {
    somme += analogRead(batteriePin);
    delay(5);
  }
  float valeurBruteMoyenne = somme / nbLectures;
  float tensionBatterie = ((valeurBruteMoyenne / 1023.0) * 3.3) * 2.13;
  batteriePct = ((tensionBatterie - tensionMin) / (tensionMax - tensionMin)) * 100;

  if (batteriePct > 100) batteriePct = 100;
  if (batteriePct < 0) batteriePct = 0;

  if (tensionBatterie <= tensionAlerte) {
    alerteBatterie = true;
  } else {
    alerteBatterie = false;
  }

  // Protection matérielle contre la décharge profonde
  if (MODE_TEST == 0 && tensionBatterie <= tensionShutdown) {
    miseEnSecurite();
  }
}

// --- AFFICHAGE  (OLED) ---

// Gestion de la pagination des écrans avec un switch/case
void afficherMenu() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.setTextSize(1);
  display.setTextColor(WHITE);

  switch (pageMenu) {
    case 0:
      display.println("TEMP & HUM        1/9");
      display.drawLine(0, 10, 128, 10, WHITE);
      display.setCursor(0, 20);
      display.setTextSize(2);
      display.print("T= ");
      display.print(mesures.temp, 1);
      display.print("C");
      display.setCursor(0, 45);
      display.print("HR=");
      display.print(mesures.hum, 1);
      display.print("%");
      break;
    case 1:
      // ... (Codes d'affichage des autres pages identiques) ...
      display.println("CO2 & VOC         2/9");
      display.setCursor(0, 20);
      display.setTextSize(2);
      display.print("C=");
      display.print((int)mesures.co2);
      display.print("ppm");
      display.setCursor(0, 45);
      display.print("COV=");
      display.print(mesures.vocIndex);
      break;
    case 2:
      display.println("NO2 & OZONE       3/9");
      display.setCursor(0, 20);
      display.setTextSize(2);
      display.print("N=");
      display.print(mesures.no2, 1);
      display.print("ppm");
      display.setCursor(0, 45);
      display.print("O3=");
      display.print((int)mesures.o3);
      break;
    case 3:
      display.println("PRESSION          4/9");
      display.setCursor(0, 30);
      display.setTextSize(2);
      display.print("P=");
      display.print((int)mesures.press);
      display.print("hPa");
      break;
    case 4:
      display.println("PARTICULES (PM)   5/9");
      display.setTextSize(2);
      display.setCursor(0, 18);
      display.print("PM1 =");
      display.print((int)mesures.pm1);
      display.setTextSize(1);
      display.print("ug/m3");
      display.setTextSize(2);
      display.setCursor(0, 34);
      display.print("PM2=");
      display.print((int)mesures.pm25);
      display.setTextSize(1);
      display.print("ug/m3");
      display.setTextSize(2);
      display.setCursor(0, 50);
      display.print("PM10=");
      display.print((int)mesures.pm10);
      display.setTextSize(1);
      display.print("ug/m3");
      break;
    case 5:
      display.println("HORLOGE           6/9");
      display.setTextSize(3);
      display.setCursor(10, 30);
      if (rtc.getHours() < 10) display.print("0");
      display.print(rtc.getHours());
      display.print(":");
      if (rtc.getMinutes() < 10) display.print("0");
      display.print(rtc.getMinutes());
      break;
    case 6:
      display.println("SYSTEME           7/9");
      display.setCursor(0, 15);
      display.print("Fichier:");
      display.setCursor(0, 25);
      display.print(getDailyFilename());

      display.setCursor(0, 40);
      display.print(sdOk ? "SD:OK " : "SD:ERR ");
      display.print(loraOk ? "LORA:OK" : "LORA:ERR");

      display.setCursor(0, 52);
      if (alerteBatterie) {
        display.setTextColor(BLACK, WHITE);
        display.print("ALERTE: BATT FAIBLE");
        display.setTextColor(WHITE);
      } else {
        display.print("Batt: ");
        display.print(batteriePct);
        display.print("%");
      }
      break;
    case 7:
      display.println("WIFI              8/9");
      display.setCursor(0, 30);
      display.println("Appuyez D3 pour");
      display.println("WiFi On/Off");
      break;
    case 8:
      display.println("FORMALDEHYDE      9/9");
      display.setCursor(0, 30);
      display.setTextSize(2);
      if (!sfa40Ok) {
        display.print("ERR CAPTEUR");
      } else {
        display.print("HCHO=");
        display.setCursor(0, 50);
        display.print(mesures.hcho, 1);
        display.setTextSize(1);
        display.print(" ppb");
      }
      break;
  }
  display.display();
}

// Utilitaires de formatage CSV pour éviter les erreurs de parsing
void printFloatOrNaN(FsFile& file, float value, int decimals = 1) {
  if (isnan(value)) file.print("NaN");
  else file.print(value, decimals);
}

void printIntOrNaN(FsFile& file, int value) {
  if (value < 0) file.print("NaN");
  else file.print(value);
}

// --- CYCLE DE MESURE PRINCIPAL ---

void effectuerMesures() {
  String timestampStr = obtenirTimestamp();

  // Lecture BME680 (Climat)
  if (bme.performReading()) {
    mesures.temp = bme.temperature;
    mesures.hum = bme.humidity;
    mesures.press = bme.pressure / 100.0;
  }

  // Lecture SCD30 (CO2)
  uint16_t drScd = 0;
  if (!scd30.getDataReady(drScd) && drScd) {
    float c, t, h;
    if (scd30.readMeasurementData(c, t, h) == 0) mesures.co2 = c;
  }

  // Lecture SPS30 (Particules Fines) - Extinction après lecture
  uint16_t drSps = 0;
  if (!sps30.readDataReadyFlag(drSps) && drSps) {
    uint16_t m1, m25, m4, m10, n05, n1, n25, n4, n10, typ;
    if (sps30.readMeasurementValuesUint16(m1, m25, m4, m10, n05, n1, n25, n4, n10, typ) == 0) {
      mesures.pm1 = (float)m1;
      mesures.pm25 = (float)m25;
      mesures.pm10 = (float)m10;
    }
  }
  sps30.stopMeasurement();
  sps30_en_chauffe = false;

  // Lecture Ozone et NO2
  int16_t o3_ppb = ozone.readOzoneData(20);
  if (o3_ppb >= 0) mesures.o3 = (float)o3_ppb;

  float no2Value = no2_i2c.readGasConcentrationPPM();
  if (!isnan(no2Value) && no2Value >= 0) mesures.no2 = no2Value;

  // Enregistrement des données sur la carte micro-SD (Si présente)
  if (sdOk) {
    String currentFile = getDailyFilename();
    ensureDailyFileHeader(currentFile);
    FsFile dataFile = sd.open(currentFile.c_str(), FILE_WRITE);
    if (dataFile) {
      String ts = timestampStr;
      String dateISO = ts.length() >= 14 ? "20" + ts.substring(6, 8) + "-" + ts.substring(3, 5) + "-" + ts.substring(0, 2) : "0000-00-00";
      String heureCourte = ts.length() >= 14 ? ts.substring(9, 14) : "00:00";

      dataFile.print(dateISO); dataFile.print(";");
      dataFile.print(heureCourte); dataFile.print(";");
      printFloatOrNaN(dataFile, mesures.temp, 1); dataFile.print(";");
      printFloatOrNaN(dataFile, mesures.hum, 1); dataFile.print(";");
      printFloatOrNaN(dataFile, mesures.press, 1); dataFile.print(";");
      printFloatOrNaN(dataFile, mesures.co2, 1); dataFile.print(";");
      printIntOrNaN(dataFile, mesures.vocIndex); dataFile.print(";");
      printFloatOrNaN(dataFile, mesures.no2, 3); dataFile.print(";");
      printFloatOrNaN(dataFile, mesures.o3, 1); dataFile.print(";");
      printFloatOrNaN(dataFile, mesures.hcho, 2); dataFile.print(";");
      printFloatOrNaN(dataFile, mesures.pm1, 1); dataFile.print(";");
      printFloatOrNaN(dataFile, mesures.pm25, 1); dataFile.print(";");
      printFloatOrNaN(dataFile, mesures.pm10, 1); dataFile.println();
      dataFile.close();
    }
  }

  // Construction et Envoi de la trame via LoRa
  String payload = "";
  payload += (isnan(mesures.temp) ? "NaN" : String(mesures.temp, 1)) + ";";
  payload += (isnan(mesures.hum) ? "NaN" : String(mesures.hum, 1)) + ";";
  payload += (isnan(mesures.press) ? "NaN" : String(mesures.press, 1)) + ";";
  payload += (isnan(mesures.co2) ? "NaN" : String(mesures.co2, 0)) + ";";
  payload += String(mesures.vocIndex) + ";";
  payload += (isnan(mesures.no2) ? "NaN" : String(mesures.no2, 3)) + ";";
  payload += (isnan(mesures.o3) ? "NaN" : String(mesures.o3, 1)) + ";";

  // Sécurité : Remplacement par NaN si le capteur SFA40 est en défaut
  if (!sfa40Ok) {
    payload += "NaN;";
  } else {
    payload += (isnan(mesures.hcho) ? "NaN" : String(mesures.hcho, 2)) + ";";
  }

  payload += (isnan(mesures.pm1) ? "NaN" : String(mesures.pm1, 1)) + ";";
  payload += (isnan(mesures.pm25) ? "NaN" : String(mesures.pm25, 1)) + ";";
  payload += (isnan(mesures.pm10) ? "NaN" : String(mesures.pm10, 1));

  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket();
}

// ======================================================
// SETUP (Lancement de la station)
// ======================================================
void setup() {
  delay(3000);
  DEBUG_BEGIN(115200);

  // Initialisation du bus I2C (Fréquence standard 100kHz)
  Wire.begin();
  Wire.setClock(100000);
  delay(500);

  // Vérification critique du capteur de Formaldéhyde
  DEBUG_PRINTLN(F("Tentative de connexion au SFA40..."));
  int sfaTries = 0;
  while (sfa40.begin() != 0 && sfaTries < 5) {
    delay(500);
    sfaTries++;
  }
  if (sfaTries < 5) {
    sfa40.startMeasurement();
    sfa40Ok = true;
    DEBUG_PRINTLN(F("-> SFA40 OK et demarre !"));
  } else {
    sfa40Ok = false;
    DEBUG_PRINTLN(F("-> SFA40 ERR : Bus I2C trop bruyant"));
  }

  // Configuration des broches périphériques SPI
  pinMode(chipSelect, OUTPUT);
  pinMode(csLoRa, OUTPUT);
  digitalWrite(chipSelect, HIGH);
  digitalWrite(csLoRa, HIGH);
  delay(100);

  FsDateTime::setCallback(dateTime);
  initialiserCarteSD();

  // Démarrage et configuration fine du module LoRa
  pinMode(resetLoRa, OUTPUT);
  digitalWrite(resetLoRa, LOW);
  delay(20);
  digitalWrite(resetLoRa, HIGH);
  delay(50);

  LoRa.setPins(csLoRa, resetLoRa, irqLoRa);
  loraOk = false;
  int tentatives = 0;
  while (tentatives < 5 && !loraOk) {
    if (!LoRa.begin(868E6)) {
      DEBUG_PRINTLN(F("Echec LoRa, nouvel essai..."));
      tentatives++;
      delay(1000);
    } else {
      DEBUG_PRINTLN(F("Module LoRa initialise avec succes !"));
      LoRa.setSyncWord(0xF3);  
      LoRa.setTxPower(20);     
      LoRa.setSpreadingFactor(9);
      loraOk = true;
    }
  }

  scannerI2C();
  initialiserHeureParWiFi();

  pinMode(pinBoutonWiFi, INPUT_PULLUP);
  pinMode(pinBoutonSuivant, INPUT_PULLUP);

  // Lancement de l'IHM
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    DEBUG_PRINTLN(F("Erreur OLED"));
  } else {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);
    display.println("  STATION");
    display.println(" QAI CSTB");
    display.display();
    delay(2000);
  }

  initialiserCapteurs();

  // Initialisation des chronomètres (Permet de forcer une mesure au premier tour)
  chronoMesure = millis() - 570000;
  chronoSGP40 = millis();
  dernierAppui = millis();
}

// ======================================================
// BOUCLE PRINCIPALE
// ======================================================
void loop() {
  unsigned long maintenant = millis();

  // --- Surveillance Batterie ---
  if (maintenant - chronoBatterie >= 2000) {
    chronoBatterie = maintenant;
    lireBatterie();
  }

  // --- Lecture haute fréquence (COV et Formaldéhyde) ---
  // Le SGP40 nécessite une lecture fréquente de son algorithme pour calibrer sa ligne de base
  if (maintenant - chronoSGP40 >= INTERVALLE_SGP40) {
    chronoSGP40 = maintenant;

    uint16_t rh_ticks = (uint16_t)(mesures.hum * 65535 / 100);
    uint16_t temp_ticks = (uint16_t)((mesures.temp + 45) * 65535 / 175);
    uint16_t sVoc = 0, eVoc = 0;
    eVoc = sgp40.measureRawSignal(rh_ticks, temp_ticks, sVoc);
    if (!eVoc) mesures.vocIndex = (int)voc_algorithm.process(sVoc);
    else mesures.vocIndex = -1;

    if (sfa40Ok) {
      sfa40.readMeasurementData();
      if (!isnan(sfa40.HCHO) && sfa40.HCHO >= 0) {
        mesures.hcho = sfa40.HCHO;
      }
    }
  }

  // --- Gestion du préchauffage ventilateur (Particules fines) ---
  if (!sps30_en_chauffe && (maintenant - chronoMesure >= (TEMPS_CYCLE - DELAI_CHAUFFE))) {
    if (!sps30.startMeasurement(SPS30_OUTPUT_FORMAT_OUTPUT_FORMAT_UINT16)) {
      sps30_en_chauffe = true;
    }
    if (ecranActif && !modeWifi) {
      display.clearDisplay();
      display.setCursor(0, 20);
      display.setTextSize(1);
      display.println("Mesure imminente...");
      display.println("Prechauffe SPS30");
      display.display();
    }
  }

  // --- Déclenchement du grand cycle de mesure (Toutes les 10 min) ---
  if (maintenant - chronoMesure >= TEMPS_CYCLE) {
    chronoMesure = maintenant;
    effectuerMesures();
  }

  // --- Interruption matérielle (Bouton WiFi) ---
  bool etatBoutonWifiActuel = digitalRead(pinBoutonWiFi);
  if (etatBoutonWifiActuel == LOW && etatBoutonWifiPrecedent == HIGH && (maintenant - tempsAntiRebondWifi > 50)) {
    if (!modeWifi) {
      ecranActif = true;
      display.ssd1306_command(SSD1306_DISPLAYON);
      allumerAlimWiFi();
      lancerModeWifi();
      chronoWifi = maintenant;
    } else {
      WiFi.disconnect();
      couperAlimWiFi();
      modeWifi = false;
      display.clearDisplay();
      display.setCursor(0, 20);
      display.setTextSize(2);
      display.println("  WIFI");
      display.println(" OFF");
      display.display();
      delay(1000);
    }
    dernierAppui = maintenant;
    tempsAntiRebondWifi = maintenant;
  }
  etatBoutonWifiPrecedent = etatBoutonWifiActuel;

  // --- Interruption matérielle (Bouton Page/Réveil IHM) ---
  bool etatBoutonActuel = digitalRead(pinBoutonSuivant);
  if (etatBoutonActuel == LOW && etatBoutonPrecedent == HIGH && (maintenant - tempsAntiRebondBouton > 50)) {
    if (!ecranActif) {
      ecranActif = true;
      display.ssd1306_command(SSD1306_DISPLAYON); // Réveille l'écran
    } else {
      pageMenu = (pageMenu + 1) % 9; // Navigue dans les 9 pages
    }
    dernierAppui = maintenant;
    tempsAntiRebondBouton = maintenant;
  }
  etatBoutonPrecedent = etatBoutonActuel;

  // --- Gestion Serveur Web et Déconnexion auto ---
  if (modeWifi) {
    gererServeurWeb();
    if (maintenant - chronoWifi > DELAI_WIFI) {
      couperAlimWiFi();
      modeWifi = false;
    }
  }

  // --- Optimisation de la batterie : Extinction de l'écran (Timeout) ---
  if (ecranActif && !modeWifi && (maintenant - dernierAppui > TEMPS_VEILLE)) {
    ecranActif = false;
    display.ssd1306_command(SSD1306_DISPLAYOFF);
  }

  // --- Rafraîchissement de l'interface ---
  if (ecranActif && !modeWifi && !sps30_en_chauffe) afficherMenu();
  
  delay(10); // Soulage le processeur
}

// ======================================================
// FONCTIONS DE GESTION DE L'ENERGIE (Deep Sleep)
// ======================================================
void allumerAlimWiFi() {
  pinMode(NINA_RESETN, OUTPUT);
  digitalWrite(NINA_RESETN, HIGH);
  delay(750);
  DEBUG_PRINTLN(F("Module WiFi sous tension (Hard ON)"));
}

void couperAlimWiFi() {
  WiFi.end();
  pinMode(NINA_RESETN, OUTPUT);
  digitalWrite(NINA_RESETN, LOW);
  DEBUG_PRINTLN(F("Module WiFi completement eteint (Hard OFF)"));
}

// Procédure d'arrêt critique en cas de batterie vide
void miseEnSecurite() {
  if (sdOk) sd.end(); // Sécurise la carte mémoire avant arrêt

  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("BATTERIE CRITIQUE");
  display.println("ARRET IMMINENT...");
  display.display();
  delay(2000);

  // Coupe tous les composants gourmands en énergie
  display.ssd1306_command(SSD1306_DISPLAYOFF);
  couperAlimWiFi();
  LoRa.sleep();
  scd30.stopPeriodicMeasurement();
  sps30.stopMeasurement();

  if (sfa40Ok) {
    sfa40.stopMeasurement();
  }

  delay(2000);

#if defined(ARDUINO_ARCH_SAMD)
  USBDevice.detach(); // Déconnecte l'USB pour réduire la consommation de la puce
#endif

  // Verrouillage de la carte en veille profonde perpétuelle
  while (true) {
    LowPower.deepSleep(10000);
  }
}