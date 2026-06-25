"""
Fichier : ftp_loraV4.py
Auteur : Maël Lefebvre
Date : Juin 2026
Description : Script Python (Passerelle Raspberry Pi) pour la réception
des trames LoRa via liaison série et la synchronisation FTP périodique
vers un serveur distant. Gère la création de fichiers journaliers,
la sauvegarde locale sécurisée et la reprise sur erreur (rattrapage de synchronisation).
"""

import serial
import time
import os
import ftplib
from datetime import datetime

# ==========================================
# CONFIGURATION SERIE
# ==========================================
PORT_SERIE = '/dev/ttyACM0'
BAUD_RATE = 9600

# ==========================================
# CONFIGURATION DOSSIER & LOG
# ==========================================
DOSSIER_SAUVEGARDE = "/home/pi/Donnees_QAI/"
FICHIER_LOG_SYNC = os.path.join(DOSSIER_SAUVEGARDE, "synclog.txt")

# ==========================================
# CONFIGURATION FTP
# ==========================================
FTP_HOST = "ftpperso.free.fr"
FTP_USER = "lenoir.dominique"
FTP_PASS = "krkf20xr"
FTP_DIR = "Brewal_essai_arduino"

INTERVALLE_FTP = 600  # Synchronisation toutes les 10 minutes

print("Lancement de la passerelle LoRa -> FTP...")

# En-tête CSV complet incluant les 7 paramètres environnementaux
EN_TETE_CSV = "Date;Heure;Temp(C);Hum(%);Pres(hPa);CO2(ppm);VOC(idx);NO2(ppm);O3(ppb);HCHO(ppb);PM1.0(ug/m3);PM2.5(ug/m3);PM10(ug/m3)\n"


# ==========================================
# FONCTIONS DE SYNCHRONISATION
# ==========================================
def obtenir_chemin_fichier_jour():
    maintenant = datetime.now()
    nom_fichier = maintenant.strftime("%y%m%d.CSV")
    return os.path.join(DOSSIER_SAUVEGARDE, nom_fichier)


def est_deja_synchronise(nom_fichier):
    if not os.path.exists(FICHIER_LOG_SYNC):
        return False
    with open(FICHIER_LOG_SYNC, 'r') as f:
        lignes = f.read().splitlines()
    return nom_fichier in lignes


def marquer_synchronise(nom_fichier):
    with open(FICHIER_LOG_SYNC, 'a') as f:
        f.write(nom_fichier + '\n')
    print(f"[SYNC] Le fichier {nom_fichier} est marque comme termine.")


def synchroniser_dossier_entier():
    try:
        print(f"[FTP] Connexion au serveur {FTP_HOST}...")
        ftp = ftplib.FTP(FTP_HOST)
        ftp.login(user=FTP_USER, passwd=FTP_PASS)
        ftp.cwd(FTP_DIR)

        # Liste des fichiers d'archives locales
        fichiers_locaux = [f for f in os.listdir(DOSSIER_SAUVEGARDE) if f.upper().endswith('.CSV')]
        fichier_jour_actuel = os.path.basename(obtenir_chemin_fichier_jour())

        for fichier in fichiers_locaux:
            chemin_complet = os.path.join(DOSSIER_SAUVEGARDE, fichier)

            # Mise à jour continue du fichier du jour
            if fichier == fichier_jour_actuel:
                print(f"[FTP] Mise a jour du fichier en cours : {fichier}")
                with open(chemin_complet, 'rb') as f:
                    ftp.storbinary(f'STOR {fichier}', f)

            # Rattrapage des fichiers non synchronisés (hors ligne la veille par ex.)
            else:
                if not est_deja_synchronise(fichier):
                    print(f"[FTP] RATTRAPAGE RETARD : Envoi de l'ancien fichier {fichier}...")
                    with open(chemin_complet, 'rb') as f:
                        ftp.storbinary(f'STOR {fichier}', f)

                    # Marquage de validation pour éviter les doublons futurs
                    marquer_synchronise(fichier)

        ftp.quit()
        print("[FTP] Fin de la synchronisation totale.")

    except Exception as e:
        print(f"[FTP] ERREUR de synchronisation (le reseau est peut-etre coupe) : {e}")


# ==========================================
# INITIALISATION DU SYSTEME
# ==========================================
if not os.path.exists(DOSSIER_SAUVEGARDE):
    os.makedirs(DOSSIER_SAUVEGARDE)
    print(f"[Info] Dossier cree : {DOSSIER_SAUVEGARDE}")

try:
    arduino = serial.Serial(PORT_SERIE, BAUD_RATE, timeout=1)
    time.sleep(2)
    print(f"[OK] Connecte a l'Arduino recepteur sur {PORT_SERIE}")
except Exception as e:
    print(f"Erreur de connexion serie : {e}")
    exit()

dernier_envoi_ftp = time.time()

# ==========================================
# BOUCLE PRINCIPALE DE RECEPTION ET ROUTAGE
# ==========================================
try:
    while True:

        # ECOUTE DU LORA (PRIORITE ABSOLUE EN TEMPS REEL)
        if arduino.in_waiting > 0:
            ligne_brute = arduino.readline().decode('utf-8', errors='ignore').strip()

            if ";" in ligne_brute and len(ligne_brute) > 10:
                maintenant = datetime.now()
                chemin_fichier_jour = obtenir_chemin_fichier_jour()

                # Création automatique du fichier à minuit
                if not os.path.exists(chemin_fichier_jour):
                    with open(chemin_fichier_jour, "w") as fichier:
                        fichier.write(EN_TETE_CSV)
                    print(f"\n[Info] Nouveau fichier journalier cree : {os.path.basename(chemin_fichier_jour)}")

                date_iso = maintenant.strftime("%Y-%m-%d")
                heure_courte = maintenant.strftime("%H:%M")
                donnee_complete = f"{date_iso};{heure_courte};{ligne_brute}\n"

                print(f"\n[LoRa] Mesure recue et sauvegardee : {donnee_complete.strip()}")

                with open(chemin_fichier_jour, "a") as fichier:
                    fichier.write(donnee_complete)

        # ENVOI FTP PERIODIQUE (AVEC SCAN DU DOSSIER)
        temps_actuel = time.time()

        if temps_actuel - dernier_envoi_ftp > INTERVALLE_FTP:
            print("\n[Info] Chrono FTP atteint. Verification du dossier...")
            synchroniser_dossier_entier()
            dernier_envoi_ftp = temps_actuel

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\n[FIN] Arret manuel de la passerelle LoRa.")