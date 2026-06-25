"""
Fichier : acquisition_keithley.py
Auteur : Maël Lefebvre
Date : Juin 2026
Description : Script d'acquisition des mesures de courant via un multimètre Keithley (réseau).
Enregistre les données de courant en continu, calcule la consommation énergétique
en temps réel (intégration numérique) et génère un bilan de la capacité consommée
(mAh) pour valider l'autonomie sur batterie du prototype QAI.
"""

import socket
import time
import csv
from datetime import datetime, timezone

# --- PARAMÈTRES DE CONNEXION ET DE CONFIGURATION ---
IP_KEITHLEY = '169.254.34.125'
PORT = 1394
INTERVALLE_SEC = 0.2
FICHIER_CSV = 'donnees_courant.csv'
FICHIER_BILAN = 'rapport_batterie.txt'

def envoyer_commande(sock, commande):
    sock.sendall((commande + '\n').encode('utf-8'))

def lire_reponse(sock):
    return sock.recv(1024).decode('utf-8').strip()

# --- INITIALISATION ET ACQUISITION ---
print(f"Tentative de connexion au Keithley sur {IP_KEITHLEY}...")
try:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(5.0)
        s.connect((IP_KEITHLEY, PORT))
        print("Connecté avec succès !")

        # Configuration de l'instrument
        envoyer_commande(s, '*RST')
        time.sleep(1)
        envoyer_commande(s, "FUNC 'CURR:DC'")

        # Préparation du fichier d'enregistrement CSV
        with open(FICHIER_CSV, mode='w', newline='') as fichier:
            writer = csv.writer(fichier, delimiter=';')
            writer.writerow(['Date', 'Heure (GMT+0)', 'Courant (A)', 'Capacité Cumulée (mAh)'])

            print(f"Début de l'acquisition (Attente de {INTERVALLE_SEC}s entre chaque boucle).")
            print("Appuyez sur CRTL+C dans cette fenêtre pour arrêter.")
            print("-" * 60)

            charge_cumulee_mAh = 0.0
            somme_courant_A = 0.0
            nb_mesures = 0

            # Initialisation du chronomètre pour le calcul précis de la consommation
            temps_precedent = time.time()

            try:
                while True:
                    envoyer_commande(s, 'READ?')
                    reponse = lire_reponse(s)

                    # Calcul du temps réel écoulé (delta_t) entre deux mesures pour l'intégration
                    temps_actuel = time.time()
                    delta_t = temps_actuel - temps_precedent
                    temps_precedent = temps_actuel  # Réinitialisation pour le cycle suivant

                    valeur_brute = reponse.split(',')[0]
                    valeur_texte = valeur_brute.replace('ADC', '').replace('A', '').strip()

                    try:
                        valeur_nombre = float(valeur_texte)

                        somme_courant_A += valeur_nombre
                        nb_mesures += 1

                        # Intégration du courant sur l'intervalle de temps réel pour obtenir la charge
                        conso_intervalle = (valeur_nombre * 1000) * (delta_t / 3600.0)
                        charge_cumulee_mAh += conso_intervalle

                        # Formatage à la française pour Excel (virgules)
                        valeur_courant = f"{valeur_nombre:.6f}".replace('.', ',')
                        valeur_capacite = f"{charge_cumulee_mAh:.4f}".replace('.', ',')

                    except ValueError:
                        valeur_courant = valeur_texte
                        valeur_capacite = "ERREUR"

                    # Horodatage
                    maintenant = datetime.now(timezone.utc)
                    date_actuelle = maintenant.strftime('%Y-%m-%d')
                    heure_actuelle = maintenant.strftime('%H:%M:%S')

                    print(f"[{heure_actuelle} GMT] Courant : {valeur_courant} A | Consommé : {charge_cumulee_mAh:.4f} mAh")

                    # Sauvegarde sur disque
                    writer.writerow([date_actuelle, heure_actuelle, valeur_courant, valeur_capacite])
                    fichier.flush()

                    time.sleep(INTERVALLE_SEC)

            # Gestion de l'arrêt manuel par l'utilisateur
            except KeyboardInterrupt:
                print("\n" + "=" * 60)
                print("🏁 FIN DE L'ACQUISITION - GÉNÉRATION DU BILAN")

                # Génération du rapport de synthèse post-mesure
                if nb_mesures > 0:
                    moyenne_A = somme_courant_A / nb_mesures

                    with open(FICHIER_BILAN, mode='w', encoding='utf-8') as f_bilan:
                        f_bilan.write("=========================================\n")
                        f_bilan.write("      RAPPORT DE CONSOMMATION BATTERIE   \n")
                        f_bilan.write("=========================================\n\n")
                        f_bilan.write(f"Date de fin du test : {date_actuelle} à {heure_actuelle} (GMT)\n")
                        f_bilan.write(f"Nombre de mesures effectuées : {nb_mesures}\n")
                        f_bilan.write(f"-> COURANT MOYEN : {moyenne_A:.6f} Ampères\n")
                        f_bilan.write(f"-> CAPACITÉ TOTALE VRAIE CONSOMMÉE : {charge_cumulee_mAh:.2f} mAh\n")

                    print(f"Les courbes sont dans : {FICHIER_CSV}")
                    print(f"Le résultat final exact est dans : {FICHIER_BILAN}")

                print("=" * 60)

except Exception as e: 
    print(f"Erreur : {e}")