import time  # Gestion du temps
from random import random

from kivy import app
# bibliothèque  nécessaire pour la mise en place de la l'interface graphqie
from kivy.app import App
from kivy.clock import Clock
from kivy.properties import ObjectProperty
from kivy.properties import StringProperty
from kivy.uix.widget import Widget
from kivy.core.window import Window

import math

import threading  # Nécessaire pour lancer l'asynchrone
import socket  # Necessaire pour connecter l'interface graphique (ce script) à TOBOR (notre robot)

socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # Création d'un socket de communication


class Config:
    """
    class contenant des infos de configurations
    """
    # Pour les décomptes
    time_count_down_for_connection = 60  # Temps d'attente pour une se connecter au serveur
    time_count_down_deconnection = 10  # Décompte avant la fermeture de la fenêtre après une deconnexion/ou impossibilté de se connecter au serveur

    tobor_say_str_init = "please wait a few seconds..."

    # Infos de config serveur-client
    host, port = ("localhost", 4455)  # Infos pour se connecter au serveur localhost =127.0.0.1, info
    numClient = "1"  # Numéro du client,
    size_Buffer_data_rec = 1024  # Nombre de octets max des données venant du serveur
    caractere_question = "?"


class MyWidget(Widget):
    """
    class permettant de gérer l'interface graphique :
    - positionnement des widgets(déclarer dans le fichier toborinterface.kv),
    - gestion des interractions de l'utilisateur avec l'interface graphique (appuie sur les boutons)
    Cette class n'a pas besoin de constructeur
    """

    tobor_say = StringProperty('')  # Contiendra le texte à afficher à l'utilisateur lorsque  Tobor enverra  un message
    tobor_say = Config.tobor_say_str_init  # Initialisation

    ok_button = ObjectProperty(None)
    no_button = ObjectProperty(None)

    btn_pressed = None  # Contiendra le nom du bouton appuyé par l'utilisateur (OK ou No)

    # Nécessaire pour gérer le positionnement des élements dans l'interface graphique
    window_size_height_percent = ObjectProperty(None)
    window_size_height_percent = 5 * Window.size[0] / 100

    # Executé quand l'utilisateur appuie sur le bouton OK
    def ok_pressed(self):
        self.btn_pressed = "OK"

    # Executé quand l'utilisateur appuie sur le bouton No
    def no_pressed(self):
        self.btn_pressed = "No"

    # Permet de mettre à jour le texte afficher à l'écran quand Tobor envoie un nouveau message
    def set_Tobor_say(self, text):
        self.ids.tobor_say.text = text
        # fait apparaître ou disparaître les boutons suivants les messages recus de Tobor
        if Config.caractere_question in text:
            self.ids.ok_button.background_color = 1, 0, 1, 1
            self.ids.ok_button.text = "OK"
            self.ids.ok_button.disabled = False

            self.ids.no_button.background_color = 1, 0, 1, 1
            self.ids.no_button.text = "NO"
            self.ids.no_button.disabled = False

        else:
            self.ids.ok_button.background_color = 0, 0, 0, 0
            self.ids.ok_button.text = ""
            self.ids.ok_button.disabled = True
            self.ids.no_button.background_color = 0, 0, 0, 0
            self.ids.no_button.text = ""
            self.ids.no_button.disabled = True


class ToborInterface(App):
    """
    class permettant de générer l'interface utilisateur et de lancer la communication avec Tobor en asynchrone
    Constructeur non necessaire grâce à l'héritage de la class App
    """

    fin_communication: bool = False  # Passe à true, à la fin de la communication

    def build(self):
        """
        Rédéfinition de la fonction build, qui existe dans la class mère App,
        cette fonction permet de générer la fênetre de l'interface graphique et de l'afficher à l'écran
        :return: L'interface graphique
        """
        my_widget = MyWidget()  # Instantiation de l'interface graphique

        # Connexion avec TOBOR en asynchrone afin d'échanger des données, tout en rafraichissant l'interface graphique
        thread_tobor_say = threading.Thread(target=ExchangeBetweenToborAndInterface(my_widget).initialize_exchange)
        thread_tobor_say.start()


        return my_widget


class ExchangeBetweenToborAndInterface:
    """
    class gérant la communication entre TOBOR (Le serveur) et l'interface graphique (le client)
    """

    rep_auto = None

    def __init__(self, widget: MyWidget):
        self.my_widget = widget
        self.time_sec = None
        self.stop_time_sec = False;

    def count_down_for_connection(self, time_user=Config.time_count_down_for_connection):
        self.time_sec = time_user
        while self.time_sec > 0:
            time.sleep(1)
            self.time_sec -= 1
            if self.stop_time_sec:
                self.time_sec = 0
                break

    def reset_time_sec(self):
        self.stop_time_sec = True
        self.time_sec = Config.time_count_down_deconnection;

    def down_count_deconnection(self):
        max_sec = Config.time_count_down_deconnection
        while max_sec > 0:
            self.my_widget.set_Tobor_say(
                f"Disconnection to the server/impossible to connect,\n this window will close in {max_sec} seconds")
            time.sleep(1)
            max_sec -= 1

    def initialize_exchange(self):
        """
        Connexion de l'interface graphique avec Tobor-> connexion client-serveur
        """
        th1 = threading.Thread(target=self.count_down_for_connection)
        th1.start()
        while True:
            try:
                socket.connect((Config.host, Config.port))  # Connexion au serveur
                # data_to_send: str = f"Hello, je suis le client {Config.numClient}"  # Message d'initialisation à envoyer au serveur
                # data_to_send = data_to_send.encode("utf8")  # Encodage du message en utf8
                # socket.sendall(data_to_send)  # Envoie du message
                self.reset_time_sec()
                # Affichage du succès de la connexion à l'utilisateur
                self.my_widget.set_Tobor_say("Connection establishes... waiting for Tobor")
                # Lancement des échanges client-serveur, le script passera au break en cas de fin de connexion
                self.question_answer_com()
                break

            except ConnectionRefusedError:
                if self.time_sec != 0:
                    self.my_widget.set_Tobor_say(f"Refused connection, the server is not started  or check "
                                                 f"the value of host and port \n wait : {self.time_sec}")
                else:
                    self.down_count_deconnection()

            except:
                if self.time_sec != 0:
                    self.my_widget.set_Tobor_say(f"Impossible connection to the server \n wait :{self.time_sec}")
                else:
                    self.down_count_deconnection()

            if self.time_sec < 1:
                break

        socket.close()
        App.get_running_app().stop()
        Window.close()
        print("quitter")

    def question_answer_com(self):
        """
        Cette fonction assure les échanges de données server-client tant que la connexion n'est pas rompue
        """
        while True:
            # Lancement en ansynchrone de la fonction des fonctions d'envoie et de receptions de données
            self.rec_data()
            self.send_data()
            # thread_rec = threading.Thread(target=self.rec_data())
            # thread_send = threading.Thread(target=self.send_data())
            # thread_rec.start()
            # thread_send.start()

    def send_data(self):
        """
        Gestion des envois de données (automatique ou bouton appuyé par l'utilisateur) vers le serveur
        """
        if self.rep_auto is None:  # Si on attend une réponse de l'utilisateur
            while True:  # Listen... l'aapuie des boutons de l'interface graphique
                if self.my_widget.btn_pressed is not None:  # Si l'utilisateur a appuyé  sur un bouton
                    data_to_send = str(self.my_widget.btn_pressed)  # Recupération de la clé associée au bouton appuyé
                    data_to_send = data_to_send.encode("utf8")  # encodage du message en utf8
                    socket.sendall(data_to_send);  # envoie du message
                    self.my_widget.set_Tobor_say(f"Sent : {self.my_widget.btn_pressed}")
                    self.my_widget.btn_pressed = None  # Plus de bouton appuyé (réinitialisation)
                    break  # quitter la boucle
        else:  # En cas de réponse auto
            data_to_send = self.rep_auto  # Recupération de la clé associée au bouton appuyé
            data_to_send = data_to_send.encode("utf8")  # encodage du message en utf8
            socket.sendall(data_to_send)  # envoie du message

        self.rep_auto = None
        time.sleep(1)  # pour test

    def rec_data(self):
        """
        Gestion des messages reçu du serveur (TOBOR)
        """
        dataRecu = socket.recv(Config.size_Buffer_data_rec)  # On s'attend à recevoir une donnée de 1024 octets maximum
        dataRecu = dataRecu.decode("utf8")  # Décodage de la donnée réçu
        self.my_widget.set_Tobor_say(f"Received : {dataRecu}")  # Affichage sur l'écran
        if Config.caractere_question not in dataRecu:  # Si ce n'est pas une question
            self.rep_auto = "auto"


if __name__ == "__main__":
    # test de ce script
    ToborInterface().run()
