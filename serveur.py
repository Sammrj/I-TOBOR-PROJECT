import socket
import threading
import time

host, port = ('', 4455)  # vu que c'est le serveur, c'est le host en même temps
socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)  # ces deux sockets sont suffisantesvoivi
socket.bind((host, port))
print("le serveur est démarré...")


class ThreadForClient(threading.Thread):
    """
    class permettant de garder la connexion Tobor-InterfaceGraphique ouverte afin d'envoyer et de recevoir des messages
    """
    fin_communication = False  # sera utilisé pour mettre fin proprement à la communication

    def __init__(self, connexion):
        threading.Thread.__init__(self)
        self.connexion = connexion
        self.send_rec_data = SendAndReceiveData(self.connexion)

    def run(self):
        try:
            self.send_rec_data.tobor_say_hello()  # premier echange auto entre tobor et l'interface graphique
            while True:
                if self.send_rec_data.messageFromTobor is not None and self.send_rec_data.messageFromTobor != "Erreur":
                    self.send_rec_data.send_data()  # Envoi de données
                    self.send_rec_data.rec_data()  # Reception de données
        except:
            ExchangeWithUser.connected = False
            socket.close()


class SendAndReceiveData:
    """
    classe permettant de gérer les envois et réceptions de données
    """
    data_rec: str = None  # message reçu de l'interface graphique,S
    message_from_tobor: str = None  # message que Tobor envoie à l'interface graphique
    success_send: str = None  # message bien envoyé ou pas

    def __init__(self, connexion):
        self.connexion = connexion

    def _set_message_from_tobor(self, message):
        self.message_from_tobor = message

    def _get_message_from_tobor(self):
        return self.message_from_tobor

    def reset_message_from_tobor(self):
        self.message_from_tobor = None

    def tobor_say_hello(self):
        self.message_from_tobor = "Hello, c'est Tobor !"
        self.send_data()
        self.rec_data()  # attends une réponse du serveur pour démarrer

    def send_data(self):
        if self.message_from_tobor is not None:
            try:
                self.connexion.send(self.message_from_tobor.encode("utf8"))  # envoi du message
                self.success_send = "T"  # message envoyé avec succès
                self.reset_message_from_tobor()  # suppression (côté Tobor) du message qui vient d'être envoyé
            except:
                self.success_send = "F"

    # Reception des messages venant de l'interface graphique
    def rec_data(self):
        try:
            data = self.connexion.recv(1024)  # Attente ici, tant que aucun message n'est reçu
            self.data_rec = data.decode("utf8")  # decodage du message reçu
        except:
            self.data_rec = "Erreur"  # en cas d'erreur

    def _get_success_send(self):
        return self.success_send

    def _get_data_rec(self):
        return self.data_rec

    # suppression des données venant de l'échange précédent
    def reset_previous_data_(self):
        self.data_rec = None
        self.success_send = None

    # encapsulation

    getDataRec = property(_get_data_rec)
    messageFromTobor = property(_get_message_from_tobor, _set_message_from_tobor)


class ExchangeWithUser:
    """
    class permettant de connecter Tobor à l'interface graphique
    """

    def __init__(self):
        pass

    connected: bool = False

    def launch(self):
        while True:
            socket.listen(10)  # nombre de connexion refuse avant que le serveur refuse des co supplémentaires
            conn, address = socket.accept()  # connexion,addresse de la connexion
            print("en écoute, un client vient de se connecter...")
            time.sleep(1)
            self.myThread = ThreadForClient(conn)
            self.myThread.start()  # lancement du programme en asychrone
            self.connected = True  # connexion établie avec l'interface graphique

    # pour envoyer des messages à l'interface graphique
    def message_to_send(self, message):
        self.myThread.send_rec_data._set_message_from_tobor(message);


def test():
    """
    La partie de code suivant correspond à des tests (simulations)
    -->Tout fonctionne comme prévu
    """

    """
        #initialisation :
        exc = ExchangeWithUser()  # class à initialiser dans le code de Tobor
        th1 = threading.Thread(target=exc.launch)  # Il faut lancer l'échange à asynchrone afin de faire fonctionner tobor en parallèle
        th1.start()  # lancement de l'échange
        
        Methodes à Utiliser
                
        exc.connected ->True ou False :  verifier si TOBOR est connecté avec l'interface graphique
        exc.myThread.send_rec_data.getDataRec-> str ou None : recupère le message envoyer par l'interface graphique
                                                contient None si aucun message recu
        exc.message_to_send(str message) : pour envoyer un message à l'utilisateur
        exc.myThread.send_rec_data.reset_previous_data_() :  # reset des valeurs de certains attributs utilisés lors la précédente échange
        exc.myThread.send_rec_data._get_success_send() : Pour verifier si un message a été envoyer correctement

        + lire les commentaires de l'exemple l'exemple
    """

    exc = ExchangeWithUser()  # class à initialiser dans le code de Tobor
    th1 = threading.Thread(target=exc.launch)  # Il faut lancer l'échange à asynchrone afin de faire fonctionner tobor en parallèle
    th1.start()  # lancement de l'échange
    i = 0  # pour le teste
    while True:  # boucle pour le test
        if exc.connected:  # verifier si TOBOR est connecté avec l'interface graphique
            if exc.myThread.send_rec_data.getDataRec is not None:  # verifier Si tobor a bien recu un message
                print("Tobor a reçu", exc.myThread.send_rec_data.getDataRec, "\n")
                exc.message_to_send(str(100 * i)) # exc.message_to_send (donnees)# pour envoyer des données
                exc.myThread.send_rec_data.reset_previous_data_() # toujours appeler cette méthode après un echange
                i += 1
                """  if exc.myThread.send_rec_data.getDataRec== "No":
                    print("c'est un ok")
                    exc.message_to_send(str(p))
                    print("alors",p)
                    p-=1
                elif exc.myThread.send_rec_data.getDataRec== "OK":
                    print(exc.myThread.send_rec_data.getDataRec)
                    exc.message_to_send(str(i))
                    i+=1
                else:
                    print("mdrrr")
                exc.myThread.send_rec_data.reset_data_rec()"""

    socket.close()

test()
