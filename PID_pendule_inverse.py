import matplotlib.pyplot as plt
import numpy as np

class ControlPID_angle:
    """ Contrôleur PID pour la vitesse d'un moteur à courant continu """

    def __init__(self, K_P, K_I, K_D):

        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D
        self.angle_des= 0 #l'angle désiré
        self.force_calculee = 0 #perturbations

        #definition des erreurs 
        self.erreur = [0]
        self.erreur_integrale = 0 #c'est la somme des erreurs
        self.erreur_derivee = 0 #c'est la dérivée de l'erreur


    def __str__(self):
        msg = f"Régulateur PID pour la vitesse du moteur à courant continu avec K_P={self.K_P}, K_I={self.K_I} et K_D={self.K_D}"
        return msg
    
    def __repr__(self): 
        return str(self)
    
    def setTarget(self, angle) : 
        """ Permet de définir l'angle désiré à atteindre """
        self.angle_des = angle

    
    def simule(self, step, angle_actuel) : 
        """ Permet de simuler le contrôleur PID pour la vitesse du moteur à courant continu """
    
        # 1) On calcule l'erreur
        err = self.angle_des - angle_actuel

        # 2) On calcule l'erreur dérivée : 
        self.erreur_derivee = (err - self.erreur[-1])/step

        # 3) : On calcule l'erreur intégrale : 
        self.erreur.append(err) #màj de l'erreur ici sinon ça pose un pb pour dérivée
        self.erreur_integrale += err*step 

        #3) On applique la loi de commande PI
        self.force_calculee = self.K_P*err + self.K_I*self.erreur_integrale  + self.K_D*self.erreur_derivee

        return self.force_calculee
    