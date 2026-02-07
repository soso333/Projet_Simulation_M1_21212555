from MoteurCCL0 import MoteurCC
import matplotlib.pyplot as plt
import numpy as np

class ControlPID_vitesse_2 :
    """ Contrôleur PID pour la vitesse d'un moteur à courant continu """

    def __init__(self, K_P, K_I, K_D, moteurCC):

        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D
        self.moteurCC = moteurCC

        # Entrée et sortie du contrôleur :
        self.vitesse_des = 0
        self.tension_envoyee_moteur = 0
        self.vitesse_moteur = 0

        #definition des erreurs 
        self.erreur = [0]
        self.erreur_integrale = 0 #c'est la somme des erreurs
        self.erreur_derivative = 0


    def __str__(self):
        msg = f"Régulateur PID pour la vitesse du moteur à courant continu avec K_P={self.K_P}, K_I={self.K_I} et K_D={self.K_D}"
        return msg
    
    def __repr__(self): 
        return str(self)
    
    def setTarget(self, vitesse) : 
        """ Permet de définir la vitesse désirée à atteindre par le moteur """
        self.vitesse_des = vitesse

    def getVoltage(self) : 
        """ Permet de récupérer la tension envoyée au moteur"""
        return self.tension_envoyee_moteur
    
    def simule(self, step) : #revenir plus tard dessus
        """ Permet de simuler le contrôleur PID pour la vitesse du moteur à courant continu """
    
        # 1) On commence par mesurer la vitesse actuelle du moteur : 
        vitesse_actuelle = self.moteurCC.getSpeed()

        # 2) On calcule l'erreur
        err = self.vitesse_des - vitesse_actuelle
        self.erreur.append(err)

        # 3) : On calcule l'erreur intégrale : 
        self.erreur_integrale += err*step 

        #3) On applique la loi de commande PI
        self.tension_envoyee_moteur = self.K_P*err + self.K_I*self.erreur_integrale 

        # 4) On applique la tension au moteur
        self.moteurCC.setVoltage(self.tension_envoyee_moteur) 

        # 5) On met à jour la vitesse du moteur
        self.moteurCC.simule(step) 

    def plot(self, temps) : 
        """ permet de tracer, directement importée du cours """
        
        from pylab import plot
        return plot(temps, self.moteurCC.getSpeed())  

if __name__=='__main__':
    from pylab import figure, show, legend, title

    m_bo = MoteurCC() # moteur en boucle ouverte

    K = m_bo.kc / (m_bo.R * m_bo.f + m_bo.kc * m_bo.ke) #On reprend Um(t), on change i(t) à l'intérieur et on a la formule : K = omega/Um, ça donne ça pour K

    m_bf = MoteurCC() # moteur boucle fermé pour le controleur

    P = m_bf.Kp = 10 # valeur arbitraire
    I = m_bf.Ki = 50 # valeur arbitraire
    D = m_bf.Kd = 0


    control = ControlPID_vitesse_2(P,I,D , m_bf)
    t = 0
    step = 0.01
    temps = [t]
    while t<2 :
        t=t+step
        temps.append(t)
        m_bo.setVoltage(1/K) # pour avoir la même réponse en régime permanent
        control.setTarget(1) # rad/s
        control.simule(step) # m_bf.setVoltage et simule seront appelés ici
        m_bo.simule(step)
    
    figure()
    m_bo.plot(temps, m_bo.omega, color = 'red')
    m_bf.plot(temps, m_bf.omega, color = 'blue')
    legend(['Boucle ouverte', 'Boucle fermée'])
    show()