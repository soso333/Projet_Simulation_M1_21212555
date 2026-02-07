from MoteurCCL0 import MoteurCC
import matplotlib.pyplot as plt
import numpy as np

class ControlPID_vitesse_3:
    """ Contrôleur PID pour la vitesse d'un moteur à courant continu """

    def __init__(self, K_P, K_I, K_D, moteurCC, color = 'blue', position_ecran=(512, 390)):

        self.K_P = K_P
        self.K_I = K_I
        self.K_D = K_D
        self.moteurCC = moteurCC

        #ajout pour l'implémentation de la classe univers
        self.color = color
        self.position_x, self.position_y = position_ecran #position dans la fenetre pygame

        # Entrée et sortie du contrôleur :
        self.vitesse_des = 0
        self.tension_envoyee_moteur = 0
        self.vitesse_moteur = 0

        #definition des erreurs 
        self.erreur = [0]
        self.erreur_integrale = 0 #c'est la somme des erreurs
        self.erreur_derivee = 0 #c'est la dérivée de l'erreur


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
    
    def simule(self, step) : 
        """ Permet de simuler le contrôleur PID pour la vitesse du moteur à courant continu """
    
        # 1) On commence par mesurer la vitesse actuelle du moteur : 
        vitesse_actuelle = self.moteurCC.getSpeed()

        # 2) On calcule l'erreur
        err = self.vitesse_des - vitesse_actuelle

        # On calcule l'erreur dérivée : 
        self.erreur_derivee = (err - self.erreur[-1])/step

        # 3) : On calcule l'erreur intégrale : 
        self.erreur.append(err) #màj de l'erreur ici sinon ça pose un pb pour dérivée
        self.erreur_integrale += err*step 

        #3) On applique la loi de commande PI
        self.tension_envoyee_moteur = self.K_P*err + self.K_I*self.erreur_integrale  + self.K_D*self.erreur_derivee

        # 4) On applique la tension au moteur
        self.moteurCC.setVoltage(self.tension_envoyee_moteur) 

        # 5) On met à jour la vitesse du moteur
        self.moteurCC.simule(step) 

    def plot(self, temps) : 
        """ permet de tracer, directement importée du cours """
        
        from pylab import plot
        return plot(temps, self.moteurCC.getSpeed())  
    
    #ajout pour l'implémentation de la classe univers - aide extérieure pour l'interface graphique (en toute honneteté)
    def gameDraw(self,scale,screen):
        import pygame
        
        # dessin stator 
        pygame.draw.circle(screen,(100,100,100),(self.position_x,self.position_y),50,2)

        # dessin du rotor : 
        angle = self.moteurCC.getPosition() 
        
        # calcul de la position du rotor :
        X = self.position_x + 45*np.cos(angle)
        Y = self.position_y + 45*np.sin(angle)
          
        pygame.draw.line(screen,self.color,(self.position_x,self.position_y),(X,Y),2)
        pygame.draw.circle(screen,self.color,(X,Y),5)

        font = pygame.font.Font(None, 24)
        img = font.render(f"PID: {self.moteurCC.getSpeed():.2f} rad/s", True, (0, 0, 0))
        screen.blit(img, (self.position_x - 40, self.position_y + 60))
        

if __name__=='__main__':
    from pylab import figure, show, legend, title

    m_bo = MoteurCC() # moteur en boucle ouverte

    K = m_bo.kc / (m_bo.R * m_bo.f + m_bo.kc * m_bo.ke) #On reprend Um(t), on change i(t) à l'intérieur et on a la formule : K = omega/Um, ça donne ça pour K

    m_bf = MoteurCC() # moteur boucle fermé pour le controleur

    P = m_bf.Kp = 10 # valeur arbitraire
    I = m_bf.Ki = 50 # valeur arbitraire
    D = m_bf.Kd = 0.1


    control = ControlPID_vitesse_3(P,I,D , m_bf)
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