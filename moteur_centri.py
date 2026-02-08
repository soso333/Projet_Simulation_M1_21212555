from matplotlib import pylab
import numpy as np
import math
from particule import ParticuleLagrange
from MoteurCCL0 import MoteurCC

# cette classe est pour la boucle ouverte ! 

class MoteurCentrifugeuse:  
    """ Simulateur d'un moteur à courant continu """
    
    def __init__(self, moteurCC, particule, position_ecran=(512, 390), color = 'red'):
        
        self.moteurCC = moteurCC
        self.particule = particule

        #ajout pour l'implémentation de la classe univers
        self.color = color
        self.position_x, self.position_y = position_ecran

        #calcul de d_eq pour valider la théorie
        self.d_eq = []
        
        
    def __str__(self):
        """ Affichage des paramètres donnés du moteur à courant continu """
        
        msg = ("Moteur centrifugeuse\n"
        f"MoteurCC = {self.moteurCC}\n", 
        f"ParticuleLagrange = {self.particule}\n"
        f"Couleur = {self.color}\n"
        )
        
        return msg
        
    def __repr__(self) : 
        return str(self)
    
    def simule(self, step) : 
        """ Permet de simuler le moteur à CC et la particule """
        
        # 1) On commence par simuler le moteur à courant continu : 
        self.moteurCC.simule(step)

        # 2) On récupère la vitesse du moteur : 
        omega_moteur = self.moteurCC.getSpeed()

        # 3) On simule la particule avec la vitesse du moteur :
        self.particule.simule(step, omega_moteur)
        
        
    def plot(self, temps, omega, color):
        """ permet de tracer, directement importée du cours """
        
        from pylab import plot
        return plot(temps, omega, color)  
        
    #ajout pour l'implémentation de la classe univers - aide extérieure : 
    def gameDraw(self,scale,screen):
        import pygame
        
        # dessin stator 
        pygame.draw.circle(screen,(150,150,150),(self.position_x,self.position_y),50,2)

        # dessin du rotor : 
        X = self.position_x + 45*np.cos(self.moteurCC.getPosition())
        Y = self.position_y - 45*np.sin(self.moteurCC.getPosition())

        #dessin particule : 
        self.particule.gameDraw(scale, screen, centre=(self.position_x,self.position_y), angle=self.moteurCC.getPosition())
        
        pygame.draw.line(screen,self.color,(self.position_x,self.position_y),(X,Y),2)
        pygame.draw.circle(screen,self.color,(X,Y),5)

        font = pygame.font.Font(None, 24)
        img = font.render(f"Vitesse: {self.moteurCC.getSpeed():.2f} rad/s", True, (0, 0, 0))
        screen.blit(img, (self.position_x - 40, self.position_y + 60))
       

    def calcul_d_eq(self) : 
       
       #récupération des paramètres de la particule pour calculer d_eq
        k = self.particule.k
        L0 = self.particule.L0
        m = self.particule.mass
        omega = self.moteurCC.getSpeed()
        
        denominateur = (k - m*omega**2)

        if denominateur == 0 :
            print("il n'y a pas de position d'équilibre stable")
            return None
        else : 
            return (k*L0)/denominateur
        
        
# Début du main : 
     
if __name__=='__main__':
    from pylab import figure, show, legend, title

    #Définition du moteur en boucle ouverte et de la particule puis du moteur centrifugeuse : 
    m = MoteurCC()
    p = ParticuleLagrange()
    moteur_centrifugeuse = MoteurCentrifugeuse(m, p)

    t = 0
    step = 0.01
    temps = [t]

    #calcul d_eq initial : 
    deq = [moteur_centrifugeuse.calcul_d_eq()]

    while t<2 : #tester pour d'autre temps, il s'amorti 
        t=t+step
        temps.append(t)

        # On applique une tension de 2V au moteur (car défini à 0 de base dans MoteurCCL0) mais elle est l'entrée du systeme 
        moteur_centrifugeuse.moteurCC.setVoltage(5)
        moteur_centrifugeuse.simule(step)

        #on ajoute à la liste d_eq calculé
        deq.append(moteur_centrifugeuse.calcul_d_eq())

    #  On trace la vitesse du moteur (numérique vs analytique) pour être sûr que le moteurCC tourne bien
    figure()
    moteur_centrifugeuse.plot(temps, moteur_centrifugeuse.moteurCC.omega, color='red')
    title("Vitesse du moteur à courant continu en boucle ouverte")
    legend()
    
    # Visualisation de la position de la particule (vu comme une masse qui fait osciller le ressort)
    figure()
    moteur_centrifugeuse.plot(temps, moteur_centrifugeuse.particule.d, color='green')
    #ajout position d'quilibfre tracé
    moteur_centrifugeuse.plot(temps, deq, color='blue')
    title("Position de la particule en fonction du temps")
    legend()

    show()



