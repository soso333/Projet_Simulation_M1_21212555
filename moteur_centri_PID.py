from matplotlib import pylab
import numpy as np
import math
from particule import ParticuleLagrange
from controleur_PID import ControlPID_vitesse_3
from MoteurCCL0 import MoteurCC

# cette classe est pour la boucle fermée ! 

class MoteurCentrifugeuse_2:  
    """ Simulateur d'un moteur à courant continu """
    
    def __init__(self, moteur_bf, particule, position_ecran=(512, 390), color = 'red'):
        
        self.moteur_bf = moteur_bf
        self.particule = particule

        #ajout pour l'implémentation de la classe univers
        self.color = color
        self.position_x, self.position_y = position_ecran

        #calcul de d_eq pour valider la théorie
        self.d_eq = []
        
        
    def __str__(self):
        """ Affichage des paramètres donnés du moteur à courant continu """
        
        msg = ("Moteur centrifugeuse\n"
        f"MoteurBF = {self.moteur_bf}\n", 
        f"ParticuleLagrange = {self.particule}\n"
        f"Couleur = {self.color}\n"
        )
        
        return msg
        
    def __repr__(self) : 
        return str(self)
    
    def simule(self, step) : 
        """ Permet de simuler le moteur à CC et la particule """
        
        # 1) On commence par simuler le moteur à courant continu : 
        self.moteur_bf.simule(step)

        # 2) On récupère la vitesse du moteur : 
        omega_moteur = self.moteur_bf.moteurCC.getSpeed()

        # 3) On simule la particule avec la vitesse du moteur :
        self.particule.simule(step, omega_moteur)
        
        
    def plot(self, temps, omega, color):
        """ permet de tracer, directement importée du cours """
        
        from pylab import plot
        return plot(temps, omega, color)  
        
    #ajout pour l'implémentation de la classe univers - aide extérieure : 
    def gameDraw(self, scale, screen):
        import pygame
        import math

        # 1. Dessin d'un rail de guidage (le bras de la centrifugeuse)
        # On dessine un bras qui va jusqu'à une distance fixe (ex: 2m) pour montrer le chemin
        longueur_bras = 2.0 * scale 
        angle = self.moteur_bf.moteurCC.getPosition()
        ax = self.position_x + longueur_bras * math.cos(angle)
        ay = self.position_y - longueur_bras * math.sin(angle)
        
        # On dessine le bras en gris épais pour simuler un rail rigide
        pygame.draw.line(screen, (200, 200, 200), (self.position_x, self.position_y), (ax, ay), 10)
        
        # 2. On appelle le dessin de la particule (qui dessine son ressort par-dessus)
        self.particule.gameDraw(scale, screen, centre=(self.position_x, self.position_y), angle=angle)
        
        # 3. Dessin du moyeu central (le moteur)
        pygame.draw.circle(screen, (50, 50, 50), (self.position_x, self.position_y), 20) # Centre moteur
        
        # 4. --- AJOUT : PANNEAU D'INFORMATIONS ---
        font = pygame.font.Font(None, 28)
        # Fond blanc pour le texte
        pygame.draw.rect(screen, (255, 255, 255), (10, 10, 250, 120))
        pygame.draw.rect(screen, (0, 0, 0), (10, 10, 250, 120), 2) # Bordure
        
        # Textes
        txt_vitesse = font.render(f"Vitesse: {self.moteur_bf.moteurCC.getSpeed():.2f} rad/s", True, (0, 0, 0))
        txt_pos = font.render(f"Distance d: {self.particule.getPosition():.2f} m", True, (0, 100, 0))
        txt_deq = font.render(f"Equilibre d_eq: {self.calcul_d_eq():.2f} m", True, (0, 0, 200))
        txt_pid = font.render(f"Kp: {self.moteur_bf.K_P} | Ki: {self.moteur_bf.K_I}", True, (150, 0, 0))

        screen.blit(txt_vitesse, (20, 20))
        screen.blit(txt_pos, (20, 45))
        screen.blit(txt_deq, (20, 70))
        screen.blit(txt_pid, (20, 95))
       

    def calcul_d_eq(self) : 
       
       #récupération des paramètres de la particule pour calculer d_eq
        k = self.particule.k
        L0 = self.particule.L0
        m = self.particule.mass
        omega = self.moteur_bf.moteurCC.getSpeed()
        
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

    #ajout PID pour le moteur centrifugeuse :
    P = 10
    I = 50
    D = 0.1

    controleur = ControlPID_vitesse_3(P,I,D , m)

    moteur_centrifugeuse = MoteurCentrifugeuse_2(controleur, p)

    t = 0
    step = 0.01
    temps = [t]

    #on donne la vitesse désirée (PID) : 
    moteur_centrifugeuse.moteur_bf.setTarget(1) # rad/s

    #calcul d_eq initial : 
    deq = [moteur_centrifugeuse.calcul_d_eq()]

    while t<2 : #tester pour d'autre temps, il s'amorti 
        t=t+step
        temps.append(t)
        moteur_centrifugeuse.simule(step)
        #on ajoute à la liste d_eq calculé
        deq.append(moteur_centrifugeuse.calcul_d_eq())

    #  On trace la vitesse du moteur (numérique vs analytique) pour être sûr que le moteurCC tourne bien
    figure()
    moteur_centrifugeuse.plot(temps, moteur_centrifugeuse.moteur_bf.moteurCC.omega, color='red')
    title("Vitesse du moteur à courant continu en boucle fermee")
    legend()
    
    # Visualisation de la position de la particule (vu comme une masse qui fait osciller le ressort)
    figure()
    moteur_centrifugeuse.plot(temps, moteur_centrifugeuse.particule.d, color='green')
    #ajout position d'quilibfre tracé
    moteur_centrifugeuse.plot(temps, deq, color='blue')
    title("Position de la particule en fonction du temps")
    legend()

    show()



