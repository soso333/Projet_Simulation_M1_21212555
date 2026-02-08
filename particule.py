from math import pi,atan2
from MoteurCCL0 import MoteurCC
import numpy as np

class ParticuleLagrange(object):
    """ Nouvelle version de la classe particule : Elle ne repose plus sur le modèle du PFD, mais sur le modèle de Lagrange (même si c'est le même résultat)"""

    def __init__(self, mass=1, k = 10, L0 = 0.5, C = 0.1, name = "paf", color = "green"):

        self.k = k          # Constante de raideur du ressort
        self.L0 = L0        # Longueur du ressort au repos
        self.C = C          # Coefficient d'amortissement du ressort
        self.mass = mass    # Masse de la particule
        self.d = [L0]       # Position de la particule à l'instant t, à t=0 la position est L0
        self.dot_d = [0]     # Vitesse de translation de la particule à l'instant t
        self.omega = [0]    # Vitesse angulaire de la particule à l'instant t
        self.name = name
        self.color = color

    def __str__(self):
        msg = 'Particule ('+str(self.mass)+', '+str(self.d[-1])+', '+str(self.omega[-1])+', "'+self.name+'", "'+str(self.color)+'", '+str(self.k)+', '+str(self.L0)+', '+str(self.C)+', '+str(self.dot_d[-1])+')'
        return msg

    def __repr__(self):
        return str(self)


    def getPosition(self) : 
        """ Permet de récupérer la dernière position de notre système """
        return self.d[-1] #[-1] pour l'historique des positions
    
    def getSpeed(self):
        return self.dot_d[-1]


    def getVitesseAngulaire(self):
        return self.omega[-1]
    
    def f1(self, d,  dot_d):
        """ Permet de définir l'équation différentielle de notre modèle"""
        return dot_d
    
    def f2(self, d, dot_d, omega_moteur):
        """ Permet de définir l'équation différentielle de notre modèle"""
        return (self.k*self.L0 - self.C*dot_d + (omega_moteur**2*self.mass-self.k)*d)/self.mass
    
    def simule(self,step, omega_moteur):
        # On commence par définir les états : 
        x1 = self.d[-1] # la position à l'instant t
        x2 = self.dot_d[-1] # la vitesse de translation à l'instant t

        #Mise à jour de la vitesse angulaire :
        self.omega.append(omega_moteur) # vitesse angulaire calculé par moteurCC

        # étape 1, définition du système 
        x_point_1 = self.f1(x1, x2)
        x_point_2 = self.f2(x1, x2, omega_moteur)

        # étape 2, euler explicite 
        x1_bar = x1 + step*x_point_1
        x2_bar = x2 + step*x_point_2

        # étape 3 : correction des dérivées d'Euler : 
        x_point_1_bar = self.f1(x1_bar, x2_bar)
        x_point_2_bar = self.f2(x1_bar, x2_bar, omega_moteur)

        # étape 4 : on passe à l'itération suivante 
        x1_n_plus1 = x1 + (step/2)*(x_point_1 + x_point_1_bar)
        x2_n_plus1 = x2 + (step/2)*(x_point_2 + x_point_2_bar)  

        # mise à jour des variables d'état : 
        self.d.append(x1_n_plus1)
        self.dot_d.append(x2_n_plus1)
 
    def plot(self, temps, systeme, color):
        """ permet de tracer, directement importée du cours """
        
        from pylab import plot
        return plot(temps, systeme, color)

    # Fait avec de l'aide extérieure, pour l'implémentation de la classe univers
    def gameDraw(self,scale,screen, centre=(512, 390), angle=0):
        import pygame, math
        
        # 1) Position de la masse
        dist_px = self.getPosition() * scale
        X = int(centre[0] + dist_px * math.cos(angle))
        Y = int(centre[1] - dist_px * math.sin(angle))
        
        # 2) Calcul des points du ressort
        nodes = 10
        points_ressort = []
        for i in range(nodes + 1):
            p = i / nodes
            # Position de base sur la ligne droite
            curr_x = centre[0] + (dist_px * p) * math.cos(angle)
            curr_y = centre[1] - (dist_px * p) * math.sin(angle)
            
            # --- CORRECTION INDENTATION : Tout ceci doit être DANS la boucle for ---
            if 0 < i < nodes:
                # On calcule l'angle perpendiculaire pour le zigzag
                perp_angle = angle + math.pi/2
                # On fait osciller l'amplitude (10 pixels de chaque côté)
                amplitude = 10 if i % 2 == 0 else -10 
                curr_x += amplitude * math.cos(perp_angle)
                curr_y -= amplitude * math.sin(perp_angle)
                
            points_ressort.append((curr_x, curr_y)) # On ajoute chaque point du zigzag
        # -----------------------------------------------------------------------

        # 3) Dessin du ressort (Ligne brisée)
        if len(points_ressort) > 1:
            pygame.draw.lines(screen, (120, 120, 120), False, points_ressort, 2)
        
        # 4) Dessin de la masse (Un beau cercle avec contour)
        pygame.draw.circle(screen, self.color, (X, Y), 12) 
        pygame.draw.circle(screen, (0, 0, 0), (X, Y), 12, 2) # Contour noir pour le relief

    def solution_analytique(self, temps, omega_moteur):

        delta = self.C**2 - 4*self.mass*self.k + 4*self.mass**2*omega_moteur**2
        alpha = -self.C/(2*self.mass)
        beta = np.sqrt(-delta)/(2*self.mass)

        A = (-self.L0*self.mass*omega_moteur**2)/(self.k - self.mass*omega_moteur**2)
        B = -alpha*A/beta
        return A*np.exp(alpha*temps)*np.cos(beta*temps) + B*np.exp(alpha*temps)*np.sin(beta*temps) + (self.k*self.L0)/(-self.mass*omega_moteur**2 + self.k)

if __name__=='__main__':
    from pylab import figure, show, legend, title

    # Initialisation de la particule 
    P0 =ParticuleLagrange()

    # paramètre simulation : 
    step = 0.01
    t = 0
    temps = [0]
    
    while t < 10 : 
        t=t+step
        temps.append(t)
        P0.simule(step, omega_moteur=2) # on suppose que le moteur tourne à une vitesse constante de 2 rad/s - pour ne pas ajouter moteurCC pour le momen
        
    print()
    figure()
    P0.plot(temps, P0.d, 'blue')
    title("Position de la particule en fonction du temps")
    legend()

    figure()
    P0.plot(temps, P0.dot_d, 'red')
    title("Vitesse de la particule en fonction du temps")
    legend()

    # Tracer la solution analytique
    temps_np = np.array(temps)
    sol_ana = P0.solution_analytique(temps_np, omega_moteur=2)
    figure()
    P0.plot(temps, sol_ana, 'green')
    title("Solution analytique de la position de la particule")
    show()

    
    
