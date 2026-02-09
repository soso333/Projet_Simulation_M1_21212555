from random import random,randint
from vector3D import Vector3D as V3D
from barre_2D import *
import pygame
from pygame.locals import *
from types import MethodType
import math

class Univers(object):
    def __init__(self,name='ici',t0=0,step=0.1,dimensions=(100,100),game=False,gameDimensions=(1024,780),fps=60):
        self.name=name
        self.time=[t0]
        self.population = []
        self.generators = []
        self.step = step
        
        self.dimensions = dimensions
        
        self.game = game
        self.gameDimensions = gameDimensions
        self.gameFPS = fps
        
        self.scale =  gameDimensions[0] / dimensions[0]
        
    
    def __str__(self):
        return 'Univers (%s,%g,%g)' % (self.name, self.time[0], self.step)
        
    def __repr__(self):
        return str(self)
        
    def addParticule(self,*members):
        for i in members:
            self.population.append(i)
        
    def addGenerators(self,*members):
        for i in members:
            self.generators.append(i)
        
        
        
    def simulateAll(self):
        #On calcule le mouvement pur un pas pour chaque agent
        for p in self.population:
            for source in self.generators :
                source.setForce(p)
            p.simulate(self.step)
        
        self.time.append(self.time[-1]+self.step)

    def simulateFor(self,duration):
        # On calcule autant de pas que nécessaire pendant duration
        while duration > 0:
            self.simulateAll()
            duration -= self.step
        
    def plot(self):
        from pylab import figure,legend,show
        
        figure(self.name)
        
        for agent in self.population :
            agent.plot()
            
        legend()
        show()
       
    def gameInteraction(self,events,keys):
        # Fonctin qui sera surchargée par le client pour définir ses intéractions
        pass
    
    def simulateRealTime(self):
        # initilisation de l'environnement pygmae, création de la fenetre
        import pygame
        
        running = self.game
    
        successes, failures = pygame.init()
        W, H = self.gameDimensions
        screen = pygame.display.set_mode((W, H))        
        clock = pygame.time.Clock()
                
        # début simulation
        while running:
            screen.fill((240,240,240)) # effacer les images du pas précédent
            
            for i in self.population : 
                i.gameDraw(self.scale, screen)

            for i in self.generators : #vérifier si l'objet sait se dessiner ou non
                if hasattr(i, 'gameDraw') : 
                    i.gameDraw(self.scale, screen)

            pygame.event.pump() # process event queue
            keys = pygame.key.get_pressed() # It gets the states of all keyboard keys.
            events = pygame.event.get()
            
            # gestion de la fermeture de la fenetre / touche Echap
            if keys[pygame.K_ESCAPE]:
                running = False
                
            for event in events:
                if event.type == pygame.QUIT:
                    running = False
            
            # Allons gérer les interactions ailleurs
            self.gameInteraction(events,keys) 
            
            # simuler les mouvement des chaque agent pendant la durée de ce pas
            self.simulateFor(1/self.gameFPS)    
            
            # demander à chaque agent sondessin en pixels sur la fenêtre
            for t in self.population:
                t.gameDraw(self.scale,screen)
            
            
            # get y axis upwards, origin on bottom left : La fenetre pygame a l'axe y vers le bas. On le retourne.
            flip_surface = pygame.transform.flip(screen, False, True)
            screen.blit(flip_surface, (0, 0))
            
            font_obj = pygame.font.Font('freesansbold.ttf', 12)
            text_surface_obj = font_obj.render(('time: %.2f' % self.time[-1]), True, 'green', (240,240,240))
            text_rect_obj = text_surface_obj.get_rect()
            text_rect_obj.topleft = (0, 0)
            
            screen.blit(text_surface_obj, text_rect_obj)
            
            pygame.display.flip()  # envoie de la fenetre vers l'écran
            clock.tick(self.gameFPS) # attendre le prochain pas d'affichage
        
        pygame.quit()

#debug ------------------------------------
try:
    print("Test de la physique des barres...")
    # Création d'objets temporaires pour tester le PFD
    b_test1 = Barre2D(nom="test1", fixed=True)
    b_test2 = Barre2D(nom="test2", mass=2)
    s_test = SpringDumper(b_test1, b_test2, k=100, c=10, l0=0)
    g_test = Gravity()
    
    # Simulation d'un pas de calcul
    g_test.setForce(b_test2)
    s_test.setForce(b_test2)
    b_test2.simulate(0.01)
    
    print("La physique (Translation/Rotation) fonctionne !")
except Exception as e:
    print(f"ERREUR PHYSIQUE DÉTECTÉE : {e}")
    # On peut lever l'erreur pour arrêter le script si le débug échoue
    raise e

#--------------------------------------------

from random import random,randint
from vector3D import Vector3D as V3D
from barre_2D import *
import pygame
from pygame.locals import *
from types import MethodType
import math

class Univers(object):
    def __init__(self,name='ici',t0=0,step=0.1,dimensions=(100,100),game=False,gameDimensions=(1024,780),fps=60):
        self.name=name
        self.time=[t0]
        self.population = []
        self.generators = []
        self.step = step
        
        self.dimensions = dimensions
        
        self.game = game
        self.gameDimensions = gameDimensions
        self.gameFPS = fps
        
        self.scale =  gameDimensions[0] / dimensions[0]
        
    
    def __str__(self):
        return 'Univers (%s,%g,%g)' % (self.name, self.time[0], self.step)
        
    def __repr__(self):
        return str(self)
        
    def addParticule(self,*members):
        for i in members:
            self.population.append(i)
        
    def addGenerators(self,*members):
        for i in members:
            self.generators.append(i)
        
        
        
    def simulateAll(self):
        #On calcule le mouvement pur un pas pour chaque agent
        for p in self.population:
            for source in self.generators :
                source.setForce(p)
            p.simulate(self.step)
        
        self.time.append(self.time[-1]+self.step)

    def simulateFor(self,duration):
        # On calcule autant de pas que nécessaire pendant duration
        while duration > 0:
            self.simulateAll()
            duration -= self.step
        
    def plot(self):
        from pylab import figure,legend,show
        
        figure(self.name)
        
        for agent in self.population :
            agent.plot()
            
        legend()
        show()
       
    def gameInteraction(self,events,keys):
        # Fonctin qui sera surchargée par le client pour définir ses intéractions
        pass
    
    def simulateRealTime(self):
        # initilisation de l'environnement pygmae, création de la fenetre
        import pygame
        
        running = self.game
    
        successes, failures = pygame.init()
        W, H = self.gameDimensions
        screen = pygame.display.set_mode((W, H))        
        clock = pygame.time.Clock()
                
        # début simulation
        while running:
            screen.fill((240,240,240)) # effacer les images du pas précédent
            
            for i in self.population : 
                i.gameDraw(self.scale, screen)

            for i in self.generators : #vérifier si l'objet sait se dessiner ou non
                if hasattr(i, 'gameDraw') : 
                    i.gameDraw(self.scale, screen)

            pygame.event.pump() # process event queue
            keys = pygame.key.get_pressed() # It gets the states of all keyboard keys.
            events = pygame.event.get()
            
            # gestion de la fermeture de la fenetre / touche Echap
            if keys[pygame.K_ESCAPE]:
                running = False
                
            for event in events:
                if event.type == pygame.QUIT:
                    running = False
            
            # Allons gérer les interactions ailleurs
            self.gameInteraction(events,keys) 
            
            # simuler les mouvement des chaque agent pendant la durée de ce pas
            self.simulateFor(1/self.gameFPS)    
            
            # demander à chaque agent sondessin en pixels sur la fenêtre
            for t in self.population:
                t.gameDraw(self.scale,screen)
            
            
            # get y axis upwards, origin on bottom left : La fenetre pygame a l'axe y vers le bas. On le retourne.
            flip_surface = pygame.transform.flip(screen, False, flip_y=True)
            screen.blit(flip_surface, (0, 0))
            
            font_obj = pygame.font.Font('freesansbold.ttf', 12)
            text_surface_obj = font_obj.render(('time: %.2f' % self.time[-1]), True, 'green', (240,240,240))
            text_rect_obj = text_surface_obj.get_rect()
            text_rect_obj.topleft = (0, 0)
            
            screen.blit(text_surface_obj, text_rect_obj)
            
            pygame.display.flip()  # envoie de la fenetre vers l'écran
            clock.tick(self.gameFPS) # attendre le prochain pas d'affichage
        
        pygame.quit()

#debug ------------------------------------
try:
    print("Test de la physique des barres...")
    # Création d'objets temporaires pour tester le PFD
    b_test1 = Barre2D(nom="test1", fixed=True)
    b_test2 = Barre2D(nom="test2", mass=2)
    s_test = SpringDumper(b_test1, b_test2, k=100, c=10, l0=0)
    g_test = Gravity()
    
    # Simulation d'un pas de calcul
    g_test.setForce(b_test2)
    s_test.setForce(b_test2)
    b_test2.simulate(0.01)
    
    print("La physique (Translation/Rotation) fonctionne !")
except Exception as e:
    print(f"ERREUR PHYSIQUE DÉTECTÉE : {e}")
    # On peut lever l'erreur pour arrêter le script si le débug échoue
    raise e

def solution_analytique(temps, theta, L, theta0, a) : 
    import numpy as np
    g = 10
    omega = np.sqrt((3*g)/(2*L))

    #creation du vecteur temps 
    tps = np.array(temps)

    #solution ana
    theta_ana = (theta0/2)*(np.exp(omega*tps) + np.exp(-omega*tps)) + a/g
    return theta_ana


#--------------------------------------------

if __name__=='__main__':

    from pylab import figure, show, legend
    
    monUnivers = Univers(game=True)
    monUnivers.step=0.001

    base = Barre2D(mass=5, long=12, pos=V3D(50,50,0), theta=0, fixed=False, nom="fixe") 

    #Longueur du pendule : 
    l_pendule = 15
    angle_pendule = math.pi/2 + 0.1 #on met le pendule tête en haut pour qu'il soit cohérent avec la théorie
    pos_x = 50 + l_pendule * math.cos(angle_pendule)
    pos_y = 50 + l_pendule*math.sin(angle_pendule)

    pendule = Barre2D(mass=1,long=l_pendule,pos=V3D(pos_x, pos_y), fixed=False, color="green", nom="mobile", theta=angle_pendule) 
    
    #liaison pivot et base mobile
    liaison = Liaison.pivot(barre1=base, barre2=pendule, k=40000, c=300, pos1=V3D(0, 0), pos2=V3D(-l_pendule/2,0))
    
    force = Gravity(V3D(0,-10))
    monUnivers.addParticule(base,pendule)
    monUnivers.addGenerators(force,liaison) 

    #on s'occupe de la base mobile
    def base_mobile(self,events,keys) : 
        force_moteur = 0 #force appliquée au pendule

        #contrôle utilisateur
        if keys[pygame.K_LEFT] : 
            force_moteur = 500 #N

        if keys[pygame.K_RIGHT]: 
            force_moteur = -500
           
        # on applique la force au cdm
        base.applyEffort(Force=V3D(force_moteur, 0, 0))

        #nouvelle approche : comme les liaisons avec les ressorts marchaient pas du tout pour bloquer l'axe y et l'angle de la base:
        #on bloque manuellement

        #y
        base.pos[-1].y = 50.0
        base.vitesse[-1].y = 0
        base.acceleration[-1].y = 0
        
        # rotation
        base.theta[-1] = 0
        base.omega[-1] = 0
        base.omegadot[-1] = 0

    # Injection de la logique dans l'univers
    monUnivers.gameInteraction = MethodType(base_mobile, monUnivers)
    monUnivers.simulateRealTime()

    #solution analytique
    import numpy as np
    import matplotlib.pyplot as plt
    g = 10
    temps_sim = np.array(monUnivers.time)
    omega = np.sqrt((3*g)/(2*l_pendule))
    theta_ana = solution_analytique(temps=temps_sim, theta=np.array(pendule.theta), L = l_pendule, theta0= 0.1, a = 0)
    plt.figure()
    plt.plot(temps_sim, (np.array(pendule.theta) - (math.pi/2)), label="Simulation", color="blue")
    plt.plot(temps_sim,theta_ana, label="Analytique", color="red")
    monUnivers.plot()
