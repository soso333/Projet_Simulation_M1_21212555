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

#--------------------------------------------

if __name__=='__main__':

    from pylab import figure, show, legend
    
    monUnivers = Univers(game=True)
    monUnivers.step=0.001
    
    #définition de la base mobile, uniquement horizontalement
    base_mobile = Barre2D(mass= 5.0, long=10, pos=V3D(50,50), color="blue", nom="base_mobile")

    #définition de lu pendule
    pendule = Barre2D(mass=1, long=15, pos=V3D(50,57.5), theta=0.2, color="green", nom="pendule")

    #on définit la liaison pivot entre le pendule et la base mobile : 
    liaison = Liaison.pivot(base_mobile, pendule, pos1=V3D(0,0), pos2=V3D(0,-7.5), k=20000, c=5, l0=0)

    # on définit deux liaisons : une selon rotation et une selon y car on bloque ces mouvements
    objet_fixe = Barre2D(pos=V3D(50,50), fixed=True)

    liaison_2 = TorsionSpringDumper(base_mobile, objet_fixe, k_rot=10000, c_rot=100)
    #nous avons besoin de bloquer l'objet fixe selon y, pour ne pas qu'il tombe à cause de la gravité, nous nous servons de dump pour contrôler cela 
    liaison_3 = SpringDumper(base_mobile, objet_fixe, k=10000,c=100, l0 = 0, pos0=V3D(0,0), pos1=V3D(0,0))
    
    #force supplémentaire
    force = Gravity(V3D(0,-10))

    # ajout à l'univers
    monUnivers.addParticule(base_mobile, pendule)
    monUnivers.addGenerators(liaison, liaison_2, force, liaison_3)

    def myInteraction(self,events,keys):
        # Application des forces, l'axe x est piloté en force

       for i in self.population : 
                  if i.nom == "base_mobile" :  
                       force_value = 0
                       if keys[pygame.K_LEFT] : 
                          i.applyEffort(Force=V3D(-50, 0, 0))
                       if keys[pygame.K_RIGHT] : 
                          i.applyEffort(Force=V3D(50, 0, 0))

         
# Surcharge de la fonction ici
    monUnivers.gameInteraction = MethodType(myInteraction,monUnivers)
 
    monUnivers.simulateRealTime()
    monUnivers.plot()
