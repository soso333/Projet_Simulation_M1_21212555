from random import random,randint
import pygame
from pygame.locals import *
from types import MethodType
from MoteurCCL0 import MoteurCC
from controleur_PID import ControlPID_vitesse_3
from moteur_centri_PID import MoteurCentrifugeuse_2
from particule import ParticuleLagrange

class Univers_2:
    def __init__(self,name='Moteur_centrifugeuse',t0=0,step=0.1,dimensions=(10,10),game=False,gameDimensions=(1024,780),fps=60):
        self.name=name
        self.time=[t0]
        self.population = []
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
        
    def addMoteur(self,*members):
        for i in members:
            self.population.append(i)
        
    def addGenerators(self,*members):
        for i in members:
            self.generators.append(i)
        
    def simulateAll(self):
        #On calcule le mouvement pur un pas pour chaque agent
        for p in self.population:
            p.simule(self.step) # On simule le moteurCC pendant un pas de temps
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
            
            
            #on met en commentaire car sinon l'écriture est à l'envers 
            # get y axis upwards, origin on bottom left : La fenetre pygame a l'axe y vers le bas. On le retourne.
            #flip_surface = pygame.transform.flip(screen, False, flip_y=True)
            #screen.blit(flip_surface, (0, 0))
            
            font_obj = pygame.font.Font(None, 12)
            text_surface_obj = font_obj.render(('time: %.2f' % self.time[-1]), True, 'green', (240,240,240))
            text_rect_obj = text_surface_obj.get_rect()
            text_rect_obj.topleft = (0, 0)
            
            screen.blit(text_surface_obj, text_rect_obj)
            
            pygame.display.flip()  # envoie de la fenetre vers l'écran
            clock.tick(self.gameFPS) # attendre le prochain pas d'affichage
        
        pygame.quit()

# DÉBUG !!!------------------
try:
    print("Test de la physique seule...")
    m_test = MoteurCC()
    p_test = ParticuleLagrange()
    c_test = ControlPID_vitesse_3(10, 50, 0.1, m_test)
    sys_test = MoteurCentrifugeuse_2(c_test, p_test)
    
    sys_test.simule(0.01)
    print("La physique fonctionne !")
except Exception as e:
    print(f"ERREUR PHYSIQUE DETECTEE : {e}")
#-----------------------------
    
if __name__=='__main__':
    from pylab import figure, show, legend
    
    monUnivers = Univers_2(game=True, dimensions=(10,10))
    
    monUnivers.step=0.01
    
    #Définition du moteur 
    m = MoteurCC()
    p = ParticuleLagrange()

    #gains initiaux : 
    K_p = 10
    K_i = 50
    K_d = 0.1
    controleur = ControlPID_vitesse_3(K_p, K_i, K_d , m)

    #definition moteur centrfigeuse avec PID : 
    m_centri = MoteurCentrifugeuse_2(controleur, p)

    #Ajout du systeme à l'univers : 
    monUnivers.addMoteur(m_centri)
    
    def myInteraction(self, events,keys):
        import math, pygame

        # controle de leader avec le clavier
        if keys[ord('z')] or keys[pygame.K_UP]: # And if the key is z or K_DOWN:

            #remarque : on met les memes pour bien comparer 
            vitesse_ciblee = 2 # 1 rad/s
            controleur.setTarget(vitesse_ciblee) # 1 rad/s

        if keys[ord('s')] or keys[pygame.K_DOWN]: # And if the key is s or K_DOWN:
            controleur.setTarget(0)

        # Création du contrôleur PID en temps réel - c'est nous qui choisissons les gains
        for event in events : 
            if event.type == pygame.KEYDOWN : 

                # La touche P permet d'augmmenter Kp
                if event.key == pygame.K_p : 
                    controleur.K_P += 1
                # La touche O permet de diminuer Kp
                if event.key == pygame.K_o : 
                    controleur.K_P -= 1
                #La touche i permet d'augmenter Ki
                if event.key == pygame.K_i : 
                    controleur.K_I += 1
                #La touche u permet de diminuer Ki
                if event.key == pygame.K_u : 
                    controleur.K_I -= 1
                #La touche d permet d'augmenter Kd
                if event.key == pygame.K_d : 
                    controleur.K_D += 0.1
                #La touche e permet de diminuer Kd
                if event.key == pygame.K_e : 
                    controleur.K_D -= 0.1

            if keys[pygame.K_SPACE]:
                controleur.erreur_integrale = 0 
                # On a remis à 0 l'erreur - c'est un reset

            if pygame.mouse.get_pressed()[0]: # Clic gauche
                mouse_x, mouse_y = pygame.mouse.get_pos()
                # Calcul de la distance entre le centre (512, 390) et la souris
                dx = mouse_x - 512
                dy = 390 - mouse_y # Inversion Y
                nouvelle_dist = math.sqrt(dx**2 + dy**2) / self.scale
                
                # On force la position de la particule
                p.d = [nouvelle_dist]
                p.dot_d = [0] # On arrête sa vitesse radiale
         
# Surcharge de la fonction ici
    monUnivers.gameInteraction = MethodType(myInteraction,monUnivers)
    monUnivers.simulateRealTime()
    monUnivers.plot()

    

