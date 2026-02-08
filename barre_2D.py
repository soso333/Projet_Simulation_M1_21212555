from vector3D import Vector3D as V3D

class Barre2D : 
    def __init__(self, mass=1, long=1, theta=0, pos=V3D(), fixed = False, color = 'red', nom='barre') : 

        # Paramètres donnés
        self.mass = mass        # La masse de la barre
        self.long = long        # La longueur de la barre
        self.theta = [theta]    # L'angle de la barre
        self.pos = [pos]        # La position initiale de la barre
        self.fixed = fixed      # Si la barre est fixe ou non
        self.color = color      # La couleur de la barre - pour univers
        self.nom = nom          # Le nom de la barre - pour uniers


        # Paramètres ajoutés : 
        self.forces = V3D()         # Toutes les forces appliquées à mon système
        self.moments = 0            # Tous les moments appliqués au système
        self.vitesse = [V3D()]      # Vitesse de translation
        self.acceleration = [V3D()] # Accéleration - translation 
        self.omega = [0]            # Vitesse de rotation
        self.omegadot = [0]         # Accélération angulaire


    def applyEffort(self, Force=V3D(), Torque=V3D(), pos=V3D()) : 
        """Permet d'appliquer des forces à la barre 2D"""

        #Le point d'application d'une force est imposé par la consigne : [-1,1]
        cdm = V3D()

        # Une barre fixe a une accélération nulle
        if self.fixed == True : 
            return None
        
        # Ajout de l'invariant aux parametres : 
        self.forces += Force

        # Si la force n'est pas appliquée au cdm, alors un moment est crée (on applique BABAR : Mb = MA + BA^R) : 
        if pos != cdm : 
            M_cdm = Torque.z + (pos.x*Force.y) - (pos.y*Force.x)
            self.moments +=M_cdm
        else : 
            self.moments += Torque.z
    
    def simulate(self, step) : 
        """ Application du PFD """

        if self.fixed == True : 
            acceleration = V3D()
            vitesse = V3D()
            omega = V3D().z
            omegadot = V3D().z
            p_angulaire = self.theta[-1]
            p = self.pos[-1]
        
        else : 

            #translation
            acceleration = self.forces*(1/self.mass)
            vitesse = self.vitesse[-1] + acceleration*step
            p = self.pos[-1]+0.5*acceleration*step**2 + self.vitesse[-1]*step

            #rotation 
            J = (1/12)*self.long**2*self.mass
            omegadot = self.moments*(1/J)
            omega = self.omega[-1]+omegadot*step
            p_angulaire = self.theta[-1] + 0.5*omegadot*step**2 + self.omega[-1]*step
            
        #On met à jour nos paramètres
        self.acceleration.append(acceleration)
        self.vitesse.append(vitesse)
        self.pos.append(p)
        self.omega.append(omega)
        self.omegadot.append(omegadot)
        self.theta.append(p_angulaire)

        # réinitialisation
        self.forces = V3D()
        self.moments = 0
    
    def plot(self) : 
        from pylab import plot
        X=[]
        Y=[]
        for p in self.pos:
            X.append(p.x)
            Y.append(p.y)
    
        return plot(X,Y,color=self.color,label=self.nom)+plot(X[-1],Y[-1],'o',color=self.color)   

    def plotRot(self) : 
        from pylab import plot
        return plot(self.theta,color=self.color,label=self.nom)+plot(len(self.theta)-1, self.theta[-1],'o',color=self.color) # len(self.theta)-1 pour dire à plot où placer theta

    def gameDraw(self, scale, screen) : 
        import pygame
        import math

        #on récupère la position et l'angle
        p = self.pos[-1]
        angle = self.theta[-1]

        # calcul des extremites de la barre 2D
        x_haut = p.x + (self.long/2) * math.cos(angle)
        y_haut = p.y + (self.long/2) * math.sin(angle)
        x_bas = p.x - (self.long/2) * math.cos(angle)
        y_bas = p.y - (self.long/2) * math.sin(angle)

        #conversion en pixel 
        p_1 = (x_haut*scale, y_haut*scale)
        p_2 = (x_bas*scale, y_bas*scale)
        
        # dessin de la barre
        pygame.draw.line(screen, self.color, p_1, p_2, 5)
        

class Force : 
    """ Classe permettant de définir une force - reprise du cours """
    
    def __init__(self,force=V3D(), moment=V3D(), pos=V3D(),name='force',active=True): #adaptation : ajout des moments
        self.force = force
        self.moment = moment 
        self.pos = pos
        self.name = name
        self.active = active
        
    def __str__(self):
        return "Force ("+str(self.force)+',' +str(self.moment)+', '+str(self.pos)+', '+self.name+")"
        
    def __repr__(self):
        return str(self)

    def setForce(self,barre2D):
        if self.active:
            barre2D.applyEffort(self.force, self.moment, self.pos)

class Gravity(Force) : 
    """ Force de gravité - reprise du cours"""

    def __init__(self,g=V3D(0,-9.8),name='gravity',active=True):
        self.g = g
        self.name = name
        self.active = active

    def setForce(self,barre2D):
        if self.active:
            barre2D.applyEffort(Force = self.g*barre2D.mass, Torque=V3D(), pos=V3D()) # en effet, le poids s'applique au centre de masse

    def forceSelect(self, barre2D, position_souris, k=10) : 
        """ Calcule la force et le point d'application à partir de la souris avec k raideur du ressort"""

        self.force += (position_souris - barre2D.pos[-1])*k #force de rappel
        self.pos = V3D(0,1) #valeur arbitraire choisie au hasard

class SpringDumper(Force) : 
    """ définition ressort, reprise du cours, translation """
    def __init__(self,P0,P1,k=0,c=0,l0=0,active=True,name="spring_and_damper", pos0=V3D(), pos1=V3D()):
        Force.__init__(self,V3D(),name,active)
        self.k = k
        self.c = c
        self.P0 = P0
        self.P1 = P1
        self.l0 = l0
        self.pos0 = pos0 #point d'application sur P0
        self.pos1 = pos1 #point d'application sur P1

    def setForce(self, barre2D):

        #récupération des positions 
        p0_pos = self.P0.pos[-1]
        p1_pos = self.P1.pos[-1]

        vec_dir = p1_pos - p0_pos
        v_n = vec_dir.mod() #debug
        if v_n == 0 : 
            return
        v_unit = vec_dir * (1/v_n)
        flex = vec_dir.mod()-self.l0
        
        vit = self.P1.vitesse[-1] - self.P0.vitesse[-1]
        vit_n = (vit.x * v_unit.x + vit.y* v_unit.y) * self.c 
        
        force = (self.k * flex + vit_n)* v_unit

        if barre2D == self.P0:
            barre2D.applyEffort(Force=force, pos=self.pos0)
        elif barre2D == self.P1:
            barre2D.applyEffort(Force=-force, pos=self.pos1)
        else:
            pass

class TorsionSpringDumper(Force) : 
    """ Force de rappel d'un ressort - pas selon les distances mais selon les angles cette fois, on ne s'occupe plus de la distance"""

    def __init__(self,P0,P1,k_rot=2,c_rot=1,active=True,name="torsion_spring_and_damper"):
        Force.__init__(self,V3D(),name,active)
        self.k = k_rot
        self.c = c_rot # coefficient d'amortissement
        self.P0 = P0
        self.P1 = P1

    def setForce(self, barre2D):

        #On commence par définir la différence entre l'angle de repos et l'angle actuel : 
        delta_theta = self.P1.theta[-1] - self.P0.theta[-1]

        # On définit la vitesse angulaire : 
        omega = self.P1.omega[-1] - self.P0.omega[-1]

        # On calcule le moment avec la loi de Hooke en torsion : 
        gamma = -self.k_rot*delta_theta

        # on calcule le moment d'amortissement : 
        moment_amor = -self.c_rot*omega

        #on applique les efforts aux deux barres : 
        if barre2D == self.P0:
            barre2D.applyEffort(Torque=V3D(0,0,-(gamma+moment_amor)))
        elif barre2D == self.P1:
            barre2D.applyEffort(Torque=-V3D(0,0,-(gamma+moment_amor)))
        else:
            pass

    
class Liaison : 
    def pivot(barre1, barre2, pos1=V3D(), pos2=V3D(), k=5000, c=2, l0=3) :
        return SpringDumper(barre1, barre2, k, c, l0, pos1, pos2)
    
    def prismatique(barre1, barre2, distance_gliss=V3D(), k_rot=10000, c_rot=10) :

        #on bloque la rotation
        return TorsionSpringDumper(barre1, barre2, k_rot, c_rot)
         





