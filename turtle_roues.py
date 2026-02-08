from turtlebot import Turtle
from vector3D import Vector3D as V3D
from turtlebot import Univers

class TurtleRoue(Turtle) : 
    """ Classe permettant de simuler des turtlesbot avec des roues et non vitesse de translation et de rotation directement. """

    def __init__(self, position=V3D(10,10), rotation=0, name='TurtleRoue', color="red", R = 0.2, L = 0.5):

         # On récupère les paramètres de la classe turtlebot
        super().__init__(position, rotation, name, color)
 
        self.R = R # Rayon des roues
        self.L = L # demi distance entre les roues

        #vitesse de rotation des roues
        self.vitesse_roue_haut_gauche = 0
        self.vitesse_roue_bas_gauche = 0
        self.vitesse_roue_haut_droite = 0
        self.vitesse_roue_bas_droite = 0

    def __str__(self) : 

        msg_classe_parent = super().__str__() #On récupère l'affichage de la classe turtlebot
        msg = f"{msg_classe_parent}\n, le rayon des roues est {self.R}, la distance entre le centre et les roues est {self.L}, la vitesse de la roue haut gauche est {self.vitesse_roue_haut_gauche}, la vitesse de la roue bas gauche est {self.vitesse_roue_bas_gauche}, la vitesse de la roue haut droite est {self.vitesse_roue_haut_droite} et la vitesse de la roue bas droite est {self.vitesse_roue_bas_droite}"

        return msg
    
    def __repr__(self) :
        return str(self)
    
    def move(self, step): # on l'appelle move pour écraser move de la classe parent pour bien fonctionner avec univers

        #On calcule la vitesse de rotation de chaque côté
        omega_g = (self.vitesse_roue_haut_gauche + self.vitesse_roue_bas_gauche)/2.0
        omega_d = (self.vitesse_roue_haut_droite + self.vitesse_roue_bas_droite)/2.0

        # on défini les vitesses de translation et de rotation
        v = ((self.R)/2) * (omega_d + omega_g) # translation
        omega = (self.R/(2*self.L)) * (omega_d - omega_g) # rotation
        
        #Mise à jour des paramètres de la classe turtle bot
        self.speedTrans = v
        self.speedRot = omega

        # puis on utilise la fonction move de la classe turtle bot pour faire le déplacement
        super().move(step)

if __name__ == "__main__":

    from pylab import figure, legend, title, show

    # On crée l'univers
    monUnivers = Univers(game=True)

    #on crée une turtle à roue
    tortue_roue = TurtleRoue()

    #On lui donne des vitesses de rotation des roues constantes dans un premier temps
    # choix : pour l'instant on a mis toutes les vitesses = 
    tortue_roue.vitesse_roue_haut_gauche = 4
    tortue_roue.vitesse_roue_bas_gauche = 4
    tortue_roue.vitesse_roue_haut_droite = 8
    tortue_roue.vitesse_roue_bas_droite = 8

    #ON ajout ma tortue à l'univers
    monUnivers.addUnit(tortue_roue)

    #On lance la simulation
    t = 0
    step = 0.01 
    temps = []
    while t < 10: 
        t += step
        temps.append(t)
        tortue_roue.move(step)

    figure()
    tortue_roue.plot()
    legend()
    title("Trajectoire de la tortue à roues")
    show()

    #simulation pygame : 
    monUnivers.simulateRealTime()
    
    monUnivers.plot()