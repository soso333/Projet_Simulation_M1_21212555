from turtlebot import Turtle
from vector3D import Vector3D as V3D
from turtlebot import Univers
from MoteurCCL0 import MoteurCC
from controleur_PID import ControlPID_vitesse_3

class TurtleRoue(Turtle) : 
    """ Classe permettant de simuler des turtlesbot avec des roues et non vitesse de translation et de rotation directement. """

    def __init__(self, position=V3D(10,10), rotation=0, name='TurtleRoue', color="red", rayon = 0.2, distance = 0.5):

         # On récupère les paramètres de la classe turtlebot
        super().__init__(position, rotation, name, color)
 
        self.rayon = rayon # Rayon des roues
        self.distance = distance # demi distance entre les roues

        # On ne définit plus les vitesses mais les moteursCC pour chaque roue
        self.moteur_haut_gauche = MoteurCC()
        self.moteur_bas_gauche = MoteurCC()
        self.moteur_haut_droit = MoteurCC()
        self.moteur_bas_droit = MoteurCC()

        #vitesse de rotation des roues
        self.vitesse_roue_haut_gauche = 0
        self.vitesse_roue_bas_gauche = 0
        self.vitesse_roue_haut_droite = 0
        self.vitesse_roue_bas_droite = 0


    def __str__(self) : 

        msg_classe_parent = super().__str__() #On récupère l'affichage de la classe turtlebot
        msg = f"{msg_classe_parent}\n, le rayon des roues est {self.rayon}, la distance entre le centre et les roues est {self.distance}"
        return msg
    
    def __repr__(self) :
        return str(self)
    
    def set_voltage(self, um_gauche_haut, um_droite_haut, um_gauche_bas, um_droite_bas) : 

        #partie gauche
        self.moteur_haut_gauche.setVoltage(um_gauche_haut)
        self.moteur_bas_gauche.setVoltage(um_gauche_bas)

        #partie droite 
        self.moteur_haut_droit.setVoltage(um_droite_haut)
        self.moteur_bas_droit.setVoltage(um_droite_bas)

    def move(self, step): # on l'appelle move pour écraser move de la classe parent pour bien fonctionner avec univers

        # Calcul de la vitesse des moteurs 
        self.moteur_haut_gauche.simule(step)
        self.moteur_bas_gauche.simule(step)
        self.moteur_haut_droit.simule(step)
        self.moteur_bas_droit.simule(step)
        
        #On donne la vitesse de rotation de chaque calculée avec simule : 
        self.vitesse_roue_haut_gauche = self.moteur_haut_gauche.getSpeed()
        self.vitesse_roue_bas_gauche = self.moteur_bas_gauche.getSpeed()
        self.vitesse_roue_haut_droite = self.moteur_haut_droit.getSpeed()
        self.vitesse_roue_bas_droite = self.moteur_bas_droit.getSpeed()

        # Calcul des vitesses de translation et rotation
        omega_g = (self.vitesse_roue_haut_gauche + self.vitesse_roue_bas_gauche)/2.0
        omega_d = (self.vitesse_roue_haut_droite + self.vitesse_roue_bas_droite)/2.0

        # on défini les vitesses de translation et de rotation
        v = ((self.rayon)/2) * (omega_d + omega_g) # translation
        omega = (self.rayon/(2*self.distance)) * (omega_d - omega_g) # rotation
        
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

    # On défini les PID
    P = 10
    I = 500
    D = 0.1

    pid_haut_gauche = ControlPID_vitesse_3(P,I,D, tortue_roue.moteur_haut_gauche)
    pid_bas_gauche = ControlPID_vitesse_3(P,I,D, tortue_roue.moteur_bas_gauche)
    pid_haut_droit = ControlPID_vitesse_3(P,I,D, tortue_roue.moteur_haut_droit)
    pid_bas_droit = ControlPID_vitesse_3(P,I,D, tortue_roue.moteur_bas_droit)

    #definition des vitesses desiree
    vitesse_des_gauche = 50
    vitesse_des_droite = 10

    #ON ajout ma tortue à l'univers
    monUnivers.addUnit(tortue_roue)

    #On lance la simulation
    t = 0
    step = 0.01 
    temps = []
    while t < 10: 
        t += step
        temps.append(t)

        #réglage PID
        pid_haut_gauche.setTarget(vitesse_des_gauche)
        pid_haut_gauche.simule(step)
        
        pid_bas_gauche.setTarget(vitesse_des_gauche)
        pid_bas_gauche.simule(step)

        pid_haut_droit.setTarget(vitesse_des_droite)
        pid_haut_droit.simule(step)

        pid_bas_droit.setTarget(vitesse_des_droite)
        pid_bas_droit.simule(step)

        tortue_roue.move(step)

    figure()
    tortue_roue.plot()
    legend()
    title("Trajectoire de la tortue à roues")
    show()

    #simulation pygame : 
    monUnivers.simulateRealTime()
    
    monUnivers.plot()