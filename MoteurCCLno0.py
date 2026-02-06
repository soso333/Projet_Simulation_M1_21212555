# Projet sim propre
from matplotlib import pylab
import numpy as np


class MoteurCC_2 : 
    """ Simulateur d'un moteur à courant continu sans l'approximation L = 0 """
    
    def __init__(self, R=1, L=0.01, kc=0.01, ke=0.01, J = 0.01, f=0.1 ):
        
        # Grandeur connue 
        self.R = R              # Résistance de l'induit
        self.L = L              # Inductance de l'induit
        self.kc = kc            # Constante de couple
        self.ke = ke            # Constante de la fcem
        self.J = J              # Inertie du rotor
        self.f = f              # Constante de frottement visqueux
        
        # caractéristiques qu'on ne connait que plus tard grâce aux pochaines fonctions :
        self.um = 0             # Tension aux bornes du moteur - donné par la fonction set voltage
        self.E = [0]            # La force contre éléctromagnétique : E(t) = ke*Ω(t)
        self.i = [0]            # Le courant  
        self.gamma = [0]        # Le couple moteur - Γ(t) = kc*i(t)
        self.omega = [0]        # La  vitesse - Ω(t), intégration numérique
        self.domega = [0]       # L'accélération - dΩ(t)/dt, intégration numérique
        self.position = [0]     # la liste des positions, les positions sont caractérisées par les angles car le moteur ne se déplace pas dans l'espace, l'axe tourne
        
        # Grandeur ajoutée 
        self.charge = 0         # Inertie ajoutée au moteur
        self.couple_ext = 0     # Couple extérieur
        self.viscosite = 0      # Force de frottement supplémentaire
        self.couple_res = 0     # Couple résistif

        
    def __str__(self):
        """ Affichage des paramètres donnés du moteur à courant continu """
        
        msg = ("Moteur CC\n"
        f"Paramètres électriques et mécaniques :\n"
        f"R = {self.R} Ω, L = {self.L} H\n",
        f"    kc = {self.kc} N.m/A, ke = {self.ke} V.s/rad\n"
        f"    J = {self.J} kg.m^2, f = {self.f} N.m.s/rad\n"
        f"  Caractéristiques:\n"
        f"    Um = {self.um} V\n"
        f"    i  = {self.i[-1]} A\n"
        f"    E  = {self.E[-1]} V\n"
        f"    Γ  = {self.gamma[-1]} N.m\n"
        f"    Ω  = {self.omega[-1]} rad/s\n"
        f"    θ  = {self.position[-1]} rad\n"
        f"  Autres grandeurs:\n"
        f"    charge = {self.charge} kg.m^2\n"
        f"    couple_ext = {self.couple_ext} N.m\n"
        f"    viscosite = {self.viscosite} N.m.s/rad\n"
        f"    couple_res = {self.couple_res} N.m"
    )
        return msg
        
    def __repr__(self) : 
        return str(self)
    
    def setVoltage(self, V) : 
        """ Permet de définir un voltage donc la tension aux bornes du moteur"""
        
        if isinstance(V, (int, float)):  
            self.um = V
        else : 
            raise TypeError("La tension doit être soit un entier soit un réel")
            
    def getPosition(self) : 
        """ Permet de récupérer la dernière position de notre système """
        return self.position[-1] #[-1] pour l'historique des positions
    
    def getSpeed(self) : 
        """ Permet de récupérer la dernière vitesse du rotor """
        return self.omega[-1]
    
    def getTorque(self) : 
        """ Permet de récupérer le dernier couple moteur torque = moment """
        
        return self.gamma[-1]
    
    def getIntensity(self) : 
        """ Permet de récupérer les valeurs de courant """
        
        return self.i[-1]
    
    def f1(self, x1, x2) :
        """ Permet de faire la méthode de RK2 """ 
        return x2
    
    def f2(self, x1, x2, um) :
        """ Permet de faire la méthode de RK2 """
        return (um*self.kc - (self.L*self.f + self.R*self.J)*x2 - (self.R*self.f + self.kc*self.ke)*x1) / (self.L*self.J)

    def simule(self, step) : # remarque : on appelle simule à chaque itération dans le main - alors simule ne fait qu'une seule itération de la méthode d'euler
        """ Permet de simuler le moteur à CC """
        
        # On commence par définir les états : 
        x1 = self.omega[-1] # la vitesse à l'instant t
        x2 = self.domega[-1] # l'accélération à l'instant t
        um = self.um # la tension à l'instant t

        # étape 1, définition du système 
        x_point_1 = self.f1(x1, x2)
        x_point_2 = self.f2(x1, x2, um)

        # étape 2, euler explicite 
        x1_bar = x1 + step*x_point_1
        x2_bar = x2 + step*x_point_2

        # étape 3 : correction des dérivées d'Euler : 
        x_point_1_bar = self.f1(x1_bar, x2_bar)
        x_point_2_bar = self.f2(x1_bar, x2_bar, um)

        # étape 4 : on passe à l'itération suivante 
        x1_n_plus1 = x1 + (step/2)*(x_point_1 + x_point_1_bar)
        x2_n_plus1 = x2 + (step/2)*(x_point_2 + x_point_2_bar)  

        # mise à jour des variables d'état : 
        self.omega.append(x1_n_plus1)
        self.domega.append(x2_n_plus1)

        # mise à jourdes autres paramètres : 
        # position : intégration de la vitesse

        pos = self.getPosition()
        nouvelle_position = pos + (step/2)*(x1 +x1_n_plus1)
        self.position.append(nouvelle_position)  
        
        
    def plot(self, temps, omega, color):
        """ permet de tracer, directement importée du cours """
        
        from pylab import plot
        return plot(temps, omega, color)  
    
    # Pour comparer nos résultats à celle de la solution analytique : 
    def solution_analytique(self, temps) : 
        """ Permet de calculer la solution analytique de la vitesse du moteur à courant continu en boucle ouverte """
    
        C = self.L*self.f+self.R*self.J
        D = self.R*self.f + self.kc*self.ke
        lambda_1 = (-C + np.sqrt(C**2 - 4*self.L*self.J*D))/(2*self.L*self.J)
        lambda_2 = (-C - np.sqrt(C**2 - 4*self.L*self.J*D))/(2*self.L*self.J) 
       
        omega = (self.kc*lambda_2*self.um)/(D*(lambda_1-lambda_2))*np.exp(lambda_1*temps) + (self.kc*lambda_1*self.um)/(D*(lambda_2-lambda_1))*np.exp(lambda_2*temps) + (self.kc*self.um)/D
        return omega
        
       
# Début du main : 
    
    
if __name__=='__main__':
    from pylab import figure, show, legend, title
    #boucle ouverte en vitesse
    m = MoteurCC_2()
    t = 0
    step = 0.01
    temps = [t]
    while t<2:
        t=t+step
        temps.append(t)
        m.setVoltage(1)
        m.simule(step)
    figure()
    m.plot(temps, m.omega, color = 'red')
    legend()
    title("Réponse indicielle du moteur CC en boucle ouverte")

    #On trace également la solution analytique pour comparer
    figure()
    legend()
    title("Solution analytique")
    omega_analytique = m.solution_analytique(np.array(temps))
    m.plot(temps, omega_analytique, color = 'blue')

    #Comparaison des deux solutions (numérique et analytique)
    figure()
    legend()
    title("Comparaison des solutions numérique et analytique")
    m.plot(temps, m.omega, color = 'red')
    m.plot(temps, omega_analytique, color = 'blue')
    
    show()
