# Projet sim propre
from matplotlib import pylab


class MoteurCC : 
    """ Simulateur d'un moteur à courant continu """
    
    def __init__(self, R=1, L=0, kc=0.01, ke=0.01, J = 0.01, f=0.1 ):
        
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
        self.position = [0]     # la liste des positions, les positions sont caractérisées par les angles car le moteur ne se déplace pas dans l'espace, l'axe tourne
        
        # Grandeur ajoutée 
        self.charge = 0         # Inertie ajoutée au moteur
        self.couple_ext = 0     # Couple extérieur
        self.viscosite = 0      # Force de frottement supplémentaire
        self.couple_res = 0     # Couple résistif
        
    def __str__(self): ############# À modifier! 
        """ Affichage des paramètres donnés du moteur à courant continu """
        
        msg = 'Moteur CC de paramètres : ('+str(self.R)+', '+str(self.L)+', '+str(self.kc)+', '+str(self.ke)+', '+str(self.J)+', '+str(self.f)+''
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

    def simule(self, step) : # remarque : on appelle simule à chaque itération dans le main - alors simule ne fait qu'une seule itération de la méthode d'euler
        """ Permet de simuler le moteur à CC """
        
        # On commence par intégrer la vitesse, on possède l'équation différentielle de omega(t), déterminée tout à l'heure : 
            
        # Définition des paramètres de l'équation différentielle
        a = (self.R*self.J)/(self.ke*self.kc+self.R*self.f)
        b = self.kc/(self.ke*self.kc+self.R*self.f)
        
        # Résolution d'une équation différentielle numérique - Méthode d'euler
        # https://femto-physique.fr/analyse-numerique/euler.php
        
        
        omega = self.getSpeed() #On récupère la dernière vitesse 
        
        # definition de l'équation différentielle sous la bonne forme
        d_omega = (1/a)*(b*self.um-omega)
        
        # application d'euler : 
        nouv_omega = omega + step*d_omega 
        self.omega.append(nouv_omega) # update de la vitesse
        
        # update des autres paramètres : 
            #position : intégration de la vitesse
        pos = self.getPosition()
        nouv_position = pos + step*nouv_omega
        self.position.append(nouv_position)
        
        # maintenant on met à jour E(t) qui dépend de omega(t), i(t) qui dépend aussi de omega(t) et couple moteur gamma : 
        self.E.append(self.ke*nouv_omega)
        
        nouv_i = (-nouv_omega*self.ke+self.um)/self.R
        self.i.append(nouv_i)
        self.gamma.append(self.kc*nouv_i)
        
        
        
    def plot(self, temps,):
        """ permet de tracer, directement importée du cours """
        
        from pylab import plot
        return plot(temps, self.omega, color = 'red')  
        
       
# Début du main : 
    
    
if __name__=='__main__':
    from pylab import figure, show, legend, title
    #boucle ouverte en vitesse
    m = MoteurCC()
    t = 0
    step = 0.01
    temps = [t]
    while t<2 :
        t=t+step
        temps.append(t)
        m.setVoltage(1)
        m.simule(step)
    figure()
    m.plot(temps)
    legend()
    title("Réponse indicielle du moteur CC en boucle ouverte")
    show()
        
        



    
    
    
        







#exo 2 : c'est la force centrifuge






