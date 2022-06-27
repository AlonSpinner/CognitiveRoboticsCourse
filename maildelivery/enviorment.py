import maildelivery.plotting as plotting
from maildelivery.objects import landmark, package
import matplotlib.pyplot as plt

class enviorment:    
    def __init__(self,landmarks,connectivityList, packages = None):
        #instance attributes
        self.landmarks : list[landmark]  = sorted(landmarks)
        self.connectivityList : list[(int,int)]  = connectivityList
        self.packages : list[package] = packages

    def plot(self,ax : plt.Axes = None):
        if ax == None:
            fig , ax = plotting.spawnWorld()
        for lm in self.landmarks:
            if lm.type == "house":
                plotting.plot_house(ax,lm)
            elif lm.type == "dock":
                plotting.plot_dock(ax,lm)
            else:
                plotting.plot_intersection(ax,lm)    
        for c in self.connectivityList:
            lm1 = self.landmarks[c[0]]
            lm2 = self.landmarks[c[1]]
            plotting.plot_road(ax,lm1,lm2)
        return ax

    def find_adjacent(self,lm : landmark):
        adjacent = []
        for c in self.connectivityList:
            if lm.id in c:
                i = 1 - c.index(lm.id)
                adjacent.append(c[i])
        return adjacent





    

    
