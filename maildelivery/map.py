import maildelivery.plotting as plotting
from maildelivery.datatypes import beacon, landmark
import matplotlib.pyplot as plt

class Map:    
    def __init__(self,beacons,landmarks,connectivityList):
        #instance attributes
        self.beacons : list[beacon] = beacons
        self.landmarks : list[landmark]  = landmarks
        self.connectivityList : list[(int,int)]  = connectivityList

    def plot(self,ax : plt.Axes = None):
        if ax == None:
            fig , ax = plotting.spawnWorld()

        for b in self.beacons:
            plotting.plot_beacon(ax,b)
        for lm in self.landmarks:
            if lm.type == "house":
                plotting.plot_house(ax,lm)
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
                c.remove(lm.id)
                adjacent.append(c[0])
        return adjacent




    

    
