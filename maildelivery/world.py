from dataclasses import dataclass
import matplotlib.pyplot as plt
import numpy as np

TEXT_OFFSET = 0.03

@dataclass(frozen = True, order = True) #ordered so they can be sorted!
class landmark:
    id : int
    xy : np.ndarray((2))
    type : str

    def angle(lm1,lm2): #akeen to lm2 - lm1
        dy = lm2.xy[1]-lm1.xy[1]
        dx = lm2.xy[0]-lm1.xy[0]
        return np.arctan2(dy,dx)

    def distance(lm1,lm2):
        dy = lm2.xy[1]-lm1.xy[1]
        dx = lm2.xy[0]-lm1.xy[0]
        return (dx**2 + dy**2)**0.5

@dataclass(frozen = False, order = True) #NOT FROZEN
class package:
    id : int
    owner : int #landmark id  or robot == ROBOT_INDEX_SHIFT + robot id
    goal : int 
    deliverytime : float
    xy : np.ndarray((2))
    graphics : list = None

    def plot(self,ax): #this is a moving object.. so we need a method to replot it
        if self.graphics is not None:
            [g.remove() for g in self.graphics] 
        self.graphics = plot_package(ax,self)

class enviorment:    
    def __init__(self,landmarks, connectivityList, packages = None):
        #instance attributes
        self.landmarks : list[landmark]  = landmarks #must be a sorted list
        self.connectivityList : list[(int,int)]  = connectivityList
        self.packages : list[package] = packages

    def find_adjacent(self,lm : landmark):
        adjacent = []
        for c in self.connectivityList:
            if lm.id in c:
                i = 1 - c.index(lm.id)
                adjacent.append(c[i])
        return adjacent

    def plot(self, ax : plt.Axes = None):
        if ax is None:
            _, ax = plot_spawnWorld(xrange = None, yrange = None)
        for lm in self.landmarks:
            if lm.type == "house":
                plot_house(ax,lm)
            elif lm.type == "dock":
                plot_dock(ax,lm)
            else:
                plot_intersection(ax,lm)    
        for c in self.connectivityList:
            lm1 = self.landmarks[c[0]]
            lm2 = self.landmarks[c[1]]
            plot_road(ax,lm1,lm2)
        return ax

#---------------------------------------------------------------------------
#--------------------------------PLOTTING FUNCTIONS-------------------------
#---------------------------------------------------------------------------

def plot_spawnWorld(xrange = None, yrange = None):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_xlabel('x'); ax.set_ylabel('y'); 
    ax.set_aspect('equal'); ax.grid()

    if xrange is not None: ax.set_xlim(xrange)
    if yrange is not None: ax.set_ylim(yrange)

    return fig, ax

def plot_road(ax : plt.Axes,lm1 : landmark, lm2 : landmark):
    x = lm1.xy[0], lm2.xy[0]
    y = lm1.xy[1], lm2.xy[1]
    return ax.plot(x,y,color = 'k')

def plot_intersection(ax: plt.Axes,x : landmark, markerShape = 'x', markerSize = 50, color = 'r'):
    g1 = ax.scatter(x.xy[0],x.xy[1], marker = markerShape, c = color, s = markerSize)
    g2 = ax.text(x.xy[0]+TEXT_OFFSET,x.xy[1]+TEXT_OFFSET,x.id, color = color)
    return [g1,g2]

def plot_house(ax: plt.Axes, h :landmark, markerShape = 's', markerSize = 80, color = 'k'):
    g1 = ax.scatter(h.xy[0],h.xy[1], marker = markerShape, edgecolors = color, s = markerSize)
    g1.set_facecolor('none')
    g2 = ax.text(h.xy[0]+TEXT_OFFSET,h.xy[1]+TEXT_OFFSET,h.id, color = color)
    return [g1,g2]

def plot_dock(ax: plt.Axes, h :landmark, markerShape = 's', markerSize = 80, color = 'b'):
    g1 = ax.scatter(h.xy[0],h.xy[1], marker = markerShape, edgecolors = color, s = markerSize)
    g1.set_facecolor('none')
    g2 = ax.text(h.xy[0]+TEXT_OFFSET,h.xy[1]+TEXT_OFFSET,h.id, color = color)
    return [g1,g2]

def plot_package(ax: plt.Axes, p :package, markerShape = 'o', markerSize = 30, color = 'orange'):
    g1 = ax.scatter(p.xy[0],p.xy[1], marker = markerShape, color = color, edgecolors = color, s = markerSize)
    g2 = ax.text(p.xy[0]+TEXT_OFFSET,p.xy[1]+TEXT_OFFSET,p.id, color = color)
    return [g1,g2] #attach graphics to package





    

    
