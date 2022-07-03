from dataclasses import dataclass
import matplotlib.pyplot as plt
import numpy as np

TEXT_OFFSET = 0.03

@dataclass(frozen = True, order = True) #ordered so they can be sorted!
class location:
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
    owner : int #id of owner
    owner_type : str  #'location' or 'robot'
    goal : int  #id of location
    deliverytime : float
    xy : np.ndarray((2))
    graphics : list = None

    def plot(self,ax): #this is a moving object.. so we need a method to replot it
        if self.graphics is not None:
            [g.remove() for g in self.graphics]
        if self.owner == self.goal and self.owner_type == 'location':
            self.graphics = plot_package(ax,self,color = 'green')
        else:
            self.graphics = plot_package(ax,self,color = 'orange')

class enviorment:    
    def __init__(self,locations, connectivityList, packages = None):
        #instance attributes
        self.locations : list[location]  = locations #must be a sorted list
        self.connectivityList : list[(int,int)]  = connectivityList
        self.packages : list[package] = packages

    def find_adjacent(self,lm : location):
        adjacent = []
        for c in self.connectivityList:
            if lm.id in c:
                i = 1 - c.index(lm.id)
                adjacent.append(c[i])
        return adjacent

    def plot(self, ax : plt.Axes = None):
        if ax is None:
            fig, ax = plot_spawnWorld(xrange = None, yrange = None)
        else:
            fig = ax.get_figure()
            
        for loc in self.locations:
            if loc.type == "house" or loc.type == "station":
                plot_house(ax,loc)
            elif loc.type == "dock":
                plot_dock(ax,loc)
            else:
                plot_intersection(ax,loc)    
        for c in self.connectivityList:
            loc1 = self.locations[c[0]]
            loc2 = self.locations[c[1]]
            plot_road(ax,loc1,loc2)
        return fig, ax

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

def plot_road(ax : plt.Axes,loc1 : location, loc2 : location):
    x = loc1.xy[0], loc2.xy[0]
    y = loc1.xy[1], loc2.xy[1]
    return ax.plot(x,y,color = 'k')

def plot_intersection(ax: plt.Axes,x : location, markerShape = 'x', markerSize = 50, color = 'r'):
    g1 = ax.scatter(x.xy[0],x.xy[1], marker = markerShape, c = color, s = markerSize)
    g2 = ax.text(x.xy[0]+TEXT_OFFSET,x.xy[1]+TEXT_OFFSET,x.id, color = color)
    return [g1,g2]

def plot_house(ax: plt.Axes, h :location, markerShape = 's', markerSize = 80, color = 'k'):
    g1 = ax.scatter(h.xy[0],h.xy[1], marker = markerShape, edgecolors = color, s = markerSize)
    g1.set_facecolor('none')
    g2 = ax.text(h.xy[0]+TEXT_OFFSET,h.xy[1]+TEXT_OFFSET,h.id, color = color)
    return [g1,g2]

def plot_dock(ax: plt.Axes, h :location, markerShape = 's', markerSize = 80, color = 'b'):
    g1 = ax.scatter(h.xy[0],h.xy[1], marker = markerShape, edgecolors = color, s = markerSize)
    g1.set_facecolor('none')
    g2 = ax.text(h.xy[0]+TEXT_OFFSET,h.xy[1]+TEXT_OFFSET,h.id, color = color)
    return [g1,g2]

def plot_package(ax: plt.Axes, p :package, markerShape = 'o', markerSize = 30, color = 'orange'):
    g1 = ax.scatter(p.xy[0],p.xy[1], marker = markerShape, color = color, edgecolors = color, s = markerSize)
    g2 = ax.text(p.xy[0]-2*TEXT_OFFSET,p.xy[1]+TEXT_OFFSET,p.id, color = color)
    return [g1,g2] #attach graphics to package





    

    
