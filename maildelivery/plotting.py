import numpy as np
import matplotlib.pyplot as plt
from maildelivery.enviorment import landmark, package
import gtsam

def spawnWorld(xrange = None, yrange = None):
    
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
    graphics = []
    graphics.append(ax.scatter(x.xy[0],x.xy[1], marker = markerShape, c = color, s = markerSize))
    graphics.append(ax.text(x.xy[0],x.xy[1],x.id, color = color))
    return graphics

def plot_house(ax: plt.Axes, h :landmark, markerShape = 's', markerSize = 80, color = 'k'):
    graphics = []
    g = ax.scatter(h.xy[0],h.xy[1], marker = markerShape, edgecolors = color, s = markerSize)
    g.set_facecolor('none')
    graphics.append(g)
    graphics.append(ax.text(h.xy[0],h.xy[1],h.id, color = color))
    return graphics

def plot_dock(ax: plt.Axes, h :landmark, markerShape = 's', markerSize = 80, color = 'b'):
    graphics = []
    g = ax.scatter(h.xy[0],h.xy[1], marker = markerShape, edgecolors = color, s = markerSize)
    g.set_facecolor('none')
    graphics.append(g)
    graphics.append(ax.text(h.xy[0],h.xy[1],h.id, color = color))
    return graphics

def plot_robot(ax , pose : gtsam.Pose2, scale = 20, color = 'b'):
        u = np.cos(pose.theta())
        v = np.sin(pose.theta())
        graphics_quiver = ax.quiver(pose.x(),pose.y(),u,v, color = color, scale = scale, width = 0.02)
        graphics_circle = ax.add_patch(plt.Circle((pose.x(),pose.y()),0.1,fill = False, color = 'b'))
        return [graphics_quiver,graphics_circle]

# def plot_package(ax, p : package):
#         ax.scatter(p.)