import matplotlib
import matplotlib.pyplot as plt
plt.ion()
import numpy as np
import time

class Obj:
    def __init__(self,ID, pos, length, depth):
        self.ID = ID
        self.pos = pos
        self.length = length
        self.depth = depth        

class Map:
    def __init__(self, Lx, Ly):
        self.Lx = Lx
        self.Ly = Ly
        self.obj = []
        self.fig = None        
    
    def draw(self,objs = {}):
        car_length = 400
        car_width = 300
        if len(objs) == 0:
            plt.cla()
            plt.xlim(self.Lx)
            plt.ylim(self.Ly)
            return False
        if self.fig is None:
            #self.fig, ax = plt.subplots(figsize = (9,6))
            #print("sdfgsdfgds")
            self.fig = plt.figure(figsize = (5, 5*(self.Ly[1]-self.Ly[0])/(self.Lx[1]-self.Lx[0])))            
            
            #plt.show(block=False)
            #plt.close()
        else:
            plt.cla()
        for k, obj in objs.items():
            #print(obj.ID, obj.pos, obj.length, obj.depth)
            #plt.plot(obj.pos[0],obj.pos[1], 'ro')
            if obj.ID == 11: # robot position
                rec_corner = [obj.pos[0]+30*np.cos(obj.pos[2]+np.pi)+(car_width/2)*np.cos(obj.pos[2]-np.pi/2),
                      obj.pos[1]+30*np.sin(obj.pos[2]+np.pi)+(car_width/2)*np.sin(obj.pos[2]-np.pi/2)]
                plt.gca().add_patch(plt.Rectangle(rec_corner, car_length, car_width, np.degrees(obj.pos[2]), facecolor='w', edgecolor='b'))
                plt.arrow(obj.pos[0], obj.pos[1], np.cos(obj.pos[2]), np.sin(obj.pos[2]), color='r', width=30)
            elif obj.ID < 11: # Obstacles
                rec_corner = [obj.pos[0]+obj.length/2*np.cos(obj.pos[2]+np.pi)+(obj.depth/2)*np.cos(obj.pos[2]-np.pi/2),
                      obj.pos[1]+obj.length/2*np.sin(obj.pos[2]+np.pi)+(obj.depth/2)*np.sin(obj.pos[2]-np.pi/2)]
                plt.gca().add_patch(plt.Rectangle(rec_corner, obj.length, obj.depth, np.degrees(obj.pos[2]), color = 'k'))
                plt.arrow(obj.pos[0], obj.pos[1], np.cos(obj.pos[2]), np.sin(obj.pos[2]), color='r', width=30)
            else: # Destination
                rec_corner = [obj.pos[0]+obj.length/2*np.cos(obj.pos[2]+np.pi)+(obj.depth/2)*np.cos(obj.pos[2]-np.pi/2),
                      obj.pos[1]+obj.length/2*np.sin(obj.pos[2]+np.pi)+(obj.depth/2)*np.sin(obj.pos[2]-np.pi/2)]
                plt.gca().add_patch(plt.Rectangle(rec_corner, obj.length, obj.depth, np.degrees(obj.pos[2]), color = 'b'))
                plt.arrow(obj.pos[0], obj.pos[1], np.cos(obj.pos[2]), np.sin(obj.pos[2]), color='r', width=30)
                
        plt.xlim(self.Lx)
        plt.ylim(self.Ly)
        self.fig.canvas.flush_events()
        #self.fig.canvas.draw()
        

def goalFromShelf(shelf, dis):
    rec_corner = [shelf.pos[0]+dis*np.cos(shelf.pos[2]+np.pi)-(shelf.depth/2 + 200)*np.cos(shelf.pos[2]-np.pi/2),
                      shelf.pos[1]+dis*np.sin(shelf.pos[2]+np.pi)-(shelf.depth/2 + 200)*np.sin(shelf.pos[2]-np.pi/2)]    
    return [rec_corner[0],rec_corner[1],shelf.pos[2]]

#s1 = Obj(11,[100,100,np.pi/2],300,300)
#s2 = Obj(1,[0,0,np.pi],1000,300)
#s3 = Obj(11,goalFromShelf(s2,s2.length),300,300)
#o = {}
#m = Map([-1500,1500],[-1000,1000])
#o[s1.ID] = s1
#m.draw(o)
#time.sleep(1)
#o[s2.ID] = s2
#m.draw(o)
#time.sleep(1)
#o[s3.ID] = s3
#m.draw(o)
#import math
#curPos = [-1699,-254,1.11]
#desPos = [-682.43,612,-3.11]
#direction = math.atan2(desPos[1] - curPos[1], desPos[0] - curPos[0])


