#!/usr/bin/env python
#from cmath import nan
from operator import index
from re import I
import sys
import queue
import numpy as np
import matplotlib.pyplot as plt
import math as m


# Read scan files saved in plain text format. Each row is a full scan
# of the Turtlebot LIDAR with the following format:
#
# [time] [no of readings] [reading0] [reading1]...
#
# The function returns a numpy arrray in which each row corresponds to
# a full 360 deg scan and each column corresponds to one direction of
# the robot. In the files the robot was stationary in a static
# environment.
# python splitAndMerge.py scan-files/scan-000.dat 
def read_scan_file(fname):
    print('read_scan_file():', fname)
    D = np.loadtxt(fname)
    scans = D[:,2:]
    return scans



# Segment class to store the endpoints of the segment and their
# corresponding indexes
class Segment:
    def __init__(self, p0, p1, idx0, idx1, es_de_union=0):
        self.p0 = p0
        self.p1 = p1
        self.idx0 = idx0
        self.idx1 = idx1
        #print("INDICES", idx0,idx1)
        self.x = p1[1] - p0[1]
        self.y = p1[0] - p0[0]
        self.origen = (-p0[0]*self.x+p0[1]*self.y)
        self.recta = np.array([self.x,-self.y,self.origen])
        self.alpha = np.arctan2(self.x,self.y)
        self.m=self.x/self.y
        self.union=es_de_union

    def __str__(self):
        return '[(' + str(self.p0[0]) + ',' + str(self.p0[1]) + ',' + str(self.p1[0]) + ',' + str(self.p1[1]) + ')]'
      

    # Computes the length of the segment
    def length(self):
        return np.linalg.norm(np.array(self.p1, np.float32)
                            - np.array(self.p0, np.float32))

    # Compute the distance from the segment to the point p
    def distance(self, p):
        # TODO
        recta = self.recta
        return abs(recta[0] * p[0] + recta[1] * p[1] + recta[2]) / np.linalg.norm(recta[0:2])
        
    # Compute the angle between this segment and segment s
    def angle(self, s):
        # TODO
        # Recordar que la forma de calcular los angulos alpha_i debe ser consistente para los dos 
        # segmentos, i.e. el primer punto es el inicio y el segundo punto es el fin del segmento, 
        # de lo contrario los angulos se calcularan para direcciones opuestas.

        # Es necesario normalizar la diferencia angular al rango (-pi, pi] sumando o restando 2pi
        # dependiendo del valor de la diferencia

        """ang1 = math.atan2((self.p1[1] - self.p0[1]), (self.p1[0] - self.p0[0]))
        ang2 = math.atan2((s.p1[1] - s.p0[1]), (s.p1[0] - s.p0[0]))

        angle = np.abs(ang1 - ang2)"""

        angle = abs(self.alpha - s.alpha)
        while angle > (m.pi):
            angle = angle - m.pi
        return angle


    def write(self):
        print('Segment: ', self.p0, '-', self.p1) 
        print('Segment: ', self.idx0, '-', self.idx1) 

    def __repr__(self):
        return '['+str(self.idx0)+","+str(self.idx1)+']'


class SplitAndMerge:
    def __init__(self, dist=0.1, angle=0.9, purge_pts = 4, purge_len = 0.02):
        self.d_th = dist        # Threshold distance (split)
        self.a_th = angle       # Threshold angle (merge)
        self.pur_pts = purge_pts  # Min number of points (purge)
        self.pur_len = purge_len  # Min segment length (purge)
        self.segmentions_with_idnx=[]
        self.segmentions=[]

    
    def split(self,Pts,segmention=[],idx=False): 
        # TODO


        if idx==False:
            idx=(0,len(Pts))
        segmento = Segment(Pts[0],Pts[-1],idx[0],idx[1])

        #print(segmento)
            
        distancias_todo_los_puntos=np.array([segmento.distance(Pts[i]) for i in range(len(Pts))])
        #print(distancias_todo_los_puntos)
        distancia=np.max(distancias_todo_los_puntos)
        #print(distancia)
        split_point=np.where(distancias_todo_los_puntos==distancia)[0][0]
        ##### print("distancia maxima, split punto:\n",distancia,split_point)
        
        if distancia>self.d_th:
            if split_point >1 :
                self.split(Pts[:split_point],segmention,(idx[0],split_point+idx[0]))
            else:
                segmention.append(Segment(Pts[0],Pts[split_point],idx[0],split_point+idx[0]))

            if idx[1]-split_point+idx[0]>1:
                self.split(Pts[split_point:],segmention,(split_point+idx[0],idx[1]))
            else:
                segmention.append(Segment(Pts[split_point],Pts[-1],split_point+idx[0],idx[1]))
            return segmention
        
        else:
            self.segmentions_with_idnx.append(((idx[0],idx[1]),segmento))
            segmention.append(segmento)
            return segmention
                
    def merge(self, segs_in):
        # TODO
        segmentos=[]
        i=0
        while i<len(segs_in):
            seg1=segs_in[i]
            j=i+1
            if j>=len(segs_in):
                j=0
            seg2=segs_in[j]
            angulo=seg1.angle(seg2)
            if angulo<=self.a_th and j>0:
                while angulo<=self.a_th:
        
                    if j<len(segs_in):
                        seg2=segs_in[j]
                        angulo=seg1.angle(seg2)
                        j+=1
                    else:
                        angulo=0
                seg2=segs_in[j]
                new_seg=Segment(seg1.p0,seg2.p1,seg1.idx0,seg2.idx1)
                segmentos.append(new_seg)
                i=j  
            else:
                if angulo<=self.a_th:
                    new_seg=Segment(seg1.p0,seg2.p1,seg1.idx0,seg2.idx1,es_de_union=seg1.idx1)
                    segmentos.append(new_seg)
                    segmentos=segmentos[1:]
                else:
                    segmentos.append(seg1)
            i+=1
        return segmentos

    def purge(self, segs_in):
        segmentos=[]
        for i in range(len(segs_in)):
            seg1 = segs_in[i]
            num_puntos = seg1.idx1-seg1.idx0
            if num_puntos < 0:
                num_puntos = seg1.union-seg1.idx1+seg1.idx0
                ##### print(num_puntos)
            if num_puntos >= self.pur_pts and seg1.length()>=self.pur_len:
                segmentos.append(seg1)
            else:
                comprobar = seg1.idx1 - seg1.idx0, self.pur_pts, seg1.length(), self.pur_len
                print("idx1-idx0, pur_pts, length, pur_len:", comprobar)

        return segmentos
                
    def __call__(self, Pts):
        pass
        seg0 = self.split(Pts)
        seg1 = self.merge(seg0)
        seg2 = self.purge(seg1)

        return seg0, seg1, seg2
    
    
    def plotSegments(self, segmentList,forma='o',color='r',linestyle='--',linewidth=2,debug=False):
        for s in segmentList:
            x = (s.p0[0], s.p1[0])
            y = (s.p0[1], s.p1[1])
            plt.plot(x, y, marker = forma, color= color,linestyle=linestyle,linewidth=linewidth)
        plt.draw()
        plt.pause(0.001)
        if debug:
            foo = input('Press a key and hit enter to continue')

            
    def plot(self, Pts):
        for ii in range(0, len(Pts)):
            pt = Pts[ii]
            plt.plot(pt[0],pt[1], marker='.', color='k')
        plt.draw()
        plt.pause(0.01)

  
      
if __name__ == '__main__':
    # make sure the filename is given as an argument
    if len(sys.argv) != 2:
        print('Usage: ', sys.argv[0], ' filename\n')
        sys.exit()
        
    # Read the file with the scans an get all of the scans as a matrix
    D = read_scan_file(sys.argv[1])
    
    # index of the scan to process
    idx = 10
    R = D[idx,:]
    ####print(R)
    min_ang = 0.0
    d_ang = 0.00435422640294
    # Convert scan to Cartesian coordinates
    scanCart = []
    for ii in range(len(R)):
        ang = min_ang - ii * d_ang
        if R[ii] < 20.0:
            if ang < -np.pi:
                ang += 2 * np.pi
            r = R[ii]
            pt = np.array([r*np.cos(ang), r*np.sin(ang)])
            scanCart.append(pt)
    #print(scanCart)
    # TODO: Set the parameters to the split and merge algorithm
    sm = SplitAndMerge(dist=0.02, angle=0.02, purge_pts = 4, purge_len = 0.002)
    
    # Call the algorithm
    segments,segments_mod1,segments_mod2 = sm(scanCart)

    #print(segments)
    #print(segments_mod1)
    #print(segments_mod2)
    # Plot the results
    sm.plot(scanCart)
    sm.plotSegments(segments,linestyle = '-', linewidth = 1,debug=True)
    print(len(segments))
    sm.plotSegments(segments_mod1, color = 'b', linestyle = '--',debug=True)
    print(len(segments_mod1))
    sm.plotSegments(segments_mod2, color = 'g', linestyle = '-.', debug = True)
    print(len(segments_mod2))
    #print(sm.segmentions_with_idnx)