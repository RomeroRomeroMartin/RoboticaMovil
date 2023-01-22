#!/usr/bin/env python

import sys
import queue
import numpy as np
import matplotlib.pyplot as plt
import math 

# Read scan files saved in plain text format. Each row is a full scan
# of the Turtlebot LIDAR with the following format:
#
# [time] [no of readings] [reading0] [reading1]...
#
# The function returns a numpy arrray in which each row corresponds to
# a full 360 deg scan and each column corresponds to one direction of
# the robot. In the files the robot was stationary in a static
# environment.
def read_scan_file(fname):
    print('read_scan_file():', fname)
    D = np.loadtxt(fname)
    scans = D[:,2:]
    return scans



# Segment class to store the endpoints of the segment and their
# corresponding indexes
class Segment:
    def __init__(self, p0, p1, idx0, idx1):
        self.p0 = p0
        self.p1 = p1
        self.idx0 = idx0
        self.idx1 = idx1
    def __str__(self):
        return '[('+str(self.p0[0])+','+str(self.p0[1])+'),('+str(self.p1[0])+','+str(self.p1[1])+')]'

   # Computes the length of the segment
    def length(self):
        return np.linalg.norm(np.array(self.p1, np.float32)
                                - np.array(self.p0, np.float32))

   # Compute the distance from the segment to the point p
    def distance(self,P,Ps,Pe):
        # TODO
        #dist=((self.p1[0]-self.p0[0])*(p[1]-self.p0[1])-(self.p1[1]-self.p0[0])*(p[0]-self.p0[0]))/np.sqrt((self.p1[0]-self.p0[0])**2+(self.p1[1]-self.p0[1])**2)
        #return np.abs(dist)
        if np.all(np.equal(Ps,Pe)):
            return np.linalg.norm(P-Ps)
        else:
	        return np.divide(np.abs(np.linalg.norm(np.cross(Pe-Ps,Ps-P))),np.linalg.norm(Pe-Ps))
    # Compute the angle between this segment and segment s
    def angle(self, s):
        a1=math.atan2((self.p1[1]-self.p0[1]),(self.p1[0]-self.p0[0]))
        a2=math.atan2((s.p1[1]-s.p0[1]),(s.p1[0]-s.p0[0]))
        a=np.abs(a1-a2)
        # TODO
        return a

    def pto_alejado(self,P):
        max=0
        ind=-1
        for i in range(1,P.shape[0]):
            d=self.distance(self,P[i,:],P[0,:],P[-1,:])
            if d>max:
                ind=i
                max=d 
        return max,ind


    def write(self):
        print('Segment: ', self.p0, '-', self.p1) 
        print('Segment: ', self.idx0, '-', self.idx1) 

class SplitAndMerge:
    def __init__(self, dist=0.1, angle=1.15, purge_pts = 8, purge_len = 0.1):
        self.d_th = dist        # Threshold distance (split)
        self.a_th = angle       # Threshold angle (merge)
        self.pur_pts = purge_pts  # Min number of points (purge)
        self.pur_len = purge_len  # Min segment length (purge)
      
    def split(self, Pts):
        d,ind=Segment.pto_alejado(Segment,Pts)
        #print(Segment(Pts[0],Pts[ind],0,ind))
        #print(d)
        if d<self.d_th:
            pts=np.vstack((Pts[0,:],Pts[-1,:]))
            #print('Segmento: ',Segment(Pts[0,:],Pts[-1,:],ind,ind+1),'Distancia: ',d)
            seg_split.append(Segment(Pts[0,:],Pts[-1,:],ind,ind+1))
        else:
            P1=self.split(Pts[:ind+1,:])
            P2=self.split(Pts[ind:,:])
            pts=np.vstack((P1[:-1,:],P2))
        
        return pts

    def merge(self, segs_in):
        # TODO
        i=0
        media=[]
        while True:
            try:
                if segs_in[i].angle(segs_in[i+1])<self.a_th:
                    segs_in[i]=(Segment(segs_in[i].p0,segs_in[i+1].p1,0,1))
                    segs_in.pop(i+1)
                else:
                    print('Segmento: ',segs_in[i],'Distancia: ', segs_in[i].length())
                    media.append(segs_in[i].length())
                    i+=1
            except:
                print('Media segmentos',np.mean(media) )
                break
        return segs_in
    

    '''def purge(self, segs_in):
        # TODO
        i=0
        while True:
            try:
                if segs_in[i].length()+segs_in[i+1].length()+segs_in[i+2].length()<self.pur_len:
                    segs_in[i]=(Segment(segs_in[i].p0,segs_in[i+2].p1,0,1))
                    segs_in.pop(i+1)
                    segs_in.pop(i+2)
                else:
                    i+=3
            except:
                break
        return segs_in'''
    def purge(self, segs_in):
        # TODO
        seg=[]
        media=[]
        i=0
        while True:
            try:
                if segs_in[i].length()<self.pur_len:
                    segs_in.pop(i)
                else:
                    seg.append(segs_in[i])
                    #print('Segmento: ',segs_in[i],'Distancia: ',segs_in[i].length())
                    media.append(segs_in[i].length())
                    i+=1
            except:
                #print('Media longitud: ',np.mean(media))
                break
        return seg
            
    def __call__(self, Pts):
        pass
        seg0 = self.split(Pts)
        seg1 = self.merge(seg0)
        seg2 = self.purge(seg1)

        return seg0
   
  
    def plotSegments(self, segmentList, color='r',debug=False):
        for s in segmentList:
            x = (s.p0[0], s.p1[0])
            y = (s.p0[1], s.p1[1])
            plt.plot(x, y, marker = 'o', color=color)
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
    min_ang = 0.0
    d_ang = 0.00435422640294
    seg_split=[]
    seg_merge=[]
    seg_purge=[]
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
    scanCart=np.array(scanCart)
    
    #print(scanCart)   
    '''
    seg=Segment(np.array([0,0]),np.array([1,0]),0,1)
    print(seg.distance(np.array([1,1])))
    seg2=Segment(np.array([0,0]),np.array([2,2]),0,1)
    seg.write()
    print('Distancia al punto (1,1): ',seg.distance(np.array([1,1])))
    seg2.write()
    print('Ángulo entre segmentos: ',np.rad2deg(seg.angle(seg2)))'''
    

    
    # TODO: Set the parameters to the split and merge algorithm
    sm = SplitAndMerge()
    
    # Call the algorithm
    
    sm.split(scanCart)
    print(len(seg_split))
    plt.title("Split")
    sm.plot(scanCart)
    sm.plotSegments(seg_split, debug=True)
    

    seg_merge=sm.merge(seg_split)
    print(len(seg_merge))
    plt.title("Merge")
    sm.plotSegments(seg_merge,color='b',debug=True)

    seg_purge=sm.purge(seg_merge)
    print(len(seg_purge))
    plt.title("Purge")
    sm.plotSegments(seg_purge, color='g',debug=True)
    

    