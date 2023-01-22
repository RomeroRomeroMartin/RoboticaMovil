#!/usr/bin/env python

import sys
import queue
import numpy as np
import math 
import matplotlib.pyplot as plt
from numpy.linalg import norm
# Read scan files saved in plain text format. Each row is a full scan
# of the Turtlebot LIDAR with the following format:
#
# [time] [no of readings] [reading0] [reading1]...
#
# The function returns a numpy arrray in which each row corresponds to
# a full 360 deg scan and each column corresponds to one direction of
# the robot. In the files the robot was stationary in a static
# environment.
global segmentos_split,segmentos_split_merge,SSS2
segmentos_split=[];segmentos_split_merge=[];SSS2=[]
def read_scan_file(fname):
    print('read_scan_file():', fname)
    D = np.loadtxt(fname)
    scans = D[:,2:]
    return scans



# Segment class to store the endpoints of the segment and their
# corresponding indexes
class Segment:
   def __init__(self, p0, p1, idx0, idx1):
      self.p0 = np.array(p0)
      self.p1 = np.array(p1)
      self.idx0 = idx0
      self.idx1 = idx1

   # Computes the length of the segment
   def length(self):
      return np.linalg.norm(np.array(self.p1, np.float32)
                            - np.array(self.p0, np.float32))
   def Segment2pointDistance(self,P): 
# point to line distance, where the line is given with points Ps es p0 and Pe es p1
    	if np.all(np.equal(self.p0,self.p1)):
    		return np.linalg.norm(P-self.p0)
    	return np.divide(np.abs(np.linalg.norm(np.cross(self.p1-self.p0,self.p0-P))),np.linalg.norm(self.p1-self.p0))
   # Compute the distance from the segment to the point p

   def distance(self,P,Ps,Pe): # point to line distance, where the line is given with points Ps and Pe
    	if np.all(np.equal(Ps,Pe)):
    		return np.linalg.norm(P-Ps)
    	return np.divide(np.abs(np.linalg.norm(np.cross(Pe-Ps,Ps-P))),np.linalg.norm(Pe-Ps))
   def punto_mas_alejado(self,P):
       	dmax = 0
       	index = -1;#print(P);#input('')
         
        for i in range(1,P.shape[0]):
            # print(P)
            # print('.'*30)
            # print(P[i,:],P[0,:],P[-1,:])
            d = self.distance(self,P[i,:],P[0,:],P[-1,:])
            if (d > dmax):
                index = i
                dmax = d
        return dmax,index
       
        
   def angle(self, s):
       # TODO
       # pass
    
    angle1 = math.atan2((self.p1[1])-(self.p0[1]), (self.p1[0])-(self.p0[0]))
    angle1 = int(angle1 * 180 / np.pi)
    # print('angulo segmento inicial con X',angle1)
    angle2 = math.atan2((s.p1[1])-(s.p0[1]), (s.p1[0])-(s.p0[0]))
    angle2 = int(angle2 * 180 / np.pi)
    # print('angulo segmento final con X',angle2)

    if angle1 * angle2 >= 0:
        ang_sg2sg = abs(angle1 - angle2)
       
    else:
        ang_sg2sg = abs(angle1) + abs(angle2)
        if ang_sg2sg > 180:
            ang_sg2sg = 360 - ang_sg2sg
    ang_sg2sg = ang_sg2sg % 180
    print('ANG',ang_sg2sg)
    return ang_sg2sg


   def write(self):
      print('Segment: ', self.p0, '---', self.p1) 
      print('Segment id : ', self.idx0, '---', self.idx1) 

class SplitAndMerge:
   def __init__(self, dist=0.1, angle=0.0, purge_pts = 8, purge_len = 0.3):
      self.d_th = dist        # Threshold distance (split)
      self.a_th = angle       # Threshold angle (merge)
      self.pur_pts = purge_pts  # Min number of points (purge)
      self.pur_len = purge_len  # Min segment length (purge)
      
   def split(self, P):
        threshold=self.d_th
        d,ind = Segment.punto_mas_alejado(Segment,P);#print('distancia',d,'thr',threshold)
        # print('Distancia',d)
        if (d>threshold):
            P1 = self.split(P[:ind+1,:]) 
            P2 = self.split(P[ind:,:]) 
            points = np.vstack((P1[:-1,:],P2))
        else:
            points = np.vstack((P[0,:],P[-1,:]))
            segmentos_split.append(Segment(P[0,:],P[-1,:],ind,ind+1))
        return points

   def merge(self, P):
       
    ss=P.copy()
    r=0
    while True:
        try:
            if ss[r].angle(ss[r+1])<self.a_th:
                print('----------Merge-----------')
                ss[r]=(Segment(ss[r].p0,ss[r+1].p1,0,1))
                ss.pop(r+1)
            else:
                r=r+1
        except:
            break
        
    return ss
   
    
   def purge(self,P):
    ss=P.copy()
    
    r=0
    while True:
        try:
            print(ss[r].length() , ss[r+1].length() ,ss[r+2].length())
            if (ss[r].length()+ss[r+1].length()+ss[r+2].length()<self.pur_len) \
                :
                print('**********Purge***********')
                ss[r]=(Segment(ss[r].p0,ss[r+2].p1,0,1))
                ss.pop(r+1)
                ss.pop(r+2)

            else:
                r=r+3

        except:
            break
    return ss
   
    
   def __call__(self, Pts):
       pass
       seg0 = self.split(Pts)
       seg1 = self.merge(seg0)
       seg2 = self.purge(seg1)

       return seg2
   
  
   def plotSegments2(self, segmentList, debug=False):
      for s in segmentList:
         x = (s.p0[0], s.p1[0])
         y = (s.p0[1], s.p1[1])
         plt.plot(x, y, marker = 'o', color='r')
         plt.draw()
         if debug:
          # print('Press a key and hit enter to continue')
              plt.waitforbuttonpress()
              plt.pause(0.001)
         
   def plot(self, Pts):
      for ii in range(0, len(Pts)):
         pt = Pts[ii]
         plt.plot(pt[0],pt[1], marker='.', color='k')
      plt.draw()
      plt.pause(0.01)
      

      
      
if __name__ == '__main__':
    # make sure the filename is given as an argument
    # if len(sys.argv) != 2:
    #     print('Usage: ', sys.argv[0], ' filename\n')
    #     sys.exit()
        
    # Read the file with the scans an get all of the scans as a matrix
    try:
        D = read_scan_file(sys.argv[1])
    except:
        D = read_scan_file('scan-files/scan-014.dat')
    s=[];p=[];m=[];idd=[]
    # index of the scan to process
    # for i in range(30,70,5):
    segmentos_split=[];segmentos_split_merge=[];SSS2=[]
    idx = 10
    R = D[idx,:]
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
    scanCart=np.array(scanCart)
    # TODO: Set the parameters to the split and merge algorithm
    try:
        dis=float(input('treshold '))
        angle=float(input('angle '))
        purge_len=float(input('purge_len '))
        sm = SplitAndMerge(dis, angle, purge_len )
        # print('d_thd {0} a_th {1}'.format(dis,angle))
    except:
        dis=0.1
        angle=30
        purge_len=3
        sm = SplitAndMerge(dis, angle, purge_len )  
            # print('d_thd {0} a_th {1}'.format(0.1,0.0))
        
        # experimentacion pt1
        
        # segment = Segment([0,0],[-1,-1],0,1)
        # print(segment.write())
        # segment2 = Segment([0,0],[-1,1],2,3)
        # print(segment2.write())
        # print(segment.angle(segment2))
        # s=[segment,segment2]
        # sm.plotSegments2(s,0)
        
        
        
        # print(segment.GetMostDistant(np.array([1,1])))
        
        
        
        
        # #tengo segemntos_split como lista del conjunto de segmentos hechos
        # #filtramos los demasiado largos
        
    sm.split(scanCart)
    segmentos_split_merge=sm.merge(segmentos_split)
    segmentos_split_merge_purge=sm.purge(segmentos_split_merge)
    
        # idd.append(i)
        # s.append(len(segmentos_split))
        # m.append(len(segmentos_split_merge))
        # p.append(len(segmentos_split_merge_purge))
        
    
    
    
    # plt.plot(idd,s)
    # plt.plot(idd,m)
    # plt.plot(idd,p)
    # plt.plot(idd,m)
    # plt.grid()
    # plt.ylabel('num de segmentos')
    # plt.xlabel('angulo umbral')
    
    sm.plot(scanCart)
    sm.plotSegments2(segmentos_split, 0)
    plt.title("Split "+str(dis)+' len '+str(len(segmentos_split)))
    plt.show()
    
    sm.plot(scanCart)
    sm.plotSegments2(segmentos_split_merge, 0)
    plt.title("S&Merge "+str(angle)+' len '+str(len(segmentos_split_merge)))
    plt.show()
    
    sm.plot(scanCart)
    sm.plotSegments2(segmentos_split_merge_purge, 0)
    plt.title("S&M&Purge "+str(purge_len)+' len '+str(len(segmentos_split_merge_purge)))
    plt.show()
    
    
    
    
    
    
    
    
    