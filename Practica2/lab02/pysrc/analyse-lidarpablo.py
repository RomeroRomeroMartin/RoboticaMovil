#!/usr/bin/env python3
import sys
import numpy as np
import statistics as st
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
from sklearn.linear_model import LinearRegression
import matplotlib.animation as animation

# C:\Users\Pablo\OneDrive - Universidade de Santiago de Compostela\4ºRob\Móvil\P2\lab02\lab02\pysrc>python analyse-lidar.py scan-files/scan-000.dat

# plt.figure(figsize=(800, 600), dpi=80)

# Step definition.
# The script must be completed in 3 steps
#
# 1. Obtain and plot means and std. deviations
# 2. Linear regression of the data points
# 3. Plot LIDAR scans in Cartesian coordinates including
#    the 3 sigma uncertainty
# Set the part you're working on here
step = 4

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

# This function should perform a linear regression of the form
# y= a*x_2
def linear_regression(x, y):
        
    # print(x,y)
    model = LinearRegression(fit_intercept=False)
    
    model.fit(x[:, np.newaxis], y)   
    a=model.coef_[0]
    print('coeficiente Regresion',a)
    a2=(sum(y*x**2)/sum(x**4))
    print('coeficiente Regresion2',a2)
    return a2

# This function computes the mean and standard deviation of the LIDAR
# readings for each angle (column of the scan matrix). Since the
# readings returned are sometimes "inf" the computation of the
# statistics must remove these values (e.g. using np.isfinite()) The
# function must return a matrix with pairs (mean, stddev) for
# different angles on each row.
def get_stds(D):

    # modified_array = np.delete(original_array, np.where(original_array == value_to_delete))
 
    mD=np.zeros(len(D.T))
    stdD=np.zeros(len(D.T))
    D=D.T
    # D=filtro(D)
    # print(D)
    for i in range(len(D)):
        
        # D2=np.zeros(len(D))
        # print('deleet',len(np.delete(D[i], np.where(D[i] == np.inf))))
        # D2[i]= np.delete(D[i], np.where(D[i] == np.inf))
        # D[i]= np.delete(D[i], np.where(D[i] == -np.inf))
        # mD[i]=st.mean(np.delete(D[i], np.where(abs(D[i]) <25)))
        # stdD[i]=st.stdev((np.delete(D[i], np.where(abs(D[i])  <25))))
        m=np.delete(D[i], np.where((D[i]) > 25))
        
        std=np.delete(D[i], np.where((D[i]) >25))
        
        if not(len(std) in [0,1]):
            # print(i,std)
            if not(std.all()==std[0]):
                if st.stdev(std)< 0.04:
                    mD[i]=st.mean(m)
                    stdD[i]=st.stdev(std)
        
        # stdD=np.delete(stdD, np.where((stdD[i]) <0.4))
        # mD=np.delete(mD, np.where((mD[i]) <0.4))
            
                


    
    
    return mD, stdD

# This function gets as an input the pose and covariance matrix in
# Polar coordinates and returns the position and covariance matrix in
# Cartesian coordinates

def polar2cart(p, Sp):
    
    X=np.array([])
    M=np.array([])
   
    if np.isfinite(p).all():
        # print('....................................................')
        # print('Polar',p)
        x=p[0]*np.cos(p[1])
        y=p[0]*np.sin(p[1])
        # print('x,y',x,y)
        J=np.array([[np.cos(p[1]),-p[0]*np.sin(p[1])],
                    [np.sin(p[1]),p[0]*np.cos(p[1])]])
        # print('J',J)
        # print('SP',Sp)
        M=np.dot(J,Sp)
        # print('M',M) 
        M2=np.dot(M,J.T)
        # print('M2',M2)            
        M2=M2
        X=np.append(X,[x,y])
        M=np.append(M,M2)

    
    else:
        return [[0,0],0]
    
    return [X,M2]
    

# This function plots an 2D point and uncertainty as a 3 sigma ellipse

def plot_error_ellipse(x,y, Sc):
    f = 3.0;
    try:
        for i in range(len(Sc)):
            # print('Usamos',Sc[i])
            pearson = round(Sc[i][0, 1]/np.sqrt(Sc[i][0, 0] * Sc[i][1, 1]),6)

            ell_radius_x = np.sqrt(1 + pearson)
            ell_radius_y = np.sqrt(1 - pearson)
            ellipse = Ellipse((0, 0), width=ell_radius_x * 2,
                              height=ell_radius_y * 2, edgecolor='red',fc='none')
        
            scale_x = np.sqrt(Sc[i][0, 0]) * f
            scale_y = np.sqrt(Sc[i][1, 1]) * f
        
            transf = transforms.Affine2D() \
                               .rotate_deg(45) \
                               .scale(scale_x, scale_y) \
                               .translate(x[i], y[i])
            ax = plt.gca()
            ellipse.set_transform(transf + ax.transData)
            ax.add_patch(ellipse)
    except:
        for i in range(len(Sc)):
            # print('Usamos',Sc[i])
            pearson = round(Sc[0, 1]/np.sqrt(Sc[0, 0] * Sc[1, 1]),6)

            # print('PEARSON',pearson)
            # input('0')
            # Using a special case to obtain the eigenvalues of this
            # two-dimensionl dataset.
            ell_radius_x = np.sqrt(1 + pearson)
            ell_radius_y = np.sqrt(1 - pearson)
            ellipse = Ellipse((0, 0), width=ell_radius_x * 2,
                              height=ell_radius_y * 2, edgecolor='red',fc='none')
        
            scale_x = np.sqrt(Sc[0, 0]) * f
            scale_y = np.sqrt(Sc[1, 1]) * f
        
            transf = transforms.Affine2D() \
                               .rotate_deg(45) \
                               .scale(scale_x, scale_y) \
                               .translate(x, y)
            ax = plt.gca()
            ellipse.set_transform(transf + ax.transData)
            ax.add_patch(ellipse)
        
def filtro(r)    :
    rr=np.array([])
    for i in range(len(r)):
        rr=np.append(rr,(np.delete(r[i], np.where((r[i]) > 25))))
    return rr

def plot_scan_cart(r):
    global a
    
    rr=filtro(r)

    x=0
    Sc=0
    Sp=np.array([])
    # Sc2=np.array([])
    in_tetha=0.00435422640294
    p=np.array([])
    t=np.array([])
    fig, ax = plt.subplots()

   
    
    for i in range(1,len(rr)):
        if np.isfinite(rr[i]):
            p=np.append(p,rr[i])
            t=np.append(t,in_tetha*i+i)
             
        else:
            p=np.append(p,0)
            t=np.append(t,0)
    plt.axes(projection = 'polar')            
    plt.polar(p,t, '.r') 
    plt.title('polar')
    plt.waitforbuttonpress()
    plt.close()
    plt.axes(projection=None, polar=False)   
    plt.plot(p,t, '.r') 
    plt.title('polarCART')
    plt.waitforbuttonpress()
    plt.close()    
    # print('P',p)
    # print('T',t)
    Sp=np.cov(p,t)
    Sp=np.array([[a,0],[0,0.3333333333333333*in_tetha]])
    # print('SPPPPPPPPPPPPPPPP',Sp)
    # print('---------------------------------')
    SCC=[]
    XX=np.array([])
    YY=np.array([])
    plt.axes(projection=None, polar=False)
    for i in range(len(p)):
        x,Sc=polar2cart([p[i],t[i]], Sp)
        
        # Sc2=np.append(Sc2,Sc)
            
        # print('DEberia ser 2x2',Sc)
        # print('x,y',x)
        SCC.append(Sc)
        XX=np.append(XX,x[0])
        YY=np.append(YY,x[1])
        
        
    
    # print('Vextor de Scs',SCC)
    #10 primeras
    for iii in range(10):
        
        plt.plot(XX[iii],YY[iii], '.')
        plot_error_ellipse(XX[iii],YY[iii], SCC[iii])
        plt.title('elipses_indv_solas')
        plt.draw()
        # fig.savefig('results3/Elipses_completas-'+(sys.argv[1].split('.'))[0].split('/')[-1]+'.png')  
        plt.waitforbuttonpress()
        plt.close()
        
    plt.plot(XX,YY, '.')
    
    plot_error_ellipse(XX,YY, SCC)
    plt.title('çelipse_total')
    plt.draw()
    # fig.savefig('results3/Elipses_completas-'+(sys.argv[1].split('.'))[0].split('/')[-1]+'.png')  
    plt.waitforbuttonpress()
    plt.close()
    
if __name__ == '__main__':
    # make sure the filename is given as an argument
    if len(sys.argv) != 2:
        print('Usage: ', sys.argv[0], ' filename\n')
        sys.exit()
    # Read the file with the scans an get all of the scans as a matrix
    D = read_scan_file(sys.argv[1])
    fig, ax = plt.subplots() 
    if step == 0:
        exit()
    # Calculate the mean and variance of the reading for each direction
    mD, stdD = get_stds(D)

    # Visualise the result as a cloud of points in 2D
    plt.plot(mD,stdD, '.')

    plt.draw()

    #fig.savefig('results/mD-stdD'+(sys.argv[1].split('.'))[0].split('/')[-1]+'.png')  
    plt.waitforbuttonpress()
    plt.close()
    if step == 1:
        exit()

    plt.plot(mD,stdD, '.')
    
    global a
    a = linear_regression(mD, stdD)
    x = np.arange(0.5, 25.0, 0.1)
    x2 = np.power(x, 2*np.ones(x.shape))
    y = a * x2 
    # ax = plt.gca()
    # ax.set_xlim([0, 10])
    # ax.set_ylim([0, 10])
    plt.plot(x,y)
    
    
    plt.draw()
    # fig.savefig('results2/LinearR'+(sys.argv[1].split('.'))[0].split('/')[-1]+'.png')  
    plt.waitforbuttonpress()
    plt.close()
    

    if step == 2:
        exit()
    # Visualise some of the scans with its +-3 sigma intervals
    scanIdx = range(0, 100, 5)
    ims = []
    for ii in scanIdx:
        r = D[ii, :]
    plot_scan_cart(r)  
    if step==3:
        exit()
    for ii in scanIdx:
        r = D[ii, :]

    PP=r-3 * a * np.power(r, 2*np.ones(r.shape))
    PS=r + 3 * a * np.power(r, 2*np.ones(r.shape))

    plt.plot(range(len(r)), r, '+')
    plt.plot(range(len(r)), PP, '.r')
    plt.plot(range(len(r)), PS, '.r')
    
    
    
    # fig.savefig('fotos/static_plot'+str(int(ii/5))+'.png')  
    plt.draw()
    

    plt.waitforbuttonpress(0)
    plt.close() 

   

       




    

