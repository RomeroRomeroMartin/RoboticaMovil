#!/usr/bin/env python3
import sys
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import matplotlib.transforms as transforms
import statistics
from sklearn.linear_model import LinearRegression

# Step definition.
# The script must be completed in 3 steps
#
# 1. Obtain and plot means and std. deviations
# 2. Linear regression of the data points
# 3. Plot LIDAR scans in Cartesian coordinates including
#    the 3 sigma uncertainty
# Set the part you're working on here
step = 3

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
# y= a*x_2
def linear_regression(x, y):
    """
    Obtiene el valor de "a" a partir de las medias y las desviaciones estandar obtenidas
    x -> vector de medias
    y -> vector de desviaciones
    x e y tienen la misma longitud
    """
    numerador = 0
    denominador = 0
    for i in range (len(x)):
        numerador += y[i] * x[i]**2
        denominador += x[i]**4
    a = numerador / denominador

    print("a: ", a)
    
    return a


# This function computes the mean and standard deviation of the LIDAR
# readings for each angle (column of the scan matrix). Since the
# readings returned are sometimes "inf" the computation of the
# statistics must remove these values (e.g. using np.isfinite()) The
# function must return a matrix with pairs (mean, stddev) for
# different angles on each row.

def get_stds(D):
    # TODO

    mD = []
    stdD = []
    filas, columnas = np.shape(D)
    finite = np.isfinite(D)
    
    for j in range (columnas):
        columna = []
        for i in range (filas):
            if finite[i][j] and (0.5 <= D[i][j] < 25) and (D[i][j] not in columna):
               columna.append(D[i][j])
        
        if len(columna) == 0:
            columna = [0, 0]
        mD.append(np.mean(columna))
        stdD.append(np.std(columna))

    return mD, stdD



# This function gets as an input the pose and covariance matrix in
# Polar coordinates and returns the position and covariance matrix in
# Cartesian coordinates
def polar2cart(p, Sp):

    #print("Polar: ", p)

    cartesianas = []
    r = p[0]
    theta = p[1]
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    cartesianas = [x,y]
    J = [[np.cos(theta), -r*np.sin(theta)],
        [np.sin(theta),  r*np.cos(theta)]]
    Sc = np.dot(np.dot(J, Sp), np.transpose(J))

    #print("Cartesianas: ", cartesianas)

    return (cartesianas, Sc)
    


# This function plots an 2D point and uncertainty as a 3 sigma ellipse
def plot_error_ellipse(x, Sc):
    f = 3.0;
    pearson = Sc[0, 1]/np.sqrt(Sc[0, 0] * Sc[1, 1])
    # Using a special case to obtain the eigenvalues of this
    # two-dimensionl dataset.
    ell_radius_x = np.sqrt(1 + pearson)
    ell_radius_y = np.sqrt(1 - pearson)
    ellipse = Ellipse((0, 0), width=ell_radius_x * 2,
                      height=ell_radius_y * 2, edgecolor='blue',fc='none')

    scale_x = np.sqrt(Sc[0, 0]) * f
    scale_y = np.sqrt(Sc[1, 1]) * f

    transf = transforms.Affine2D() \
                       .rotate_deg(45) \
                       .scale(scale_x, scale_y) \
                       .translate(x[0], x[1])
    ax = plt.gca()
    ellipse.set_transform(transf + ax.transData)
    ax.add_patch(ellipse)


def plot_scan_cart(r):
    global a

        # TODO

    for i in range (0, len(r)):

        p=np.array([r[i],-(i)*(2*np.pi)/1440])

        Sp=np.array([[a*r[i]**4,0],

                     [0,((2*np.pi)/1440/3)**2]])

        c,Sc = polar2cart(p,Sp)


        #print("C: ", c)
        
        plt.plot(c[0],c[1], '.')

        plot_error_ellipse(c,Sc)

    #plt.draw()

    plt.show()

    #plt.waitforbuttonpress()

    #plt.close()
    
    
    
if __name__ == '__main__':
    # make sure the filename is given as an argument
    if len(sys.argv) != 2:
        print('Usage: ', sys.argv[0], ' filename\n')
        sys.exit()
    # Read the file with the scans an get all of the scans as a matrix
    D = read_scan_file(sys.argv[1])

    if step == 0:
        exit()
    # Calculate the mean and variance of the reading for each direction
    mD, stdD = get_stds(D)
    #print("Media: ", mD, "y desviacion: ", stdD)
    print("Media: ", mD[0])
    print("Desviacion: ", stdD[0])

    # Visualise the result as a cloud of points in 2D
    plt.plot(mD,stdD, '.')
    plt.title("Media y desviacion")
    #plt.draw()
    #plt.waitforbuttonpress()
    #plt.close()
    plt.show()

    if step == 1:
        exit()
    
    plt.plot(mD,stdD, '.')

    print("\nImprimimos grafica desviacion y medias")

    global a
    a = linear_regression(mD, stdD)
    print("a: ", a)
    x = np.arange(0.5, 25.0, 0.1)
    x2 = np.power(x, 2*np.ones(x.shape))
    y = a * x2 
    plt.plot(x,y)
    #plt.draw()
    #plt.waitforbuttonpress()
    #plt.close()
    plt.show()


    plot_scan_cart(D[0])
    
    if step == 2:
        exit()
    # Visualise some of the scans with its +-3 sigma intervals
    scanIdx = range(0, 100, 5)
    for ii in scanIdx:
        r = D[ii, :]
        print(r)
        plt.plot(range(len(r)), r, '+')
        plt.plot(range(len(r)), r - 3 * a * np.power(r, 2*np.ones(r.shape)), '.r')
        plt.plot(range(len(r)), r + 3 * a * np.power(r, 2*np.ones(r.shape)),'.r')
        plt.draw()
        plt.waitforbuttonpress(0)
        plt.close()




    

