# Python code to simulate and visualise Integrator and Unicycle
# robots. The robot object derived from the classes are callable
# objects which can be used with the scipy package to integrate the
# movement and obtained samples of the trajectory
from math import atan2, sqrt
import numpy as np
import matplotlib.patches as patches
import matplotlib.pyplot as plt

class IntegratorRobot:
    def __init__(self,pos):
        pass
    
    def controller(self, x):
        # TODO Implement here your controller for the robot
        k=np.array([[10,5],[3,4]])

        u=-k@x
        print(np.linalg.eig(k))
        return u

    # This method returns the time derivative of the system
    def __call__(self, x, t):
        return self.controller(x)


class UnicycleRobot:
    def __init__(self,pi,pf):
        self.pf=pf
        pass

    def controller(self, x):
        # TODO Implement here your controller for the robot
        # u[0]-> v
        # u[1]-> w
        kp=0
        kb=-1
        ka=2
        p=sqrt(((self.pf[0]-x[0])**2)+((self.pf[1]-x[1])**2))
        a=self.angle2range(atan2(self.pf[1]-x[1],self.pf[0]-x[0])-x[2])
        b=self.angle2range(-x[2]-a)
        v=kp*p
        w=kb*b+ka*a 
        u=[v,w]
        return u
    
    # This method returns the time derivative of the system, it
    # implements the unicycle motion model
    def __call__(self, x, t):
        u = self.controller(x)
        return np.array([u[0]*np.cos(x[2]), u[0]*np.sin(x[2]), u[1]],
                        np.float32)

    # This method brings any angle to the range (-pi,pi]
    def angle2range(self, z):
        while z > np.pi:
            z -= 2 * np.pi
        while z <= -np.pi:
            z += 2 * np.pi
        return z
    
class GraphicsRobot:
    def __init__(self, range = np.array([[-15,-15],[15,15]], np.float32), dt = 0.1):
        self.range = range
        self.dt = dt
        self.l = 0.5
        self.w = 0.25
        
    def PlotRobot(self, x):
        if x.size == 2:
            c = patches.Circle((x[0],x[1]), self.l)
            return c
        else:
            p = x[0:2]
            z = x[2]
            u = np.array([np.cos(z),np.sin(z)] ,np.float32)
            up = np.array([-np.sin(z),np.cos(z)] ,np.float32)
            p0 = p + self.l/2 * u
            p1 = p - self.l/2 * u + self.w/2 * up
            p2 = p - self.l/2 * u - self.w/2 * up
            R = np.array([p0,p1,p2], np.float32)
            r = patches.Polygon(R)
            return r
    p0 = np.array([np.float32(8), np.float32(12),np.float32(3)])
    def PlotTrajectory(self, X):
        plt.figure(300)
        plt.plot(X[:,0], X[:,1])
        plt.gca().add_patch(self.PlotRobot(X[0,:]))
        plt.gca().add_patch(self.PlotRobot(X[-1,:]))
        plt.draw()
        plt.grid()
        plt.xlabel('x')
        plt.ylabel('y')
        plt.waitforbuttonpress(0)
        plt.close()

    def PlotVariables(self, T, X):
        plt.figure(200)
        sz = X[0,:].size
        fix, ax = plt.subplots(sz)
        ax[0].plot(T, X[:,0])
        ax[0].grid()
        ax[0].set_xlabel('t')
        ax[0].set_ylabel('x')

        ax[1].plot(T, X[:,1])
        ax[1].grid()
        ax[1].set_xlabel('t')
        ax[1].set_ylabel('y')

        if sz == 3:
            ax[2].plot(T, X[:,2])
            ax[2].grid()
            ax[2].set_xlabel('t')
            ax[2].set_ylabel('theta')
        
        plt.waitforbuttonpress(0)
        plt.close()
        
    
    def AnimateTrajectory(self, T, X):
        plt.figure(100)
        for ii in range(len(T)):
            plt.cla()
            x = X[ii,:]
            plt.gca().add_patch(self.PlotRobot(x))
            plt.draw()
            plt.xlim([self.range[0,0], self.range[1,0]])
            plt.ylim([self.range[0,1], self.range[1,1]])
            plt.grid()
            plt.xlabel('x')
            plt.ylabel('y')
            plt.title(f'{T[ii]:.2f}/{T[-1]}')
            plt.pause(self.dt)
        plt.waitforbuttonpress(0)
        plt.close()
