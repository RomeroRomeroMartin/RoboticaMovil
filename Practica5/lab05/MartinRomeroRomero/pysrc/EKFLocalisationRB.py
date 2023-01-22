import numpy as np
from Simulator import Landmark, Map, Robot, EnvPlot
import math
from plot_graficas import plotea
# Create a map with random landmarks
noLandmarks = 5
m = Map()
for i in range(noLandmarks):
    p = np.array([np.random.uniform(-5,5), np.random.uniform(-5,5)], np.float32)
    m.append(Landmark(p, i))

# Create an object to plot the environment (map), robot and estimate
e = EnvPlot()
    
# Set the state and measurement noise covariance
Q = np.array([[0.25**2, 0],[0, 0.25**2]], np.float32)
R = np.array([[0.2**2, 0],[0, (5 * np.pi / 180)**2]], np.float32)

# Create the robot object, by default measures the Cartesian
# coordinates of the landmarks in the environment
r = Robot(np.array([2,-2], np.float32), Q, R, type='rb', range=2.5)

class KF:
    def __init__(self, x0, P0, Q, R, m, dt = r.dt):
        self.dt = dt
        self.xk_k = x0
        self.Pk_k = P0
        self.Q = Q * self.dt
        self.R = R / self.dt
        self.type = type
        self.map = m
        pass
    
    def predict(self, u):
        # TODO: Implement the prediction step of the KF        
        A=np.identity(2)
        T=0.2525
        B=T*A

        self.xk_k=A@self.xk_k+B@u
        self.Pk_k=A@self.Pk_k@A.T+self.Q
        pass
    def update(self, y):
        # TODO: Implement the updarte step of the KF for localization
        # using the field 'map' of the class
        #JG=np.array([[-1,0],[0,-1]])
        if len(y)>0:
            Y_hat=[]
            Jarr=[]
            for i in range(len(y)):
                auX=np.sqrt((self.map[y[i].id].p[0]-self.xk_k[0])**2+(self.map[y[i].id].p[1]-self.xk_k[1])**2)
                auY=math.atan2(self.map[y[i].id].p[1]-self.xk_k[1],self.map[y[i].id].p[0]-self.xk_k[0])
                Y_hat.append([auX,auY])
                JG=[[(self.xk_k[0]-self.map[y[i].id].p[0])/np.linalg.norm(self.map[y[i].id].p-self.xk_k), (self.xk_k[1]-self.map[y[i].id].p[1])/np.linalg.norm(self.map[y[i].id].p-self.xk_k)],
                [(self.map[y[i].id].p[1]-self.xk_k[1])/(np.linalg.norm(self.map[y[i].id].p-self.xk_k))**2, -(self.map[y[i].id].p[0]-self.xk_k[0])/(np.linalg.norm(self.map[y[i].id].p-self.xk_k))**2]]
                
                Jarr.append(JG)
            Y_hat=np.array(Y_hat)
            Jarr=np.array(Jarr)
            #print('Jacobiana: ',Jarr)
            #print('Y: ',self.Pk_k)
            Y=[i.p for i in y]

            Y_hat=Y-Y_hat
            Jarr.shape=(2*len(y),2)
            I=np.eye(len(y))
            R=np.kron(I,self.R)
            

            S=(Jarr@self.Pk_k)@Jarr.T+R
            
            K=self.Pk_k@Jarr.T@np.linalg.inv(S)
            
            Y_hat.shape=(2*len(y))
    
            self.xk_k=self.xk_k+K@Y_hat
            self.Pk_k=self.Pk_k-K@S@K.T


            pass
    
# Initial estimates of the position of the error and its covariance
# matrix
xHat = np.array([0, 0], np.float32)
PHat = np.array([[3,0],[0,3]], np.float32)

# Object for the (Extended) Kalman filter estimator
kf = KF(xHat, PHat, Q,  R, m)
        
# Plot the first step of the estimation process and pause
e.plotSim(r, m, kf, True)

# Main loop:
# 1- The robot moves (action) and returns its velocity
# 2- Predict step of the Kalman filter
# 3- The robot perceives the environment (landmarks) and
#    returns a list of landmarks
# 4- Update state of the Kalman filter
f = open('datos.dat', 'w')
while r.t < r.tf:
    u = r.action()
    kf.predict(u)
    f.write(F'{r.t} {kf.xk_k[0]} {kf.xk_k[1]} {r.p[0]} {r.p[1]}\n')
    y = r.measure(m)
    kf.update(y)
    
    e.plotSim(r, m, kf)

f.close()

plotea()
