import matplotlib.pyplot as plt


def plotea():
    f = open('datos.dat')
    lineas = f.readlines()

    x_est=[]
    y_est=[]
    x_r=[]
    y_r=[]
    for i in range(len(lineas)):
        l=list(lineas[i].split(' '))
        x_est.append(float(l[1]))
        y_est.append(float(l[2]))
        x_r.append(float(l[3]))
        y_r.append(float(l[4]))

    fig, ax = plt.subplots()
    ax.set_ylabel('Y')
    ax.set_xlabel('X')
    ax.set_title('Posición del robot')
    plt.plot(x_est,y_est,color='red')
    plt.plot(x_r,y_r,color='blue')
    plt.show()