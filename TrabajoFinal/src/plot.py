import matplotlib.pyplot as plt
import numpy as np
import scienceplots, sys
for robot in ["PFM","VFH","CVM"]:
    plt.figure()
    for i in range(1,201):
        file="datos/data_"+robot+"_"+str(i)+".dat"
        # print(sys.argv)
        # print(file)
        f=open(file,"r")
        lines=f.readlines()
        x=[]
        y=[]
        t=[]
        for X in lines:
           
            # print(X.split(','))
            t.append(float(X.split(',')[0]))
            if abs(float(X.split(',')[1]))<100:
                x.append(float(X.split(',')[1]))
            if abs(float(X.split(',')[2]))<100:
                y.append(float(X.split(',')[2]))
        f.close()

        # print([x.split(' ')[1] for x in open(file).readlines()])
        # print(result)
        # print(t[0:3])
        # print(x[0:3])
        # print(y[0:3])
        # print(len(y))
        # print(len(x))

        # plt.style.use(['science','no-latex'])
       
        plt.xlim((-6,6))
        plt.ylim((-6,6))

       
        plt.title('Evolucion de la posicion del Robot '+robot)
       
        if len(x)>len(y) and len(y)>50:
            plt.plot(x[:len(y)],y)
           
        else:
            plt.plot(x,y[:len(x)])

        plt.scatter(0,0,c='r',marker='X')
    plt.grid()
plt.show()  


for robot in ["PFM","VFH","CVM"]:
   

    file="datos/"+robot+".dat"
    # print(sys.argv)
    # print(file)
    f=open(file,"r")
    lines=f.readlines()
    x=[]
    y=[]
    t=[]
    c=[]
    for X in lines:
       
        # print(X.split(','))
        t.append(float(X.split(',')[0]))
        if abs(float(X.split(',')[1]))<100:x.append(float(X.split(',')[1]))
        y.append(float(X.split(',')[2]))
        c.append(float(X.split(',')[3]))
    f.close()

    # print([x.split(' ')[1] for x in open(file).readlines()])
    # print(result)
    # print(t[0:3])
    # print(x[0:3])
    # print(y[0:3])
    # print(len(y))
    # print(len(x))
    i=1
    # plt.style.use(['science','no-latex'])
    plt.figure()
    #plt.title('Evolucion de la posicion del Robot'+robot)
    plt.suptitle('Metrica del algoritmo '+robot)
    plt.subplot(2,2,1)
    plt.plot(x)
    plt.ylim((-0.5,10))
    plt.title('Distancia minima absoluta',fontsize=8)
    plt.xlabel('Numero de simulacion',fontsize=6)
    plt.ylabel('Distancia',fontsize=6)
    plt.grid()
    plt.subplot(2,2,2)
    plt.plot(y)
    plt.title('Distacia media a obstaculos',fontsize=8)
    plt.xlabel('Numero de simulacion',fontsize=6)
    plt.ylabel('Distancia',fontsize=6)
    plt.grid()
    plt.subplot(2,2,3)
    ceros=t.count(0)
    unos=t.count(1)
    plt.bar(['Fracaso','Exito'],[ceros,unos])
    plt.title('DONE',fontsize=8)
    plt.ylabel('Numero de ejecuciones',fontsize=6)
    plt.grid()
    plt.subplot(2,2,4)
    plt.plot(c)  
    plt.title('Tiempo de ejecucion',fontsize=8)
    plt.xlabel('Numero de simulacion',fontsize=6)
    plt.ylabel('ms',fontsize=6)
    plt.grid()
plt.show()  

x={}
y={}
t={}
c={}
for robot in ["PFM","VFH","CVM"]:
   

    file="datos/"+robot+".dat"
    # print(sys.argv)
    # print(file)
    f=open(file,"r")
    lines=f.readlines()

    x_l=[]
    y_l=[]
    t_l=[]
    c_l=[]
    for X in lines:
       
        # print(X.split(','))
        t_l.append(float(X.split(',')[0]))
        if abs(float(X.split(',')[1]))<100:x_l.append(float(X.split(',')[1]))
        y_l.append(float(X.split(',')[2]))
        c_l.append(float(X.split(',')[3]))

    t[robot]=t_l
    x[robot]=x_l
    y[robot]=y_l
    c[robot]=c_l
    f.close()


plt.figure()
plt.plot(x['PFM'],color='green',label='PFM')
plt.plot(x['VFH'],color='blue',label='VFH')
plt.plot(x['CVM'],color='red',label='CVM')
plt.ylim((-0.5,10))
plt.title('Distancia minima absoluta')
plt.xlabel('Numero de simulacion',fontsize=8)
plt.ylabel('Distancia',fontsize=8)
plt.legend()
plt.grid()
plt.figure()
plt.plot(y['PFM'],color='green',label='PFM')
plt.plot(y['VFH'],color='blue',label='VFH')
plt.plot(y['CVM'],color='red',label='CVM')
plt.title('Distacia media a obstaculos')
plt.xlabel('Numero de simulacion',fontsize=8)
plt.ylabel('Distancia',fontsize=8)
plt.legend()
plt.grid()
plt.figure()
plt.plot(c['PFM'],color='green',label='PFM')
plt.plot(c['VFH'],color='blue',label='VFH')
plt.plot(c['CVM'],color='red',label='CVM')
plt.title('Tiempo de ejecucion')
plt.xlabel('Numero de simulacion',fontsize=8)
plt.ylabel('ms',fontsize=8)
plt.legend()
plt.grid()
plt.show()  


mean_PFM=np.mean(x['PFM'])
mean_CVM=np.mean(x['CVM'])
mean_VFH=np.mean(x['VFH'])
std_PFM=np.std(x['PFM'])
std_CVM=np.std(x['CVM'])
std_VFH=np.std(x['VFH'])
print('--------MEDIA  y DESVIACION TIPICA de DISTANCIAS MINIMAS---------')
print('-->PFM:   mean: ',mean_PFM,'  std: ',std_PFM)
print('-->CVM    mean: ',mean_CVM,'  std: ',std_CVM)
print('-->VFH    mean: ',mean_VFH,'  std: ',std_VFH)


mean_PFM=np.mean(y['PFM'])
mean_CVM=np.mean(y['CVM'])
mean_VFH=np.nanmean(y['VFH'])
std_PFM=np.std(y['PFM'])
std_CVM=np.std(y['CVM'])
std_VFH=np.nanstd(y['VFH'])
print('--------MEDIA y DESVIACION TIPICA de DISTANCIAS MEDIAS---------')
print('-->PFM:   mean: ',mean_PFM,'  std: ',std_PFM)
print('-->CVM    mean: ',mean_CVM,'  std: ',std_CVM)
print('-->VFH    mean: ',mean_VFH,'  std: ',std_VFH)

mean_PFM=np.mean(c['PFM'])
mean_CVM=np.mean(c['CVM'])
mean_VFH=np.nanmean(c['VFH'])
std_PFM=np.std(c['PFM'])
std_CVM=np.std(c['CVM'])
std_VFH=np.nanstd(c['VFH'])
print('--------MEDIA y DESVIACION TIPICA del TIEMPO---------')
print('-->PFM:   mean: ',mean_PFM,'  std: ',std_PFM)
print('-->CVM    mean: ',mean_CVM,'  std: ',std_CVM)
print('-->VFH    mean: ',mean_VFH,'  std: ',std_VFH)