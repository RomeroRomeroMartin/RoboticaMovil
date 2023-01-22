import matplotlib.pyplot as plt

f = open('archivo.dat')
lineas = f.readlines()

x=[]
y=[]
tiempo=[]
for i in range(len(lineas)):
    l=list(lineas[i].split(' '))
    tiempo.append(float(l[0]))
    x.append(float(l[1]))
    y.append(float(l[2]))

fig, ax = plt.subplots()
#plt.xlim((-5,5))
#plt.ylim((-5,5))
ax.set_ylabel('Y')
ax.set_xlabel('X')
ax.set_title('Posición del robot')
plt.plot(x,y)
plt.show()