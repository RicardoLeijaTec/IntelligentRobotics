import random
import matplotlib.pyplot as plt


maxPuntos = int(input("Numero de puntos a generar: "))

x = [0]*maxPuntos
y = [0]*maxPuntos
arregloMedios = []

for puntos in range(0, maxPuntos):
    x[puntos] = random.randint(-25,25)
    y[puntos] = random.randint(-25,25)
    
for puntos in range(0, maxPuntos):
    print("x: "+str(x[puntos])+"  y: "+str(y[puntos]))
    


for i in range(0, maxPuntos-1):
    m = [(x[i]+x[i+1])/2,(y[i]+y[i+1])/2]
    
    arregloMedios.append(m)
    print(m)

plt.scatter(x,y)
for j in range(0, len(arregloMedios)):
    plt.scatter(arregloMedios[j][0], arregloMedios[j][1], c='#000000')
#plt.scatter(arregloMedios[0][0], arregloMedios[0][1])
plt.show()