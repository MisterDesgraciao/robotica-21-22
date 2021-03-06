#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Rob�tica Computacional -
# Grado en Ingenier�a Inform�tica (Cuarto)
# Pr�ctica: Filtros de particulas.


from math import *
from robot import *
import decimal
import random
import numpy as np
import matplotlib.pyplot as plt
import sys
import select
from datetime import datetime
# ******************************************************************************
# Declaraci�n de funciones

def distancia(a,b):
  # Distancia entre dos puntos (admite poses)
  return np.linalg.norm(np.subtract(a[:2],b[:2]))

def angulo_rel(pose,p):
  # Diferencia angular entre una pose y un punto objetivo 'p'
  w = atan2(p[1]-pose[1],p[0]-pose[0])-pose[2]
  while w >  pi: w -= 2*pi
  while w < -pi: w += 2*pi
  return w

def pinta(secuencia,args):
  # Dibujar una secuencia de puntos
  t = np.array(secuencia).T.tolist()
  plt.plot(t[0],t[1],args)

def mostrar(bordes,objetivos,trayectoria,trayectreal,trayectideal,filtro):
  # Mostrar mapa y trayectoria
  plt.ion() # modo interactivo
  plt.clf()
  plt.axis('equal')
  # Generamos el centro
  centro = [(bordes[0]+bordes[1])/2.,(bordes[2]+bordes[3])/2.]
  radio  = max(bordes[1]-bordes[0],bordes[3]-bordes[2])
  plt.xlim(centro[0]-radio,centro[0]+radio)
  plt.ylim(centro[1]-radio,centro[1]+radio)
  # Representar mapa
  for p in filtro:
    dx = cos(p.orientation)*.05
    dy = sin(p.orientation)*.05
    plt.arrow(p.x,p.y,dx,dy,head_width=.05,head_length=.05,color='k')
  pinta(trayectoria,'--g')
  pinta(trayectreal,'-r')
  pinta(trayectideal,'-g')  # añadida línea verde para la trayectoria ideal
  pinta(objetivos,'-.ob')
  p = hipotesis(filtro)
  dx = cos(p[2])*.05
  dy = sin(p[2])*.05
  plt.arrow(p[0],p[1],dx,dy,head_width=.075,head_length=.075,color='m')
  # Mostrar y comprobar pulsaciones de teclado:
  plt.draw()
#  if sys.stdin in select.select([sys.stdin],[],[],.01)[0]:
#    line = sys.stdin.readline()
  raw_input()

def genera_filtro(bordes, num_particulas, balizas, real, centro=[2,2], radio=1):
  # Inicializaci�n de un filtro de tama�o 'num_particulas', cuyas part�culas
  filtro = []
  # imitan a la muestra dada y se distribuyen aleatoriamente sobre un �rea dada.
  for i in range(num_particulas):
    posXAleatoria = random.uniform(centro[0] - radio, centro[0] + radio)
    posYAleatoria = random.uniform(centro[1] - radio, centro[1] + radio)
    orientacionAleatoria = random.random() * 2.0 * pi
    instanceOfRobot = robot()
    instanceOfRobot.set(posXAleatoria, posYAleatoria, orientacionAleatoria)
    instanceOfRobot.set_noise(.01,.01,.1) # añadimos ruido
    filtro.append(instanceOfRobot)
  return filtro

def newRobotList(real, ideal, filtro):
  robotsList = list(filtro)
  robotsList.insert(0, real) # Para que luego al mover siempre se mueva primero el real
  robotsList.insert(1, ideal)
  return robotsList

# Dispersion espacial del filtro de particulas
# Aproximacion de la disperción a partir de la media de la distancia
# de una particula cualquiera a el resto.
def dispersion(filtro):
  particulaArbitraria = filtro[random.randrange(0, len(filtro))]
  distanciaMedia = 0 # Infinito
  for particula in filtro:
    distanciaMedida = distancia(particulaArbitraria.pose(), particula.pose())
    ratio = distanciaMedida / len(filtro)
    distanciaMedia += ratio
  return distanciaMedia

# Peso medio normalizado del filtro de particulas
# Si es alto significa que estás bien localizadas
def peso_medio(filtro, real, balizas):
  pesoTotal = 0
  for particula in filtro:
    pesoTotal += particula.measurement_prob(real.sense(balizas), balizas)
  return pesoTotal / len(filtro)


# ******************************************************************************

#random.seed(0)
random.seed(datetime.now())

# Definici�n del robot:
P_INICIAL = [2.,3.,0.] # Pose inicial (posici�n y orientacion)
V_LINEAL  = .7         # Velocidad lineal    (m/s)
V_ANGULAR = 140.       # Velocidad angular   (�/s)
FPS       = 10.        # Resoluci�n temporal (fps)
HOLONOMICO = 0         # Robot holon�mico
GIROPARADO = 0         # Si tiene que tener vel. lineal 0 para girar
LONGITUD   = .1        # Longitud del robot

N_PARTIC  = 50         # Tama�o del filtro de part�culas
N_INICIAL = 2000       # Tama�o inicial del filtro
N_PARTICULAS = 100      # Número de partículas generadas

# Definici�n de trayectorias:
trayectorias = [
    [[0,2],[4,2]],
    [[2,4],[4,0],[0,0]],
    [[2,4],[2,0],[0,2],[4,2]],
    [[2+2*sin(.4*pi*i),2+2*cos(.4*pi*i)] for i in range(5)],
    [[2+2*sin(.8*pi*i),2+2*cos(.8*pi*i)] for i in range(5)],
    [[2+2*sin(1.2*pi*i),2+2*cos(1.2*pi*i)] for i in range(5)],
    [[2*(i+1),4*(1+cos(pi*i))] for i in range(6)],
    [[2+.2*(22-i)*sin(.1*pi*i),2+.2*(22-i)*cos(.1*pi*i)] for i in range(20)],
    [[2+(22-i)/5*sin(.1*pi*i),2+(22-i)/5*cos(.1*pi*i)] for i in range(20)]
    ]

# Definici�n de los puntos objetivo:
if len(sys.argv)<2 or int(sys.argv[1])<0 or int(sys.argv[1])>=len(trayectorias):
  sys.exit(sys.argv[0]+" <�ndice entre 0 y "+str(len(trayectorias)-1)+">")
objetivos = trayectorias[int(sys.argv[1])]

# Definici�n de constantes:
EPSILON = .1                # Umbral de distancia
V = V_LINEAL/FPS            # Metros por fotograma
W = V_ANGULAR*pi/(180*FPS)  # Radianes por fotograma

trayectoria = trayectorias[int(sys.argv[1])]

real = robot()
real.set_noise(.01,.01,.1) # Ruido lineal / radial / de sensado
real.set(*P_INICIAL)

# Creamos robot ideal SIN ruido
ideal = robot()
ideal.set_noise(0, 0, .1)
ideal.set(*P_INICIAL)

trayectoriaIdeal = [ideal.pose()]
trayectoriaReal = [real.pose()]

# Creamos los bordes del gráfico
# Fijar los bordes del gr�fico
objT   = np.array(objetivos).T.tolist()
bordes = [min(objT[0]),max(objT[0]),min(objT[1]),max(objT[1])]
balizas = [1, 1]


trayectreal = [real.pose()]

# Array de robots
# Seleccionar hip�tesis de localizaci�n y actualizar la trayectoria
        # bordes, num_particulas, balizas, real, centro=[2,2], radio=1
filtro = genera_filtro(bordes, N_PARTICULAS, objetivos, real)

robotsList = newRobotList(real, ideal, filtro)

# Inicializar el array con los objetivos para cada particula
indicesObjetivosDeLosRobots = []
for i in range(len(robotsList)):
  indicesObjetivosDeLosRobots.append(0)

tiempo  = 0.
espacio = 0.

robotsHanLlegadoAlFinal = False
while not robotsHanLlegadoAlFinal:
  i = 0
  for particula in robotsList:
    posX = particula.pose()[0]
    posY = particula.pose()[1]

    # necesito localizar el mejor robot (ideal)
    aux0 = hipotesis(filtro)[0]
    aux1 = hipotesis(filtro)[1]
    aux2 = hipotesis(filtro)[2]
    ideal.set(aux0, aux1, aux2)
    trayectoriaIdeal.append(ideal.pose())

    # Compruebo si la particula ha llegado a su objetivo
    if distancia([posX, posY], trayectoria[indicesObjetivosDeLosRobots[i]]) < EPSILON:
      # Cambiar a siguiente objetivo
      indicesObjetivosDeLosRobots[i] += 1
      print "Indice objetivo final de la particula[", i, "]: ", indicesObjetivosDeLosRobots[i]
    else:
      # Mover particula hacia el objetivo
      objetivoActual = trayectoria[indicesObjetivosDeLosRobots[i]]
      poseRobot = particula.pose()

      # Si es el robot real, se mueve igual que el ideal
      if particula == real:
        trayectoriaReal.append(real.pose())
        objetivoActual = trayectoria[indicesObjetivosDeLosRobots[1]]
        poseRobot = ideal.pose()

      w = angulo_rel(poseRobot, objetivoActual)
      if w > W:  w =  W
      if w < -W: w = -W
      v = distancia(poseRobot, objetivoActual)
      if (v > V): v = V
      if (v < 0): v = 0

      if HOLONOMICO:
        if GIROPARADO and abs(w) > .01:
          v = 0
        particula.move(w, v)
      else:
        particula.move_triciclo(w,v,LONGITUD)

      espacio += v
      tiempo  += 1
    i += 1
    
  # Calcular la verosimilitud para cada robot y para la misma medida
  # para que solo varie la posicion. Modifica el peso del robot. 
  # Primero se debe mover el robot real, luego se calcula la
  # verosimilitud para el resto. Por eso en la lista de robots
  # el real se ha insertado el primero.

  # TODO: 
  # Cuando el measuremente prob da valores pequeños -> más partículas
  # y viceversa.
  # Vamos ajustando el número de partículas.
  pesoMedio = peso_medio(filtro, real, balizas)
  dispersionMedia = dispersion(filtro)
  print "peso medio: ", pesoMedio 
  print "dispersion: ", dispersionMedia
  if (pesoMedio < 1): # and dispersionMedia < 2):
    N_PARTICULAS = N_PARTICULAS * 1.5
  else:
    N_PARTICULAS = N_PARTICULAS * 0.5 + 20
  
  if (N_PARTICULAS > 2000):
    N_PARTICULAS = 1000
  
  #N_PARTICULAS = int(N_PARTICULAS)
  N_PARTICULAS = 100  # Le pongo una variable entera porque si no, peta
  filtro = resample(filtro, N_PARTICULAS)
  robotsList = newRobotList(real, ideal, filtro)

  # Mostramos
  mostrar(bordes,objetivos,trayectoria,trayectoriaReal,trayectoriaIdeal,robotsList)
  
  # Comprobar si todas las particulas han llegado al final
  numeroDeParticulasEnFinal = 0
  for j in range(len(indicesObjetivosDeLosRobots)):
    indiceUltimoObjetivo = len(trayectoria)
    if indicesObjetivosDeLosRobots[j] != indiceUltimoObjetivo:
      break
    numeroDeParticulasEnFinal += 1

  if (numeroDeParticulasEnFinal == len(robotsList)):
    robotsHanLlegadoAlFinal = True

if len(trayectoria) > 1000:
  print "<< ! >> Puede que no se haya alcanzado la posici�n final."
print "Recorrido: "+str(round(espacio,3))+"m / "+str(tiempo/FPS)+"s"
print "Error medio de la trayectoria: "+str(round(sum(\
    [distancia(trayectoria[i],trayectreal[i])\
    for i in range(len(trayectoria))])/tiempo,3))+"m"
raw_input()

