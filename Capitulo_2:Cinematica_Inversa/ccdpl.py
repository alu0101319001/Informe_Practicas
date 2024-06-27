#! /usr/bin/env python
# -*- coding: utf-8 -*-

# Robótica Computacional - 
# Grado en Ingeniería Informática (Cuarto)
# Práctica: Resolución de la cinemática inversa mediante CCD
#           (Cyclic Coordinate Descent).

import sys
import os
from math import *
import numpy as np
import matplotlib
if os.environ.get('DISPLAY'):
    matplotlib.use('Qt5Agg')  # Usar TkAgg si hay una pantalla disponible
else:
    matplotlib.use('Agg')  # Usar Agg si no hay una pantalla disponible (headless)

import matplotlib.pyplot as plt
import colorsys as cs
from matplotlib.patches import Circle


import file_reader as fr

# ******************************************************************************
# Declaración de funciones

def muestra_origenes(O,final=0):
  # Muestra los orígenes de coordenadas para cada articulación
  print('Origenes de coordenadas:')
  for i in range(len(O)):
    print('(O'+str(i)+')0\t= '+str([round(j,3) for j in O[i]]))
  if final:
    print('E.Final = '+str([round(j,3) for j in final]))

def muestra_robot(O,obj):
  # Muestra el robot graficamente
  plt.figure()
  plt.xlim(-L,L)
  plt.ylim(-L,L)
  T = [np.array(o).T.tolist() for o in O]
  for i in range(len(T)):
    plt.plot(T[i][0], T[i][1], '-o', color=cs.hsv_to_rgb(i/float(len(T)),1,1))
  plt.plot(obj[0], obj[1], '*')
  plt.pause(0.0001)
  plt.show()
  
#  input()
  plt.close()

def matriz_T(d,th,a,al):
  # Calcula la matriz T (ángulos de entrada en grados)
  
  return [[cos(th), -sin(th)*cos(al),  sin(th)*sin(al), a*cos(th)]
         ,[sin(th),  cos(th)*cos(al), -sin(al)*cos(th), a*sin(th)]
         ,[      0,          sin(al),          cos(al),         d]
         ,[      0,                0,                0,         1]
         ]

def cin_dir(th,a):
  #Sea 'th' el vector de thetas
  #Sea 'a'  el vector de longitudes
  T = np.identity(4)
  o = [[0,0]]
  for i in range(len(th)):
    T = np.dot(T,matriz_T(0,th[i],a[i],0))
    tmp=np.dot(T,[0,0,0,1])
    o.append([tmp[0],tmp[1]])
  return o

def natural_movements(th): 
  # print(f'input movement: {th}')
  while th > pi:
      th -= 2 * pi
  while th <= -pi:
      th += 2 * pi
  # print(f'natural movement: {th}')
  return th

def limit_restriction(i, limits, value):
  if value < limits[-1-i][0]:
    result = limits[-1-i][0]
  elif value > limits[-1-i][1]:
    result = limits[-1-i][1]
  else:
    result = value
  # print(f'Limite restriction: {value, limits[-1-i][0], limits[-1-i][1], result}')
  return result

def resolve_revolute(i, O, objetivo, th, limits):
  # cálculo de la cinemática inversa de una articulacion de revolución:
  al1 = (atan2((objetivo[1] - O[-1][-1-(i+1)][1]), (objetivo[0] - O[-1][-1-(i+1)][0])))
  al2 = (atan2((O[-1][-1][1] - O[-1][-1-(i+1)][1]), (O[-1][-1][0] - O[-1][-1-(i+1)][0])))
  incth = al1 - al2
  th[-1-i] = th[-1-i] + incth
  th[-1-i] = natural_movements(th[-1-i])
  th[-1-i] = limit_restriction(i, limits, th[-1-i])
  return th

def resolve_prismatic(i, O, objetivo, a, th, limits):
  # Cálculo de la cinemática inversa de una articulación prismática:
  w = 0
  for j in (range(len(sec) - i)):
    w += th[-1-i-j]
  vu = [cos(w), sin(w)]
  v = [objetivo[0] - O[-1][-1][0], objetivo[1] - O[-1][-1][1]]
  d = sum(vu[k] * v[k] for k in range(len(v)))
  a[-1-i] = a[-1-i] + d
  a[-1-i] = limit_restriction(i, limits, a[-1-i])
  # print(f'prismatic resolution: {w, vu, v, d, a[-1-i]}')
  return a

def estimar_viabilidad(objetivo, a, limits, sec):
    longitudes_maximas = []  # Almacenará las longitudes máximas de cada articulación

    for i, articulacion in enumerate(sec):
        if articulacion == 'r':
            # Para articulaciones de tipo revolución, se utiliza el valor 'a' dado
            longitudes_maximas.append(a[i])
        elif articulacion == 'p':
            # Para articulaciones de tipo prismatic, se utiliza el límite como longitud máxima
            longitudes_maximas.append(limits[i][1])

    # Calcular la distancia al objetivo
    distancia_objetivo = sqrt(objetivo[0]**2 + objetivo[1]**2)

    # Verificar si el punto objetivo es alcanzable dentro del radio de acción de los segmentos
    if distancia_objetivo <= sum(longitudes_maximas):
        return True

    return False
  
# ******************************************************************************
# Cálculo de la cinemática inversa de forma iterativa por el método CCD
REVOLUTE = 'r'
PRISMATIC = 'p'

# introducción del input
if len(sys.argv) != 2:
  sys.exit("python " + sys.argv[0])
file_name = sys.argv[1]

objetivo, EPSILON, sec, limits, th, a = fr.process_input(file_name)
if not fr.check_input(objetivo, EPSILON, sec, limits, th, a):
  raise

L = sum(a)
if not estimar_viabilidad(objetivo, a, limits, sec):
  print('El objetivo está fuera del alcance máximo, por lo tanto no hay solución.')
  sys.exit(1)

O=cin_dir(th,a)
#O=zeros(len(th)+1) # Reservamos estructura en memoria
 # Calculamos la posicion inicial
print ("- Posicion inicial:")
muestra_origenes(O)

dist = float("inf")
prev = 0.
iteracion = 1
while (dist > EPSILON and abs(prev-dist) > EPSILON/100.):
  prev = dist
  O=[cin_dir(th,a)]
  # Para cada combinación de articulaciones:
  for i in range(len(sec)):
    # cálculo de la cinemática inversa:
    if sec[-1-i] == REVOLUTE:
      th = resolve_revolute(i, O, objetivo, th, limits)
    elif sec[-1-i] == PRISMATIC:
      a = resolve_prismatic(i, O, objetivo, a, th, limits)
    else:
      print(f'ERROR: Valor de la secuencia desconocido: {sec[-1-i]} con i = {i}')
      raise
    O.append(cin_dir(th,a))
    # print(O, al1, al2, incth, th)

  dist = np.linalg.norm(np.subtract(objetivo,O[-1][-1]))
  print ("\n- Iteracion " + str(iteracion) + ':')
  print ("Valor de theta = " + str(th))
  print ("Valor de a = " + str(a))
  muestra_origenes(O[-1])
  muestra_robot(O,objetivo)
  print ("Distancia al objetivo = " + str(round(dist,5)))
  iteracion+=1
  O[0]=O[-1]

if dist <= EPSILON:
  print ("\n" + str(iteracion) + " iteraciones para converger.")
else:
  print ("\nNo hay convergencia tras " + str(iteracion) + " iteraciones.")
print ("- Umbral de convergencia epsilon: " + str(EPSILON))
print ("- Distancia al objetivo:          " + str(round(dist,5)))
print ("- Valores finales de las articulaciones:")
for i in range(len(th)):
  print ("  theta" + str(i+1) + " = " + str(round(th[i],3)))
for i in range(len(th)):
  print ("  L" + str(i+1) + "     = " + str(round(a[i],3)))
