"""
Funciones para lectura de fichero que procese una entrada concreta
para resolver el problema de CCD 

Formato: 
  Posición D:                   x y
  Epsilon:                      E
  Secuencia articulaciones:     r r p r r p r
  Límites:                      [min, max] [min, max] [min, max] [min, max] [min, max] [min, max] [min, max]
  Configuracion inicial:        
    th:                         0 0 0 0 0 0 0
     a:                         2 2 0 2 2 0 2
"""
import re
from math import *

def process_input(file_name):
    with open(file_name, 'r') as f:
        line = f.readline()
        d_split = line.split()
        d = [float(i) for i in d_split]
        
        line = f.readline()
        e = float(line)
        
        line = f.readline()
        art_sec = line.split()

        line = f.readline()
        # Esta expresión regular busca coincidencias de pares de números (positivos, negativos, enteros o decimales) dentro de corchetes cuadrados y separados por una coma.
        limits_str = re.findall(r'\[(-?[\d.]+),(-?[\d.]+)\]', line)
        # Determinar el tipo de límite (ángulo o distancia) basado en la secuencia
        limits = []
        for i, limit in enumerate(limits_str):
            if art_sec[i] == 'p':
                # Si la articulación es prismática, almacena los límites como distancia
                limits.append([float(limit[0]), float(limit[1])])
            else:
                # Si es revoluta, almacena los límites como ángulos en radianes
                limits.append([radians(float(limit[0])), radians(float(limit[1]))])

        line = f.readline()
        th_split = line.split()
        th = [radians(float(i)) for i in th_split]
        
        line = f.readline()
        a_split = line.split()
        a = [float(i) for i in a_split]

        return d, e, art_sec, limits, th, a
    
def check_input(d, e, sec, limits, th, a):
    try:
        if len(d) != 2 or not all(isinstance(pos, float) for pos in d): 
            raise Exception(f'Objetivo no válido: {d}')
        if not isinstance(e, float) or e <= 0:
            raise Exception(f'EPSILON no es válido: {e}')
        if not all(isinstance(art, str) and len(art) == 1 and (art == 'p' or art == 'r') for art in sec):
            raise Exception(f'Secuencia de articulaciones no válido: {sec}')
        if len(limits) != len(sec) or not all(isinstance(value, list) and len(value) == 2 and all(isinstance(val, float) for val in value) for value in limits):
            raise Exception(f'Límites no es válido: {limits}')
        if len(th) != len(sec) or not all(isinstance(value, float) for value in th):
            raise Exception(f'Configuración inicial th no valido: {th}')
        if len(a) != len(sec) or not all(isinstance(value, float) for value in a):
            raise Exception(f'Configuración inicial a no válida: {a}')
        else:
            return True
        

    except Exception as error:
        print(f"Error: input no correcto, revisar fichero de entrada - {error}")
        return False
    
def limit_type(sec, index):
    # Determinar el tipo de límite en el índice dado basado en la secuencia
    if sec[index] == 'p':
        return True
    else:
        return False



