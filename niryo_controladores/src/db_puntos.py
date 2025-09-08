#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from tinydb import TinyDB, Query

# Inicializar la base de datos
db = TinyDB('coordenadas.json')

# Función para escribir datos en la base de datos
def escribir_datos(coordenadas):
    # Convertir array de Float32 a un diccionario con claves 'x', 'y', 'z', 'a', 'b', 'c'
    coordenadas_dict = {'x': coordenadas[0], 'y': coordenadas[1], 'z': coordenadas[2], 
                        'a': coordenadas[3], 'b': coordenadas[4], 'c': coordenadas[5]}
    # Insertar coordenadas en la base de datos y obtener el doc_id asignado
    doc_id = db.insert(coordenadas_dict)
    # Agregar el doc_id a las coordenadas
    coordenadas_dict["id"] = doc_id

# Función para leer datos de un punto específico en la base de datos
def leer_punto(n):
    # Buscar en la base de datos el punto con el doc_id igual a n
    punto = db.get(doc_id=n)
    # Devolver solo las coordenadas sin el doc_id
    del punto.doc_id
    return punto

# Función para cambiar los datos de un punto específico en la base de datos
def cambiar_punto(n, nuevas_coordenadas):
    # Convertir array de Float32 a un diccionario con claves 'x', 'y', 'z', 'a', 'b', 'c'
    nuevas_coordenadas_dict = {'x': nuevas_coordenadas[0], 'y': nuevas_coordenadas[1], 'z': nuevas_coordenadas[2], 
                                'a': nuevas_coordenadas[3], 'b': nuevas_coordenadas[4], 'c': nuevas_coordenadas[5]}
    # Actualizar los datos del punto con doc_id igual a n
    db.update(nuevas_coordenadas_dict, doc_ids=[n])

# Función para eliminar los datos de un punto específico en la base de datos
def eliminar_punto(n):
    # Eliminar el punto con doc_id igual a n
    db.remove(doc_ids=[n])
    
# Función para leer datos de la base de datos
def leer_datos():
    return db.all()

# Función para eliminar todos los datos de la base de datos
def eliminar_todos_datos():
    #db.purge()
    db.truncate()

def main():
    rospy.init_node('coordenadas_node')

    """
    # Ejemplo de publicación de datos
    coordenadas = {"x": 10, "y": 20, "z": 30, "a": 40, "b": 50, "c": 60}
    escribir_datos(coordenadas)

    # Ejemplo de lectura de datos
    punto_leido = leer_punto(1)
    print("Datos del punto 1:")
    print(punto_leido)

    # Ejemplo de cambio de datos
    nuevas_coordenadas = {"x": 20, "y": 30, "z": 40, "a": 50, "b": 60, "c": 70}
    cambiar_punto(1, nuevas_coordenadas)

    # Ejemplo de eliminación de datos
    eliminar_punto(1)

    """

if __name__ == '__main__':
    main()
