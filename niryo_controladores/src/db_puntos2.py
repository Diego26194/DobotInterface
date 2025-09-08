#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from tinydb import TinyDB, Query

# Inicializar la base de datos
db = TinyDB('coordenadas.json')
rutina_actual = db.table('rutina_actual')

# Función para escribir datos en la base de datos
def escribir_datos(coordenadas, identificador=None):
    # Convertir array de Float32 a un diccionario con claves 'ang1', 'ang2', 'ang3', 'ang4', 'ang5', 'ang6'
    coordenadas_dict = {'ang1': coordenadas[0], 'ang2': coordenadas[1], 'ang3': coordenadas[2], 
                        'ang4': coordenadas[3], 'ang5': coordenadas[4], 'ang6': coordenadas[5]}
    
    # Insertar coordenadas en la base de datos y obtener el doc_id asignado
    doc_id = db.insert(coordenadas_dict)
    
    # Verificar si se recibió un identificador
    if identificador is None:
        # Si no se proporcionó un identificador, asignar 'p{doc_id}'
        nombre = f"p{doc_id}"
    else:
        # Si se proporcionó, usar ese identificador
        nombre = identificador

    # Actualizar el registro con el nombre asignado
    db.update({'nombre': nombre}, doc_ids=[doc_id])
    
    # Agregar el doc_id y nombre al diccionario original para referencia
    coordenadas_dict["id"] = doc_id
    coordenadas_dict["nombre"] = nombre


# Función para leer datos de un punto específico en la base de datos
# Función para leer un punto específico en la base de datos usando 'nombre'
def leer_punto(nombre):
    Punto = Query()
    # Buscar en la base de datos el punto con el nombre dado
    punto = db.get(Punto.nombre == nombre)
    if punto:
        # Devolver solo las coordenadas sin el campo 'doc_id'
        punto.pop('doc_id', None)  # Elimina el doc_id si existe
        return punto
    else:
        return None  # Si no se encuentra el punto

# Función para cambiar los datos de un punto específico en la base de datos usando 'nombre'
def cambiar_punto(nombre, nuevas_coordenadas):
    Punto = Query()
    # Convertir array de Float32 a un diccionario con claves 'ang1', 'ang2', etc.
    nuevas_coordenadas_dict = {'ang1': nuevas_coordenadas[0], 'ang2': nuevas_coordenadas[1], 'ang3': nuevas_coordenadas[2], 
                               'ang4': nuevas_coordenadas[3], 'ang5': nuevas_coordenadas[4], 'ang6': nuevas_coordenadas[5]}
    
    # Actualizar los datos del punto que tiene el nombre dado
    db.update(nuevas_coordenadas_dict, Punto.nombre == nombre)

# Función para eliminar los datos de un punto específico en la base de datos usando 'nombre'
def eliminar_punto(nombre):
    Punto = Query()
    # Eliminar el punto con el nombre dado
    db.remove(Punto.nombre == nombre)
    
# Función para leer datos de la base de datos
def leer_datos():
    return db.all()

# Función para eliminar todos los datos de la base de datos
def eliminar_todos_datos():
    #db.purge()
    db.truncate()



# Función para escribir datos en la base de datos
def escribir_datos_rutina(coordenadas):
    # Convertir array de Float32 a un diccionario con claves 'x', 'y', 'z', 'a', 'b', 'c'
    coordenadas_dict = {
        'ang1': coordenadas[0],'ang2': coordenadas[1],'ang3': coordenadas[2],
        'ang4': coordenadas[3],'ang5': coordenadas[4],'ang6': coordenadas[5],'vel_esc': coordenadas[6]}
    
    # Insertar coordenadas en la tabla 'rutina_actual' y obtener el doc_id asignado
    doc_id = rutina_actual.insert(coordenadas_dict)
    # Agregar el doc_id a las coordenadas
    coordenadas_dict["id"] = doc_id
    return coordenadas_dict  # Devuelve el diccionario con el ID incluido

# Función para leer datos de un punto específico en la base de datos
def leer_punto_rutina(n):
    # Buscar en la base de datos el punto con el doc_id igual a n
    punto = rutina_actual.get(doc_id=n)
    if punto:  # Verifica si se encontró el punto
        # Devolver solo las coordenadas sin el doc_id
        del punto.doc_id
        return punto
    else:
        return None  # Retorna None si no se encuentra el punto

# Función para eliminar todos los datos de la base de datos
def eliminar_todos_datos_rutina():
    #db.purge()
    rutina_actual.truncate()

# Función para leer datos de la base de datos
def leer_datos_rutina():
    return rutina_actual.all()

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
