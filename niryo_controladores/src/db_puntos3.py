#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
from tinydb import TinyDB, Query, where

# Inicializar la base de datos
db = TinyDB('coordenadas.json')
rutina_actual = db.table('rutina_actual')
puntos_invariables= db.table('puntos_invariables') #Puntos que no se pueden editar
Puntos_usados=db.table('Puntos_usado')
Puntos_editables=db.table('Puntos_editables')

#tablas = db.tables()  este comando devuelve todas las tablas existentes en db, deberia excluir '_default'  ejemplo de lo que devuelvee {'_default', 'usuarios', 'puntos', 'trayectorias'}

# Función para escribir datos en la base de datos
def escribir_datos(coordenadas, identificador=None):
    # Convertir array de Float32 a un diccionario con claves 'ang1', 'ang2', 'ang3', 'ang4', 'ang5', 'ang6'
    coordenadas_dict = {'ang1': coordenadas[0], 'ang2': coordenadas[1], 'ang3': coordenadas[2], 
                        'ang4': coordenadas[3], 'ang5': coordenadas[4], 'ang6': coordenadas[5]}
    
    # Insertar coordenadas en la base de datos y obtener el doc_id asignado
    doc_id = Puntos_editables.insert(coordenadas_dict)
    
    # Verificar si se recibió un identificador
    if identificador is None:
        # Si no se proporcionó un identificador, asignar 'p{doc_id}'
        nombre = f"p{doc_id}"
    else:
        # Si se proporcionó, usar ese identificador
        nombre = identificador

    # Actualizar el registro con el nombre asignado
    Puntos_editables.update({'nombre': nombre}, doc_ids=[doc_id])
    
    # Agregar el doc_id y nombre al diccionario original para referencia
    coordenadas_dict["id"] = doc_id
    coordenadas_dict["nombre"] = nombre


# Función para leer datos de un punto específico en la base de datos
# Función para leer un punto específico en la base de datos usando 'nombre'
def leer_punto(nombre):
    Punto = Query()
    # Buscar en la base de datos el punto con el nombre dado
    punto = Puntos_editables.get(Punto.nombre == nombre)
    if punto:
        # Devolver solo las coordenadas sin el campo 'doc_id'
        punto.pop('doc_id', None)  # Elimina el doc_id si existe
        return punto
    else:
        return None  # Si no se encuentra el punto

# Función para cambiar los datos de un punto específico en la base de datos usando 'nombre'
def cambiar_punto(nombre, nuevas_coordenadas):
    Punto = Query()
    if Puntos_editables.get(Punto.nombre == nombre):
    # Convertir array de Float32 a un diccionario con claves 'ang1', 'ang2', etc.
        nuevas_coordenadas_dict = {'ang1': nuevas_coordenadas[0], 'ang2': nuevas_coordenadas[1], 'ang3': nuevas_coordenadas[2], 
                                'ang4': nuevas_coordenadas[3], 'ang5': nuevas_coordenadas[4], 'ang6': nuevas_coordenadas[5]}
    
    # Actualizar los datos del punto que tiene el nombre dado
        Puntos_editables.update(nuevas_coordenadas_dict, Punto.nombre == nombre)
        return True
    else:
        return None

# Función para eliminar los datos de un punto específico en la base de datos usando 'nombre'
def eliminar_punto(nombre):
    Punto = Query()
    if Puntos_editables.get(Punto.nombre == nombre):
        # Eliminar el punto con el nombre dado
        Puntos_editables.remove(Punto.nombre == nombre)
        return True
    else:
        return None
    
# Función para leer datos de la base de datos
def leer_datos():
    return Puntos_editables.all()

# Función para eliminar todos los datos de la base de datos
def eliminar_todos_datos():
    #db.purge()
    Puntos_editables.truncate()

############# Rutina actual #############

# Función para escribir datos en la base de datos
def agregar_punto_rutina(coordenadas, plan , identificador=None, pos=None):
    # Convertir array de Float32 a un diccionario con claves 'x', 'y', 'z', 'a', 'b', 'c'
    coordenadas_dict = {
        'ang1': coordenadas[0],'ang2': coordenadas[1],'ang3': coordenadas[2],
        'ang4': coordenadas[3],'ang5': coordenadas[4],'ang6': coordenadas[5],
        'vel_esc': coordenadas[6],'ratio': coordenadas[7],'wait': 0,'plan': plan , 'tipo':'punto'}
    
    # --- Manejo del campo "pos" ---
    if pos is None:
        # Si no se pasa posición, insertar al final
        max_pos = max([p.get("pos", 0) for p in rutina_actual.all()] or [0])
        pos = max_pos + 1
    else:
        # Si se pasa posición, hay que desplazar los que están desde pos en adelante
        for punto in rutina_actual.search(where("pos") >= pos):
            rutina_actual.update({"pos": punto["pos"] + 1}, doc_ids=[punto.doc_id])

    coordenadas_dict["pos"] = pos
    
    # Insertar coordenadas en la base de datos y obtener el doc_id asignado
    doc_id = rutina_actual.insert(coordenadas_dict)
    # Verificar si se recibió un identificador
    if identificador is None:
        # Si no se proporcionó un identificador, asignar 'p{doc_id}'
        nombre = f"rut{doc_id}"
    else:
        # Si se proporcionó, usar ese identificador
        nombre = identificador

    # Actualizar el registro con el nombre asignado
    rutina_actual.update({'nombre': nombre}, doc_ids=[doc_id])
    
    # Agregar el doc_id y nombre al diccionario original para referencia
    coordenadas_dict["id"] = doc_id
    coordenadas_dict["nombre"] = nombre
    return doc_id

# Función para escribir datos en la base de datos
def agregar_rutina_rutina(identificador, pos=None):
        
    # --- Manejo del campo "pos" ---
    if pos is None:
        # Si no se pasa posición, insertar al final
        max_pos = max([p.get("pos", 0) for p in rutina_actual.all()] or [0])
        pos = max_pos + 1
    else:
        # Si se pasa posición, hay que desplazar los que están desde pos en adelante
        for punto in rutina_actual.search(where("pos") >= pos):
            rutina_actual.update({"pos": punto["pos"] + 1}, doc_ids=[punto.doc_id])

    posicion = {'pos': pos, 'tipo':'rutina', 'wait': 0}
    
    # Insertar coordenadas en la base de datos y obtener el doc_id asignado
    doc_id = rutina_actual.insert(posicion)
    # Verificar si se recibió un identificador
    if identificador is None:
        # Si no se proporcionó un identificador, asignar 'p{doc_id}'
        nombre = f"Rutina{doc_id}"
    else:
        # Si se proporcionó, usar ese identificador
        nombre = identificador

    # Actualizar el registro con el nombre asignado
    rutina_actual.update({'nombre': nombre}, doc_ids=[doc_id])
    
    # Agregar el doc_id y nombre al diccionario original para referencia
    posicion["id"] = doc_id
    posicion["nombre"] = nombre
    return doc_id

# Función para eliminar los datos de un punto específico en la base de datos usando 'nombre'
def eliminar_punto_rutina(posicion):
    Punto = Query()
    if rutina_actual.get(Punto.pos == posicion): #rutina_actual.search(Punto.pos == posicion) (usar ".search" devuelve todos los elementos con esa caracteristica, y ."get" solo el primero que encuentra)
        # Eliminar el punto con el nombre dado
        rutina_actual.remove(Punto.pos == posicion)
        return True
    else:
        return None
        
    

# Función para leer datos de un punto específico en la base de datos
def leer_punto_rutina(n):
    # Buscar en la base de datos el punto con el doc_id igual a n
    punto = rutina_actual.get(doc_id=n)
    if punto:  # Verifica si se encontró el punto
        # Devolver solo las coordenadas sin el doc_id
        #del punto.doc_id
        return punto
    else:
        return None  # Retorna None si no se encuentra el punto
    
    #leer y ordenar en ese orden pos
#puntos = sorted(tabla.all(), key=lambda x: x['pos'])
#for p in puntos:
#    print(p['nombre'], p['pos'])

# Función para eliminar todos los datos de la base de datos
def eliminar_todos_datos_rutina():
    #db.purge()
    rutina_actual.truncate()

# Función para leer datos de la base de datos
def leer_rutina_completa():
    return rutina_actual.all()

############# Manejar Rutinas #############

def eliminar_rutina(rutina):
    if rutina in db.tables():
        db.drop_table(rutina)
        return True
    else: 
        return None

def obtener_todas_rutinas():
    # Tablas que quiero excluir
    excluir = {"rutina_actual", "puntos_invariables", "Puntos_usados", "Puntos_editables", "_default"}
    
    # Obtengo todas las tablas, resto las excluidas y devuelvo como lista ordenada
    tablas_rutinas = list(db.tables() - excluir)
    
    return tablas_rutinas

def obtener_rutina(nombre_rutina):
    if nombre_rutina in db.tables():
        tabla = db.table(nombre_rutina)   
        return tabla.all()                
    else: 
        print("La no tabla existe")
        return None

def agregar_rutina(nombre):
    if nombre in db.tables():
        print("La tabla existe")
        return None
    else:
        datos = rutina_actual.all()
        db.table(nombre).insert_multiple(datos)
        return True

    

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
