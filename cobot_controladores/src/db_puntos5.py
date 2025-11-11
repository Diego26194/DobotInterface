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
    coordenadas_dict = {'coordenadasAngulares': coordenadas}
    
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
    if punto is not None and isinstance(punto, dict):
        punto.pop('doc_id', None)  # Elimina el doc_id si existe
        return punto
    else:
        return None  # Si no se encuentra el punto

# Función para cambiar los datos de un punto específico en la base de datos usando 'nombre'
def cambiar_punto(nombre, nuevas_coordenadas):
    Punto = Query()
    if Puntos_editables.get(Punto.nombre == nombre):    
        nuevas_coordenadas_dict = {'coordenadasAngulares': nuevas_coordenadas}
    
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

# Función para escribir punto en "rutina_actual"
def agregar_punto_rutina(pose, coorCartesianas_euler, vel_esc, ratio, plan , identificador=None, pos=None, wait=None):
    # Convertir array de Float32 a un diccionario con claves 'x', 'y', 'z', 'a', 'b', 'c'
    coorCartesianas_quat=pose_to_dict(pose)
    coordenadas_dict = {
        'coordenadasCEuler': coorCartesianas_euler, 'coordenadasCQuaterniones': coorCartesianas_quat, 
        'vel_esc': vel_esc, 'ratio': ratio,'plan': plan,
        'wait': wait if wait is not None else 0}
        
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
    return pos

# Función para agregar una rutina como instruccion en la rutina actual
def agregar_rutina_rutina(identificador, pos=None, wait=None):
        
    # --- Manejo del campo "pos" ---
    if pos is None:
        # Si no se pasa posición, insertar al final
        max_pos = max([p.get("pos", 0) for p in rutina_actual.all()] or [0])
        pos = max_pos + 1
    else:
        # Si se pasa posición, hay que desplazar los que están desde pos en adelante
        for punto in rutina_actual.search(where("pos") >= pos):
            rutina_actual.update({"pos": punto["pos"] + 1}, doc_ids=[punto.doc_id])

    rutina = {'nombre': identificador,'pos': pos, 'plan':"Rutina", 
        'wait': wait if wait is not None else 0}
    
    # Insertar coordenadas en la base de datos y obtener el doc_id asignado
    doc_id = rutina_actual.insert(rutina)
   
    # Agregar el doc_id y nombre al diccionario original para referencia
    rutina["id"] = doc_id
    agregar_rutina_control(identificador)
    return pos

# Función para agregar una rutina como instruccion en la rutina actual
def agregar_trayectoria_rutina(trayectoria,poseInit,identificador=None, pos=None, wait=None):
               
    # --- Manejo del campo "pos" ---
    if pos is None:
        # Si no se pasa posición, insertar al final
        max_pos = max([p.get("pos", 0) for p in rutina_actual.all()] or [0])
        pos = max_pos + 1
    else:
        # Si se pasa posición, hay que desplazar los que están desde pos en adelante
        for punto in rutina_actual.search(where("pos") >= pos):
            rutina_actual.update({"pos": punto["pos"] + 1}, doc_ids=[punto.doc_id])

    rutina = {'puntos':trayectoria, 'coordenadasCQuaterniones': poseInit,
              'pos': pos, 'plan':"Trayectoria",'wait': wait if wait is not None else 0}
    
    # Insertar coordenadas en la base de datos y obtener el doc_id asignado
    doc_id = rutina_actual.insert(rutina)
    
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
    rutina["id"] = doc_id
    rutina["nombre"] = nombre
    #agregar_rutina_control(identificador)
    return pos

def editar_punto_rutina(pose, coorCartesianas_euler, vel_esc, ratio,wait, posicion, plan, identificador=None):
    Punto = Query()
    pos=posicion
    coorCartesianas_quat=pose_to_dict(pose)
    # Buscar el punto por pos
    punto = rutina_actual.get(Punto.pos == pos)
    if not punto or (identificador is not None and punto.get('nombre') != identificador):
        return None  # No coincide posición o nombre
    
    # Crear diccionario con los nuevos valores
    nuevas_coordenadas_dict = {
        'coordenadasCEuler': coorCartesianas_euler, 'coordenadasCQuaterniones': coorCartesianas_quat, 
        'vel_esc': vel_esc, 'ratio': ratio,'wait': wait,'plan': plan
    }
    
    # Manejo de nombre
    if identificador is not None:
        nuevas_coordenadas_dict['nombre'] = identificador
    else:
        # Generar uno único basado en el doc_id
        doc_id = rutina_actual.get(doc_id=punto.doc_id).doc_id
        nuevas_coordenadas_dict['nombre'] = f"rut{doc_id}"
    
    # Actualizar en la base de datos
    rutina_actual.update(nuevas_coordenadas_dict, doc_ids=[punto.doc_id])
    
    # Recuperar y devolver el punto actualizado
    return rutina_actual.get(doc_id=punto.doc_id)

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
    Punto = Query()
    punto = rutina_actual.get(Punto.pos == n)  # Busca donde 'pos' == n
    if punto:
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
    rutina_actual.insert({
    "id": "control",       # clave fija para poder buscarlo
    "rutinas": [],         # vector de rutinas guardadas
    "error": False,        # bandera de error
    "fechas": []           # lista de fechas o timestamps
})

# Función para leer datos de la base de datos
def leer_rutina_completa():
    return rutina_actual.all()

def leer_rutina_sin_euler():
    """Devuelve la rutina completa pero excluyendo 'coordenadasCEuler' """
    docs = rutina_actual.all()
    resultado = []

    for doc in docs:
        doc_copy = dict(doc)  # copiamos para no modificar el original
        doc_copy.pop('coordenadasCEuler', None)  # elimina si existe
        resultado.append(doc_copy)

    # ordenar: control primero, luego por pos
    resultado.sort(key=lambda d: (0 if d.get("id") == "control" else 1, d.get("pos", float('inf'))))
    return resultado


def leer_rutina_sin_quaterniones():
    """Devuelve la rutina completa pero excluyendo 'coordenadasCQuaterniones' """
    docs = rutina_actual.all()
    resultado = []

    for doc in docs:
        doc_copy = dict(doc)
        doc_copy.pop('coordenadasCQuaterniones', None)  # elimina si existe
        resultado.append(doc_copy)

    resultado.sort(key=lambda d: (0 if d.get("id") == "control" else 1, d.get("pos", float('inf'))))
    return resultado

############# Registro_control #############

def agregar_rutina_control(nombre_rutina: str):
    Control = Query()
    # Obtenemos el documento de control
    doc = rutina_actual.get(Control.id == "control")
    if not doc:
        raise Exception("No existe documento de control en la tabla")

    # Actualizamos el campo 'rutinas' agregando el nuevo nombre
    nuevas_rutinas = doc["rutinas"] + [nombre_rutina]
    rutina_actual.update({"rutinas": nuevas_rutinas}, Control.id == "control")
    
def eliminar_rutina_control(nombre_rutina: str):
    Control = Query()
    doc = rutina_actual.get(Control.id == "control")
    if not doc:
        raise Exception("No existe documento de control en la tabla")

    rutinas = doc["rutinas"].copy()  # copiamos para no modificar en vivo

    try:
        rutinas.remove(nombre_rutina)  # elimina solo la primera coincidencia
    except ValueError:
        return False  # no estaba en la lista

    rutina_actual.update({"rutinas": rutinas}, Control.id == "control")
    return True

def error_true_control():
    Control = Query()
    # Obtenemos el documento de control
    doc = rutina_actual.get(Control.id == "control")
    if not doc:
        raise Exception("No existe documento de control en la tabla")
    rutina_actual.update({"error": True}, Control.id == "control")
    
def error_false_control():
    Control = Query()
    # Obtenemos el documento de control
    doc = rutina_actual.get(Control.id == "control")
    if not doc:
        raise Exception("No existe documento de control en la tabla")
    rutina_actual.update({"error": False}, Control.id == "control")

# actualiza linea de control en rutina actual
def actualizar_control(rutinas: list, fechas: list):
    Control = Query()
    doc = rutina_actual.get(Control.id == "control")
    if not doc:
        raise Exception("No existe documento de control en la tabla rutina_actual")

    rutina_actual.update({
        "rutinas": rutinas,
        "fechas": fechas
    }, Control.id == "control")

# Verifica que las rutinas registradas en el control existan en la DB.       
def verificar_rutinas_control():
    Control = Query()
    doc_control = rutina_actual.get(Control.id == "control")
    if not doc_control:
        raise Exception("No existe documento de control en la tabla")

    # Rutinas registradas en control (puede haber duplicados)
    rutinas_control = doc_control.get("rutinas", [])

    # Rutinas efectivamente existentes
    rutinas_existentes = obtener_todas_rutinas()

    # Detectamos las faltantes (sin repetidos → usamos set)
    rutinas_faltantes = list(set(rutinas_control) - set(rutinas_existentes))

    if rutinas_faltantes: # Si faltan, activa error y devuelve lista de faltantes
        error_true_control()
    else:                 # Si todas existen, desactiva error y devuelve lista vacía.
        error_false_control() 

    return rutinas_faltantes

    
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
    
### Extras ###
def pose_to_dict(pose):
    return {
        "position": {
            "x": pose.position.x,
            "y": pose.position.y,
            "z": pose.position.z
        },
        "orientation": {
            "x": pose.orientation.x,
            "y": pose.orientation.y,
            "z": pose.orientation.z,
            "w": pose.orientation.w
        }
    }

    

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
