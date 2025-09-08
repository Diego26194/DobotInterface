#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray, Int16MultiArray, String, Bool
from db_puntos3 import (
    escribir_datos,
    leer_datos,
    eliminar_todos_datos,
    leer_punto,
    cambiar_punto,
    eliminar_punto,
    agregar_punto_rutina,
    leer_rutina_completa,
    eliminar_todos_datos_rutina,
    eliminar_punto_rutina,
    leer_punto_rutina,
    eliminar_rutina,
    obtener_todas_rutinas,
    obtener_rutina,
    agregar_rutina,
    agregar_rutina_rutina,
)
from niryo_controladores.msg import punto_web, nombresPuntos
import numpy as np

class ModoLectura:
    def bit_grados(self, bit):
        grad = [(b * 360 / 4095 - 180) for b in bit]
        return grad

    def __init__(self):
        rospy.init_node('modo_lectura')
        
        # Subscribirse al tópico pos_dy
        rospy.Subscriber('p_dy', Int16MultiArray, self.callback)
        
        # Definir el nombre del nuevo tópico
        self.puntos_dy_pub = rospy.Publisher('puntos_dy', Float32MultiArray, queue_size=10)

        #   ////////  PAGINA WEB    ////////

        
        # Subscribirse al tópico pos_dy
        rospy.Subscriber('orden_web', punto_web, self.acciones_web)
        
        # Definir el nombre del nuevo tópico
        self.puntos_web = rospy.Publisher('puntodb', punto_web, queue_size=10)
        
        self.puntos_rutina = rospy.Publisher('puntoRutina', punto_web, queue_size=10)
        
        self.informe_web = rospy.Publisher('informe_web', String, queue_size=10)
        
        self.correr_rutina = rospy.Publisher('rutina', Bool, queue_size=10)
        
        self.nombres_puntos_tabla = rospy.Publisher('lista_puntosdb', nombresPuntos, queue_size=10)
        
        self.nombres_rutinas_tabla = rospy.Publisher('lista_rutinasdb', nombresPuntos, queue_size=10)
        
        self.velocidad=50
        self.ratio=0
        
        # Eliminar todos los datos de la base de datos al inicio
        #################### Eliminar todos los datos de la base de datos ####################
        eliminar_todos_datos()
        #################### Aquí termina la sección de eliminación de la base de datos ####################
        
    def callback(self, data):
        # Obtener el vector de coordenadas del mensaje recibido(bit)
        coordenadas = data.data

        # Verificar si las coordenadas tienen la longitud correcta
        if len(coordenadas) != 6:
            rospy.logwarn("El mensaje recibido no tiene el tamaño esperado (6)")
            return
        
        # Convertir de bits a grados antes de guardar en la base de datos
        coordenadas_grad = self.bit_grados(coordenadas)
        coordenadas_grad.append(self.velocidad)
        coordenadas_grad.append(self.ratio)
        
        
        id=agregar_punto_rutina(coordenadas_grad, 'PTP', None) 
            
            
        punto = leer_punto_rutina(id)  
        if punto:
            mensaje_puntoR = punto_web()
            mensaje_puntoR.orden = [punto['id'], punto['nombre'], punto['plan']]
            mensaje_puntoR.coordenadas = [
                punto['ang1'],
                punto['ang2'], 
                punto['ang3'], 
                punto['ang4'], 
                punto['ang5'], 
                punto['ang6'],
                punto['vel_esc'],
                punto['ratio'],
            ],
            self.puntos_rutina.publish(mensaje_puntoR)

            mensaje_informe = "Punto {} con los datos {} agregado correctamente"['ver'].format(punto.id, punto.nombre, mensaje_puntoR.coordenadas)
            self.informe_web.publish(mensaje_informe)
        else:
            self.informe_web.publish(f"Punto {id} no agregado correctamente.")
        

    # /////// pagina web ////////
    def acciones_web(self, data):
    # Extraer la acción y el nombre (si existe) desde el mensaje recibido
        accion = data.orden[0]
        nombre = data.orden[1] if len(data.orden) > 1 else None  # Asigna None si no hay segundo términonombre
        id = data.orden[2] if len(data.orden) > 2 else None
        plan= data.orden[3] if len(data.orden) > 3 else None
        rospy.loginfo(accion)
        rospy.loginfo(nombre)
    
    # Diccionario para formatear mensajes
        mensajes_informe = {
            'agregar': "Punto {} con los datos {} agregado correctamente",
            'eliminar': "Punto {} eliminado correctamente",
            'errorP': "ERROR- punto {} no se encuentra el Base de datos",
            'modificar': "Punto {} con los datos {} modificado correctamente",
            'subir_db': "Puntos de la base de datos cargados correctamente",
            'eliminarPunRut': "Punto {} de posicion {} eliminado correctamente",
            'errorEliminarPunRut': "Punto NO ELIMINADO, ERROR en Punto {} de posicion {} ",
            'addR': "Punto {} con los datos {} agregado correctamente",
            'ver': "Punto {} con los datos {}",
            'correr_rutina': "Punto de rutina agregado correctamente",
            'fin_rutina': "Rutina completada exitosamente",
            'comienzo_rutina': "Rutina iniciada, todos los puntos anteriores eliminados",
            'eliminarRutina':"Rutina {} eliminada correctamente",
            'ErrorR':'ERROR- Rutina {} no se encuentra en la base de datos',
            'addRT':"Rutina {} agregada correctamente a la tabla", #agrega rutina a tabla de rutina actual
            'subirR_db': "Rutinas de la base de datos cargados correctamente",
        }
    # Switch que maneja las diferentes acciones en base al valor de 'accion'
        if accion == 'agregar':
            if nombre:
                escribir_datos(data.coordenadas, nombre)  # Llamar a la función escribir_datos
                mensaje_informe = mensajes_informe['agregar'].format(nombre, data.coordenadas)
            else:
                escribir_datos(data.coordenadas, None)  # Si no se especifica nombre, pasa None
                mensaje_informe = "Punto agregado correctamente con nombre automático"

        # Publicar en el tópico 'informe_web'
            self.informe_web.publish(mensaje_informe)

        elif accion == 'eliminar':
            if nombre:
                resultado = eliminar_punto(nombre)  # Llamar a la función eliminar_punto
                if resultado:  
                    mensaje_informe = mensajes_informe['eliminar'].format(nombre)                    
                else:
                    mensaje_informe = mensajes_informe['errorModificar'].format(nombre)
                    mensaje_errorAddP = punto_web()
                    mensaje_errorAddP.orden=['errorP'].format(nombre)
                self.informe_web.publish(mensaje_informe)
                

        elif accion == 'modificar':
            if nombre:
                resultado = cambiar_punto(nombre, data.coordenadas)
                if resultado:  
                    mensaje_informe = mensajes_informe['modificar'].format(nombre, data.coordenadas)                    
                else:
                    mensaje_informe = mensajes_informe['errorModifica'].format(nombre, data.coordenadas)
                    mensaje_errorAddP = punto_web()
                    mensaje_errorAddP.orden=['errorP'].format(nombre)
                self.informe_web.publish(mensaje_informe)
                    

        elif accion == 'subir_db':
        # Obtener todos los puntos de la base de datos
            puntos = leer_datos()  # Esto retorna db.all()
            nombres = [punto['nombre'] for punto in puntos]  # Extraer todos los nombres
        
        # Publicar los nombres de los puntos en el tópico 'puntodb'
            mensaje_puntodb = nombresPuntos()
            mensaje_puntodb.nombres = nombres
            self.nombres_puntos_tabla.publish(mensaje_puntodb)
        
        # Publicar en el tópico 'informe_web'
            self.informe_web.publish(mensajes_informe['subir_db'])
            
        elif accion == 'addR':
            if nombre:
                id=agregar_punto_rutina(data.coordenadas, plan, nombre)  # Llamar a la función escribir_datos
                mensaje_informe = mensajes_informe['addR'].format(nombre, data.coordenadas)
            else:
                id=agregar_punto_rutina(data.coordenadas, plan, None)  # Si no se especifica nombre, pasa None
                mensaje_informe = "Punto agregado correctamente con nombre automático"
                
                
            punto = leer_punto_rutina(id)  
            if punto:
                mensaje_puntoR = punto_web()
                mensaje_puntoR.orden = ['addP', punto['nombre'], punto['plan']]
                mensaje_puntoR.coordenadas = [
                    punto['pos'],
                    punto['ang1'],
                    punto['ang2'], 
                    punto['ang3'], 
                    punto['ang4'], 
                    punto['ang5'], 
                    punto['ang6'],
                    punto['vel_esc'],
                    punto['ratio'],
                    punto['wait'],
                ],
                self.puntos_rutina.publish(mensaje_puntoR)

                mensaje_informe = mensajes_informe['ver'].format(id, nombre, mensaje_puntoR.coordenadas)
                self.informe_web.publish(mensaje_informe)
            else:
                self.informe_web.publish(f"Punto {id} no agregado correctamente.")


        # Publicar en el tópico 'informe_web'
            self.informe_web.publish(mensaje_informe)

        
        elif accion == 'eliminarPunRut':          
            mensaje_puntoR = punto_web()
            mensaje_puntoR.orden=['ErrordelPR']   
            errores = False         
            for i in range(len(data.coordenadas)):
              punto = leer_punto_rutina(data.coordenadas[i])        
              if not punto or punto.nombre != data.orden[i+1]:
                mensaje_puntoR.coordenadas.append(data.coordenadas[i])
                mensaje_informe = mensajes_informe['errorEliminarPunRut'].format(data.orden[i+1], item)                
                errores = True
            if not errores:
              mensaje_puntoR.orden=['delPR']
              for item in data.coordenadas:
                punto = leer_punto_rutina(item)
                eliminar_punto_rutina(item)
                mensaje_informe = mensajes_informe['eliminarPunRut'].format(punto.nombre, item)
              mensaje_puntoR.coordenadas=data.coordenadas
            self.puntos_rutina.publish(mensaje_puntoR)
             
             

        elif accion == 'ver':
            if nombre:
                punto = leer_punto(nombre)  # Llamar a la función leer_punto
                if punto:
                    mensaje_puntodb = punto_web()
                    mensaje_puntodb.orden = [nombre]
                    mensaje_puntodb.coordenadas = [punto['ang1'], punto['ang2'], punto['ang3'], punto['ang4'], punto['ang5'], punto['ang6']]
                    self.puntos_web.publish(mensaje_puntodb)

                    mensaje_informe = mensajes_informe['ver'].format(nombre, mensaje_puntodb.coordenadas)
                    self.informe_web.publish(mensaje_informe)
                else:
                    self.informe_web.publish(f"Punto {nombre} no encontrado.")
            else:
                self.informe_web.publish("Error: no se ha proporcionado un nombre para el punto.")

        elif accion == 'eliminarRutina':
            if nombre:
                resultado=eliminar_rutina(nombre)
                if resultado:
                    mensaje_informe = mensajes_informe['eliminarRutina'].format(nombre)
                else:
                    mensaje_informe = mensajes_informe['eliminarRutina'].format(nombre)
                    mensaje_errorDelR = punto_web()
                    mensaje_errorDelR.orden=['errorR']
                self.informe_web.publish(mensaje_informe)     

        elif accion == 'addRT':
            if data.coordenadas:
                posicion=data.coordenadas[0]
            else:
                posicion=None
            if nombre:
                resultado=agregar_rutina_rutina(nombre,posicion)
                
                punto = leer_punto_rutina(resultado)  
            if punto:
                mensaje_rutinaR = punto_web()
                mensaje_rutinaR.orden = ['addRT', punto['nombre']]
                mensaje_rutinaR.coordenadas = [
                    punto['wait'],
                ],
                self.puntos_rutina.publish(mensaje_rutinaR)

                mensaje_informe = mensajes_informe['ver'].format(id, nombre, mensaje_rutinaR.coordenadas)
                self.informe_web.publish(mensaje_informe)
            else:
                self.informe_web.publish(f"Rutina {nombre} no agregado correctamente.")
                
        elif accion == 'subirR_db':
        # Publicar los nombres de los puntos en el tópico 'puntodb'
            mensaje_puntodb = nombresPuntos()
            mensaje_puntodb.nombres = obtener_todas_rutinas()
            self.nombres_rutinas_tabla.publish(mensaje_puntodb)
        
        # Publicar en el tópico 'informe_web'
            self.informe_web.publish(mensajes_informe['subirR_db'])
                    



if __name__ == '__main__':
    try:
        modo_lectura = ModoLectura()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
