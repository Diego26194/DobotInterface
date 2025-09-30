#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray, Int16MultiArray, String, Bool
from db_puntos2 import escribir_datos, leer_datos, eliminar_todos_datos,leer_punto,cambiar_punto,eliminar_punto, escribir_datos_rutina,leer_datos_rutina, eliminar_todos_datos_rutina
from cobot_controladores.msg import punto_web
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
        # Definir el nombre del nuevo tópico
        self.informe_web = rospy.Publisher('informe_web', String, queue_size=10)
        # Definir el nombre del nuevo tópico
        self.correr_rutina = rospy.Publisher('rutina', Bool, queue_size=10)
        
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
        coordenadas_rad = self.bit_grados(coordenadas)
        
        # Guardar las coordenadas en grados en la base de datos
        escribir_datos_rutina(coordenadas_rad)
        
        # Publicar todos los puntos en el tópico puntos_dy
        puntos = leer_datos_rutina()
        for punto in puntos:
            puntos_msg = Float32MultiArray()
            puntos_msg.data = [punto['ang1'], punto['ang2'], punto['ang3'], punto['ang4'], punto['ang5'], punto['ang6']]
            self.puntos_dy_pub.publish(puntos_msg)

    # /////// pagina web ////////
    def acciones_web(self, data):
    # Extraer la acción y el nombre (si existe) desde el mensaje recibido
        accion = data.orden[0]
        nombre = data.orden[1] if len(data.orden) > 1 else None  # Asigna None si no hay segundo términonombre
        rospy.loginfo(accion)
        rospy.loginfo(nombre)
    
    # Diccionario para formatear mensajes
        mensajes_informe = {
            'agregar': "Punto {} con los datos {} agregado correctamente",
            'eliminar': "Punto {} eliminado correctamente",
            'modificar': "Punto {} con los datos {} modificado correctamente",
            'subir_db': "Puntos de la base de datos cargados correctamente",
            'ver': "Punto {} con los datos {}",
            'correr_rutina': "Punto de rutina agregado correctamente",
            'fin_rutina': "Rutina completada exitosamente",
            'comienzo_rutina': "Rutina iniciada, todos los puntos anteriores eliminados"
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
                eliminar_punto(nombre)  # Llamar a la función eliminar_punto
                mensaje_informe = mensajes_informe['eliminar'].format(nombre)
                self.informe_web.publish(mensaje_informe)

        elif accion == 'modificar':
            if nombre:
                cambiar_punto(nombre, data.coordenadas)  # Llamar a la función cambiar_punto
                mensaje_informe = mensajes_informe['modificar'].format(nombre, data.coordenadas)
                self.informe_web.publish(mensaje_informe)

        elif accion == 'subir_db':
        # Obtener todos los puntos de la base de datos
            puntos = leer_datos()  # Esto retorna db.all()
            nombres = [punto['nombre'] for punto in puntos]  # Extraer todos los nombres
        
        # Publicar los nombres de los puntos en el tópico 'puntodb'
            mensaje_puntodb = punto_web()
            mensaje_puntodb.orden = nombres
            self.puntos_web.publish(mensaje_puntodb)
        
        # Publicar en el tópico 'informe_web'
            self.informe_web.publish(mensajes_informe['subir_db'])

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

        elif accion == 'correr_rutina':
            if nombre == 'comienzo_rutina':
            # Eliminar todos los datos de la tabla rutina_actual
                eliminar_todos_datos_rutina()

            # Publicar mensaje de que la rutina ha comenzado y se han eliminado los datos previos
                mensaje_informe = mensajes_informe['comienzo_rutina']
                self.informe_web.publish(mensaje_informe)
        
            elif nombre == 'fin_rutina':
            # Mensaje de finalización de rutina
                self.informe_web.publish(mensajes_informe['fin_rutina'])
                

                self.correr_rutina.publish(Bool(data=True))
        
            else:
            # Almacenar las coordenadas de este punto en la tabla rutina_actual

            # Publicar confirmación de que el punto fue agregado a la rutina
                mensaje_informe = mensajes_informe['correr_rutina']
                self.informe_web.publish(mensaje_informe)
            
            escribir_datos_rutina(data.coordenadas)



if __name__ == '__main__':
    try:
        modo_lectura = ModoLectura()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
