#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray, Int16MultiArray, String, Bool

from cobot_controladores.msg import punto_web
import numpy as np


import re


import time

class RUN:
    def __init__(self):
        rospy.init_node('run_Trayectoria')
        
        # Subscribirse al tópico pos_dy
        rospy.Subscriber('p_dy', Int16MultiArray, self.callback)
        

        #   ////////  PAGINA WEB    ////////

        
        # Subscribirse al tópico pos_dy
        rospy.Subscriber('orden_web', punto_web, self.acciones_web)
        
        # Definir el nombre del nuevo tópico
        
        self.informe_web = rospy.Publisher('informe_web', String, queue_size=10)
        
        
        ##################### Modo Trayectoria ###################
        rospy.Subscriber('/tipo_modo_lectura', Bool, self.modoT)
        self.modoTrayectoria=True
        self.rutina=[]
        self.rutinaPunto=[2047,2047,2047,2047,2047,2047]
        
        self.pub_cord_dy = rospy.Publisher('cord_dy', Int16MultiArray, queue_size=10)

    def modoT(self,data):
        self.modoTrayectoria=data.data
        self.rutina.clear()
        
        
    def callback(self, data):
        # Obtener el vector de coordenadas del mensaje recibido(bit)
        coordenadas = data.data
        
        if self.modoTrayectoria:
            for i, valor in enumerate(coordenadas):
                if valor != -1 or valor != 0:
                    self.rutinaPunto[i] = valor

            # Guarda una copia del vector actual
            self.rutina.append(list(self.rutinaPunto))
        

    def acciones_web(self, data):
    # Extraer la acción y el nombre (si existe) desde el mensaje recibido
        accion = data.orden[0]
        nombre = data.orden[1] if len(data.orden) > 1 else None  # Asigna None si no hay segundo términonombre
        
        rospy.loginfo(accion)
        rospy.loginfo(nombre)
    
    # Diccionario para formatear mensajes
        mensajes_informe = {
            #Prueba de Rutina
            'runTrayectoria': "corriendo trayectoria",
        }
    # Switch que maneja las diferentes acciones en base al valor de 'accion'
        if accion == 'runTrayectoria':
            rate = rospy.Rate(20)  
            for punto in self.rutina:
                msg = Int16MultiArray()
                msg.data = punto
                self.pub_cord_dy.publish(msg)
                rate.sleep()                        

if __name__ == '__main__':
    try:
        RUNTrayectoria = RUN()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
