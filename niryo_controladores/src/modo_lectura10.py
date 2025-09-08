#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray, Int16MultiArray
from db_puntos import escribir_datos, leer_datos, eliminar_todos_datos
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
        escribir_datos(coordenadas_rad)
        
        # Publicar todos los puntos en el tópico puntos_dy
        puntos = leer_datos()
        for punto in puntos:
            puntos_msg = Float32MultiArray()
            puntos_msg.data = [punto['x'], punto['y'], punto['z'], punto['a'], punto['b'], punto['c']]
            self.puntos_dy_pub.publish(puntos_msg)


if __name__ == '__main__':
    try:
        modo_lectura = ModoLectura()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
