#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from pruebas.msg import msg_estados  # Importa el nuevo tipo de mensaje
from serial import Serial, SerialException # Importar la excepción SerialException para manejar errores de conexión

class Inicializacion:
    def __init__(self):
        # Inicializar el nodo
        rospy.init_node('inicializacion')

        # Crear el publicador para el tópico 'estados'
        self.estados_pub = rospy.Publisher('estados', msg_estados, queue_size=10)

        # Vector de estado inicial
        self.estado_inicial_deseado = msg_estados()
        self.estado_inicial_deseado.arduino = True
        self.estado_inicial_deseado.motores = True
        self.estado_inicial_deseado.modo = True
        self.estado_inicial_deseado.rutina = False
        self.estado_inicial_deseado.estado5 = False

        # Publicar el estado inicial
        self.publicar_estado_inicial()

    def publicar_estado_inicial(self):
        # Publicar el estado inicial en el tópico 'estados'
        self.estados_pub.publish(self.estado_inicial_deseado)
        rospy.loginfo("Estado inicial publicado: arduino={}, motores={}, modo={}, rutina={}, estado5={}".format(
            self.estado_inicial_deseado.arduino,
            self.estado_inicial_deseado.motores,
            self.estado_inicial_deseado.modo,
            self.estado_inicial_deseado.rutina,
            self.estado_inicial_deseado.estado5
        ))

    def monitorear_conexion_arduino(self):
        try:
            # Intentar abrir un puerto serie al dispositivo Arduino
            ser = Serial('/dev/ttyACM0', 115200, timeout=1)
            ser.close()  # Cerrar el puerto serie después de abrirlo para verificar si está disponible
            arduino_conectado = True
        except SerialException:
            arduino_conectado = False
            rospy.logwarn("No se pudo establecer conexión con el dispositivo Arduino.")


        # Actualizar el estado del Arduino en el estado inicial
        self.estado_inicial_deseado.arduino = arduino_conectado

        # Si el Arduino está desconectado, también se considera que los motores están desconectados
        if not arduino_conectado:
            self.estado_inicial_deseado.motores = False

        # Publicar el nuevo estado
        self.estados_pub.publish(self.estado_inicial_deseado)
        rospy.loginfo("Estado actualizado: arduino={}, motores={}, modo={}, rutina={}, estado5={}".format(
            self.estado_inicial_deseado.arduino,
            self.estado_inicial_deseado.motores,
            self.estado_inicial_deseado.modo,
            self.estado_inicial_deseado.rutina,
            self.estado_inicial_deseado.estado5
        ))

if __name__ == '__main__':
    try:
        nodo_inicializacion = Inicializacion()
       
        """"
        # Monitorear la conexión del Arduino cada 1 segundo
        rate = rospy.Rate(1)  # 1 Hz
        while not rospy.is_shutdown():
            nodo_inicializacion.monitorear_conexion_arduino()
            rate.sleep()
        """
    except rospy.ROSInterruptException:
        pass
