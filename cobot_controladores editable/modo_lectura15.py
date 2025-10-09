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
    editar_punto_rutina,
    verificar_rutinas_control,
    actualizar_control,
    leer_rutina_sin_quaterniones,
)
from cobot_controladores.msg import punto_web, nombresPuntos, punto_real
import numpy as np

import tf
from moveit_commander import MoveGroupCommander
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionFK, GetPositionFKRequest
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

class ModoLectura:
    def bit_grados(self, bit):
        grad = [(b * 360 / 4095 - 180) for b in bit]
        return grad
    
    def grados_rad(self, grad):
        rad = [g * np.pi / 180 for g in grad]
        return rad
    
    def rad_grados(self, rad):
        grad=[r * 180 / np.pi for r in rad]
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
        
        self.pub_pos_real = rospy.Publisher('pos_real', punto_real, queue_size=10)
        
        rospy.Subscriber('pos_dy', Int16MultiArray, self.pasar_punto_real)
        
        self.velocidad=50
        self.ratio=0
        
        # Eliminar todos los datos de la base de datos al inicio
        #################### Eliminar todos los datos de la base de datos ####################
        eliminar_todos_datos()
        eliminar_todos_datos_rutina()
        #################### Aquí termina la sección de eliminación de la base de datos ####################
        
        self.angulos = [0] * 6
        
        
        #################### Inicializa el conversor de coordenadas para un robot definido en MoveIt. ####################
        self.move_group = MoveGroupCommander("cobot_arm")
        self.group_name = "cobot_arm"
        self.base_frame = self.move_group.get_planning_frame()
        self.move_group.set_planner_id("PTP")
        
    def pasar_punto_real(self, data):        
        entrada = data.data
        
        errores = []
        for i, valor in enumerate(entrada):
            if valor == -1:
                errores.append(False)
            else:       
                errores.append(True)        
                self.angulos[i] = self.bit_grados([valor])[0]  
        
        coordenadas_rad = self.grados_rad(self.angulos)
        coordenadas_cart = self.AngulosArticulares_a_cartesianasEuler(coordenadas_rad)
        
        msg = punto_real()
        msg.ang = [int(a) for a in self.angulos]          
        msg.cart = [int(c) for c in coordenadas_cart]    
        msg.error = errores                              
        
        self.pub_pos_real.publish(msg)
        
    def callback(self, data):
        # Obtener el vector de coordenadas del mensaje recibido(bit)
        coordenadas = data.data

        # Verificar si las coordenadas tienen la longitud correcta
        if len(coordenadas) != 6:
            rospy.logwarn("El mensaje recibido no tiene el tamaño esperado (6)")
            return
        
        # Filtrar valores -1 (motores desconectados)
        for i, valor in enumerate(coordenadas):
            if valor != -1:        
                self.angulos[i] = self.bit_grados([valor])[0]

        # Intentar convertir ángulos a pose
        coorCartesianas_quat = self.AngulosArticulares_a_pose(self.angulos)

        # Si los ángulos están fuera de los límites, cortar el proceso
        if coorCartesianas_quat is None:
            msg = "Ángulos fuera de los límites de trabajo"
            rospy.logwarn(msg)
            self.informe_web.publish(msg)
            return

        # Si pasó el chequeo, calculamos coordenadas en Euler
        coorCartesianas_euler = self.AngulosArticulares_a_cartesianasEuler(self.angulos)

        # Guardar el punto en la rutina
        id = agregar_punto_rutina(
            coorCartesianas_quat,
            coorCartesianas_euler,
            self.velocidad,
            self.ratio,
            'PTP',
            None
        ) 

        # Recuperar el punto recién agregado
        punto = leer_punto_rutina(id)  
        if punto:
            mensaje_puntoR = punto_web()
            mensaje_puntoR.orden = [punto.doc_id, punto['nombre'], punto['plan']]
            mensaje_puntoR.coordenadas = list(punto['coordenadasCEuler']) + [
                int(punto['vel_esc']),
                int(punto['ratio']),
                int(punto['wait']),
                int(punto['pos']),
            ]
            self.puntos_rutina.publish(mensaje_puntoR)

            mensaje_informe = (
                f"Punto {punto['nombre']} agregado correctamente "
                f"con datos {mensaje_puntoR.coordenadas}, {self.velocidad}, {self.ratio}"
            )
            self.informe_web.publish(mensaje_informe)
        else:
            self.informe_web.publish(f"Punto {id} no agregado correctamente.")

        

    # /////// pagina web ////////
    
    # Refresca los puntos de la rutina ingresada en la pagina web
    def refrescarRutinaActual(self): # (self, rutina: list)
        
        rutina=leer_rutina_sin_quaterniones()
        
        rospy.logwarn(rutina)
        
        mensaje_puntoR = punto_web()
        mensaje_puntoR.orden = ['vaciarTabla']
        self.puntos_rutina.publish(mensaje_puntoR)
       
        for punto in rutina:
            # Saltar la fila de control
            if punto.get("id") == "control":
                continue

            if punto.get("rutina") is False:
                
                rospy.logwarn(punto)
                
                # Punto normal
                mensaje_puntoR = punto_web()
                mensaje_puntoR.orden = ['addP', punto['nombre'], punto['plan']]
                mensaje_puntoR.coordenadas = list(punto['coordenadasCEuler']) + [
                    int(punto['vel_esc']),
                    int(punto['ratio']),
                    int(punto['wait']),
                    int(punto['pos']),
                ]
                
                rospy.logwarn(mensaje_puntoR.coordenadas)
                rospy.logwarn('hasta aca todo bien')
                rospy.logwarn('hasta aca todo bien')
                rospy.logwarn('hasta aca todo bien')
                
                rospy.logwarn("DEBUG coordenadas -> %s", mensaje_puntoR.coordenadas)
                for i, c in enumerate(mensaje_puntoR.coordenadas):
                    rospy.logwarn("%d: %s (%s)", i, c, type(c))
                
                self.puntos_rutina.publish(mensaje_puntoR)

            elif punto.get("rutina") is True:
                # Sub-rutina
                mensaje_rutinaR = punto_web()
                mensaje_rutinaR.orden = ['addRT', punto['nombre']]
                mensaje_rutinaR.coordenadas = [
                    punto['wait'],
                    punto['pos'],
                ]
                self.puntos_rutina.publish(mensaje_rutinaR)

    def acciones_web(self, data):
    # Extraer la acción y el nombre (si existe) desde el mensaje recibido
        accion = data.orden[0]
        nombre = data.orden[1] if len(data.orden) > 1 else None  # Asigna None si no hay segundo términonombre
        plan= data.orden[2] if len(data.orden) > 2 else None
        rospy.loginfo(accion)
        rospy.loginfo(nombre)
    
    # Diccionario para formatear mensajes
        mensajes_informe = {
            #Tabla DB Puntos
            'agregar': "Punto {} con los datos {} agregado correctamente",
            'Erroragregar': "Punto con los datos nombre {} ya existente",            
            'eliminar': "Punto {} eliminado correctamente",
            'errorP': "ERROR- punto {} no se encuentra el Base de datos",
            'modificar': "Punto {} con los datos {} modificado correctamente",
            'subir_db': "Puntos de la base de datos cargados correctamente",
            'ver': "Punto {} con los datos {}",
            'ang-cart':"",
            'cart-ang':"",
            #Tabla Rutina actual
            'eliminarPunRut': "Punto {} de posicion {} eliminado correctamente",
            'errorPunRut': " ERROR en Punto {} de posicion {} ",
            'addPT': "Punto {} con los datos {},{},{} agregado correctamente",  
            'addRT':"Rutina {} agregada correctamente a la tabla",
            'editPR': "Punto {} con los datos {} modificado correctamente",
            'refresRut':' Rutina actual refrescada ',
                        
            'correr_rutina': "Punto de rutina agregado correctamente",
            'fin_rutina': "Rutina completada exitosamente",
            'comienzo_rutina': "Rutina iniciada, todos los puntos anteriores eliminados",
            # Rutinas
            'eliminarRutina':"Rutina {} eliminada correctamente",
            'ErrorR':'ERROR- Rutina {} no se encuentra en la base de datos', #agrega rutina a tabla de rutina actual
            'subirR_db': "Rutinas de la base de datos cargados correctamente",            
            'cargarRRA': "Rutinas {} cargados ",        
            'addRutina': "Rutinas {} agregada correctamente ",
        }
    # Switch que maneja las diferentes acciones en base al valor de 'accion'
        if accion == 'agregar':
            if nombre:
                if not leer_punto(nombre):                    
                    escribir_datos(data.coordenadas, nombre)  # Llamar a la función escribir_datos
                    mensaje_informe = mensajes_informe['agregar'].format(nombre, data.coordenadas)
                    
                    #### Refresco la lista de puntos ####
                    puntos = leer_datos()  # Esto retorna db.all()
                    nombres = [punto['nombre'] for punto in puntos]  # Extraer todos los nombres
                
                # Publicar los nombres de los puntos en el tópico 'puntodb'
                    mensaje_puntodb = nombresPuntos()
                    mensaje_puntodb.nombres = nombres
                    self.nombres_puntos_tabla.publish(mensaje_puntodb)
                
                # Publicar en el tópico 'informe_web'
                    self.informe_web.publish(mensajes_informe['subir_db'])
                else:
                    mensaje_informe = mensajes_informe['Erroragregar'].format(nombre)                    
            else:
                escribir_datos(data.coordenadas, None)  # Si no se especifica nombre, pasa None
                mensaje_informe = "Punto agregado correctamente con nombre automático"
                
                 #### Refresco la lista de puntos ####
                puntos = leer_datos()  # Esto retorna db.all()
                nombres = [punto['nombre'] for punto in puntos]  # Extraer todos los nombres
            
            # Publicar los nombres de los puntos en el tópico 'puntodb'
                mensaje_puntodb = nombresPuntos()
                mensaje_puntodb.nombres = nombres
                self.nombres_puntos_tabla.publish(mensaje_puntodb)
            
            # Publicar en el tópico 'informe_web'
                self.informe_web.publish(mensajes_informe['subir_db'])

        # Publicar en el tópico 'informe_web'
            self.informe_web.publish(mensaje_informe)

        elif accion == 'eliminar':
            if nombre:
                resultado = eliminar_punto(nombre)  # Llamar a la función eliminar_punto
                if resultado:  
                    mensaje_informe = mensajes_informe['eliminar'].format(nombre)                    
                else:
                    mensaje_informe = mensajes_informe['errorP'].format(nombre)
                    mensaje_error = punto_web()
                    mensaje_error.orden=['errorP'].format(nombre)
                    self.puntos_rutina.publish(mensaje_error)
                self.informe_web.publish(mensaje_informe)
                

        elif accion == 'modificar':
            if nombre:
                resultado = cambiar_punto(nombre, data.coordenadas)
                if resultado:  
                    mensaje_informe = mensajes_informe['modificar'].format(nombre, data.coordenadas)                    
                else:
                    mensaje_informe = mensajes_informe['errorModifica'].format(nombre, data.coordenadas)
                    mensaje_error = punto_web()
                    mensaje_error.orden=['errorP'].format(nombre)
                    self.puntos_rutina.publish(mensaje_error)
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
            
        elif accion == 'addPT':
            coorCartesianas_quat=self.AngulosArticulares_a_pose(data.coordenadas[:6])
            coorCartesianas_euler=self.AngulosArticulares_a_cartesianasEuler(data.coordenadas[:6])
            vel_esc=data.coordenadas[6]
            ratio=data.coordenadas[7]
            if nombre:
                posicion=agregar_punto_rutina(coorCartesianas_quat, coorCartesianas_euler, vel_esc, ratio, plan, nombre)                 
            else:
                posicion=agregar_punto_rutina(coorCartesianas_quat, coorCartesianas_euler, vel_esc, ratio, plan, None)  # Si no se especifica nombre, pasa None
                mensaje_informe = "Punto agregado correctamente con nombre automático"
                self.informe_web.publish(mensaje_informe)
                
            punto = leer_punto_rutina(posicion)  
            if punto:
                mensaje_puntoR = punto_web()
                mensaje_puntoR.orden = ['addP', punto['nombre'], punto['plan']]
                mensaje_puntoR.coordenadas = list(punto['coordenadasCEuler']) + [
                    int(punto['vel_esc']),
                    int(punto['ratio']),
                    int(punto['wait']),
                    int(punto['pos']),
                ]
                rospy.logwarn(mensaje_puntoR.orden)
                rospy.logwarn(mensaje_puntoR.coordenadas)
                rospy.logwarn("DEBUG coordenadas -> %s", mensaje_puntoR.coordenadas)
                for i, c in enumerate(mensaje_puntoR.coordenadas):
                    rospy.logwarn("%d: %s (%s)", i, c, type(c))
                self.puntos_rutina.publish(mensaje_puntoR)

                mensaje_informe = mensajes_informe['addPT'].format(nombre, coorCartesianas_euler, vel_esc, ratio)
                self.informe_web.publish(mensaje_informe)
            else:
                self.informe_web.publish(f"Punto {posicion} no agregado correctamente.")


        # Publicar en el tópico 'informe_web'
            
            

        elif accion == 'editPR':            
            coorCartesianas_euler=data.coordenadas[:6]
            coorCartesianas_quat=self.cartesianasEuler_a_pose(coorCartesianas_euler)
            vel_esc=data.coordenadas[6]
            ratio=data.coordenadas[7]
            wait=data.coordenadas[8]
            posicion=data.coordenadas[9]
            mensaje_puntoR = punto_web() 
            if self.pose_a_AngulosArticulares(coorCartesianas_quat):
                
                if nombre:
                    point=editar_punto_rutina(coorCartesianas_quat, coorCartesianas_euler, vel_esc, ratio,wait, posicion, plan, nombre)  
                    # mensaje_informe = mensajes_informe['editPR'].format(nombre, data.coordenadas)
                else:
                    point=editar_punto_rutina(coorCartesianas_quat, coorCartesianas_euler, vel_esc, ratio,wait, posicion, plan, None)  # Si no se especifica nombre, pasa None
                    # mensaje_informe = "Punto agregado correctamente con nombre automático"
                   
                if point:    
                    #punto = leer_punto_rutina(posicion)  
                                    
                    mensaje_puntoR.orden = ['editPR']
                    mensaje_puntoR.orden = ['editPR', point['nombre'], point['plan']]
                    mensaje_puntoR.coordenadas = list(point['coordenadasCEuler']) + [
                        int(point['vel_esc']),
                        int(point['ratio']),
                        int(point['wait']),
                        int(point['pos']),
                    ]
                    
                    
                    rospy.logwarn("DEBUG coordenadas -> %s", mensaje_puntoR.coordenadas)
                    for i, c in enumerate(mensaje_puntoR.coordenadas):
                        rospy.logwarn("%d: %s (%s)", i, c, type(c))
                        
                        
                    self.puntos_rutina.publish(mensaje_puntoR)

                    mensaje_informe = mensajes_informe['editPR'].format(point['nombre'], mensaje_puntoR.coordenadas)
                    self.informe_web.publish(mensaje_informe)
                else:
                    mensaje_puntoR.orden = ['errorPunRut']
                    self.puntos_rutina.publish(mensaje_puntoR)
                    mensaje_informe ='Rutina NO Editada,'+ mensajes_informe['errorPunRut'].format(nombre, posicion)
                    self.informe_web.publish(mensaje_informe)
            else:
                mensaje_puntoR.orden = ['errorPunRut2']
                self.puntos_rutina.publish(mensaje_puntoR)
                self.informe_web.publish(f"Error Coordenadas fuera de limites")

        
        elif accion == 'refresRut':          
            self.refrescarRutinaActual()
            mensaje_informe = mensajes_informe['refresRut']
            self.informe_web.publish(mensaje_informe)

        
        elif accion == 'eliminarPunRut':          
            mensaje_puntoR = punto_web()
            mensaje_puntoR.orden=['ErrordelPR']   
            errores = False         
            for i in range(len(data.coordenadas)):
              punto = leer_punto_rutina(data.coordenadas[i])        
              if not punto or punto['nombre'] != data.orden[i+1]:
                mensaje_puntoR.coordenadas.append(data.coordenadas[i])
                mensaje_informe ='Punto NO ELIMINADO,'+ mensajes_informe['errorPunRut'].format(data.orden[i+1], item)
                self.informe_web.publish(mensaje_informe)                
                errores = True
            if not errores:
              mensaje_puntoR.orden=['delPR']
              for item in data.coordenadas:
                punto = leer_punto_rutina(item)
                eliminar_punto_rutina(item)
                mensaje_informe = mensajes_informe['eliminarPunRut'].format(punto['nombre'], item)
              mensaje_puntoR.coordenadas=data.coordenadas
              self.informe_web.publish(mensaje_informe)
              self.refrescarRutinaActual() #ver si dejar esto o no
             
             

        elif accion == 'ver':
            if nombre:
                punto = leer_punto(nombre)  # Llamar a la función leer_punto
                if punto:
                    CoordenadasCartesianas=self.AngulosArticulares_a_cartesianasEuler(punto['coordenadasAngulares'])
                    if CoordenadasCartesianas:
                        mensaje_puntodb = punto_web()
                        mensaje_puntodb.orden = ['Punto', nombre]                        
                        mensaje_puntodb.coordenadas = [*(punto['coordenadasAngulares']),*CoordenadasCartesianas]
                        self.puntos_web.publish(mensaje_puntodb)

                        mensaje_informe = mensajes_informe['ver'].format(nombre, mensaje_puntodb.coordenadas)
                        self.informe_web.publish(mensaje_informe)
                    else:
                        self.informe_web.publish(f"Coordenadas no validas")
                else:
                    self.informe_web.publish(f"Punto {nombre} no encontrado.")
            else:
                self.informe_web.publish("Error: no se ha proporcionado un nombre para el punto.")             
             

        elif accion == 'ang-cart':            
            CoordenadasCartesianas=self.AngulosArticulares_a_cartesianasEuler(data.coordenadas)
            mensaje_puntodb = punto_web()
            if CoordenadasCartesianas:
                mensaje_puntodb.orden = ['Cart']                        
                mensaje_puntodb.coordenadas = CoordenadasCartesianas
                self.puntos_web.publish(mensaje_puntodb)

                #mensaje_informe = mensajes_informe['ang-cart'].format(nombre, mensaje_puntodb.coordenadas)
                #self.informe_web.publish(mensaje_informe)
            else:
                mensaje_puntodb.orden = ['ECart']                        
                self.puntos_web.publish(mensaje_puntodb)
                self.informe_web.publish(f"Coordenadas no validas")             
             

        elif accion == 'cart-ang':
            CoordenadasAngulares=self.cartesianaEuler_a_AngulosArticulares(data.coordenadas)
            mensaje_puntodb = punto_web()
            if CoordenadasAngulares:
                mensaje_puntodb.orden = ['Ang']                        
                mensaje_puntodb.coordenadas = CoordenadasAngulares
                self.puntos_web.publish(mensaje_puntodb)

                #mensaje_informe = mensajes_informe['ang-cart'].format(nombre, mensaje_puntodb.coordenadas)
                #self.informe_web.publish(mensaje_informe)
            else:
                mensaje_puntodb.orden = ['EAng']                        
                self.puntos_web.publish(mensaje_puntodb)
                self.informe_web.publish(f"Coordenadas no validas")


        elif accion == 'eliminarRutina':
            if nombre:
                resultado=eliminar_rutina(nombre)
                if resultado:
                    mensaje_informe = mensajes_informe['eliminarRutina'].format(nombre)
                else:
                    mensaje_informe = mensajes_informe['ErrorR'].format(nombre)
                    mensaje_errorDelR = punto_web()
                    mensaje_errorDelR.orden=['errorR']
                    self.puntos_rutina.publish(mensaje_errorDelR)
                self.informe_web.publish(mensaje_informe)  
        
        
        elif accion == 'cargarRRA':
            if nombre:
                rutina=obtener_rutina(nombre)
                faltantes = verificar_rutinas_control()
                
                if rutina:
                    if faltantes:  
                        # Al menos una rutina declarada en control no existe en DB
                        mensaje_informe = f"⚠️ Rutinas faltantes: {', '.join(faltantes)}"
                        mensaje_errorDelR = punto_web()
                        mensaje_errorDelR.orden = ['errorRR'] #['errorRR', *faltantes]  #cambiar por esto al agregar el cambio de color en la tabla
                        self.puntos_rutina.publish(mensaje_errorDelR)  
                        self.informe_web.publish(mensaje_informe)                       
                    for doc in rutina:
                        if doc.get("id") == "control":
                            actualizar_control(
                                doc.get("rutinas", []),
                                doc.get("fechas", [])
                            )

                        elif doc.get("rutina") == False:
                            coorCartesianas_quat = self.dict_to_pose( doc.get("coordenadasCQuaterniones", []) )
                            coorCartesianas_euler = self.pose_a_cartesianasEuler(coorCartesianas_quat)
                            vel_esc = doc.get("vel_esc", 0)
                            ratio = doc.get("ratio", 0)
                            plan = doc.get("plan", "")
                            identificador = doc.get("nombre")
                            pos = doc.get("pos")

                            agregar_punto_rutina(coorCartesianas_quat, coorCartesianas_euler,
                                                vel_esc, ratio, plan,
                                                identificador, pos)

                        elif doc.get("rutina") == True:
                            identificador = doc.get("nombre")
                            pos = doc.get("pos")
                            agregar_rutina_rutina(identificador, pos)
                            
                    self.refrescarRutinaActual()                            
                            
                    mensaje_informe = mensajes_informe['cargarRRA'].format(nombre)
                    
                else:
                    mensaje_informe = mensajes_informe['ErrorR'].format(nombre)
                    mensaje_errorDelR = punto_web()
                    mensaje_errorDelR.orden=['errorR']
                    self.puntos_rutina.publish(mensaje_errorDelR)
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
                    punto['pos'],
                ],
                self.puntos_rutina.publish(mensaje_rutinaR)

                mensaje_informe = mensajes_informe['addRT'].format(nombre)
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
        
                
        elif accion == 'addRutina':
        # Publicar los nombres de los puntos en el tópico 'puntodb'
            if not nombre:
                rutinas = obtener_todas_rutinas()
                
                # Filtrar solo las que son "rutina N"
                genericas = []
                for r in rutinas:
                    m = re.match(r"rutina(\d+)$", r)
                    if m:
                        genericas.append(int(m.group(1)))
                
                # Encontrar el próximo número disponible
                if genericas:
                    nuevo_num = max(genericas) + 1
                else:
                    nuevo_num = 1
                nombre=f"rutina{nuevo_num}"
                        
            if agregar_rutina(nombre):
                mensaje_informe = mensajes_informe['addRutina'].format(nombre)
                self.informe_web.publish(mensaje_informe)
                
                ###### Refrescar tabla de rutinas ########
                
                # Publicar los nombres de los puntos en el tópico 'puntodb'
                mensaje_puntodb = nombresPuntos()
                mensaje_puntodb.nombres = obtener_todas_rutinas()
                self.nombres_rutinas_tabla.publish(mensaje_puntodb)
            
            # Publicar en el tópico 'informe_web'
                self.informe_web.publish(mensajes_informe['subirR_db'])
            else:                
                self.informe_web.publish(f"Ya existe rutina con el nombre {nombre}")
            
            
            
    ###### ConversorCoordenadas ######## 
    
    def dict_to_pose(self, pose_dict):
        
        pose = Pose()
        pose.position = Point(
            x=pose_dict["position"]["x"],
            y=pose_dict["position"]["y"],
            z=pose_dict["position"]["z"]
        )
        pose.orientation = Quaternion(
            x=pose_dict["orientation"]["x"],
            y=pose_dict["orientation"]["y"],
            z=pose_dict["orientation"]["z"],
            w=pose_dict["orientation"]["w"]
        )
        return pose       
        
    # ===============================
    # 1. Ángulos articulares -> Pose
    # ===============================
    def AngulosArticulares_a_pose(self, cord_ang_grados):   
        	
        try:                     
            
            cord_ang_rad = self.grados_rad(cord_ang_grados)
            
            rospy.wait_for_service('/compute_fk')
            fk = rospy.ServiceProxy('/compute_fk', GetPositionFK)

            req = GetPositionFKRequest()
            req.header.frame_id = self.base_frame
            #req.header.frame_id = "world" 
            req.fk_link_names = [self.move_group.get_end_effector_link()]

            # Estado articular
            robot_state = RobotState()
            js = JointState()
            js.name = self.move_group.get_active_joints()
            js.position = cord_ang_rad
            
            print("Joints activos:", self.move_group.get_active_joints())
            
            robot_state.joint_state = js
            req.robot_state = robot_state

            resp = fk(req)
            
            if resp.error_code.val == 1:  # SUCCESS
                return resp.pose_stamped[0].pose
            else:
                rospy.logerr(f"FK falló con código {resp.error_code.val}")
                return None
        except Exception as e:
            rospy.logerr(f"Error en FK: {e}")
            return None


    # ===============================
    # 2. Pose -> Ángulos articulares
    # ===============================
    def pose_a_AngulosArticulares(self, pose: Pose):
        """
        pose: geometry_msgs/Pose
        return: lista [j1..j6] en grados o None si falla
        """
        rospy.wait_for_service('/compute_ik')
        compute_ik = rospy.ServiceProxy('/compute_ik', GetPositionIK)

        self.move_group.set_goal_tolerance(0.001)
        self.move_group.set_goal_position_tolerance(0.001)
        self.move_group.set_goal_orientation_tolerance(0.01)
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.base_frame
        pose_stamped.pose = pose
        pose_stamped.header.stamp = rospy.Time.now()

        ik_req = GetPositionIKRequest()
        ik_req.ik_request.group_name = self.group_name
        ik_req.ik_request.pose_stamped = pose_stamped
        ik_req.ik_request.robot_state = self.move_group.get_current_state()
        ik_req.ik_request.avoid_collisions = False

        resp = compute_ik(ik_req)
        
        if resp.error_code.val == 1:  # SUCCESS
            cord_ang_rad = list(resp.solution.joint_state.position)
            #return self.rad_grados(cord_ang_rad)
            return [int(round(ang)) for ang in self.rad_grados(cord_ang_rad)]
        else:
            rospy.logerr(f"IK falló con código: {resp.error_code.val}")
            return None


    # ===============================
    # 3. Cartesianas + Euler -> Pose
    # ===============================
    def cartesianasEuler_a_pose(self, coord):
        """
        coord: [x,y,z,roll,pitch,yaw] con x,y,z en mm y ángulos en grados
        return: geometry_msgs/Pose
        """
        # 1. Separar parte cartesiana y angular
        coord_cart = [c / 1000.0 for c in coord[:3]]   # mm → m
        coord_ang  = self.grados_rad(coord[3:])        # grados → radianes

        # 2. Calcular cuaternión a partir de las coordenadas angulares
        quat = tf.transformations.quaternion_from_euler(
            coord_ang[0], coord_ang[1], coord_ang[2]
        )
        
        # 3. Construir objeto Pose
        pose = Pose()
        pose.position.x = coord_cart[0]
        pose.position.y = coord_cart[1]
        pose.position.z = coord_cart[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
       
        return pose

    # ===============================
    # 4. Pose -> Cartesianas + Euler
    # ===============================
    def pose_a_cartesianasEuler(self, pose: Pose):
        """
        return: [x,y,z,roll,pitch,yaw] en grados
        """
        quat = [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        
        coordenadas = [
           int(round(float(pose.position.x * 1000))),
           int(round(float(pose.position.y * 1000))),
           int(round(float(pose.position.z * 1000))),
           ] + [int(round(a)) for a in self.rad_grados([roll, pitch, yaw])
        ]
        return coordenadas
        
       
    # ===============================
    # 5. Ángulos articulares -> Cartesianas + Euler
    # ===============================
    def AngulosArticulares_a_cartesianasEuler(self, cord_ang_grados):
        """
        cord_ang_grados: lista [j1..j6] en grados
        return: [x,y,z,roll,pitch,yaw] en grados
        """
        pose = self.AngulosArticulares_a_pose(cord_ang_grados)
        return self.pose_a_cartesianasEuler(pose)


    # ===============================
    # 6. Cartesianas + Euler -> Ángulos articulares
    # ===============================
    def cartesianaEuler_a_AngulosArticulares(self, coord_grados):
        """
        coord_grados: [x,y,z,roll,pitch,yaw] en (m, grados)
        return: lista [j1..j6] en grados o None si falla
        """
        pose = self.cartesianasEuler_a_pose(coord_grados)
        
        if pose:
            return self.pose_a_AngulosArticulares(pose)
        else: 
            None
                        



if __name__ == '__main__':
    try:
        modo_lectura = ModoLectura()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
