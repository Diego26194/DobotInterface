#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int16, Int16MultiArray
from moveit_msgs.msg import DisplayTrajectory, MoveGroupSequenceActionFeedback
import numpy as np
import time
from cobot_controladores.msg import waits, trayectorias
from db_puntos6 import (obtener_rutina)

from Normalizacion_Robot import NormalizacionRobot

norm = NormalizacionRobot()

class ControladorCobot:
    def __init__(self):
        rospy.init_node('controlador_cobot', anonymous=False)
        rospy.loginfo("🦾 Nodo 'controlador_cobot' iniciado")

        # === Variables internas ===
        self.indicesT=[]
        self.trayectoras=[]
        self.recibido_trayectorias =False
        self.orden=None        
        self.indicesWait=[]
        self.wait=[]
        self.plan_fragmentos = []           # partes del plan recibido        # partes del plan recibido
        self.plan_fragmentos_state = False  
        self.feedback_state = None          # estado textual del feedback ("PLANNING", "MONITOR", "IDLE", etc.)
        self.recibido_feedback = False      # bandera para saber si llegó feedback
        self.planificacion_valida = False   # bandera de éxito o error
        self.feedback_relevante = False
        self.recibido_waits = False 

        # === Publicadores ===
        self.pub_cord_dy = rospy.Publisher('cord_dy', Int16MultiArray, queue_size=10)
        self.pub_planificacion_trayectoria = rospy.Publisher('planificacion_A_trayectoria', Int16, queue_size=2)

        # === Subscriptores ===
        rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, self.callback_display_rutina)
        rospy.Subscriber('/sequence_move_group/feedback', MoveGroupSequenceActionFeedback, self.callback_feedback)
        rospy.Subscriber('trayectoria_A_planificacion', Int16, self.callback_planificacion)  
        self.rutine_waits = rospy.Subscriber('rutine_waits', waits, self.callback_waits)
        self.rutine_trayectorias = rospy.Subscriber('rutine_trayectorias', trayectorias, self.callback_trayectorias)

        # === Parámetros ===
        self.PAUSA_ENTRE_TRAYECTORIAS = 0.1  # segundos
        self.TIMEOUT_PLANIFICACION = 5.0    # segundos máximos de espera
        
        self._limpiar_variables()

    # ----------------------------------------------------
    # Callbacks
    # ----------------------------------------------------
    def callback_waits(self, msg):
        """Guarda los waits y sus idices."""      
        self.indicesWait=list(msg.indice)
        self.wait=list(msg.wait)
        self.recibido_waits =True
        
    def callback_trayectorias(self, msg):
        """Guarda las trayectorias y sus idices."""      
        self.indicesT=list(msg.indice)
        posicionT=list(msg.posicion)
        rutinasT=list(msg.Rutina)
        
        for i, nomRut in enumerate(rutinasT):
            rut=obtener_rutina(nomRut)
            tray=next((r for r in rut if r.get('pos') == posicionT[i]))
            self.trayectoras.append(tray.get('puntos'))   
            
        self.recibido_trayectorias =True
        
            
    def callback_display_rutina(self, msg):
        """Guarda los fragmentos de trayectorias planificadas."""
        if not (self.orden==-2) :
            self.plan_fragmentos.append(msg)
            self.plan_fragmentos_state=True
            
            rospy.loginfo(f"📦 Fragmento de plan recibido ({len(self.plan_fragmentos)})")

    def callback_feedback(self, msg):
        """Registra el estado de la planificación (feedback)."""
        self.feedback_state = msg.feedback.state

        rospy.loginfo(f"📡 Feedback: estado='{self.feedback_state}'")

        # Evaluar estado
        if self.feedback_state == "MONITOR":
            self.planificacion_valida = True
        elif self.feedback_state == "IDLE":
            self.feedback_relevante = True
        # Solo marcar como fallo si el texto menciona error
            if "error" in msg.status.text.lower() or "failed" in msg.status.text.lower():
                rospy.logerr("❌ Estado IDLE con error detectado.")
                self.planificacion_valida = False
                
        self.recibido_feedback = True

    def callback_planificacion(self, msg):
        self.orden=msg.data
        
        if self.orden==0:
            rospy.loginfo("Limpieza inicial")
            self._limpiar_variables() 
            self.pub_planificacion_trayectoria.publish(0)     
            
        elif self.orden==-1:
            rospy.loginfo("🔄 Señal -1 recibida. Esperando planificación (feedback + plan)...")

            # Esperar feedback y trayectorias planificadas
            inicio = time.time()
            while (not self.plan_fragmentos_state or not self.feedback_relevante) and (time.time() - inicio < self.TIMEOUT_PLANIFICACION):
                rospy.sleep(0.1)

            if not self.recibido_feedback:
                rospy.logwarn("⚠️ No se recibió feedback en el tiempo establecido.")
                self._limpiar_variables()
                self.pub_planificacion_trayectoria.publish(-2)
                return

            # Evaluar resultado            
            if self.planificacion_valida and self.plan_fragmentos_state and self.plan_fragmentos:
                rospy.loginfo("✅ Planificación válida (estado MONITOR)")
                self.pub_planificacion_trayectoria.publish(1)
                self.plan_fragmentos_state=False
            else:
                rospy.logerr("❌ Planificación fallida (estado IDLE o sin trayectoria). No se ejecutará.")
                self.pub_planificacion_trayectoria.publish(-1)                
                self._limpiar_variables()
        elif self.orden==-2:
            self.ejecutar_plan_completo()
            self._limpiar_variables()      
        elif self.orden==-3:
            self.pub_planificacion_trayectoria.publish(2)
            
            rospy.loginfo("🔄 Señal -3 recibida. Esperando waits")
            
            # Esperar feedback y trayectorias planificadas
            inicio = time.time()
            while (not self.recibido_waits) and (time.time() - inicio < self.TIMEOUT_PLANIFICACION):
                rospy.sleep(0.1)

            if not self.recibido_waits:
                rospy.logwarn("⚠️ No se recibió feedback en el tiempo establecido.")
                self._limpiar_variables()
                self.pub_planificacion_trayectoria.publish(-2)
                return
            
            self.ejecutar_plan_completo()
            self._limpiar_variables() 
            
        elif self.orden==-4:
                
            self.pub_planificacion_trayectoria.publish(2)
            
            rospy.loginfo("🔄 Señal -4 recibida. Esperando Trayectorias")
            
            # Esperar feedback y trayectorias planificadas
            inicio = time.time()
            while (not self.recibido_trayectorias) and (time.time() - inicio < self.TIMEOUT_PLANIFICACION):
                rospy.sleep(0.1)

            if not self.recibido_trayectorias:
                rospy.logwarn("⚠️ No se recibió feedback en el tiempo establecido.")
                self._limpiar_variables()
                self.pub_planificacion_trayectoria.publish(-2)
                return
            
            self.ejecutar_plan_completo()
            self._limpiar_variables()   
            
        elif self.orden==-5:
                
            self.pub_planificacion_trayectoria.publish(2)
            
            rospy.loginfo("🔄 Señal -5 recibida. Esperando waits y Trayectorias")
            
            # Esperar feedback y trayectorias planificadas
            inicio = time.time()
            while (not self.recibido_waits or not self.recibido_trayectorias) and (time.time() - inicio < self.TIMEOUT_PLANIFICACION):
                rospy.sleep(0.1)

            if (not self.recibido_waits) and (not self.recibido_trayectorias):
                rospy.logwarn("⚠️ No se recibió feedback de waits ni trayectorias en el tiempo establecido.")
                self._limpiar_variables()
                self.pub_planificacion_trayectoria.publish(-2)
                return
            elif not self.recibido_waits:
                rospy.logwarn("⚠️ No se recibió feedback de waits en el tiempo establecido.")
                self._limpiar_variables()
                self.pub_planificacion_trayectoria.publish(-2)
                return
            elif not self.recibido_trayectorias:
                rospy.logwarn("⚠️ No se recibió feedback de trayectorias en el tiempo establecido.")
                self._limpiar_variables()
                self.pub_planificacion_trayectoria.publish(-2)
                return
            
            self.ejecutar_plan_completo()
            self._limpiar_variables()         
            
        elif self.orden==-6:
            rospy.loginfo("🔄 Señal 0 recibida. Esperando planificación (feedback + plan)...")

            # Esperar feedback y trayectorias planificadas
            inicio = time.time()
            while (not self.plan_fragmentos) and (time.time() - inicio < self.TIMEOUT_PLANIFICACION):
                rospy.sleep(0.1)

            if not self.plan_fragmentos:
                rospy.logwarn("⚠️ No se recibió feedback en el tiempo establecido.")
                self._limpiar_variables()
                self.pub_planificacion_trayectoria.publish(-2)
                return

            rospy.loginfo("✅ Planificación válida (estado MONITOR)")
                
            self.ejecutar_plan_completo()
            self._limpiar_variables()           

    # ----------------------------------------------------
    # Ejecución
    # ----------------------------------------------------
    def ejecutar_plan_completo(self):
        """Ejecuta todas las trayectorias acumuladas."""  
        for i, frag in enumerate(self.plan_fragmentos):
            
            rospy.loginfo(f"▶ Ejecutando fragmento {i+1}/{len(self.plan_fragmentos)}...")            
            for trajectory in frag.trajectory:
                self.ejecutar_trayectoria_individual(trajectory.joint_trajectory)   
            
            if (i) in self.indicesT:
                idx = self.indicesT.index(i)
                self.ejecutar_trayectoria_bruta(self.trayectoras[idx])  
            
            if (i) in self.indicesWait:  
                # Obtener el tiempo de espera correspondiente
                idx = self.indicesWait.index(i)
                tiempo_wait = self.wait[idx]
                rospy.loginfo(f"⏸ Esperando {tiempo_wait}s después del punto {i+1}")
                time.sleep(tiempo_wait)
            else:
                # Pausa corta estándar
                time.sleep(self.PAUSA_ENTRE_TRAYECTORIAS)
    
    def ejecutar_trayectoria_bruta(self, puntos):
        rate = rospy.Rate(20)  
        for punto in puntos:
            msg = Int16MultiArray()
            msg.data = punto
            self.pub_cord_dy.publish(msg)
            rate.sleep()

    def ejecutar_trayectoria_individual(self, joint_trajectory):
        """Envía los puntos de una trayectoria."""
        puntos = joint_trajectory.points
        t0 = time.time()

        for punto in puntos:
            posiciones_bits = norm.rad_bit(punto.positions)
            msg = Int16MultiArray(data=posiciones_bits)
            self.pub_cord_dy.publish(msg)

            t_obj = punto.time_from_start.to_sec()
            while (time.time() - t0) < t_obj:
                time.sleep(0.001)
        self.pub_planificacion_trayectoria.publish(3)

    # ----------------------------------------------------
    # Limpieza
    # ----------------------------------------------------
    def _limpiar_variables(self):
        """Reinicia variables internas para evitar datos residuales."""
        self.indicesT.clear()
        self.trayectoras.clear()
        self.recibido_trayectorias =False
        self.orden=None
        self.indicesWait.clear()
        self.wait.clear()
        self.plan_fragmentos.clear()
        self.plan_fragmentos_state = False
        self.feedback_state = None
        self.recibido_feedback = False
        self.planificacion_valida = False
        self.feedback_relevante = False
        self.recibido_waits = False
        rospy.loginfo("🧹 Variables internas limpiadas y nodo listo para próxima planificación.")

if __name__ == '__main__':
    try:
        nodo = ControladorCobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


