#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Int16, Int16MultiArray
from moveit_msgs.msg import DisplayTrajectory, MoveGroupSequenceActionFeedback
import numpy as np
import time


class ControladorCobot:
    def __init__(self):
        rospy.init_node('controlador_cobot', anonymous=False)
        rospy.loginfo("🦾 Nodo 'controlador_cobot' iniciado")

        # === Variables internas ===
        self.plan_fragmentos = []           # partes del plan recibido
        self.feedback_state = None          # estado textual del feedback ("PLANNING", "MONITOR", "IDLE", etc.)
        self.recibido_feedback = False      # bandera para saber si llegó feedback
        self.planificacion_valida = False   # bandera de éxito o error

        # === Publicadores ===
        self.pub_cord_dy = rospy.Publisher('cord_dy', Int16MultiArray, queue_size=10)
        self.pub_planificacion_trayectoria = rospy.Publisher('planificacion_A_trayectoria', Int16, queue_size=10)

        # === Subscriptores ===
        rospy.Subscriber('/move_group/display_planned_path', DisplayTrajectory, self._callback_display_trajectory)
        rospy.Subscriber('/sequence_move_group/feedback', MoveGroupSequenceActionFeedback, self._callback_feedback)
        rospy.Subscriber('trayectoria_A_planificacion', Int16, self._callback_planificacion)

        # === Parámetros ===
        self.PAUSA_ENTRE_TRAYECTORIAS = 0.1  # segundos
        self.TIMEOUT_PLANIFICACION = 25.0    # segundos máximos de espera

        rospy.loginfo("✅ Esperando señal (-4) para iniciar planificación...")

    # ----------------------------------------------------
    # Conversión
    # ----------------------------------------------------
    def _radianes_a_bits(self, rad):
        """Convierte una lista de radianes a valores 0–4095."""
        return [int((r + np.pi) * 4095 / (2 * np.pi)) for r in rad]

    # ----------------------------------------------------
    # Callbacks
    # ----------------------------------------------------
    def _callback_display_trajectory(self, msg):
        """Guarda los fragmentos de trayectorias planificadas."""
        self.plan_fragmentos.append(msg)
        rospy.loginfo(f"📦 Fragmento de plan recibido ({len(self.plan_fragmentos)})")

    def _callback_feedback(self, msg):
        """Registra el estado de la planificación (feedback)."""
        self.feedback_state = msg.feedback.state
        self.recibido_feedback = True

        rospy.loginfo(f"📡 Feedback: estado='{self.feedback_state}'")

        # Evaluar estado
        if self.feedback_state == "MONITOR":
            self.planificacion_valida = True
        elif self.feedback_state == "IDLE":
            self.planificacion_valida = False

    def _callback_planificacion(self, msg):
        """Lógica principal de control."""
        if msg.data != -4:
            return

        rospy.loginfo("🔄 Señal -4 recibida. Esperando planificación (feedback + plan)...")

        # Esperar feedback y trayectorias planificadas
        inicio = time.time()
        while (not self.recibido_feedback or not self.plan_fragmentos) and (time.time() - inicio < self.TIMEOUT_PLANIFICACION):
            rospy.sleep(0.1)

        if not self.recibido_feedback:
            rospy.logwarn("⚠️ No se recibió feedback en el tiempo establecido.")
            self._limpiar_variables()
            return

        # Evaluar resultado
        if self.planificacion_valida and self.plan_fragmentos:
            rospy.loginfo("✅ Planificación válida (estado MONITOR). Ejecutando plan...")
            self._ejecutar_plan_completo()
            self.pub_planificacion_trayectoria.publish(0)
        else:
            rospy.logerr("❌ Planificación fallida (estado IDLE o sin trayectoria). No se ejecutará.")

        # Limpieza
        self._limpiar_variables()

    # ----------------------------------------------------
    # Ejecución
    # ----------------------------------------------------
    def _ejecutar_plan_completo(self):
        """Ejecuta todas las trayectorias acumuladas."""
        for i, frag in enumerate(self.plan_fragmentos):
            rospy.loginfo(f"▶ Ejecutando fragmento {i+1}/{len(self.plan_fragmentos)}...")
            for trajectory in frag.trajectory:
                self._ejecutar_trayectoria_individual(trajectory.joint_trajectory)
                time.sleep(self.PAUSA_ENTRE_TRAYECTORIAS)

    def _ejecutar_trayectoria_individual(self, joint_trajectory):
        """Envía los puntos de una trayectoria."""
        puntos = joint_trajectory.points
        t0 = time.time()

        for punto in puntos:
            posiciones_bits = self._radianes_a_bits(punto.positions)
            msg = Int16MultiArray(data=posiciones_bits)
            self.pub_cord_dy.publish(msg)

            t_obj = punto.time_from_start.to_sec()
            while (time.time() - t0) < t_obj:
                time.sleep(0.001)

    # ----------------------------------------------------
    # Limpieza
    # ----------------------------------------------------
    def _limpiar_variables(self):
        """Reinicia variables internas para evitar datos residuales."""
        self.plan_fragmentos.clear()
        self.feedback_state = None
        self.recibido_feedback = False
        self.planificacion_valida = False
        rospy.loginfo("🧹 Variables internas limpiadas y nodo listo para próxima planificación.")


if __name__ == '__main__':
    try:
        nodo = ControladorCobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


