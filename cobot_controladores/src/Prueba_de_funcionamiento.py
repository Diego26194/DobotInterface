#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander
import tf

# Importá tu clase con las funciones
from modo_lectura13 import ModoLectura   # <- Cambiá esto según corresponda


def main():
    rospy.init_node("tester_nodo", anonymous=True)

    # Instanciamos la clase que tiene las funciones
    mi_robot = ModoLectura()

    print("\n=== Nodo de prueba interactivo ===")
    print("Escribe el nombre de la función y se ejecutará.")
    print("Funciones disponibles:")
    print("1. AngulosArticulares_a_pose [j1..j6 en grados]")
    print("2. pose_a_AngulosArticulares [pose random de ejemplo]")
    print("3. cartesianasEuler_a_pose [x,y,z,roll,pitch,yaw]")
    print("4. pose_a_cartesianasEuler [pose random de ejemplo]")
    print("5. AngulosArticulares_a_cartesianasEuler [j1..j6 en grados]")
    print("6. cartesianaEuler_a_AngulosArticulares [x,y,z,roll,pitch,yaw]")
    print("Escribe 'exit' para salir\n")

    while not rospy.is_shutdown():
        try:
            comando = input(">> ").strip()
            if comando == "exit":
                print("Saliendo...")
                break

            # Ejemplos de prueba
            if comando == "AngulosArticulares_a_pose":
                valores = [0, 30, 45, 0, 60, 90]  # ejemplo en grados
                res = mi_robot.AngulosArticulares_a_pose(valores)
                print("Resultado:", res)

            elif comando == "pose_a_AngulosArticulares":
                pose = Pose()
                pose.position.x = 0.2
                pose.position.y = 0.0
                pose.position.z = 0.2
                pose.orientation.w = 1.0
                res = mi_robot.pose_a_AngulosArticulares(pose)
                print("Resultado:", res)

            elif comando == "cartesianasEuler_a_pose":
                valores = [200, 0, 200, 0, 90, 0]  # [mm, mm, mm, roll, pitch, yaw]
                res = mi_robot.cartesianasEuler_a_pose(valores)
                print("Resultado:", res)

            elif comando == "pose_a_cartesianasEuler":
                pose = Pose()
                pose.position.x = 0.1
                pose.position.y = 0.2
                pose.position.z = 0.3
                pose.orientation.x = 0.0
                pose.orientation.y = 0.0
                pose.orientation.z = 0.0
                pose.orientation.w = 1.0
                res = mi_robot.pose_a_cartesianasEuler(pose)
                print("Resultado:", res)

            elif comando == "AngulosArticulares_a_cartesianasEuler":
                valores = [0, 30, 45, 0, 60, 90]
                res = mi_robot.AngulosArticulares_a_cartesianasEuler(valores)
                print("Resultado:", res)

            elif comando == "cartesianaEuler_a_AngulosArticulares":
                valores = [200, 0, 200, 0, 90, 0]
                res = mi_robot.cartesianaEuler_a_AngulosArticulares(valores)
                print("Resultado:", res)

            else:
                print("Comando no reconocido. Intenta de nuevo.")

        except Exception as e:
            print("Error al ejecutar:", e)


if __name__ == "__main__":
    main()
