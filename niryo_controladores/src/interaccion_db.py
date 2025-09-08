#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from db_puntos2 import escribir_datos, leer_datos, eliminar_punto, cambiar_punto

def menu():
    while True:
        print("\nOpciones:")
        print("1. Escribir datos")
        print("2. Leer todos los datos")
        print("3. Cambiar un punto")
        print("4. Eliminar un punto")
        print("5. Salir")

        opcion = input("Selecciona una opción: ")

        if opcion == '1':
            coord = [float(x) for x in input("Introduce las coordenadas x, y, z, a, b, c separadas por espacios: ").split()]
            escribir_datos(coord)
            print("Datos escritos.")
        elif opcion == '2':
            datos = leer_datos()
            print("Datos de la base de datos:")
            for d in datos:
                print(d)
        elif opcion == '3':
            n = int(input("Introduce el ID del punto a cambiar: "))
            nuevas_coord = [float(x) for x in input("Introduce las nuevas coordenadas x, y, z, a, b, c separadas por espacios: ").split()]
            cambiar_punto(n, nuevas_coord)
            print("Punto cambiado.")
        elif opcion == '4':
            n = int(input("Introduce el ID del punto a eliminar: "))
            eliminar_punto(n)
            print("Punto eliminado.")
        elif opcion == '5':
            break
        else:
            print("Opción no válida.")

if __name__ == "__main__":
    rospy.init_node('coordenadas_node')
    menu()
