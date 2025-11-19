#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Normalizacion_Robot.py
import numpy as np

class NormalizacionRobot:
    """
    Clase encargada de conversiones entre radianes, grados y bits,
    además de proveer límites articulares.
    """

    def __init__(self):
        # Límites articulares en grados
        self.angMin = [-175, -115, -160, -175, -175, -175]
        self.angMax = [ 175,  115,  160,  175,  175,  175]

    # ------------------------------
    # Conversiones
    # ------------------------------
    
    def rad_bit(self, rad):
        bit = []
        for i, r in enumerate(rad):
            if i < 3:
                # XL430: de [-pi, pi] a [0,4095]
                b = int((r + np.pi) * 4095 / (2 * np.pi))
            else:
                # XL320: de [-150°, +150°] a [0,1023] 150° = 2.61799 rad y 300º = 5.23599
                b = int((r + 2.61799) * 1023 / 5.23599)
            bit.append(b)

        return np.int32(bit)


    def bit_rad(self, bit):
        rad = []
        for i, b in enumerate(bit):
            if i < 3:
                # XL430
                r = (b * (2*np.pi) / 4095.0) - np.pi
            else:
                # XL320
                r = (b * 5.23599 / 1023.0) - 2.61799
            rad.append(r)

        return rad


    def grados_bit(self, deg):
        bit = []
        for i, g in enumerate(deg):
            if i < 3:
                # XL430: [–180°, +180°]
                b = int((g + 180) * 4095 / 360)
            else:
                # XL320: [–150°, +150°]
                b = int((g + 150) * 1023 / 300)
            bit.append(b)

        return np.int32(bit)
    
    def grados_rad(self, grad):
        """Convierte grados a radianes"""
        return [g * np.pi / 180.0 for g in grad]

    def rad_grados(self, rad):
        """Convierte radianes a grados"""
        return [r * 180.0 / np.pi for r in rad]
    
    def bit_grados(self, bit):
        grados = []
        for i, b in enumerate(bit):
            if i < 3:
                # Para XL430 (0–4095 → 360°)
                g = (b * 360.0 / 4095.0) - 180.0
            else:
                # Para XL320 (0–1023 → 300°)
                g = (b * 300.0 / 1023.0) - 150.0
            grados.append(g)
        return grados

    # ------------------------------
    # Validaciones
    # ------------------------------
    def esta_en_limites(self, grados):
        """Devuelve True si todos los ángulos están dentro de los límites"""
        for g, lo, hi in zip(grados, self.angMin, self.angMax):
            if g < lo or g > hi:
                return False
        return True
