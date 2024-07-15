#!/usr/bin/env python3

import numpy as np
import control

class ReferenceFilter:
    def __init__(self):
        zeta = 0.8
        omega_b = 0.05
        omega_n = omega_b/np.sqrt(1-2*zeta**2 + np.sqrt(4*zeta**4 - 4*zeta**2 + 2))

        I = np.eye(3)
        Omega = 2*zeta*omega_n*I
        # Gamma = omega_n**2*I
        Delta = zeta*I
        Ad = np.zeros((9,9))
        Ad[0:3,3:6] = I
        Ad[3:6,6:9] = I
        Ad[6:9,0:3] = -Omega**3
        Ad[6:9,3:6] = -(2*Delta+I)@Omega**2
        Ad[6:9,6:9] = -(2*Delta+I)@Omega

        Bd = np.zeros((9,3))
        Bd[6:9,:] = Omega**3

        sys = control.ss(Ad, Bd, np.zeros((9,9)), np.zeros((9,3)))
        sysd = control.c2d(sys, 0.1)

        self.Ad = sysd.A
        self.Bd = sysd.B

    def step(self, r, xd):
        x_next = self.Ad@xd + self.Bd@r
        return x_next
    
    def get_eta(self, xd):
        return xd[:,0:3]
    
    def get_nu(self, xd):
        nu = np.zeros((len(xd),3))
        for i in range(len(xd)):
            psi = xd[i,2]
            nu[i,:] = (self.rotationMatrix(psi).transpose())@xd[i,3:6]
        return nu

    @staticmethod
    def rotationMatrix(psi):
        R = np.array([[np.cos(psi), -np.sin(psi), 0],
                [np.sin(psi), np.cos(psi), 0],
                [0, 0, 1]])
        return R
