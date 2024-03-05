import numpy as np

class AdaptiveBackstep:
    def __init__(self, kappa):
        self.init_system(kappa)

    def init_system(self, kappa):

        I = np.eye(3)

        K_1 = np.diag([10, 10, 10])
        self.K_1_tilde = K_1 + kappa*I
        self.K_2 = np.diag([60, 60, 30])
        self.tau_max = np.array([41.0, 50.0, 55.0]) # Må tilpasses thrusterne

        ## Forenklet modell ## Bør også endres
        m = 50
        self.M = np.diag([m, m, m])
        self.D = np.diag([10, 10, 5])

    def control_law(self, eta, nu, w, v_ref, dt_v_ref, dtheta_v_ref, eta_d, dtheta_eta_d, ddtheta_eta_d): # dtheta == ds
        _, R_trps = self.R(eta[2])
        S = self.S(nu[2])

        eta_error = eta - eta_d
        eta_error[2] = self.ssa(eta_error[2])

        z1 = R_trps @ eta_error
        alpha1 = -self.K_1_tilde @ z1 + R_trps @ dtheta_eta_d * v_ref

        z2 = nu - alpha1

        sigma1 = self.K_1_tilde @ (S @ z1) - self.K_1_tilde @ nu - S @ (R_trps @ dtheta_eta_d) * v_ref + R_trps @ dtheta_eta_d * dt_v_ref

        dtheta_alpha1 = self.K_1_tilde @ (R_trps @ dtheta_eta_d) + R_trps @ ddtheta_eta_d * v_ref + R_trps @ dtheta_eta_d * dtheta_v_ref

        # Control law ## Må endres når system-matrisene endres
        tau = -self.K_2 @ z2 + self.D @ nu + self.M @ sigma1 + self.M @ dtheta_alpha1 * (v_ref + w)

        # Add constraints to tau #

        if np.absolute(tau[0]) > self.tau_max[0] or np.absolute(tau[1]) > self.tau_max[1] or np.absolute(tau[2]) > self.tau_max[2]:
            if np.absolute(tau[0]) > self.tau_max[0]:
                tau[2] = np.sign(tau[2]) * np.absolute(self.tau_max[0] / tau[0]) * np.absolute(tau[2])
                tau[1] = np.sign(tau[1]) * np.absolute(self.tau_max[0] / tau[0]) * np.absolute(tau[1])
                tau[0] = np.sign(tau[0]) * self.tau_max[0]
            if np.absolute(tau[1]) > self.tau_max[1]:
                tau[2] = np.sign(tau[2]) * np.absolute(self.tau_max[1] / tau[1]) * np.absolute(tau[2])
                tau[0] = np.sign(tau[0]) * np.absolute(self.tau_max[1] / tau[1]) * np.absolute(tau[0])
                tau[1] = np.sign(tau[1]) * self.tau_max[1]
            if np.absolute(tau[2]) > self.tau_max[2]:
                tau[1] = np.sign(tau[1]) * np.absolute(self.tau_max[2] / tau[2]) * np.absolute(tau[1])
                tau[0] = np.sign(tau[0]) * np.absolute(self.tau_max[2] / tau[2]) * np.absolute(tau[0])
                tau[2] = np.sign(tau[2]) * self.tau_max[2]
        return tau

    def calculate_coriolis_matrix(self, nu):
        # u = nu[0]
        # v = nu[1]
        # r = nu[2]

        # C_RB = np.array([[0.0, 0.0, -self.m * (self.xg * r + v)], [0.0, 0.0, self.m * u],
        #                   [self.m*(self.xg*r+v), -self.m*u, 0.0]])
        # C_A = np.array([[0.0, 0.0, -self.M_A[1,1] * v + (-self.M_A[1,2])*r],[0.0,0.0,-self.M_A[0,0]*u],
        #                  [self.M_A[1,1]*v-(-self.M_A[1,2])*r, self.M_A[0,0]*u, 0.0]])
        # C = C_RB + C_A

        #return C
        pass

    def R(self,psi):
        R = np.array([[np.cos(psi), -np.sin(psi), 0],
                    [np.sin(psi), np.cos(psi), 0],
                    [0, 0, 1]])
        R_T = np.transpose(R)
        return R, R_T
    
    def S(self,r):
        S = np.array([[0, -r, 0],
                    [r, 0, 0],
                    [0, 0, 0]])
        return S
    
    def ssa(self,angle):
        wrpd_angle = (angle + np.pi) % (2.0*np.pi) - np.pi
        return wrpd_angle
