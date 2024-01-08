import numpy as np

class ASV():

    def __init__(self, M, D):

        """
        Initialize some system matrices     
        """
        
        #m = 50
        #d = np.array([10, 10, 5])

        #self.M = np.diag(m*np.ones(3))
        #self.D = np.diag(d)
        
        
        self.M = M
        self.M_inv = np.linalg.inv(self.M)
        self.D = D


    def state_dot(self, state, tau_actuation, V_current = np.zeros(2)):
        
        """
        Derivative of the state calculated with the non-linear kinematix
        """

        heading = state[2]

        J = np.array(
            [[np.cos(heading), -np.sin(heading), 0],
            [np.sin(heading), np.cos(heading), 0],
            [0, 0, 1]]
        )

        A = np.zeros((6,6))

        A[:3,3:] = J
        A[3:, 3:] = - self.M_inv @ self.D

        B = np.zeros((6,3))
        B[3:,:] = self.M_inv

        x_dot = A @ state + B @ tau_actuation
        x_dot[0:2] += V_current # add current drift term at velocity level

        return x_dot
    
    def linearize_model(self, heading):
        """
        Get a linearization about some heading
        """
        J = np.array(
            [[np.cos(heading), -np.sin(heading), 0],
            [np.sin(heading), np.cos(heading), 0],
            [0, 0, 1]]
        )

        A = np.zeros((6,6))

        A[:3,3:] = J
        A[3:, 3:] = - self.M_inv @ self.D

        B = np.zeros((6,3))
        B[3:,:] = self.M_inv

        return A, B
    
    def RK4_integration_step(self, x, u, dt):
        
        # integration scheme for simulation, implements the Runge-Kutta 4 integrator

        k1 = self.state_dot(x,         u)
        k2 = self.state_dot(x+dt/2*k1, u)
        k3 = self.state_dot(x+dt/2*k2, u)
        k4 = self.state_dot(x+dt*k3,   u)
        
        x_next = x + dt/6*(k1+2*k2+2*k3+k4)

        return x_next