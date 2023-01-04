import numpy as np
import control
import scipy.signal

"""
Single object tracking 

Need a model for how the object moves.
Need a model/distribution for detection? 

Compute probability of matching obsercations to your track
    Based on mahalanobis distance, give a weight to each observation. 
    It's aslo a posibilty not to obsevre anything.

Update step 
    Use a weighted combination of observations
    x^ = sum(p_i*(o_i - Cx))
    P = (see pdf from psu edu)


Prediction step
    Seems to be the same as for a KF.

"""


class PDAF:
    def __init__(self):
        # x = [r, thetha, r', theta']

        self.time_step = 0.1
        self.x_pri = np.ndarray(
            (4,), buffer=np.array([0.0, 0.0, 0.0, 0.0]), dtype=float
        )
        self.P_pri = np.ndarray(
            (4, 4),
            buffer=np.array(
                [
                    [0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0],
                    [0.0, 0.0, 0.0, 0.0],
                ]
            ),
            dtype=float,
        )

        self.x_post = np.ndarray((4,), dtype=float)
        self.P_post = np.ndarray((4, 4), dtype=float)

        self.L = np.ndarray((2, 2), dtype=float)

        self.C = np.ndarray(
            (2, 4),
            buffer=np.array([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]]),
            dtype=float,
        )

        self.A = np.ndarray(
            (4, 4),
            buffer=np.array(
                [
                    [1.0, 0.0, self.time_step, 0],
                    [0, 1.0, 0, self.time_step],
                    [0, 0, 1.0, 0],  # assuming constnat velocity
                    [0, 0, 0, 1.0],
                ]
            ),  # assuming constnat velocity
            dtype=float,
        )

        self.Q = np.ndarray(
            (4, 4),
            buffer=np.array(
                [[0.001, 0, 0, 0], [0, 0.001, 0, 0], [0, 0, 0.1, 0], [0, 0, 0, 0.1]]
            ),
            dtype=float,
        )

        self.R = np.ndarray((2, 2), buffer=np.array([[0.1, 0], [0, 0.1]]), dtype=float)


        self.gate_radius = 5 #the gating window will be a circle around the predicted position (5 is just a random number for now) 
        self.residual_vector = np.ndarray((2,), dtype=float)

    def compute_probability_of_matching_observations(self, y):
        a = np.ndarray((len( y+ 1),), dtype=float) #score for each observation based on distance from predicted position
        p = np.ndarray((len( y+ 1),), dtype=float) #probability of matching observation i to the track

        a[0] = 1.0 #considering the probability that no observations match. Choose an approriate value. 
        for i, y_i in enumerate(y):
            delta_r = y_i[0] - self.x_pri[0] #use euclidian distance for now
            if delta_r <= 0.1: #In order to avoid infinte high weights. Choose an approriate threshold.
                a[i+1] = 10
            else:
                a[i+1] = 1/delta_r

        a_sum = np.sum(a)
        for i, a_i in enumerate(a):
            p[i] = a_i/a_sum

        return p


    def compute_residual_vector(self, y, p):

        self.residual_vector[0] = 0 #r
        self.residual_vector[1] = 0 #theta

        for i in range(len(y)):
            self.residual_vector[0] += p[i+1]*(y[i][0] - self.x_pri[0])
            self.residual_vector[1] += p[i+1]*(y[i][1] - self.x_pri[1])
        

    def prediction_step(self):
        self.x_pri = np.matmul(self.A, self.x_post)
        self.P_pri = (
            np.matmul(self.A, np.matmul(self.P_post, np.transpose(self.A))) + self.Q
        )

    def correction_step(self, y):


        P_CT = np.matmul(self.P_pri, np.transpose(self.C))
        C_P_CT = np.matmul(self.C, P_CT)
        self.L = np.matmul(P_CT, np.linalg.inv(C_P_CT + self.R))

        p = self.compute_probability_of_matching_observations(y)
        self.compute_residual_vector(y, p)
        self.x_post = self.x_pri + np.matmul(self.L, self.residual_vector)

        self.P_post = p[0]*self.P_pri + (1-p[0])*

        #I_LC = np.identity(len(self.x_pri)) - np.matmul(self.L, self.C)
        #correction_term = np.matmul(self.L, np.matmul(self.R, np.transpose(self.L)))
        #self.P_post = (#must be modified to incorporate weighted matches
        #    np.matmul(I_LC, np.matmul(self.P_pri, np.transpose(I_LC))) + correction_term
        #)


# -----------------------------------