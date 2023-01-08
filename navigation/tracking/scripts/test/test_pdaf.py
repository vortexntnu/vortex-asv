import numpy as np
import control
import scipy.signal

"""
Single object tracking

Sub tasks: 

    Need a model for how the object moves.

    Compute probability of matching observations to your track
        Based on mahalanobis distance, give a weight to each observation. 
        Remeber it's a posibilty not to obsevre anything.

    Update step 
        Use a weighted combination of observations when calculating the residual vector. But how??
        How do we compute the posterior covariance? How to we incorporate the uncertanty from the data assiciation? 

    Prediction step
        Seems to be the same as for a KF.

    How to initialize tracks? How to delete tracks? 
        Don't think we have to worry about this for single object tracking. 
        Initially we assume there is one and only one object. 

    Include an exitance variable in the state vector (e.g. implement ipda)

    Define a gating window based on the predicted variance. 

    Find a good way to deal with scenario of no observations.

    Find a good way to deal with varying array lengths.
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
                    [0, 0, 0, 1.0],  # assuming constnat velocity
                ]
            ),
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

        self.gate_radius = 5  # the gating window will be a circle around the predicted position, or elipsiod based on the predicted variance; eq (7.17).

        self.residual_vector = np.ndarray((2,), dtype=float)
        self.p_no_match = 0.5  # probabiity that no observations matches the track
        self.p_match_arr = np.ndarray(
            (2,), dtype=float
        )  # Lengt of this array will vary based on how many observations there are.
        self.o_within_gate_arr = np.ndarray(
            (2,2), dtype=float
        )  # Lengt of this array will vary based on how many observations there are.

    def filter_observations_outside_gate(self, o):

        within_gate = []

        for o_i in o:
            if (o_i[0]-self.x_pri[0]) < self.gate_radius:
                within_gate.append(o_i)

        self.o_within_gate_arr = np.array(within_gate)
        

    def compute_probability_of_matching_observations(self):

        score = np.ndarray(
            (len(self.o_within_gate_arr),), dtype=float
        )  # score for each observation based on distance from predicted position

        self.p_match_arr = np.ndarray(
            (len(self.o_within_gate_arr) +1,), dtype=float
        )

        if len(self.o_within_gate_arr) == 0:  # no observations
            self.p_match_arr[0] = 1.0  # probability that no observations match the track

        else:
            self.p_match_arr[0] = self.p_no_match

            for i, y_i in enumerate(self.o_within_gate_arr):
                delta_r = abs(y_i[0] - self.x_pri[0])  # use euclidian distance for now
                if (
                    delta_r <= 0.1
                ):  # In order to avoid infinte high weights. Choose an approriate threshold.
                    score[i] = 10
                else:
                    score[i] = 1 / delta_r

            score_sum = np.sum(score)
            for i in range(len(self.o_within_gate_arr)):
                self.p_match_arr[i + 1] = (score[i] / score_sum) * (1 - self.p_no_match)

    def compute_residual_vector(self):
        #Can this be correct??

        self.residual_vector[0] = 0  # r
        self.residual_vector[1] = 0  # theta

        for i in range(len(self.o_within_gate_arr)):
            self.residual_vector[0] += self.p_match_arr[i + 1] * (self.o_within_gate_arr[i][0] - self.x_pri[0])
            self.residual_vector[1] += self.p_match_arr[i + 1] * (self.o_within_gate_arr[i][1] - self.x_pri[1])

    def prediction_step(self):
        self.x_pri = np.matmul(self.A, self.x_post)
        self.P_pri = (
            np.matmul(self.A, np.matmul(self.P_post, np.transpose(self.A))) + self.Q
        )

    def correction_step(self, o):

        P_CT = np.matmul(self.P_pri, np.transpose(self.C))
        C_P_CT = np.matmul(self.C, P_CT)
        self.L = np.matmul(P_CT, np.linalg.inv(C_P_CT + self.R))

        self.filter_observations_outside_gate(o)

        if len(self.o_within_gate_arr) == 0:  
            self.x_post = self.x_pri
            self.P_post = self.P_pri

        else:
            self.compute_probability_of_matching_observations()
            self.compute_residual_vector()
            self.x_post = self.x_pri + np.matmul(self.L, self.residual_vector)

            I_LC = np.identity(len(self.x_pri)) - np.matmul(self.L, self.C)
            correction_term = np.matmul(
                self.L, np.matmul(self.R, np.transpose(self.L))
            )  # OBS: correction term should reflect uncertain data association
            self.P_post = (
                self.p_match_arr[0] * self.P_pri 
                + (1 - self.p_match_arr[0]) * I_LC * self.P_pri 
                + correction_term
            )


# -----------------------------------

def test_filter_observations_outside_gate():

    pdaf = PDAF()

    n_obs = 10
    r = 4
    theta = 0.5

    observations = np.ndarray((n_obs, 2), dtype=float)
    for i in range(n_obs):
        observations[i, 0] = r 
        observations[i, 1] = theta 

    for i in range(n_obs-5):
        observations[i, 0] = r +2
        observations[i, 1] = theta 

    print("observations: ", observations)

    pdaf.filter_observations_outside_gate(observations)

    print("observations within gate: ", pdaf.o_within_gate_arr)

def test_compute_probability_of_matching_observations():

    pdaf = PDAF()

    n_obs = 10
    r = 4
    theta = 0.5

    observations = np.ndarray((n_obs, 2), dtype=float)
    
    for i in range(n_obs):
        observations[i, 0] = r + i*0.1
        observations[i, 1] = theta 

    pdaf.filter_observations_outside_gate(observations)
    pdaf.compute_probability_of_matching_observations()

    print("p of matches: ", pdaf.p_match_arr)

    assert(np.sum(pdaf.p_match_arr) - 1 < 0.00001)
    assert(pdaf.p_match_arr[0] == pdaf.p_no_match or len(pdaf.o_within_gate_arr)==0)


def test_compute_residual_vector():

    pdaf = PDAF()

    n_obs = 10
    r = 4
    theta = 0.5

    observations = np.ndarray((n_obs, 2), dtype=float)
    
    for i in range(n_obs):
        observations[i, 0] = r 
        observations[i, 1] = theta 

    pdaf.filter_observations_outside_gate(observations)
    pdaf.compute_probability_of_matching_observations()
    pdaf.compute_residual_vector()

    print(pdaf.residual_vector)

def test_pdaf_zero_velocity():

    r = 5
    theta = 1
    tollerance = 0.3
    n_time_steps = 100
    n_obs = 5 #this value will in relaity vary


    pdaf = PDAF()

    pdaf.x_pri[0] = r
    pdaf.x_pri[1] = theta
    pdaf.x_pri[2] = 10
    pdaf.x_pri[3] = 0.5

    for i in range(len(pdaf.x_post)):
        pdaf.Q[i, i] = 0.1

    for i in range(len(pdaf.C)):
        pdaf.R[i, i] = 0.1

    observations= np.ndarray((n_time_steps, n_obs, 2), dtype=float)

    #for i in range(n_time_steps):
    #   for j in range(n_obs):
    #       observations[i, j, 0] = r + np.random.randn(1) * pdaf.R[0, 0]
    #        observations[i, j, 1] = theta + np.random.randn(1) * pdaf.R[1, 1]

    observations[0] = None

    for o_time_k in observations:

        pdaf.correction_step(o_time_k)

        pdaf.prediction_step()

    print(pdaf.x_post)

    assert abs(pdaf.x_post[0] - r) < tollerance
    assert abs(pdaf.x_post[1] - theta) < tollerance
    assert abs(pdaf.x_post[2]) < tollerance
    assert abs(pdaf.x_post[3]) < tollerance





