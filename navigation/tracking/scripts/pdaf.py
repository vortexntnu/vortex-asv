import numpy as np

"""
Single object tracking

Sub tasks: 

    Use a kalam gain so that the filter is numerically stable. 
    SINGULAR MATRICIES

    TEST
        - Visualize position estimates with clutter.
        - Visualize velocity estimates with vectors. 
        - Visualize validation gate. 
        - Does the frequency of "zero detections within gate" correspond with p of detection? 

    Observations: 
        P pri and post are not symmetrical.
        P post grows to values over 1000.
        When a measurment not orriginating from the target is within the validation gate -> P post explodes and the estimates jumps. But then rains its self inn pretty fast. 
        Negative elements in P post!!

"""


class PDAF:
    def __init__(self):
        # x = [x, y, x', y']

        self.time_step = 0.1
        self.state_pri = np.ndarray(
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

        self.state_post = np.ndarray((4,), dtype=float)
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

        self.o_pri =  np.ndarray((2,), dtype=float) #predicted observation

        self.Q = np.ndarray(
            (4, 4),
            buffer=np.array(
                [[0.001, 0, 0, 0], [0, 0.001, 0, 0], [0, 0, 0.1, 0], [0, 0, 0, 0.1]]
            ),
            dtype=float,
        )

        self.R = np.ndarray((2, 2), buffer=np.array([[0.1, 0], [0, 0.1]]), dtype=float)

        self.S = np.ndarray((2, 2), buffer=np.array([[0.1, 0], [0, 0.1]]), dtype=float)

        self.validation_gate_scaling_param = (
            5  # number of standard deviations we are willing to consider.
        )

        self.residual_vector = np.ndarray((2,), dtype=float)
        self.p_no_match = 0.01  # probabiity that no observations matches the track
        self.p_match_arr = np.ndarray(
            (2,), dtype=float
        )  # Lengt of this array will vary based on how many observations there are.
        self.o_within_gate_arr = np.ndarray(
            (2, 2), dtype=float
        )  # Lengt of this array will vary based on how many observations there are.

        self.n = 0  # used for N/M track manager
        self.m = 0  # used for N/M track manager

    def compute_mah_dist(self, o):
        "Compute mahaloanobis distance between observation and predicted observation."
        o_predicted = np.matmul(self.C, self.state_pri)
        diff = o - o_predicted
        mah_dist = np.matmul(
            diff.reshape(2, 1).T, np.matmul(np.linalg.inv(self.S), diff.reshape(2, 1))
        )
        return mah_dist

    def filter_observations_outside_gate(self, o):

        within_gate = []

        for o_i in o:
            mah_dist = self.compute_mah_dist(o_i)
            if mah_dist < self.validation_gate_scaling_param**2:
                within_gate.append(o_i)

        self.o_within_gate_arr = np.array(within_gate)

    def compute_probability_of_matching_observations(self):

        score = np.ndarray(
            (len(self.o_within_gate_arr),), dtype=float
        )  # score for each observation based on distance from predicted position

        self.p_match_arr = np.ndarray((len(self.o_within_gate_arr) + 1,), dtype=float)

        if len(self.o_within_gate_arr) == 0:
            self.p_match_arr[0] = 1.0
        else:
            self.p_match_arr[0] = self.p_no_match

        for i, o_i in enumerate(self.o_within_gate_arr):

            mah_distance = self.compute_mah_dist(o_i)
            if (
                mah_distance <= 0.1
            ):  # In order to avoid infinte high weights. Choose an approriate threshold.
                score[i] = 10
            else:
                score[i] = 1 / mah_distance

        score_sum = np.sum(score)
        for i in range(len(self.o_within_gate_arr)):
            self.p_match_arr[i + 1] = (score[i] / score_sum) * (1 - self.p_no_match)

    def compute_residual_vector(self):
        self.residual_vector[0] = 0  # x
        self.residual_vector[1] = 0  # y

        for i in range(len(self.o_within_gate_arr)):
            self.residual_vector[0] += self.p_match_arr[i + 1] * (
                self.o_within_gate_arr[i][0] - self.state_pri[0]
            )
            self.residual_vector[1] += self.p_match_arr[i + 1] * (
                self.o_within_gate_arr[i][1] - self.state_pri[1]
            )

    def compute_S(self):
        C_P = np.matmul(self.C, self.P_pri)
        self.S = np.matmul(C_P, self.C.T) + self.R

    def compute_L(self):
        P_CT = np.matmul(self.P_pri, self.C.T)
        C_P_CT = np.matmul(self.C, P_CT)
        self.L = np.matmul(P_CT, np.linalg.inv(C_P_CT + self.R))

        # print("\n --- L ---\n")
        # print(self.L)

    def correct_state_vector(self):
        self.state_post = self.state_pri + np.matmul(self.L, self.residual_vector)

    def correct_P(self):
        temp1 = np.ndarray((2, 2), dtype=float)
        for i, o_i in enumerate(self.o_within_gate_arr):
            ny_ak = o_i - np.matmul(self.C, self.state_pri)
            temp1 += self.p_match_arr[i + 1] * np.matmul(ny_ak.reshape(2,1), ny_ak.reshape(2,1).T)
        temp2 = temp1 - np.matmul(self.residual_vector.reshape(2,1), self.residual_vector.reshape(2,1).T)

        spread_of_innovations = np.matmul(
            self.L, np.matmul(temp2, self.L.T)
        )  # given by (7.26) Brekke
        L_S_LT = np.matmul(self.L, np.matmul(self.S, self.L.T))

        # print("\n -------- P pri --------------- \n", self.P_pri)
        # print("\n -------- L*S*L^T --------------- \n", L_S_LT)
        # print("\n -------- P ~ --------------- \n", spread_of_innovations)
        # print("\n -------- temp 2 --------------- \n", temp2)

        self.P_post = (
            self.P_pri - (1 - self.p_no_match) * L_S_LT + spread_of_innovations
        )  # given by (7.25) Brekke

        #print("\n -------- P post --------------- \n", self.P_post)

    def prediction_step(self):
        self.state_pri = np.matmul(self.A, self.state_post)
        self.P_pri = np.matmul(self.A, np.matmul(self.P_post, self.A.T)) + self.Q
        self.o_pri = np.matmul(self.C, self.state_pri)

        # print("\n -------- P pri --------------- \n", self.P_pri)

    def correction_step(self, o):

        self.compute_L()
        self.compute_S()

        self.filter_observations_outside_gate(o)
        print("obs within gate: ", self.o_within_gate_arr)

        if len(self.o_within_gate_arr) == 0:
            self.state_post = self.state_pri
            self.P_post = self.P_pri

        else:
            self.compute_probability_of_matching_observations()
            self.compute_residual_vector()

            self.correct_state_vector()
            self.correct_P()

    def create_observations_for_one_timestep(self, x, y):
        "Only used for testing. Not part of the tracker algorithm."

        n_obs = np.random.randint(0, 10)

        obs = np.ndarray((n_obs, 2), dtype=float)
        # add obs that are scaterd far apart
        for i in range(n_obs):
            obs[i, 0] = x + np.random.randn(1) * 100
            obs[i, 1] = y + np.random.randn(1) * 100

        # add obs that corresponds to the acctual track (1-p_no_match)*100 prosent of the time.
        random_int = np.random.randint(0, 100)
        if (random_int < 100 * (1 - self.p_no_match)) and (n_obs > 0):
            obs[-1, 0] = x + np.random.randn(1) * self.R[0, 0]
            obs[-1, 1] = y + np.random.randn(1) * self.R[1, 1]

        return obs

    def create_observations_for_one_timestep_simple_version(self, x, y):
        "Only used for testing. Not part of the tracker algorithm."

        n_obs = np.random.randint(0, 10)

        obs = np.ndarray((n_obs, 2), dtype=float)
        for i in range(n_obs):
            obs[i, 0] = x + np.random.randn(1) * self.R[0, 0]
            obs[i, 1] = y + np.random.randn(1) * self.R[1, 1]

        return obs
