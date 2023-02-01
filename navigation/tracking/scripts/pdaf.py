import numpy as np

"""
Single object tracking

Implementation based on chapter 7 in "Fundemental in Sensor Fusion" by Brekke, 2021 edition. 
Slides from PSU are nice for vizualization. https://www.cse.psu.edu/~rtc12/CSE598C/datassocPart2.pdf 

"""


class PDAF:
    def __init__(self, config):
        # x = [x, y, x', y']

        self.time_step = config["pdaf"]["time_step"]  # can vary for each time step
        self.state_post = np.array(config["pdaf"]["state_post"]).reshape(
            (4, 1)
        )  # posterior state estiamte
        self.P_post = np.array(config["pdaf"]["P_post"]).reshape(
            (4, 4)
        )  # posterior error covariance estimate

        self.state_pri = self.state_post  # prior state estiamte
        self.P_pri = self.P_post  # prior error covariance estimate

        self.L = np.zeros((4, 2))  # kalam gain

        self.C = np.array(
            [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0]]
        )  # C as in, y = C @ x

        self.A = np.array(  # A as in, x' = A @ x
            [
                [1.0, 0.0, self.time_step, 0],
                [0, 1.0, 0, self.time_step],
                [0, 0, 1.0, 0],  # assuming constnat velocity
                [0, 0, 0, 1.0],  # assuming constnat velocity
            ]
        )

        self.o_pri = np.zeros((2, 1))  # predicted observation

        self.Q = np.array(config["pdaf"]["Q"])  # covariance of disturbance

        self.R = np.array(config["pdaf"]["R"])  # covariance of noise

        self.S = np.zeros((2, 2))  # covariance of predicted measurment

        self.validation_gate_scaling_param = config["pdaf"][
            "validation_gate_scaling_param"
        ]

        self.minimal_mahalanobis_distance = config["pdaf"][
            "minimal_mahalanobis_distance"
        ]  # observations that are closer then this, will be set to min distance

        self.residual_vector = np.zeros((2, 1))

        self.p_no_match = config["pdaf"][
            "p_no_match"
        ]  # probabiity that no observations originates from the track

        self.p_match_arr = None  # probability the respective observations orriginate from the track. p_math_arr[0] always equal to probability that no observation originates from the track.

        self.o_within_gate_arr = None

    def cb(self, o_arr, time_step):
        self.update_model(time_step)
        self.prediction_step()
        self.correction_step(o_arr)

    def prediction_step(self):
        self.state_pri = self.A @ self.state_post
        self.P_pri = self.A @ self.P_post @ self.A.T + self.Q
        self.o_pri = self.C @ self.state_pri

    def correction_step(self, o):

        self.compute_L()
        self.compute_S()

        self.filter_observations_outside_gate(o)

        if len(self.o_within_gate_arr) == 0:
            self.state_post = self.state_pri
            self.P_post = self.P_pri

        else:
            self.compute_probability_of_matching_observations()
            self.compute_residual_vector()

            self.correct_state_vector()
            self.correct_P()

    def filter_observations_outside_gate(self, o):

        within_gate = []

        for o_i in o:
            o_i_arr = np.array(o_i).reshape(2, 1)
            mah_dist = self.compute_mah_dist(o_i_arr)
            if mah_dist < self.validation_gate_scaling_param**2:
                within_gate.append(o_i_arr)

        self.o_within_gate_arr = np.array(within_gate)

    def compute_mah_dist(self, o):
        "Compute mahaloanobis distance between observation and predicted observation."

        diff = o - self.o_pri
        mah_dist = diff.T @ np.linalg.inv(self.S) @ diff

        return mah_dist

    def compute_probability_of_matching_observations(self):

        score = np.zeros((len(self.o_within_gate_arr),))

        self.p_match_arr = np.zeros((len(self.o_within_gate_arr) + 1,))

        if len(self.o_within_gate_arr) == 0:
            self.p_match_arr[0] = 1.0
        else:
            self.p_match_arr[0] = self.p_no_match

        for i, o_i in enumerate(self.o_within_gate_arr):

            mah_distance = self.compute_mah_dist(o_i)
            if mah_distance <= self.minimal_mahalanobis_distance:
                score[i] = 1 / self.minimal_mahalanobis_distance
            else:
                score[i] = 1 / mah_distance

        score_sum = np.sum(score)
        for i in range(len(self.o_within_gate_arr)):
            self.p_match_arr[i + 1] = (score[i] / score_sum) * (1 - self.p_no_match)

    def update_model(self, time_step):
        self.time_step = time_step
        self.A = np.array(
            [
                [1.0, 0.0, self.time_step, 0],
                [0, 1.0, 0, self.time_step],
                [0, 0, 1.0, 0],  # assuming constnat velocity
                [0, 0, 0, 1.0],  # assuming constnat velocity
            ]
        )

    def compute_residual_vector(self):
        self.residual_vector = self.residual_vector * 0
        for i in range(len(self.o_within_gate_arr)):
            self.residual_vector += self.p_match_arr[i + 1] * (
                self.o_within_gate_arr[i] - self.o_pri
            )

    def compute_S(self):
        self.S = self.C @ self.P_pri @ self.C.T + self.R

    def compute_L(self):
        C_P_CT = self.C @ self.P_pri @ self.C.T
        self.L = self.P_pri @ self.C.T @ np.linalg.inv(C_P_CT + self.R)

    def correct_state_vector(self):
        self.state_post = self.state_pri + self.L @ self.residual_vector

    def correct_P(self):
        # qf - quadratic form
        qf_weighted_residual_vector = np.zeros((2, 2))
        for i, o_i in enumerate(self.o_within_gate_arr):
            conditional_innovations = o_i - self.o_pri

            qf_weighted_residual_vector += (
                self.p_match_arr[i + 1]
                * conditional_innovations
                @ conditional_innovations.T
            )

        qf_residual_vector = self.residual_vector @ self.residual_vector.T
        diff = qf_weighted_residual_vector - qf_residual_vector
        spread_of_innovations = self.L @ diff @ self.L.T  # given by (7.26) Brekke

        L_S_LT = self.L @ self.S @ self.L.T

        self.P_post = (
            self.P_pri - (1 - self.p_no_match) * L_S_LT + spread_of_innovations
        )  # given by (7.25) Brekke

    def create_observations_for_one_timestep(self, x, y):
        "Only used for testing. Not part of the tracker algorithm."

        n_obs = 10  # np.random.randint(0, 10)

        obs = np.ndarray((n_obs, 2), dtype=float)
        # add obs that are scaterd far apart
        for i in range(n_obs):
            obs[i, 0] = x + np.random.randn(1) * 1
            obs[i, 1] = y + np.random.randn(1) * 1

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
