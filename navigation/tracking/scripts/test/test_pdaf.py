import numpy as np

"""
Single object tracking

Sub tasks: 

    Include an exitance variable in the state vector (e.g. implement ipda)

    Track manager. See 2&M/N in Brekke.

    Use a kalam gain so that the filter is numerically stable. 

    Integrate with ROS. 
        use std ros msgs

    Port to CPP ? 

    Vizualize test. 

    TEST
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

        self.Q = np.ndarray(
            (4, 4),
            buffer=np.array(
                [[0.001, 0, 0, 0], [0, 0.001, 0, 0], [0, 0, 0.1, 0], [0, 0, 0, 0.1]]
            ),
            dtype=float,
        )

        self.R = np.ndarray((2, 2), buffer=np.array([[0.1, 0], [0, 0.1]]), dtype=float)

        self.S = np.ndarray((2, 2), buffer=np.array([[0.1, 0], [0, 0.1]]), dtype=float)

        self.validation_gate_scaling_param = 5  #number of standard deviations we are willing to consider. 

        self.residual_vector = np.ndarray((2,), dtype=float)
        self.p_no_match = 0.2  # probabiity that no observations matches the track
        self.p_match_arr = np.ndarray(
            (2,), dtype=float
        )  # Lengt of this array will vary based on how many observations there are.
        self.o_within_gate_arr = np.ndarray(
            (2,2), dtype=float
        )  # Lengt of this array will vary based on how many observations there are.

    def compute_mah_dist(self, o):
        "Compute mahaloanobis distance between observation and predicted observation."
        o_predicted = np.matmul(self.C, self.state_pri)
        diff = o-o_predicted
        mah_dist = np.matmul(diff.reshape(2,1).T, np.matmul(np.linalg.inv(self.S), diff.reshape(2,1)))
        return mah_dist

    def filter_observations_outside_gate(self, o):

        self.compute_S()

        within_gate = []

        for o_i in o:
            mah_dist = self.compute_mah_dist(o_i)
            if mah_dist < self.validation_gate_scaling_param**2:
                within_gate.append(o_i)
            else: 
                print("o outside gate! o: ", o_i)

        self.o_within_gate_arr = np.array(within_gate)


    def compute_probability_of_matching_observations(self):

        score = np.ndarray(
            (len(self.o_within_gate_arr),), dtype=float
        )  # score for each observation based on distance from predicted position

        self.p_match_arr = np.ndarray(
            (len(self.o_within_gate_arr) +1,), dtype=float
        )

        if len(self.o_within_gate_arr)==0:
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
            self.residual_vector[0] += self.p_match_arr[i + 1] * (self.o_within_gate_arr[i][0] - self.state_pri[0])
            self.residual_vector[1] += self.p_match_arr[i + 1] * (self.o_within_gate_arr[i][1] - self.state_pri[1])

    def compute_S(self):
        C_P = np.matmul(self.C, self.P_pri)
        self.S = np.matmul(C_P, self.C.T) + self.R

    def compute_L(self):
        P_CT = np.matmul(self.P_pri, self.C.T)
        C_P_CT = np.matmul(self.C, P_CT)
        self.L = np.matmul(P_CT, np.linalg.inv(C_P_CT + self.R))

    def correct_state_vector(self):
        self.state_post = self.state_pri + np.matmul(self.L, self.residual_vector)

    def correct_P(self):
        temp1 = np.ndarray((2, 2), dtype=float)
        for i, o_i in enumerate(self.o_within_gate_arr):
            ny_ak = o_i - np.matmul(self.C, self.state_pri)
            temp1 += self.p_match_arr[i + 1] * np.matmul(ny_ak, ny_ak.T)

        temp2 = temp1 - np.matmul(self.residual_vector, self.residual_vector.T)

        spread_of_innovations =  np.matmul(self.L, np.matmul(temp2, self.L.T)) #given by (7.26) Brekke
        L_S_LT = np.matmul(self.L, np.matmul(self.S, self.L.T))

        self.P_post = self.P_pri -(1-self.p_no_match)*L_S_LT + spread_of_innovations #given by (7.25) Brekke


    def prediction_step(self):
        self.state_pri = np.matmul(self.A, self.state_post)
        self.P_pri = (
            np.matmul(self.A, np.matmul(self.P_post, self.A.T)) + self.Q
        )

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

    def create_observations_for_one_timestep(self, x, y):
        "Only used for testing. Not part of the tracker algorithm."

        n_obs = np.random.randint(0, 10)

        obs = np.ndarray((n_obs, 2), dtype=float)
        #add obs that are scaterd far apart
        for i in range(n_obs):
            obs[i, 0] = x + np.random.randn(1) * 2
            obs[i, 1] = y + np.random.randn(1) * 2

        #add obs that corresponds to the acctual track (1-p_no_match)*100 prosent of the time. 
        random_int = np.random.randint(0, 100)
        if (random_int < 100*(1-self.p_no_match)) and (n_obs > 0):
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

def compute_r(x, y):
    return np.sqrt(x**2 + y**2)

# -----------------------------------

def test_filter_observations_outside_gate():

    pdaf = PDAF()

    n_obs = 10
    x = 1
    y = 0.5

    observations = np.ndarray((n_obs, 2), dtype=float)
    for i in range(n_obs):
        observations[i, 0] = x 
        observations[i, 1] = y 

    for i in range(n_obs-5):
        observations[i, 0] = x +2
        observations[i, 1] = y 

    print("observations: ", observations)

    pdaf.filter_observations_outside_gate(observations)

    print("observations within gate: ", pdaf.o_within_gate_arr)

def test_compute_probability_of_matching_observations():

    pdaf = PDAF()

    n_obs = 10
    x = 1
    y = 0.5

    observations = np.ndarray((n_obs, 2), dtype=float)
    
    for i in range(n_obs):
        observations[i, 0] = x + i*0.1
        observations[i, 1] = y 

    pdaf.filter_observations_outside_gate(observations)
    pdaf.compute_probability_of_matching_observations()

    print("p of matches: ", pdaf.p_match_arr)
    print("p sum: ", np.sum(pdaf.p_match_arr))

    assert(abs(np.sum(pdaf.p_match_arr) - 1 )< 0.00001)
    assert(pdaf.p_match_arr[0] == pdaf.p_no_match or len(pdaf.o_within_gate_arr)==0)


def test_compute_residual_vector():

    pdaf = PDAF()

    n_obs = 10
    x = 4
    y = 0.5

    observations = np.ndarray((n_obs, 2), dtype=float)
    
    for i in range(n_obs):
        observations[i, 0] = x 
        observations[i, 1] = y 

    pdaf.filter_observations_outside_gate(observations)
    pdaf.compute_probability_of_matching_observations()
    pdaf.compute_residual_vector()

    print(pdaf.residual_vector)

def test_correct_P():
    pdaf = PDAF()

    n_obs = 3
    x = 4
    y = 0.5

    observations = np.ndarray((n_obs, 2), dtype=float)
    
    for i in range(n_obs):
        observations[i, 0] = x 
        observations[i, 1] = y 

    pdaf.compute_L()
    pdaf.compute_S()

    pdaf.filter_observations_outside_gate(observations)
    pdaf.compute_probability_of_matching_observations()
    pdaf.compute_residual_vector()
    pdaf.correct_state_vector()

    pdaf.correct_P()

def test_pdaf_zero_velocity():

    x = 5
    y = 1
    tollerance = 0.5
    n_timesteps = 200

    pdaf = PDAF()

    pdaf.state_pri[0] = 0
    pdaf.state_pri[1] = 0
    pdaf.state_pri[2] = 10
    pdaf.state_pri[3] = 5

    for i in range(len(pdaf.state_post)):
        pdaf.Q[i, i] = 0.1

    for i in range(len(pdaf.C)):
        pdaf.R[i, i] = 0.01

    for k in range(n_timesteps):

        o_time_k = pdaf.create_observations_for_one_timestep_simple_version(x, y)

        pdaf.correction_step(o_time_k)

        pdaf.prediction_step()

        #print("observations: ", o_time_k)
        #print("estimates: ", pdaf.x_post)

    print(pdaf.state_post)

    assert abs(pdaf.state_post[0] - x) < tollerance
    assert abs(pdaf.state_post[1] - y) < tollerance
    assert abs(pdaf.state_post[2]) < tollerance
    assert abs(pdaf.state_post[3]) < tollerance

def test_pdaf_constant_vel():
    """
    We simulate a boat with constant velocity in both r and theta.
    """

    x = 5
    x_der = 0.9
    y = 1.2
    y_der = 0.8
    tollerance = 0.5
    n_timesteps = 200

    pdaf = PDAF()

    pdaf.state_pri[0] = 0
    pdaf.state_pri[1] = 0
    pdaf.state_pri[2] = 10
    pdaf.state_pri[3] = 10

    for i in range(len(pdaf.state_post)):
        pdaf.Q[i, i] = 0.1

    for i in range(len(pdaf.C)):
        pdaf.R[i, i] = 0.1

    measurments = np.ndarray((n_timesteps, 2), dtype=float)
    for i in range(n_timesteps):
        measurments[i, 0] = (
            x 
            + i * x_der * pdaf.time_step 
            + np.random.randn(1) * pdaf.R[0, 0]
        )
        measurments[i, 1] = (
            y
            + i * y_der * pdaf.time_step
            + np.random.randn(1) * pdaf.R[1, 1]
        )

    for k in range(n_timesteps):

        o_time_k = pdaf.create_observations_for_one_timestep_simple_version(
            x + k * x_der * pdaf.time_step , 
            y + k * y_der * pdaf.time_step)

        pdaf.correction_step(o_time_k)

        pdaf.prediction_step()

    print("final true state: ", 
    (x + x_der * (n_timesteps - 1) * pdaf.time_step), 
    (y + y_der * (n_timesteps - 1) * pdaf.time_step), 
    x_der, 
    y_der)
    
    print("final observations: ", o_time_k)

    print("final estimates: ", pdaf.state_post)

    assert (
        abs(pdaf.state_post[0] - (x + x_der * (n_timesteps - 1) * pdaf.time_step))
        < tollerance
    )
    assert (
        abs(pdaf.state_post[1] - (y + y_der * (n_timesteps - 1) * pdaf.time_step))
        < tollerance
    )
    assert abs(pdaf.state_post[2] - x_der) < tollerance
    assert abs(pdaf.state_post[3] - y_der) < tollerance







