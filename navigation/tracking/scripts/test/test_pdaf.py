from pdaf import PDAF
import numpy as np

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

    # measurments = np.ndarray((n_timesteps, 2), dtype=float)
    # for i in range(n_timesteps):
    #     measurments[i, 0] = (
    #         x 
    #         + i * x_der * pdaf.time_step 
    #         + np.random.randn(1) * pdaf.R[0, 0]
    #     )
    #     measurments[i, 1] = (
    #         y
    #         + i * y_der * pdaf.time_step
    #         + np.random.randn(1) * pdaf.R[1, 1]
    #     )

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







