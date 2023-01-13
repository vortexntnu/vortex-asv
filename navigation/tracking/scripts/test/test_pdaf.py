from pdaf import PDAF
import numpy as np

import sys
sys.path.insert(0,'/home/hannahcl/Documents/vortex/monkey_tracking/data_generation')
from scenarios import BaseScenario
from load_config import load_yaml_into_dotdict

import plots

from track_manager import TRACK_MANAGER


def data_generation():

    config = load_yaml_into_dotdict('scenario.yaml')

    scenario = BaseScenario(config)

    measurements, ground_truths = scenario.run()

    # for measurements_at_t in measurements:
    #     for measurement in measurements_at_t:
    #         print(measurement)
    #     print()

    #plot(scenario, measurements, ground_truths

    return scenario, measurements, ground_truths

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


def test_pdaf_constant_vel_data():
    """
    We simulate a boat with constant velocity in both x and y.
    """

    pdaf = PDAF()

    pdaf.state_pri[0] = 0
    pdaf.state_pri[1] = 0
    pdaf.state_pri[2] = 0
    pdaf.state_pri[3] = 0

    for i in range(len(pdaf.state_post)):
        pdaf.Q[i, i] = 0.1

    for i in range(len(pdaf.C)):
        pdaf.R[i, i] = 0.1

    scenario, measurements, ground_truths = data_generation()
    estimates = []

    for o_time_k in measurements:

        # print("new time step")
        # print(o_time_k)

        o_list = []
        for o in o_time_k:
            o_list.append(o.pos)

        pdaf.correction_step(o_list)

        pdaf.prediction_step()   

        estimates.append(pdaf.state_post[:2])
    
    print("final observations: ", measurements[-1])

    print("final gt: ", ground_truths[-1])

    print("final estimates: ", pdaf.state_post)

    plots.plot_with_estimates(scenario, measurements, ground_truths, estimates)

# def test_plot_pos_and_vel():

#     pdaf = PDAF()

#     pdaf.state_pri[0] = 0
#     pdaf.state_pri[1] = 0
#     pdaf.state_pri[2] = 10
#     pdaf.state_pri[3] = 5

#     for i in range(len(pdaf.state_post)):
#         pdaf.Q[i, i] = 0.1

#     for i in range(len(pdaf.C)):
#         pdaf.R[i, i] = 0.1

#     scenario, measurements, ground_truths = data_generation()
#     estimates = []

#     for o_time_k in measurements:
 
#         o_list = []
#         for o in o_time_k:
#             o_list.append(o.pos)

#         pdaf.correction_step(o_list)

#         pdaf.prediction_step()   

#         estimates.append(pdaf.state_post)
    
#     print("final observations: ", measurements[-1])

#     print("final gt: ", ground_truths[-1])

#     print("final estimates: ", pdaf.state_post)

#     plot_pos_and_vel(estimates)









