import sys

sys.path.insert(0, "/home/hannahcl/Documents/vortex/monkey_tracking/data_generation")
sys.path.insert(0, "/home/hannahcl/Documents/vortex/monkey_tracking/config")
from scenarios import BaseScenario
from load_config import load_yaml_into_dotdict
import yaml

from track_manager import TRACK_MANAGER, TRACK_STATUS
import plots

import numpy as np


def data_generation():

    config = load_yaml_into_dotdict(
        "/home/hannahcl/Documents/vortex/monkey_tracking/config/scenario.yaml"
    )

    scenario = BaseScenario(config)

    measurements, ground_truths = scenario.run()

    return scenario, measurements, ground_truths


def test_cb():

    with open(
        "/home/hannahcl/Documents/vortex/asv_ws/src/vortex-asv/navigation/tracking/scripts/config_traking_sys.yaml",
        "r",
    ) as stream:
        config_loaded = yaml.safe_load(stream)

    manager = TRACK_MANAGER(config_loaded)

    x = 0
    y = 0
    n_timesteps = 50

    manager.main_track.pdaf.p_no_match = 0.01

    for i in range(n_timesteps):
        print("\n timestep", i, "\n")
        o_arr = manager.main_track.pdaf.create_observations_for_one_timestep(x, y)
        print("observations: ", o_arr)
        manager.cb(o_arr)

        for track in manager.tentative_tracks:
            print("state: ", track.pdaf.state_pri[:2])
            print("n: ", track.n, "m: ", track.m)

    print("final estimates: ", manager.main_track.pdaf.state_post)


def test_tentative_confirm_del():

    with open(
        "/home/hannahcl/Documents/vortex/asv_ws/src/vortex-asv/navigation/tracking/scripts/config_traking_sys.yaml",
        "r",
    ) as stream:
        config_loaded = yaml.safe_load(stream)

    manager = TRACK_MANAGER(config_loaded)

    manager.N = 3
    manager.M = 8

    scenario, measurements, ground_truths = data_generation()

    manager.main_track.pdaf.p_no_match = 1 - scenario.config.probability.detection
    manager.main_track.pdaf.time_step = scenario.config.dt

    manager.initial_measurement_covariance = scenario.config.noise.measurement

    for i in range(len(manager.main_track.pdaf.state_post)):
        manager.main_track.pdaf.Q[i, i] = scenario.config.noise.process

    for i in range(len(manager.main_track.pdaf.C)):
        manager.main_track.pdaf.R[i, i] = scenario.config.noise.measurement

    manager.max_vel = 3

    tentative_estimates = []
    conf_estimates = []
    tentative_del_estimates = []

    for o_time_k in measurements:

        o_list = []
        for o in o_time_k:
            o_list.append(o.pos)
        o_arr = np.array(o_list)

        # update
        manager.cb(o_arr)

        # add updates to lists that will be plottee
        if manager.main_track.track_status == TRACK_STATUS.tentative_confirm:
            last_addition_to_tentative_tracks = []
            for track in manager.tentative_tracks:
                last_addition_to_tentative_tracks.append(track.pdaf.state_post[:2])

            tentative_estimates.append(last_addition_to_tentative_tracks)

        if manager.main_track.track_status == TRACK_STATUS.confirmed:
            conf_estimates.append(manager.main_track.pdaf.state_post[:2])

        if manager.main_track.track_status == TRACK_STATUS.tentative_delete:
            tentative_del_estimates.append(manager.main_track.pdaf.state_post[:2])

    plots.plot_tentative_confirm_del(
        scenario,
        measurements,
        ground_truths,
        tentative_estimates,
        conf_estimates,
        tentative_del_estimates,
    )


# @pytest.mark.plot
def test_plot_interactive():
    with open(
        "/home/hannahcl/Documents/vortex/asv_ws/src/vortex-asv/navigation/tracking/scripts/config_traking_sys.yaml",
        "r",
    ) as stream:
        config_loaded = yaml.safe_load(stream)

    wait_for_btn_press = False

    manager = TRACK_MANAGER(config_loaded)

    manager.N = 3
    manager.M = 8

    scenario, measurements, ground_truths = data_generation()

    manager.main_track.pdaf.p_no_match = 1 - scenario.config.probability.detection
    manager.main_track.pdaf.time_step = scenario.config.dt

    manager.initial_measurement_covariance = 1  # scenario.config.noise.measurement

    for i in range(len(manager.main_track.pdaf.state_post)):
        manager.main_track.pdaf.Q[i, i] = scenario.config.noise.process

    for i in range(len(manager.main_track.pdaf.C)):
        manager.main_track.pdaf.R[i, i] = scenario.config.noise.measurement

    manager.max_vel = 4

    tentative_estimates = []
    conf_estimates = []
    tentative_del_estimates = []
    estimate_status = []

    for o_time_k in measurements:

        o_list = []
        for o in o_time_k:
            o_list.append(o.pos)
        o_arr = np.array(o_list)

        # update
        manager.cb(o_arr)

        # add updates to lists that will be plottee
        if manager.main_track.track_status == TRACK_STATUS.tentative_confirm:
            last_addition_to_tentative_tracks = []
            for track in manager.tentative_tracks:
                last_addition_to_tentative_tracks.append(track.pdaf.state_post[:2])
            tentative_estimates.append(last_addition_to_tentative_tracks)

        if manager.main_track.track_status == TRACK_STATUS.confirmed:
            conf_estimates.append(manager.main_track.pdaf.state_post[:2])

        if manager.main_track.track_status == TRACK_STATUS.tentative_delete:
            tentative_del_estimates.append(manager.main_track.pdaf.state_post[:2])

        estimate_status.append(manager.main_track.track_status)

    # print(estimate_status)

    plots.plot_interactive(
        scenario,
        measurements,
        ground_truths,
        tentative_estimates,
        conf_estimates,
        tentative_del_estimates,
        estimate_status,
        wait_for_btn_press,
    )

    assert True
