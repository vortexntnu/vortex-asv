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


# @pytest.mark.plot
def test_plot_interactive():
    with open(
        "/home/hannahcl/Documents/vortex/asv_ws/src/vortex-asv/navigation/tracking/scripts/config_traking_sys.yaml",
        "r",
    ) as stream:
        config_loaded = yaml.safe_load(stream)

    wait_for_btn_press = False

    manager = TRACK_MANAGER(config_loaded)

    scenario, measurements, ground_truths = data_generation()

    tentative_estimates = []
    conf_estimates = []
    tentative_del_estimates = []
    estimate_status = []

    time_step = 0.1

    for i in range(len(measurements)):

        o_list = []
        for o in measurements[i]:
            o_list.append(o.pos)
        o_arr = np.array(o_list)

        # update
        manager.cb(o_arr, time_step)

        # add updates to lists that will be plottee
        if manager.main_track.track_status == TRACK_STATUS.tentative_confirm:
            last_addition_to_tentative_tracks = []
            for track in manager.tentative_tracks:
                last_addition_to_tentative_tracks.append(track.pdaf.state_post)
            tentative_estimates.append(last_addition_to_tentative_tracks)

        if manager.main_track.track_status == TRACK_STATUS.confirmed:
            conf_estimates.append(manager.main_track.pdaf.state_post)

        if manager.main_track.track_status == TRACK_STATUS.tentative_delete:
            tentative_del_estimates.append(manager.main_track.pdaf.state_post)

        estimate_status.append(manager.main_track.track_status)

    plots.plot_interactive_velocity(
        scenario,
        measurements,
        ground_truths,
        tentative_estimates,
        conf_estimates,
        tentative_del_estimates,
        estimate_status,
        wait_for_btn_press,
    )

    # plots.plot_vel(
    # ground_truths,
    # conf_estimates,
    # estimate_status,
    # )

    assert True
