import sys
sys.path.insert(0,'/home/hannahcl/Documents/vortex/monkey_tracking/data_generation')
from scenarios import BaseScenario, plot
import argparse
import matplotlib.pyplot as plt

from constant_velocity_object import CVObject, Measurement

from utility import time_from_step
from load_config import load_yaml_into_dotdict

from track_manager import TRACK_MANAGER


def data_generation():

    config = load_yaml_into_dotdict('scenario.yaml')

    scenario = BaseScenario(config)

    measurements, ground_truths = scenario.run()

    for measurements_at_t in measurements:
        for measurement in measurements_at_t:
            print(measurement)
        print()

    #plot(scenario, measurements, ground_truths

def test_cb():

    manager = TRACK_MANAGER()

    x = 20
    y = 20
    tollerance = 0.5
    n_timesteps = 50


    for i in range(n_timesteps):
        print("\n timestep", i, "\n")
        o_arr = manager.main_track.create_observations_for_one_timestep(x, y)
        print("observations: ", o_arr)
        manager.cb(o_arr)

        for track in manager.tentative_tracks:
            print("state: ", track.state_pri[:2])
            print("n: ", track.n, "m: ", track.m)

    print("final estimates: ", manager.main_track.state_post)

def test_add_tentative_tracks():

    manager = TRACK_MANAGER()

    x = 0
    y = 0
    tollerance = 0.5
    n_timesteps = 50

    manager.main_track.R[0,0] = 0.001
    manager.main_track.R[1,1] = 0.001


    for i in range(n_timesteps):
        print("\n timestep", i, "\n")

        o_arr = manager.main_track.create_observations_for_one_timestep(x, y)
        print(len(manager.tentative_tracks),"tentative tracks: ")
        for track in manager.tentative_tracks:
            print("state: ", track.state_pri[:2])
            print("n: ", track.n)

        print("observations: ", o_arr)

        manager.update_status_on_tentative_tracks(o_arr)
        new_o_arr = manager.remove_o_incorporated_in_tracks(o_arr)
        manager.add_tentative_tracks(new_o_arr)

        manager.prev_observations = new_o_arr






