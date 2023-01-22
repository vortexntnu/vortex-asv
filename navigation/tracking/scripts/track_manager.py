import numpy as np
from enum import Enum
from typing import List
from dataclasses import dataclass

from pdaf import PDAF

"""

Track manager:

Implementation based on section 7.4.3 in chapter 7 in "Fundemental in Sensor Fusion" by Brekke, 2021 edition.

Manage single object tracking based on M/N method. 
Subtasks: 

    Read NIS NEES 
    How should velocity and position be initialized? 
    How to take into account noise when defining validation gate max size? 
    Is there a way to model velocity constaints? 

    P: Seems like track jumps far away from previous estimates, even though it had status "confirmed".
        Seems like this is due to estimates jumping a lot in one step. Look an evolution of P and L.
    P: Sometimes L and P are Nan. 

    observations: 
        Estimate is largely affected when there is more then one obs inside validation gate (when status is confirmed).
        Seems like standard validation gate is larger then max size validation gate :) 

    TEST

        - Are traks deleted when tracks disaperas from surveilance region? 
        - Visualize tentative trakcs, confirmed trakcs, and tentative deletion tracks.  
        - How low can the detection probability be, and the track is still confirmed? 
        - How high can false detection probability be before false tracks are confirmed? 

        -Are estimates within a resonable tollerance from gt when there is resonable
            measurment noise
            prosses noise
            p of detection
            lidar range 
            size of validation gate
            size of max validation gate
            N
            M
            time_step

Integration:
    Integrate with ros
    Publish nav_msgs/Pose
"""


class TRACK_STATUS(Enum):
    tentative_confirm = 1
    confirmed = 2
    tentative_delete = 3


@dataclass
class PDAF_2MN:
    pdaf = PDAF()
    m = 0
    n = 0
    track_status = TRACK_STATUS.tentative_confirm

class TRACK_MANAGER:
    def __init__(self):
        # subscribe to topic with detections from point cloud
        # publish state of main track if status is confirmed

        self.prev_observations: List[np.ndarray] = []
        self.tentative_tracks: List[PDAF_2MN] = []

        self.N = 4
        self.M = 8
        self.main_track = PDAF_2MN()

        self.max_vel = 7  # [m/s]
        self.initial_measurement_covariance = 20
        self.initial_update_error_covariance = 1.0

    def cb(self, o_arr):
        if self.main_track.track_status == TRACK_STATUS.tentative_confirm:
            print("\n ------------ tentative confirm with ", len(self.tentative_tracks), " tracks.")

            for track in self.tentative_tracks:
                print("state: ", track.pdaf.state_pri[:2])
                print("n: ", track.n, "m: ", track.m)
                #print("S: ", track.S)

            print("update status on tentative tracks")
            self.update_status_on_tentative_tracks(o_arr)
            remaining_o_arr = self.remove_o_incorporated_in_tracks(
                o_arr
            )  # make this prettier/better
            print("add tentative tracks")
            self.add_tentative_tracks(remaining_o_arr)
            self.prev_observations = remaining_o_arr



        elif self.main_track.track_status == TRACK_STATUS.confirmed:
            print("\n ------------track still confirmed")
            print("state: ", self.main_track.pdaf.state_post)

            self.main_track.pdaf.correction_step(o_arr)
            self.main_track.pdaf.prediction_step()

            if len(self.main_track.pdaf.o_within_gate_arr) == 0:
                self.main_track.track_status = TRACK_STATUS.tentative_delete
                self.main_track.m = 0
                self.main_track.n = 0


        elif self.main_track.track_status == TRACK_STATUS.tentative_delete:
            print("\n ------------tentative delete")
            print("state: ", self.main_track.pdaf.state_post)
            print("n: ", self.main_track.n, "m: ", self.main_track.m)


            self.main_track.pdaf.correction_step(o_arr)
            self.main_track.pdaf.prediction_step()

            self.main_track.m += 1
            if len(self.main_track.pdaf.o_within_gate_arr) > 0:
                self.main_track.n += 1

            if self.main_track.n == self.N:
                self.main_track.track_status = TRACK_STATUS.confirmed

            elif self.main_track.m == self.M:
                self.main_track.track_status = TRACK_STATUS.tentative_confirm
                print("track deleted")


    def add_tentative_tracks(self, o_arr):
        # can this be written in a more efficient way?
        for prev_o in self.prev_observations:
            n = self.n_observations_inside_max_size_gate(prev_o, o_arr)
            if n > 0:
                print("track added")
                tentative_track = PDAF_2MN()

                tentative_track.pdaf.state_post[0] = prev_o[
                    0
                ]  # x #can improve. this point is one timestep delayed.
                tentative_track.pdaf.state_post[1] = prev_o[1]  # y
                tentative_track.pdaf.state_post[2] = 0  # x'
                tentative_track.pdaf.state_post[3] = 0  # y'

                for i in range(len(tentative_track.pdaf.state_pri)):
                    tentative_track.pdaf.P_pri[i][i] = self.initial_update_error_covariance

                self.tentative_tracks.append(tentative_track)

    def remove_o_incorporated_in_tracks(self, o_arr):
        remaining_o = o_arr.tolist()

        for track in self.tentative_tracks:
            for i, o in enumerate(remaining_o):
                # print(
                #     "\n --- dist ---- \n",
                #     o[0] - track.state_pri[0],
                #     o[1] - track.state_pri[0],
                # )
                dist = np.sqrt(
                    (o[0] - track.pdaf.state_pri[0]) ** 2 + (o[1] - track.pdaf.state_pri[1]) ** 2
                )
                if dist < self.max_vel * self.main_track.pdaf.time_step + self.initial_measurement_covariance:
                    remaining_o.pop(i)
                    # print(o, "deleted from o_arr")

        return remaining_o

    def update_status_on_tentative_tracks(self, o_arr):
        for track in self.tentative_tracks:

            track.n, track.m = self.update_confirmation_count(track, o_arr)

            track.pdaf.prediction_step()
            track.pdaf.correction_step(o_arr)

            if track.n == self.N:
                print("track confirmed")
                self.main_track = track
                self.main_track.track_status = TRACK_STATUS.confirmed
                self.tentative_tracks = []
            elif track.m == self.M:
                print("track deleted")
                self.tentative_tracks.remove(track)

    def update_confirmation_count(self, track: PDAF_2MN, o_arr):
        m = track.m + 1

        predicted_o = track.pdaf.C@track.pdaf.state_pri
        if self.n_observations_inside_max_size_gate(predicted_o, o_arr) > 0:
            n = track.n + 1
        else:
            n = track.n

        return n, m

    def n_observations_inside_max_size_gate(self, predicted_position, o_arr):
        n = 0

        for o in o_arr:
            # print("\n --- dist ---- \n", o[0]-predicted_position[0], o[1]-predicted_position[1])
            dist = np.sqrt(
                (o[0] - predicted_position[0]) ** 2
                + (o[1] - predicted_position[1]) ** 2
            )
            print("dist:", dist, "range:", self.max_vel*self.main_track.pdaf.time_step + self.initial_measurement_covariance)
            if dist < self.max_vel * self.main_track.pdaf.time_step + self.initial_measurement_covariance:
                n += 1

        return n



