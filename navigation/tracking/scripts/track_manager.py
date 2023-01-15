import numpy as np
from enum import Enum
from typing import List

from pdaf import PDAF

"""

Track manager:

Manage single object tracking based on M/N method. 
Subtasks: 

    Read NIS NEES 
    How should velocity and position be initialized? 
    How to take into account noise when defining validation gate max size? 

    P: Seems like track jumps far away from previous estimates, even though it had status "confirmed".
        Seems like this is due to estimates jumping a lot in one step. Look an evolution of P and L.
    P: Seems like estimates are confirmed even though there has not been a tentative track close by. 
        See if this is solved by deleting tentative tracks when transitioning state. 


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

    observations: 
        Tentative tracks follow GT for a long time without confirming the track. 
        When computing dist under "n obs inside gate" the dist is unaturally large. Range is only 0.4.


Integration:
    Integrate with ros
    Publish nav_msgs/Pose
"""


class TRACK_MANAGER:
    def __init__(self):
        # subscribe to topic with detections from point cloud
        # publish state of main track if status is confirmed

        self.prev_observations: List[np.ndarray] = []
        self.tentative_tracks: List[PDAF] = []

        self.N = 4
        self.M = 8
        self.main_track = PDAF()

        self.main_track_status = TRACK_STATUS.tentative_confirm
        self.main_track_status_trans = TRACK_STATUS_TRANSITION.none

        self.max_vel = 5  # [m/s]
        self.sd = 1.0  # [m/s] standard deviation for measurments. Same thing as R, but I want it in 1x1, not 2x2.
        self.initial_update_error_covariance = 1.0

    def cb(self, o_arr):
        if self.main_track_status == TRACK_STATUS.tentative_confirm:
            self.update_status_on_tentative_tracks(o_arr)
            remaining_o_arr = self.remove_o_incorporated_in_tracks(
                o_arr
            )  # make this prettier/better
            self.add_tentative_tracks(remaining_o_arr)
            self.prev_observations = remaining_o_arr

            print("tentative confirm with ", len(self.tentative_tracks), " tracks.")

            for track in self.tentative_tracks:
                print("state: ", track.state_pri[:2])
                print("n: ", track.n, "m: ", track.m)


        elif self.main_track_status == TRACK_STATUS.confirmed:
            self.main_track.correction_step(o_arr)
            self.main_track.prediction_step()

            if len(self.main_track.o_within_gate_arr) == 0:
                self.main_track_status = TRACK_STATUS.tentative_delete
                self.main_track.m = 0
                self.main_track.n = 0

            print("track still confirmed")
            print("state: ", self.main_track.state_post)

        elif self.main_track_status == TRACK_STATUS.tentative_delete:
            self.main_track.correction_step(o_arr)
            self.main_track.prediction_step()

            self.main_track.m += 1
            if len(self.main_track.o_within_gate_arr) == 0:
                self.main_track.n += 1

            if self.main_track.n == self.N:
                self.main_track_status = TRACK_STATUS.confirmed
            elif self.main_track.m == self.M:
                self.main_track_status = TRACK_STATUS.tentative_confirm
                print("track deleted")

            print("tentative delete")
            print("state: ", self.main_track.state_post)

        #actions when transitionning between states
        if self.main_track_status_trans == TRACK_STATUS_TRANSITION.tentative_conf_to_conf:
            self.tentative_tracks = [] #delete tentative trakcs
            self.main_track_status_trans = TRACK_STATUS_TRANSITION.none

    def add_tentative_tracks(self, o_arr):
        # can this be written in a more efficient way?
        for prev_o in self.prev_observations:
            n = self.n_observations_inside_max_size_gate(prev_o, o_arr)
            if n > 0:
                print("track added")
                tentative_track = PDAF()

                tentative_track.state_post[0] = prev_o[
                    0
                ]  # x #can improve. this point is one timestep delayed.
                tentative_track.state_post[1] = prev_o[1]  # y
                tentative_track.state_post[2] = 0  # x'
                tentative_track.state_post[3] = 0  # y'

                for i in range(len(tentative_track.state_pri)):
                    tentative_track.P_pri[i][i] = self.initial_update_error_covariance

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
                    (o[0] - track.state_pri[0]) ** 2 + (o[1] - track.state_pri[1]) ** 2
                )
                if dist < self.max_vel * self.main_track.time_step + self.sd:
                    remaining_o.pop(i)
                    # print(o, "deleted from o_arr")

        return remaining_o

    def update_status_on_tentative_tracks(self, o_arr):
        for track in self.tentative_tracks:

            track.n, track.m = self.update_confirmation_count(track, o_arr)

            track.prediction_step()
            track.correction_step(o_arr)

            if track.n == self.N:
                print("track confirmed")
                self.main_track_status = TRACK_STATUS.confirmed
                self.main_track_status_trans = TRACK_STATUS_TRANSITION.tentative_conf_to_conf
                self.main_track = track
                self.tentative_tracks.remove(track)
            elif track.m == self.M:
                print("track deleted")
                self.tentative_tracks.remove(track)

    def update_confirmation_count(self, pdaf: PDAF, o_arr):
        m = pdaf.m + 1

        if self.n_observations_inside_max_size_gate(pdaf.state_pri[:2], o_arr) > 0:
            n = pdaf.n + 1
        else:
            n = pdaf.n

        return n, m

    def n_observations_inside_max_size_gate(self, predicted_position, o_arr):
        n = 0

        for o in o_arr:
            # print("\n --- dist ---- \n", o[0]-predicted_position[0], o[1]-predicted_position[1])
            dist = np.sqrt(
                (o[0] - predicted_position[0]) ** 2
                + (o[1] - predicted_position[1]) ** 2
            )
            print("dist:", dist, "range:", self.max_vel*self.main_track.time_step + self.sd )
            if dist < self.max_vel * self.main_track.time_step + self.sd:
                n += 1

        return n


class TRACK_STATUS(Enum):
    tentative_confirm = 1
    confirmed = 2
    tentative_delete = 3

class TRACK_STATUS_TRANSITION(Enum):
    none = 0
    tentative_conf_to_conf = 1
    conf_to_tentative_del = 2
    tentative_del_to_tentative_conf = 3
