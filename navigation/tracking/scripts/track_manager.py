import numpy as np
from enum import Enum
from typing import List

from pdaf import PDAF

"""

Track manager:
    Only works when init pos is close to measurments. 

    How should velocity and position be initialized? 
    How to take into account noise when defining validation gate max size? 
    TEST
"""

class TRACK_MANAGER:
    def __init__(self):
        #subscribe to topic with detections from point cloud 
        #publish state of main track if status is confirmed

        self.prev_observations: List[np.ndarray]= [] 
        self.tentative_tracks: List[PDAF] = [] 

        self.N = 4
        self.M = 10
        self.main_track = PDAF()
        
        self.main_track_status = TRACK_STATUS.tentative_confirm

        self.max_vel = 2 #[m/s]
        self.sd = 1.0 #[m/s] standard deviation for measurments. Same thing as R, but I want it in 1x1, not 2x2. 
        self.time_step = self.main_track.time_step

    def cb(self, o_arr):
        if self.main_track_status == TRACK_STATUS.tentative_confirm:
            self.update_status_on_tentative_tracks(o_arr) #check if observation are close to tentative trakcs
            remaining_o_arr = self.remove_o_incorporated_in_tracks(o_arr) #make this prettier/better
            self.add_tentative_tracks(remaining_o_arr) #check if observation are close to previous observations. Obs: will end up with multiple tentative trakcs for the same track. 
            self.prev_observations = remaining_o_arr

            print("tentative confirm with ", len(self.tentative_tracks), " tracks.")


        elif self.main_track_status == TRACK_STATUS.confirmed:
            self.main_track.correction_step(o_arr)
            self.main_track.prediction_step()

            if len(self.main_track.o_within_gate_arr) == 0:
                self.main_track_status = TRACK_STATUS.tentative_delete
                self.main_track.m = 0
                self.main_track.n = 0

            print("track confirmed")

        elif self.main_track_status == TRACK_STATUS.tentative_delete:
            self.main_track.correction_step(o_arr)
            self.main_track.prediction_step()  

            self.main_track.m += 1
            if len(self.main_track.o_within_gate_arr) == 0:
                self.main_track.n += 1

            if self.main_track.n == self.main_track.N:
                self.main_track_status = TRACK_STATUS.confirmed
            elif self.main_track.m == self.main_track.M:
                self.main_track_status = TRACK_STATUS.tentative_confirm 

            print("tentative delete")  
   

    def add_tentative_tracks(self, o_arr):
        #can this be written in a more efficient way? 
        for prev_o in self.prev_observations:
            n = self.n_observations_inside_max_size_gate(prev_o, o_arr)
            if n > 0:
                print("track added")
                tentative_track = PDAF() 

                tentative_track.state_pri[0] = prev_o[0] #x #can improve. this point is one timestep delayed.
                tentative_track.state_pri[1] = prev_o[1] #y
                tentative_track.state_pri[2] = 0    #x'
                tentative_track.state_pri[3] = 0    #y'

                self.tentative_tracks.append(tentative_track) 

    def remove_o_incorporated_in_tracks(self, o_arr):
        remaining_o = o_arr.tolist()

        for track in self.tentative_tracks:
            for i, o in enumerate(remaining_o):
                dist = np.sqrt((o[0]-track.state_pri[0])**2 + (o[1]-track.state_pri[1])**2)
                if dist < self.max_vel*self.time_step + self.sd:
                    remaining_o.pop(i)
                    print(o, "deleted from o_arr")

        return remaining_o

    def update_status_on_tentative_tracks(self, o_arr):
        for track in self.tentative_tracks:
            
            track.n, track.m = self.update_confirmation_count(track, o_arr)

            track.prediction_step()
            track.correction_step(o_arr)

            if track.n == self.N:
                print("track confirmed")
                self.main_track_status = TRACK_STATUS.confirmed
                self.tentative_tracks.remove(track)
            elif track.m == self.M:
                print("track deleted")
                self.tentative_tracks.remove(track)

    def update_confirmation_count(self, pdaf: PDAF, o_arr):
        m = pdaf.m + 1

        if self.n_observations_inside_max_size_gate(pdaf.state_pri[:2], o_arr) > 0: 
            n = pdaf.n +1 
        else: 
            n = pdaf.n

        return n, m

    def n_observations_inside_max_size_gate(self, curr_pos, o_arr):
        n = 0

        for o in o_arr:
            dist = np.sqrt((o[0]-curr_pos[0])**2 + (o[1]-curr_pos[1])**2)
            #print("dist:", dist, "range:", self.max_vel*self.time_step + self.sd )
            if dist < self.max_vel*self.time_step + self.sd:
                n += 1
                
        return n

class TRACK_STATUS(Enum):
    tentative_confirm = 1
    confirmed = 2
    tentative_delete = 3