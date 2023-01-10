import numpy as np
from enum import Enum
from typing import List

from test_pdaf import PDAF

"""

Track manager:

    Define a different validation gate then what is used for confirmed tracks.
    How should velocity be initialized? 
    Write "add tentative tracks".
    Can we make do with fewer classes? 
    Multilpe fiels? 
    TEST
"""

class TRACK_MANAGER:
    def __init__(self):
        #subscripe to topic with detections from point cloud 
        #publish state of main track if status is confirmed

        self.prev_detections: List[np.ndarray]= [] 
        self.tentative_tracks: List[TRACK] = [] 

        self.main_track = TRACK()

    def cb(self, o_arr):
        if self.main_track.status == TRACK_STATUS.tentative_confirm:
            self.update_status_on_tentative_tracks(o_arr)
            self.add_tentative_tracks(o_arr)

        elif self.main_track.status == TRACK_STATUS.confirmed:
            self.main_track.pdaf.correction_step(o_arr)
            self.main_track.pdaf.prediction_step()

            if self.main_track.pdaf.o_within_gate_arr == 0:
                self.main_track.status = TRACK_STATUS.tentative_delete
                self.main_track.m = 0
                self.main_track.n = 0

        elif self.main_track.status == TRACK_STATUS.tentative_delete:
            self.main_track.pdaf.correction_step(o_arr)
            self.main_track.pdaf.prediction_step()  

            self.main_track.m += 1
            if self.main_track.pdaf.o_within_gate_arr == 0:
                self.main_track.n += 1

            if self.main_track.n == self.main_track.N:
                self.main_track.status = TRACK_STATUS.confirmed
            elif self.main_track.m == self.main_track.M:
                self.main_track.status = TRACK_STATUS.tentative_confirm        

    def add_tentative_tracks(self, o_arr):
        a = 2
        #for each prev detection:
        #   search for new detections within a validation gate based on maks dist to new predicted state pluss noise. 

    def update_status_on_tentative_tracks(self, o_arr):
        for track in self.tentative_tracks:
            track.update_confirmation_count(o_arr)
            if track.n == track.N:
                self.main_track.status = TRACK_STATUS.confirmed
                self.tentative_tracks.remove(track)
            elif track.m == track.M:
                self.tentative_tracks.remove(track)

class TRACK_STATUS(Enum):
    tentative_confirm = 1
    confirmed = 2
    tentative_delete = 3

class TRACK:
    def __init__(self, o):
        self.N = 3
        self.M = 5
        self.n = 0
        self.m = 0

        self.status = TRACK_STATUS.tentative

        self.pdaf = PDAF()
        self.pdaf.state_pri[0] = o[0] #x
        self.pdaf.state_pri[1] = o[1] #y
        self.pdaf.state_pri[2] = 0    #x'
        self.pdaf.state_pri[3] = 0    #y'

    def update_confirmation_count(self, o_arr):
        self.m += 1
        
        self.pdaf.prediction_step()

        self.pdaf.correction_step(o_arr)

        if self.pdaf.o_within_gate_arr > 0: #OBS: might not be good enough, considering boats with high speed. 
            self.n += 1
