import numpy as np
import matplotlib.pyplot as plt

class LOSGuidance:
    def __init__(self, p0: list[float], p1: list[float]):
        self.set_path(p0, p1)
        self.heading_ref = 50*np.pi/180 # magic init number!!!

    def set_path(self, p0: list[float], p1: list[float]):
        self.p0 = np.array(p0)
        self.p1 = np.array(p1)

    def calculate_R_Pi_p(self):
        self.Pi_p = np.arctan2(self.p1[1]-self.p0[1], self.p1[0]-self.p0[0])
        self.R_Pi_p = np.array(
            [[np.cos(self.Pi_p), -np.sin(self.Pi_p)],
            [np.sin(self.Pi_p), np.cos(self.Pi_p)]]
        )
    
    def calculate_LOS_x_ref(self, x: np.ndarray, look_ahead: float) -> np.ndarray:
        self.set_path(self.p0, self.p1)
        self.calculate_R_Pi_p()
        p_asv = np.array([x[0], x[1]])
        errors = self.R_Pi_p.T @ (p_asv - self.p0)
        along_track_error = errors[0]
        p_los_world = self.R_Pi_p @ np.array([along_track_error + look_ahead, 0]) + self.p0
        x_ref = np.array([p_los_world[0], p_los_world[1], self.heading_ref])
        
        return x_ref
