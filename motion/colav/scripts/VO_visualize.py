import numpy as np
from dataclasses import dataclass
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.patches as mpatches
import time as time
import VOMATH as VO
import math
@dataclass
class Vessel:
    x: float = 0.0    
    y: float = 0.0    
    vx: float = 0.0    
    vy: float = 0.0    
    radius: float = 1.0    
    color: str = "b"
    
class Intruder(Vessel):

    def __init__(self, x, y, vx, vy, radius, color, process_noise=0.1):
        super().__init__(x, y, vx, vy, radius, color)
        
        self.process_noise = process_noise        
        self.history = []
        self.last_update = None        
        self.history.append((self.x, self.y))

    def update(self):
        if self.last_update is None:
            self.last_update = time.monotonic()
            return 
        
        current_time = time.monotonic()
        elapsed_time = current_time - self.last_update        
        self.last_update = current_time        
        F = np.array([[1, 0, elapsed_time, 0],
                    [0, 1, 0, elapsed_time],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])
        
        Q = np.array([[0.25*elapsed_time**4, 0, 0.5*elapsed_time**3, 0],
                      [0, 0.25*elapsed_time**4, 0, 0.5*elapsed_time**3],
                      [0.5*elapsed_time**3, 0, elapsed_time**2, 0],
                      [0, 0.5*elapsed_time**3, 0, elapsed_time**2]]) * self.process_noise**2        
        
        w = np.random.multivariate_normal([0, 0, 0, 0], Q)
        X = np.array([[self.x], [self.y], [self.vx], [self.vy]])
        X = F @ X + w[:, np.newaxis]
        self.x = X[0, 0]
        self.y = X[1, 0]
        self.vx = X[2, 0]
        self.vy = X[3, 0]
        self.history.append((self.x, self.y))


class Ownship(Vessel):
    def __init__(self, waypoints, desired_velocity, x=0.0, y=0.0):
        super().__init__(x=x, y=y, color="r")
        
        self.waypoints = waypoints        
        self.history = []
        self.desired_velocity = desired_velocity        
        self.last_update = None        
        self.history.append((self.x, self.y))
        self.intruders = []
        self.stop_range = 0.5
        self.min_range = 3.0        
        self.collision_sector = np.pi / 2        
        self.other_ownships = []

    def colav_heading(self):
        targets = self.intruders + self.other_ownships 
        if not targets:
            return 0.0, False
            
        # Check if any target is inside the collision sector and closer than min_range / 4        
        for target in targets:
            
            dx = target.x - self.x            
            dy = target.y - self.y            
            dist = np.sqrt(dx**2 + dy**2)
            if dist < self.stop_range and dy>0:
                # Calculate distance and angle to closest intruder        
                return 0.0, True
        closest_intruder = min(targets,  key=lambda target: np.sqrt((target.x - self.x)**2 + (target.y - self.y)**2))
        dx = closest_intruder.x - self.x
        dy = closest_intruder.y - self.y
        dist = np.sqrt(dx**2 + dy**2)
        theta = np.arctan2(dy, dx)
        # Check if closest intruder is within safe zone
        if dist > self.min_range:
            return 0.0, False
        
        v_robot = [self.vx,self.vy]
      #  for target in targets:

        v_obstacle =[closest_intruder.vx,closest_intruder.vy]

        
            # Compute the relative velocity between the robot and the obstacle
        v_relative = [v_obstacle[0] - v_robot[0], v_obstacle[1] - v_robot[1]]

        # Compute the angle of the relative velocity vector
        theta_relative = math.atan2(v_relative[1], v_relative[0])

        # Define the maximum angular velocity of the robot
        max_angular_velocity = math.pi/4  # 45 degrees per second

        # Define the time horizon for collision avoidance
        time_horizon = 5  # seconds

        # Compute the left and right VO cone angles
        theta_l = theta_relative - max_angular_velocity * time_horizon
        theta_r = theta_relative + max_angular_velocity * time_horizon
        v_current = math.sqrt(self.vx**2+self.vy**2)
        v_pref = self.desired_velocity
        # Calculate the velocity obstacle
        t = math.tan(theta_l) + math.tan(theta_r)
        s = 2 * v_current / t
        v_hat = math.atan2(-math.sin(theta_l) - math.sin(theta_r), -math.cos(theta_l) - math.cos(theta_r))
        v_bar = v_hat + math.pi / 2

        # Calculate the direction of the new heading
        # if v_current < v_pref:
        #     # If the robot is slower than its preferred speed, move in the preferred direction
        #     new_heading = v_bar
        # else:
            # Otherwise, move in the direction of the velocity obstacle
        new_heading = math.atan2(s * math.sin(v_hat), v_current + s * math.cos(v_hat))
        print(new_heading)
        return -new_heading,False

        # Convert the heading to degrees for display purposes

        # Print the new heading
      ##  print("New heading: %.2f degrees" % new_heading_degrees)



        # # Calculate relative bearing        
        # ownship_bearing = np.arctan2(self.vy, self.vx)
        # relative_bearing = theta - ownship_bearing 
        # if relative_bearing < 0:
        #     relative_bearing += 2 * np.pi
        #     # Check if intruder is within the conic section in front of ownship
        # if abs(relative_bearing) <= self.collision_sector / 2:
        #     # # If intruder is directly in front of the ownship, stop to avoid collision            # if dist < self.min_range / 2:            #     return 0.0, True            
        #     # # Calculate the distance from the intruder to the line perpendicular to ownship's heading            
        #     distance_to_line = abs(dy * np.cos(ownship_bearing) - dx * np.sin(ownship_bearing))
        #     # Check if intruder is within the conic section 
        #     if distance_to_line < dist * np.sin(self.collision_sector / 2):
        #     # Adjust heading to steer away from the intruder                
        #         adjustment = np.pi /3             # If intruder is approaching from the right, go around the back of the intruder                
        #         if relative_bearing > np.pi:
        #             print("Intruder on the right")
        #             return adjustment, False
        #     # If intruder is approaching from the left, steer away from it                
        #         if relative_bearing < np.pi:
        #             print("Intruder on the left")
        #             return -adjustment, False
        # return 0.0, False 
    def update(self):
        if not self.waypoints:
            return 
        if self.last_update is None:
            self.last_update = time.monotonic()
            return 
        current_time = time.monotonic()
        dt = current_time - self.last_update        
        self.last_update = current_time        
        current_waypoint = self.waypoints[0]
        dx = current_waypoint[0] - self.x        
        dy = current_waypoint[1] - self.y        
        d = np.sqrt(dx**2 + dy**2)
        if d < self.radius:
            self.waypoints.pop(0)
            if not self.waypoints:
                return
            current_waypoint = self.waypoints[0]
            dx = current_waypoint[0] - self.x            
            dy = current_waypoint[1] - self.y        
        adjustment, do_stop = self.colav_heading()
        desired_velocity = 0
        if not do_stop:
            desired_velocity = self.desired_velocity        
        theta = np.arctan2(dy, dx) + adjustment        
        desired_vx = desired_velocity * np.cos(theta)
        desired_vy = desired_velocity * np.sin(theta)
        self.vx += (desired_vx - self.vx) * dt        
        self.vy += (desired_vy - self.vy) * dt        
        self.x += self.vx * dt        
        self.y += self.vy * dt        
        self.history.append((self.x, self.y))

    def update_intruders(self, intruders):
        self.intruders = intruders
class VesselVisualiser:

    def __init__(self, ownships, intruders):
        self.ownships = ownships 
        for i, ownship in enumerate(self.ownships):
            other_ownships = [o for j, o in enumerate(ownships) if i != j]
            ownship.other_ownships = other_ownships        
        self.intruders = intruders        
        self.fig, self.ax = plt.subplots()
        self.ax.set_facecolor("lightgray")
        self.ownship_lines = [self.ax.plot([], [], color=ownship.color)[0] for ownship in self.ownships]
        self.intruder_lines = [self.ax.plot([], [], color=v.color)[0] for v in self.intruders]
        self.intruder_patches = [self.ax.add_patch(mpatches.Circle(xy=(v.x, v.y), radius=self.ownships[0].min_range/10 , fill=False, linestyle='dashed', linewidth=0.5, color='black')) for v in self.intruders]
        self.ownship_patches = [self.ax.add_patch(mpatches.Circle(xy=(ownship.x, ownship.y), radius=ownship.min_range , fill=False, linestyle='dashed', linewidth=0.5, color='darkorange')) for ownship in self.ownships]
        self.ownship_stop_patches = [self.ax.add_patch(mpatches.Circle(xy=(ownship.x, ownship.y), radius=ownship.stop_range , fill=False, linestyle='dashed', linewidth=0.5, color='red')) for ownship in self.ownships]
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.grid(True)
        self.ax.set_aspect("equal")

    def animate(self, frame):
        for ownship in self.ownships:
            ownship.update_intruders(self.intruders)
            ownship.update()
        for intruder in self.intruders:
            intruder.update()
        for ownship, line, patch,stop_patch in zip(self.ownships, self.ownship_lines, self.ownship_patches,self.ownship_stop_patches):
            line.set_data(*zip(*ownship.history))
            patch.center = (ownship.x, ownship.y)
            stop_patch.center = (ownship.x, ownship.y)
        for intruder, line, patch in zip(self.intruders, self.intruder_lines, self.intruder_patches):
            line.set_data(*zip(*intruder.history))
            patch.center = (intruder.x, intruder.y)
        return self.ownship_lines + self.intruder_lines + self.ownship_patches +self.ownship_stop_patches + self.intruder_patches 
    def start(self):
        anim = FuncAnimation(self.fig, self.animate,frames=100, blit=True, interval=50)
        plt.show()


if __name__ == "__main__":
    # create vessels       # Square move    
    ownship = Ownship(waypoints=[(6, 0), (6, 6), (0, 6), (0, 0), (6, 0), (6, 6), (0, 6), (0, 0), (6, 0), (6, 6), (0, 6), (0, 0), (6, 0), (6, 6), (0, 6), (0, 0),], desired_velocity=1.0)    
    ownships = [
    Ownship(x=0.0, y=-9.0, waypoints=[(0, 9)], desired_velocity=1.0),

    #Ownship(x=0.0, y=9.0, waypoints=[(0, -9)], desired_velocity=1.0), # head on with compliant target    
    ]
    # Left approach    
    #intruders = [
       # Intruder(-6, 0, 0.7, 0.0, 1.0, 'g', process_noise=0.0), # Left to right        
        #Intruder(6, 0, -0.7, 0.0, 0.8, 'g', process_noise=0.0), # Right to left        
        #Intruder(-5, -5, 0.5, 0.5, 1.0, 'g', 0.0),
    
        #Intruder(0, 0, 0.1, 0.05, 1.0, 'b', 0.2),        
        #Intruder(0, 0, -0.1, 0.05, 1.0, 'b', 0.2),        
        #Intruder(0, 0, 0.1, -0.05, 1.0, 'b', 0.2),        
        #Intruder(0, 0, -0.1, -0.05, 1.0, 'b', 0.2),    
   # ]
    # Right approach    
    intruders = [    Intruder(0.1, 0, 0, 0, 1.0, 'g', process_noise=0.0),]    
    ownship = Ownship(x=0.0, y=-9.0, waypoints=[(0, 9)], desired_velocity=1.0)    
    
    # create visualizer    
    visualizer = VesselVisualiser(ownships, intruders)
    # animate vessel positions    
    visualizer.start()


