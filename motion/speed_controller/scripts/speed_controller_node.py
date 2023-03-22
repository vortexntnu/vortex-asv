#!/usr/bin/python3

import rospy

from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from pid_controller import PIDRegulator
from speed_controller.cfg import speedControllerConfig

from dynamic_reconfigure.server import Server


class SpeedControllerPID:
    """
    Wrapper for the PID controller to make the speed_controller
    code cleaner.
    """

    def __init__(self):
        """
        Initialize the PID controller with fixed gains and saturation limit.
        """

        self.controller_u = PIDRegulator(25, 0.024, 3.5, 5.0)  # Args: p, i, d, sat

    def update_gains(self, u_p, u_i, u_d, u_sat):
        """
        Update the controller gains and saturation limit.
        Args:
                u_p	  proportional gain for u (speed)
                u_i	  integral gain for u
                u_d	  derivative gain for u
                u_sat	  saturation limit u
        """

        self.controller_u.p = u_p
        self.controller_u.i = u_i
        self.controller_u.d = u_d
        self.controller_u.sat = u_sat


    def speed_controller(self, u_d, u, t):
        """
        Calculate force to maintain fixed speed.
        Args:
                u_d	desired speed
                u     current speed
                t       time
        Returns:
                float:	A restoring force output by the controller.
        """

        # error ENU

        e_rot = u_d - u

        # regulate(err, t)
        tau_u = self.controller_u.regulate(e_rot, t)
        return tau_u


class SpeedControllerROS:
    """
    The ROS wrapper class for the SpeedControllerPID. 
    The speed_controller is made up
    of a PID controller.
    Nodes created:
            speed_controller
    Subscribes to:
            "/guidance_desired_speed"
    Publishes to:
            "/thrust/force_input"
            
    """

    def __init__(self):
        """
        Initialize the speed_controller node, subscribers, publishers and the
        objects for the PID controller.
        """

        rospy.init_node("speed_controller")

        self.u = 0

        # Create controller
        self.PID = SpeedControllerPID()

        # Subscribers
        self.guidance_sub = rospy.Subscriber(
            rospy.get_param("/guidance_interface/desired_speed"),
            Float64,
            self.speed_data_callback,
            queue_size = 1,
        )

        self.odometry_sub = rospy.Subscriber(
            "/pose_gt", # change to /odometry/filtered
            Odometry,
            self.odometry_callback,
            queue_size = 1,
        )

        # Publisher
        self.force_pub = rospy.Publisher(
            rospy.get_param("/asv/thruster_manager/force"), Wrench, queue_size=1
        )

        # Dynamic reconfigure
        self.config = {}
        self.srv_reconfigure = Server(speedControllerConfig, self.config_callback)


    def odometry_callback(self, odom_msg):
        self.u = odom_msg.twist.twist.linear.x
    

    def speed_data_callback(self, msg):
        """
        Handle guidance data
        msg: the guidance data message
        """

        force_msg = Wrench()

        # desired speed message
        force_msg.force.x = self.PID.speed_controller(0.5, self.u, rospy.Time.now().to_sec())
        self.force_pub.publish(force_msg)


    def config_callback(self, config, level):
        """Handle updated configuration values.

        Args:
                config	The dynamic reconfigure server's Config type variable
                level	Ununsed variable
        Returns:
                A Config type containing the updated config argument.
        """

        # Old parameters for u (speed)
        u_p_old = self.PID.controller_u.p
        u_i_old = self.PID.controller_u.i
        u_d_old = self.PID.controller_u.d
        u_sat_old = self.PID.controller_u.sat

        # Reconfigured PID parameters for u
        u_p = config["PID_u_p"]
        u_i = config["PID_u_i"]
        u_d = config["PID_u_d"]
        u_sat = config["PID_u_sat"]

        rospy.loginfo("speed_controller reconfigure: ")

        self.log_value_if_updated("u_p", u_p_old, u_p)
        self.log_value_if_updated("u_i", u_i_old, u_i)
        self.log_value_if_updated("u_d", u_d_old, u_d)
        self.log_value_if_updated("u_sat", u_sat_old, u_sat)

        # Update controller gains
        self.PID.update_gains(u_p, u_i, u_d, u_sat)

        # update config
        self.config = config

        return config
    
    def log_value_if_updated(self, name, old_value, new_value):
        """
        A helper function for the config_callback() method
        Args:
                name		The string name of the variable
                old_value	A real number
                new_value	A real number
        """

        if old_value != new_value:
            rospy.loginfo("\t {:}: {:.4f} -> {:.4f}".format(name, old_value, new_value))
            

if __name__ == "__main__":
    speed_controller = SpeedControllerROS()
    rospy.spin()
