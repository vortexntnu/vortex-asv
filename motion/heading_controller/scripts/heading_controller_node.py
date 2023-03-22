#!/usr/bin/python3

import rospy

from geometry_msgs.msg import Wrench
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf

from pid_controller import PIDRegulator
from heading_controller.cfg import headingControllerConfig

from dynamic_reconfigure.server import Server



class HeadingControllerPID:
    """
    Wrapper for the PID controller to make the heading_controller
    code cleaner.
    """

    def __init__(self):
        """
        Initialize the PID controller with fixed gains and saturation limit.
        """

        self.controller_psi = PIDRegulator(25, 0.024, 3.5, 5.0)  # Args: p, i, d, sat

    def update_gains(self, psi_p, psi_i, psi_d, psi_sat):
        """
        Update the controller gains and saturation limit.
        Args:
                psi_p	  proportional gain for psi (heading)
                psi_i	  integral gain for psi
                psi_d	  derivative gain for psi
                psi_sat	  saturation limit psi
        """

        self.controller_psi.p = psi_p
        self.controller_psi.i = psi_i
        self.controller_psi.d = psi_d
        self.controller_psi.sat = psi_sat


    def heading_controller(self, psi_d, psi, t):
        """
        Calculate force to maintain fixed heading.
        Args:
                psi_d	desired heading
                psi     current heading
                t       time
        Returns:
                float:	A restoring force output by the controller.
        """

        # error ENU
        e_rot = psi_d - psi

        # regulate(err, t)
        tau_psi = self.controller_psi.regulate(e_rot, t)
        return tau_psi



class HeadingControllerROS:
    """
    The ROS wrapper class for the HeadingControllerPID. 
    The heading_controller is made up
    of a PID controller.
    Nodes created:
            heading_controller
    Subscribes to topic:
            /guidance/desired_heading
    Publishes to topic:
            "/thrust/torque_input"
    """

    def __init__(self):
        """
        Initialize the heading_controller node, subscribers, publishers and the
        objects for the PID controller.
        """

        rospy.init_node("heading_controller")

        self.psi = 0

        # Create controller
        self.PID = HeadingControllerPID()

        # Subscribers
        self.heading_sub = rospy.Subscriber(
            rospy.get_param("/guidance_interface/desired_heading"),
            Float64,
            self.heading_data_callback,
            queue_size = 1,
        )

        self.odometry_sub = rospy.Subscriber(
            "/pose_gt", # change to /odometry/filtered
            Odometry,
            self.odometry_callback,
            queue_size = 1,
        )

        # Publisher
        self.torque_pub = rospy.Publisher(
            rospy.get_param("/asv/thruster_manager/torque"), Wrench, queue_size=1
        )

        # Dynamic reconfigure
        self.config = {}
        self.srv_reconfigure = Server(headingControllerConfig, self.config_callback)


    def odometry_callback(self, odom_msg):
        q = odom_msg.pose.pose.orientation
        q_l = [q.x, q.y, q.z, q.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(q_l)
        self.psi = yaw

    def heading_data_callback(self, msg):
        """
        Handle guidance data
        msg: the guidance data message
        """

        torque_msg = Wrench()

        # Desired heading message
        torque_msg.torque.z = self.PID.heading_controller(msg.data, self.psi, rospy.Time.now().to_sec())
        self.torque_pub.publish(torque_msg)
        

    def config_callback(self, config, level):
        """Handle updated configuration values.

        Args:
                config	The dynamic reconfigure server's Config type variable
                level	Ununsed variable
        Returns:
                A Config type containing the updated config argument.
        """

        # Old parameters for psi (heading)
        psi_p_old = self.PID.controller_psi.p
        psi_i_old = self.PID.controller_psi.i
        psi_d_old = self.PID.controller_psi.d
        psi_sat_old = self.PID.controller_psi.sat

        # Reconfigured PID parameters for psi
        psi_p = config["PID_psi_p"]
        psi_i = config["PID_psi_i"]
        psi_d = config["PID_psi_d"]
        psi_sat = config["PID_psi_sat"]

        rospy.loginfo("heading_controller reconfigure: ")

        self.log_value_if_updated("psi_p", psi_p_old, psi_p)
        self.log_value_if_updated("psi_i", psi_i_old, psi_i)
        self.log_value_if_updated("psi_d", psi_d_old, psi_d)
        self.log_value_if_updated("psi_sat", psi_sat_old, psi_sat)

        # Update controller gains
        self.PID.update_gains(psi_p, psi_i, psi_d, psi_sat)

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
    heading_controller = HeadingControllerROS()
    rospy.spin()
        





