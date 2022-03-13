#!/usr/bin/python3
# Written by Christopher Strom
# Copyright (c) 2020, Vortex NTNU.
# All rights reserved.

import rospy
import numpy as np

from geometry_msgs.msg import Wrench
from vortex_msgs.msg import GuidanceData

from pid.pid_controller import PIDRegulator
from backstepping.backstepping_controller import BacksteppingDesign, BacksteppingControl

from dynamic_reconfigure.server import Server
from los_controller.cfg import LOSControllerConfig

class LOSControllerPID:
	"""
	Wrapper for the PID controller to make the los_controller
	code cleaner.
	"""

	def __init__(self):
		"""
		Initialize the PID controller with fixed gains and saturation limit.
		"""

		self.controller_psi = PIDRegulator(25, 0.024, 3.5, 5.0)	# Args: p, i, d, sat
		self.controller_u = PIDRegulator(25, 0.024, 3.5, 5.0)	# Args: p, i, d, sat



	def updateGains(self, psi_p, psi_i, psi_d, psi_sat, u_p, u_i, u_d, u_sat):
		"""
		Update the controller gains and saturation limit.
		Args:
			psi_p	  proportional gain for psi (heading)
			psi_i	  integral gain for psi
			psi_d	  derivative gain for psi
			psi_sat	  saturation limit psi
			u_p	  proportional gain for u (velocity)
			u_i	  integral gain for u
			u_d	  derivative gain for u
			u_sat	  saturation limit for u
		"""

		self.controller_psi.p = psi_p
		self.controller_psi.i = psi_i
		self.controller_psi.d = psi_d
		self.controller_psi.sat = psi_sat

		self.controller_u.p = u_p
		self.controller_u.i = u_i
		self.controller_u.d = u_d
		self.controller_u.sat = u_sat


	def headingController(self, psi_d, psi, t):
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

	def speedController(self, u_d, u, t):
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
		e_speed = u_d - u

		# regulate(err, t)
		tau_u = self.controller_u.regulate(e_speed, t)
		return tau_u


class LOSControllerBackstepping:
	"""
	Wrapper for the backstepping controller.
	"""

	def __init__(self):
		"""
		Initialize the backstepping controller with fixed parameters.
		"""
												# 0.75, 30, 12, 2.5
		self.controller = BacksteppingControl(3.75, 45.0, 28.0, 10.5)

#k3 is probably not necessary. Could be removed, but need to find dependencies.
	def updateGains(self, c, k1, k2, k3):
		"""
		Update the backstepping controller gains.
        Args:
            c   a double containing heading gain
            k1  a double containing surge speed gain
            k2  a double containing sway speed gain
            k3  a double containing heave speed gain
		"""
		
		self.controller.c = c
		self.controller.K = np.array(( (k1, 0, 0),
                            		   (0, k2, 0), 
                            		   (0, 0, k3) )) 


	def regulate(self, u, u_dot, u_d, u_d_dot, v, psi, psi_d, r, r_d, r_d_dot):
		"""
		A wrapper for the controlLaw() method in the BacksteppingContoller
		class, to make the los_controller code cleaner.
		Args:
			u         current velocity in the body-fixed x-direction	
			u_dot     current acceleration in the body-fixed x-direction
			u_d       desired velocity in the body-fixed x-direction
			u_d_dot	  desired acceleration in the body-fixed x-direction
			v         current velocity in the body-fixed y-direction
			psi       current heading angle in the NED frame
			psi_d     desired heading angle in the NED frame
			r         current angular velocity around the body-fixed z-axis
			r_d       desired angular velocity around the body-fixed z-axis
			r_d_dot   desired angular acceleration around the body-fixed z-axis
		Returns:
			float[3]:	The control force vector tau
		"""
		
		tau = self.controller.controlLaw(u, u_dot, u_d, u_d_dot, v, psi, psi_d, r, r_d, r_d_dot)
		return tau



class LOSController:
	"""
	The ROS wrapper class for the LOSController. The los_controller is made up
	of a PID and a backstepping controller, and is mainly used in
	conjunction with the LOS guidance system.
	Nodes created:
		los_controller
	Subscribes to:
		/guidance/los_data
	Publishes to:
		/auv/thruster_manager/input
	"""

	def __init__(self):
		"""
		Initialize the los_controller node, subscribers, publishers and the
		objects for the PID and the backstepping controllers.
		"""

		rospy.init_node('los_controller')

		# Create controllers
		self.Backstepping = LOSControllerBackstepping()
		self.PID = LOSControllerPID()
	
		# Subscribers
		self.sub_guidance = rospy.Subscriber('/guidance/los_data', GuidanceData, self.guidance_data_callback, queue_size=1)

		# Publishers
		self.pub_thrust = rospy.Publisher(rospy.get_param("/asv/thruster_manager/input"), Wrench, queue_size=1)

		# Dynamic reconfigure 
		self.config = {}
		self.srv_reconfigure = Server(LOSControllerConfig, self.config_callback)

	def guidance_data_callback(self, msg):
		"""
		Handle guidance data whenever it
		u_p = config['PID_u_p']
			msg:	The guidance data message
		"""
		thrust_msg = Wrench()

		# Thrust message forces and torque
		thrust_msg.force.x = self.PID.speedController(0.5, msg.u, msg.t)
		thrust_msg.torque.z = self.PID.headingController(msg.psi_d, msg.psi, msg.t)

		# Publish the thrust message to /auv/thruster_manager/input
		self.pub_thrust.publish(thrust_msg)

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


	def config_callback(self, config, level):
		"""motion/los_controller/src/los_controller_node.pychange
		Handle updated configuration values.
		
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

		c_old = self.Backstepping.controller.c		
		K = self.Backstepping.controller.K

		# Old parameters for u (velocity)
		u_p_old = self.PID.controller_u.p
		u_i_old = self.PID.controller_u.i
		u_d_old = self.PID.controller_u.d
		u_sat_old = self.PID.controller_u.sat

		# Reconfigured PID parameters for psi
		psi_p = config['PID_psi_p']
		psi_i = config['PID_psi_i']
		psi_d = config['PID_psi_d']
		psi_sat = config['PID_psi_sat']

		# Reconfigured PID parameters for u
		u_p = config['PID_u_p']
		u_i = config['PID_u_i']
		u_d = config['PID_u_d']
		u_sat = config['PID_u_sat']

		# Reconfigured Backstepping parameters
		c = config['Backstepping_c']
		k1 = config['Backstepping_k1']
		k2 = config['Backstepping_k2']
		k3 = config['Backstepping_k3']

		rospy.loginfo("los_controller reconfigure: ")

		self.log_value_if_updated('psi_p', psi_p_old, psi_p)
		self.log_value_if_updated('psi_i', psi_i_old, psi_i)
		self.log_value_if_updated('psi_d', psi_d_old, psi_d)
		self.log_value_if_updated('psi_sat', psi_sat_old, psi_sat)

		self.log_value_if_updated('u_p', u_p_old, u_p)
		self.log_value_if_updated('u_i', u_i_old, u_i)
		self.log_value_if_updated('u_d', u_d_old, u_d)
		self.log_value_if_updated('u_sat', u_sat_old, u_sat)

		self.log_value_if_updated('c', c_old, c)
		self.log_value_if_updated('k1', K[0, 0], k1)
		self.log_value_if_updated('k2', K[1, 1], k2) 
		self.log_value_if_updated('k3', K[2, 2], k3) 
		
		# Update controller gains
		self.PID.updateGains(psi_p, psi_i, psi_d, psi_sat, u_p, u_i, u_d, u_sat)
		self.Backstepping.updateGains(c, k1, k2, k3)

		# update config
		self.config = config

		return config


if __name__ == '__main__':
	try:
		los_controller = LOSController()
		rospy.spin()
	except rospy.ROSInterruptException:
		pass