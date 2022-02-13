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



	def updateGains(self, p, i, d, sat):
		"""
		Update the controller gains and saturation limit.

		Args:
			p	  proportional gain
			i	  integral gain
			d	  derivative gain
			sat	  saturation limit
		"""

		self.controller.p = p
		self.controller.i = i
		self.controller.d = d
		self.controller.sat = sat


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
		tau = self.controller_psi.regulate(e_rot, t)
		return tau

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
		tau = self.controller_u.regulate(e_speed, t)
		return tau


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
		Handle guidance data whenever it is published by calculating
		a control vector based on the given data.

		Args:
			msg:	The guidance data message
		"""

		# Control forces
		#tau_d = self.PID.headingController(msg.psi_d, msg.psi, msg.t)


		# add speed controllers here

		thrust_msg = Wrench()

		# Thrust message forces and torque
		thrust_msg.force.x = self.PID.speedController(msg.u_d, msg.u, msg.t)
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

		# Old parameters							
		p_old = self.PID.controller.p
		i_old = self.PID.controller.i
		d_old = self.PID.controller.d
		sat_old = self.PID.controller.sat

		c_old = self.Backstepping.controller.c		
		K = self.Backstepping.controller.K

		# Reconfigured PID parameters
		p = config['PID_p']
		i = config['PID_i']
		d = config['PID_d']
		sat = config['PID_sat']

		# Reconfigured Backstepping parameters
		c = config['Backstepping_c']
		k1 = config['Backstepping_k1']
		k2 = config['Backstepping_k2']
		k3 = config['Backstepping_k3']

		rospy.loginfo("los_controller reconfigure: ")

		self.log_value_if_updated('p', p_old, p)
		self.log_value_if_updated('i', i_old, i)
		self.log_value_if_updated('d', d_old, d)
		self.log_value_if_updated('sat', sat_old, sat)

		self.log_value_if_updated('c', c_old, c)
		self.log_value_if_updated('k1', K[0, 0], k1)
		self.log_value_if_updated('k2', K[1, 1], k2) 
		self.log_value_if_updated('k3', K[2, 2], k3) 
		
		# Update controller gains
		self.PID.updateGains(p, i, d, sat)
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
