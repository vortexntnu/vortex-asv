#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import ast


class ParameterManagerNode(Node):
    
    def __init__(self):
        super().__init__('parameter_manager_node')

        self.get_logger().info("Parameter manager node is up and running")


        #Declaration of parameters
        self.declare_parameter('configuration_matrix',"")
        self.declare_parameter('configuration_matrix_rows', 0)
        self.declare_parameter('configuration_matrix_columns', 0)
        self.declare_parameter('asv_thruster_min_thrust', 0)
        self.declare_parameter('asv_thruster_min_thrust', 0)
   
        self.declare_parameter('asv_thruster_manager_output',"")
        self.declare_parameter('asv_thruster_manager_input',"")
        self.declare_parameter('asv_thruster_manager_wrench',"")

        self.configuration_matrix = self.get_parameter('configuration_matrix').value #Is a string because 2D Array is not supported as a ros2 param
        self.asv_thruster_manager_output = self.get_parameter('asv_thruster_manager_output').value
        self.asv_thruster_manager_input = self.get_parameter('asv_thruster_manager_input').value
        self.asv_thruster_manager_wrench = self.get_parameter('asv_thruster_manager_wrench').value

        # self.configuration_matrix = StringInto2DArray(self.configuration_matrix)


def StringInto2DArray(input_string):
    two_d_array = ast.literal_eval(input_string)
    if isinstance(two_d_array, list) and all(isinstance(row, list) for row in two_d_array):
        return two_d_array
    else:
        print("The input string is not a valid 2D array.")


def main():
    rclpy.init()
    parameter_manager_node = ParameterManagerNode()
    rclpy.spin(parameter_manager_node)
    
    #test = parameter_manager_node.configuration_matrix
    #print(test)
    parameter_manager_node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
