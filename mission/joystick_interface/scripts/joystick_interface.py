from geometry_msgs.msg import Wrench

class JoystickInterface:
    def __init__(self):
        self.wrench_pub 
    
    def create_2d_wrench_message(self,x,y,yaw):
        wrench_msg = Wrench()
        wrench_msg.force.x = x
        wrench_msg.force.y = y
        wrench_msg.torque.z = yaw
        return wrench_msg
    
