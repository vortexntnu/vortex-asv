#!/usr/bin/env python

import rospy
from std_msgs.msg import String
#from my_msgs.msg import MyData # custom message type for the combined data. Talk with hannah
from nav_msgs.msg import Odometry

class UpdateDataNode:
    def __init__(self):
        rospy.init_node('update_data_node')

        # #                                        Position Name
        # self.data.objects.CurrentRedBouy      = (0,0,'red')
        # self.data.objects.CurrentGreenBouy    = (0,0,'green')
        # self.data.objects.CurrentNorthMarker  = (0,0,'north')
        # self.data.objects.CurrentSouthMarker  = (0,0,'south')
        # self.data.objects.CurrentEastMarker   = (0,0,'east')
        # self.data.objects.CurrentWestMarker   = (0,0,'west')
        # #                              Distance Name
        # self.data.ClosestObject       = (0, '')
        # self.data.SecondClosestObject = (0, '')
        # #                                       
        # self.data.vessel = Odometry(Position...)
        
        # Initialize subscribers to topic1 and topic2
        self.sub1 = rospy.Subscriber('detected_objects', MyData, self.callback1) #detected_objects is not a real topic
        self.sub2 = rospy.Subscriber('vessel_position', String, self.callback2) #vessel_position is not a real topic
        
        # Initialize publisher to data topic
        self.pub = rospy.Publisher('updated_data_Nav_tasks_Njord', MyData, queue_size=1)

    def callback1(self, msg):
        # Extract data from topic1 and combine with topic2 data
        combined_data = MyData()
        combined_data.objects.CurrentRedBouy = msg.objects.CurrentRedBouy
        combined_data.objects.CurrentGreenBouy = msg.objects.CurrentGreenBouy
        combined_data.objects.CurrentNorthMarker = msg.objects.CurrentNorthMarker
        combined_data.objects.CurrentSouthMarker = msg.objects.CurrentSouthMarker
        combined_data.objects.CurrentEastMarker = msg.objects.CurrentEastMarker
        combined_data.objects.CurrentWestMarker = msg.objects.CurrentWestMarker
        
        if hasattr(self, 'vessel_position'):
            combined_data.vessel_position = self.vessel_position
        
        # Publish combined data on data topic
        self.pub.publish(combined_data)
    
    def callback2(self, msg):
        # Extract data from vessel_position and store for later use
        data_parts = msg.split(',')
        self.vessel_position = (float(data_parts[0]), float(data_parts[1]))
    
if __name__ == '__main__':
    try:
        UpdateDataNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

