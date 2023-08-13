#!/usr/bin/env python3
import rospy
from vortex_msgs.msg import DetectedObjectArray, DetectedObject
import time
import numpy as np


class Buoy:

    def __init__(self, x, y, type_):
        """
        x: float
        y: float
        type_: str
        """
        self.x = x
        self.y = y
        self.type_ = type_
        self.last_observed = time.time()

    def assert_type(self, other):
        """
        other: Buoy
        """
        if self.type_ != other.type_:
            raise ValueError(
                f"Expected same buoy type, got: '{self.type_}' and '{other.type_}'"
            )

    # Overloads
    def __sub__(self, other):
        """
        other: Buoy
        """
        self.assert_type(other)
        return Buoy(self.x - other.x, self.y - other.y, self.type_)

    def __abs__(self):
        return np.sqrt(self.x**2 + self.y**2)


class UpdateBuoyMarkerNode:

    def __init__(self):
        rospy.init_node('UpdateBuoyMarkerNode', anonymous=True)

        self.subscribeTo = rospy.Subscriber('bouys/all_buoys_in_world_frame',
                                            DetectedObjectArray,
                                            self.new_buoy_marker_callback)
        self.pub = rospy.Publisher('bouys_and_markers',
                                   DetectedObjectArray,
                                   queue_size=10)

        self.buoys = []

        self.dist_treshold = 1.  # Threshold for updating buey position
        self.time_treshold = 3.  # Seconds that object is not seen

    def convert_topic_buoys(self, msg):
        """
        Convert topic 'bueys/all_buoys_in_world_frame' to Buoy-type, then return array of all buoys.
        """
        buoys = []
        for topic_element in msg.DetectedObjectArray:
            buoys.append(
                Buoy(topic_element.x, topic_element.y, topic_element.type))

        return buoys

    def publish_buoys(self):
        """
        Publish the internally stored buoys to topic as DetectedObjectArray type.
        """
        msg = DetectedObjectArray()

        for buoy in self.buoys:
            single_message = DetectedObject()
            single_message.x = buoy.x
            single_message.y = buoy.y
            single_message.type = buoy.type_
            msg.DetectedObjectArray.append(single_message)

        self.pub.publish(msg)

    def new_buoy_marker_callback(self, msg):
        """
        Callback function used to sort out buoys of interrest (?).
        msg: DetectedObjectArray
        """
        new_buoys = self.convert_topic_buoys(msg)
        for i, buoy in enumerate(self.buoys):
            # Delete buoys that have not been observed for a while
            if time.time() - buoy.last_observed >= self.time_treshold:
                self.buoys.pop(i)
                break

            # Update buoy positions
            for new_buoy in new_buoys:
                if abs(new_buoy - buoy) <= self.dist_treshold:
                    self.buoys[i] = new_buoy
                    new_buoys.remove(new_buoy)

                    break  # This is assuming there are no topic-buoys within
                    # threshold proximity to each other.
        # Add new bouys
        self.buoys += new_buoys
        self.publish_buoys()


if __name__ == "__main__":
    try:
        UpdateBuoyMarkerNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass