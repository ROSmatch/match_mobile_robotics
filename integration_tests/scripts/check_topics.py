#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


class TopicChecker:
    def __init__(self):
        self.topics = {'/mur620/UR10_r/global_tcp_pose','/mur620/UR10_l/global_tcp_pose','/mur620/UR10_l/wrench','/mur620/UR10_r/wrench', '/mur620/ground_truth','/mur620/mir_pose_simple','/mur620/mobile_base_controller/odom','/mur620/scan'}
        self.topic_data = {topic: False for topic in self.topics}
        self.subscribers = [rospy.Subscriber(topic, rospy.AnyMsg, self.callback, topic) for topic in self.topics]

    def callback(self, msg, topic):
        self.topic_data[topic] = True

    def check_topics(self):
        rospy.sleep(1)  # Give some time for topics to receive data
        for topic, has_data in self.topic_data.items():
            if not has_data:
                rospy.logerr(f"ERROR: No data received on topic: {topic}")
                return False
            else: 
                rospy.loginfo(f"Data received on topic: {topic}")
        rospy.loginfo("All topics have data.")
        return True

if __name__ == '__main__':
    rospy.init_node('check_topics')

    checker = TopicChecker()
    if checker.check_topics():
        rospy.loginfo("Test passed.")
    else:
        rospy.logerr("Test failed.")