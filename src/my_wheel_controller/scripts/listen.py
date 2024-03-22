#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import message_filters

def callback(data1, data2):
    rospy.loginfo( "I heard %s %s", data1.data,data2.data)

def listener():
    rospy.init_node('listener', anonymous=True)
    t1= message_filters.Subscriber("chatter1", String)
    t2 =message_filters.Subscriber("chatter2", String)
    ts = message_filters.ApproximateTimeSynchronizer([t1, t2], 10, 1, allow_headerless=True)
    ts.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
