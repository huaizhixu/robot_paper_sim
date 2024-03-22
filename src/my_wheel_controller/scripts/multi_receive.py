#!/usr/bin/env python
# 多传感器融合，将多个信号订阅到一个节点并发布
import rospy
import message_filters
from sensor_msgs.msg import Imu
from multi_msgs.msg import multi_info
from multi_msgs.msg import ak80_info
# from rosbot3_ctrl.msg import system_K
#from robot_control.msg import robot_orientation
#from robot_control.msg import multi_info
#from sensor_msgs.msg import JointState
#from robot_control.msg import cmd

class multi_receive_node:
    def __init__(self):
        rospy.init_node('multi_receive')
        # sys_k_sub = message_filters.Subscriber('/sys_K', system_K)
        imu_sub = message_filters.Subscriber('/imu', Imu)
        ak_info_sub = message_filters.Subscriber("/motor_ak80_info",ak80_info)
        self.pub = rospy.Publisher('/multi_info_motor_and_imu', multi_info, queue_size=10)
        # 时间序列处理
        self.ts = message_filters.ApproximateTimeSynchronizer([imu_sub, ak_info_sub], 10, 1,allow_headerless=True)

        self.ts.registerCallback(self.callback)
        rospy.spin()
    def callback(self, imu, ak80):
        print('ok')
        # x1=js.position[2]
        # x2=js.position[3]
        # x=0.5*(x1+x2)
        # dx1=0.05
        # dx2=0.05

        # info = multi_info()
        # info.x=1.6
        # info.dx=0.5*(dx1+dx2)
        # info.theta = 1
        # info.dtheta = 1
        # info.delta_yaw = 2
        # info.ddelta_yaw = 2

        # 时间戳
        # info.header.stamp = rospy.Time.now()
        print("start publish")
        # self.pub.publish(info)
        # print(log_msg)


if __name__ == "__main__":
    multi_receive = multi_receive_node()

# #!/usr/bin/env python
# import rospy
# from std_msgs.msg import String
# import message_filters
# from sensor_msgs.msg import Imu
# from multi_msgs.msg import multi_info
# from multi_msgs.msg import ak80_info

# def callback(data1, data2):
#     rospy.loginfo( "I heard ")

# def listener():
#     rospy.init_node('listener', anonymous=True)
#     t1= message_filters.Subscriber("imu", Imu)
#     t2 =message_filters.Subscriber("motor_ak80_info", ak80_info)
#     ts = message_filters.ApproximateTimeSynchronizer([t1, t2], 10, 1, allow_headerless=True)
#     ts.registerCallback(callback)
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()

# if __name__ == '__main__':
#     listener()