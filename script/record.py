#!/usr/bin/env python
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import time
from scipy.spatial.transform import Rotation as R
import numpy as np
import threading

class Logger:
    def __init__(self):
        self.f = open("/home/ldd/msckf_vio/src/msckf_vio/path/record_2_2.txt", 'w')
        self.msckf_pose = [str(0),str(0),str(0),str(0),str(0),str(0), str(0)]
        self.opti_pose = [str(0),str(0),str(0),str(0),str(0),str(0), str(0)]
        
        self.start_time = 0 
        self.cur_time = 0
        rospy.Subscriber("/firefly_sbx/vio/odom", Odometry,self.msckf_Cb)
        rospy.Subscriber("/vrpn_client_node/liudandi/pose",PoseStamped,self.opti_Cb)
        
        self.add_thread = threading.Thread(target = self.thread_job)
        self.add_thread.start()

    def thread_job(self):
        rospy.spin()
        
    def write_data(self):
        self.cur_time = time.perf_counter() 
        self.f.write(str(self.cur_time - self.start_time))
        self.f.write(',')
        self.f.write(','.join(self.msckf_pose))
        self.f.write(',')
        self.f.write(','.join(self.opti_pose))
        self.f.write('\r\n')
        
    def write_title(self):
        self.f.write("time, msckf_x, msckf_y, msckf_z, msckf_q_x, msckf_q_y, msckf_q_z, msckf_q_w, opti_x, opti_y, opti_z, opti_q_x, opti_q_y, opti_q_z, opti_q_w")
        self.f.write('\r\n')

    def opti_Cb(self, msg):
        self.opti_pose = [str(msg.pose.position.x), str(msg.pose.position.y), str(msg.pose.position.z), str(msg.pose.orientation.x),str(msg.pose.orientation.y),str(msg.pose.orientation.z), str(msg.pose.orientation.w)]

    def msckf_Cb(self,msg):
        self.msckf_pose = [str(msg.pose.pose.position.x), str(msg.pose.pose.position.y), str(msg.pose.pose.position.z), str(msg.pose.pose.orientation.x),str(msg.pose.pose.orientation.y),str(msg.pose.pose.orientation.z), str(msg.pose.pose.orientation.w)]
        print(str(msg.pose.pose.orientation.x),str(msg.pose.pose.orientation.y),str(msg.pose.pose.orientation.z), str(msg.pose.pose.orientation.w))
def main():
    print("start record!")
    rospy.init_node('record_node', anonymous=True)
    logger = Logger()
    rate = rospy.Rate(200)
    logger.start_time = time.perf_counter() 
    logger.write_title()
    while not rospy.is_shutdown():
        logger.write_data()
        rate.sleep()
    logger.f.close()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass