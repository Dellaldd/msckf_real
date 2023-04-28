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


class Logger:
    def __init__(self):
        self.f = open("/home/ldd/msckf_vio/src/msckf_vio/path/record.txt", 'w')
        self.msckf_pose = [str(0),str(0),str(0),str(0),str(0),str(0)]
        self.opti_pose = [str(0),str(0),str(0),str(0),str(0),str(0)]
        self.extri = [str(0),str(0),str(0),str(0),str(0),str(0)]
        
        self.T_imu0_gt = np.identity(4)
        self.T_imu0_gt[1,1] = -1
        self.T_imu0_gt[2,2] = -1
        
        self.init_opti_pose = np.identity(4)
        self.is_first_pose = True
        
        self.start_time = 0 
        self.cur_time = 0
        rospy.Subscriber("/firefly_sbx/vio/odom", Odometry,self.msckf_Cb)
        rospy.Subscriber("/vrpn_client_node/liudandi/pose",PoseStamped,self.opti_Cb)

    def write_data(self):
        self.cur_time = time.perf_counter() 
        self.f.write(str(self.cur_time - self.start_time))
        self.f.write(',')
        self.f.write(','.join(self.msckf_pose))
        self.f.write(',')
        self.f.write(','.join(self.opti_pose))
        self.f.write('\r\n')
        
    def write_title(self):
        self.f.write("time, msckf_x, msckf_y, msckf_z, msckf_roll, msckf_pitch, msckf_yaw, opti_x, opti_y, opti_z, opti_roll, opti_pitch, opti_yaw")
        self.f.write('\r\n')

    def opti_Cb(self, msg):
        msg = PoseStamped()
        quaternion = [msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w]
        if self.is_first_pose:
            r = R.from_quat(quaternion)
            r_matrix = r.as_matrix()
            self.init_opti_pose[:3,:3] = r_matrix
            self.init_opti_pose[0,3] = msg.pose.position.x
            self.init_opti_pose[1,3] = msg.pose.position.y
            self.init_opti_pose[2,3] = msg.pose.position.z
            self.is_first_pose = False
        else:
            r = R.from_quat(quaternion)
            r_matrix = r.as_matrix()
            opti_pose = np.identity(4)
            opti_pose[:3,:3] = r_matrix
            opti_pose[0, 3] = msg.pose.position.x
            opti_pose[1, 3] = msg.pose.position.y
            opti_pose[2, 3] = msg.pose.position.z
            
            T_gt_gt0 = np.dot(np.linalg.inv(self.init_opti_pose), opti_pose)
            r_gt_gt0 = R.from_matrix(T_gt_gt0[:3,:3])
            euler = r_gt_gt0.as_euler('zyx', degrees=True)
            self.opti_pose = [str(T_gt_gt0[0,3]), str(T_gt_gt0[1,3]), str(T_gt_gt0[2,3]), str(euler[0]),str(euler[1]),str(euler[2])]
    
    def msckf_Cb(self,msg):
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        r = R.from_quat(quaternion)
        r_matrix = r.as_matrix()
        T_imu_imu0 = np.identity(4)
        T_imu_imu0[:3,:3] = r_matrix
        T_imu_imu0[0,3] = msg.pose.pose.position.x
        T_imu_imu0[1,3] = msg.pose.pose.position.y
        T_imu_imu0[2,3] = msg.pose.pose.position.z
       
        T_imu_gt = np.dot(self.T_imu0_gt,  T_imu_imu0)
        r_imu_gt = R.from_matrix(T_imu_gt[:3,:3])
        euler = r_imu_gt.as_euler('zyx', degrees=True)
        self.msckf_pose = [str(T_imu_gt[0,3]), str(T_imu_gt[1,3]), str(T_imu_gt[2,3]), str(euler[0]),str(euler[1]),str(euler[2])]
        

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
