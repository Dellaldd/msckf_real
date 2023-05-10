import numpy as np
import rospy
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def main():
    
    # T_gt1_w = np.identity(4)
    # euler = [0, 0, 0]
    # r = R.from_euler('zyx',euler)
    # T_gt1_w[:3, :3] = r.as_matrix()
    # T_gt1_w[0,3] = 1
    # T_gt1_w[1,3] = 1
    # T_gt1_w[2,3] = 1
    # print(T_gt1_w)
    
    # T_gt2_w = np.identity(4)
    # T_gt2_w[0,3] = 3
    # T_gt2_w[1,3] = 3
    # T_gt2_w[2,3] = 3
    
    # T_gt2_gt1 = np.dot(np.linalg.inv(T_gt1_w), T_gt2_w)
    # print(T_gt2_gt1)
    
    
    file_name = "record_2.txt"
    save_name = "record_2.png"
    read_path = "/home/ldd/msckf_vio/src/msckf_vio/path/" + file_name
    data = np.loadtxt(read_path, delimiter=',', skiprows=1)
    fig, ax = plt.subplots(2, 3)
    first_index = 0
    for i in range(data.shape[0]):
        if data[i,1] and data[i,7]:
            first_index = i
            break
        else:
            continue
    
    T_gt0_w = np.identity(4)
    euler = [data[first_index, 10],data[first_index, 11], data[first_index, 12]] # zyx
    print(euler)
    r = R.from_euler('zyx',euler, degrees=True)
    
    T_gt0_w[:3, :3] = r.as_matrix()
    T_gt0_w[0,3] = data[first_index, 7]
    T_gt0_w[1,3] = data[first_index, 8]
    T_gt0_w[2,3] = data[first_index, 9]
    
    matrix = R.from_matrix(T_gt0_w[:3, :3])
    euler_gt = matrix.as_euler('zyx', degrees=True)
    print(euler_gt)
    
    T_gt_w = np.identity(4)
    T_imu_imu0 = np.identity(4)
    
    T_imu0_gt0 = np.identity(4)
    T_imu0_gt0[1,1] = -1
    T_imu0_gt0[2,2] = -1
    
    gt = []
    imu = []
    for i in range(first_index, data.shape[0]):
        euler = [data[i, 10],data[i, 11], data[i, 12]]
        r = R.from_euler('zyx',euler, degrees=True)
        T_gt_w[:3, :3] = r.as_matrix()
        T_gt_w[0,3] = data[i, 7]
        T_gt_w[1,3] = data[i, 8]
        T_gt_w[2,3] = data[i, 9]
        T_gt_gt0 = np.dot(np.linalg.inv(T_gt0_w), T_gt_w)
        matrix = R.from_matrix(T_gt_gt0[:3,:3])
        euler_gt = matrix.as_euler('zyx', degrees=True)
        gt.append(np.array([T_gt_gt0[0,3], T_gt_gt0[1,3], T_gt_gt0[2,3], euler_gt[0], euler_gt[1], euler_gt[2]]))
        
        euler = [data[i, 4],data[i, 5], data[i, 6]]
        r = R.from_euler('zyx',euler,degrees=True)
        T_imu_imu0[:3, :3] = r.as_matrix()
        T_imu_imu0[0,3] = data[i, 1]
        T_imu_imu0[1,3] = data[i, 2]
        T_imu_imu0[2,3] = data[i, 3]
        T_imu_gt0 = np.dot(T_imu0_gt0, T_imu_imu0)
        matrix = R.from_matrix(T_imu_gt0[:3,:3])
        euler_imu = matrix.as_euler('zyx', degrees=True)
        imu.append(np.array([T_imu_gt0[0,3], T_imu_gt0[1,3], T_imu_gt0[2,3], euler_imu[0], euler_imu[1], euler_imu[2]]))
        
    imu = np.array(imu)
    gt = np.array(gt)
    
    ax[0][0].plot(data[first_index:,0], imu[:,0], 'b-', label = "msckf")
    ax[0][1].plot(data[first_index:,0], -imu[:,1], 'b-')
    ax[0][2].plot(data[first_index:,0], imu[:,2], 'b-')

    ax[0][0].plot(data[first_index:,0], gt[:,0], 'r-', label = "opti")
    ax[0][1].plot(data[first_index:,0], gt[:,1], 'r-')
    ax[0][2].plot(data[first_index:,0], gt[:,2], 'r-')
    
    ax[1][0].plot(data[first_index:,0], imu[:,5], 'b-')
    ax[1][1].plot(data[first_index:,0], -imu[:,4], 'b-')
    ax[1][2].plot(data[first_index:,0], -imu[:,3], 'b-')

    ax[1][0].plot(data[first_index:,0], gt[:,5], 'r-')
    ax[1][1].plot(data[first_index:,0], gt[:,4], 'r-')
    ax[1][2].plot(data[first_index:,0], gt[:,3], 'r-')
    
    ax[0, 0].set_title("position x(m)")
    ax[0, 1].set_title("position y(m)")
    ax[0, 2].set_title("position z(m)")
    ax[1, 0].set_title("roll(deg)")
    ax[1, 1].set_title("pitch(deg)")
    ax[1, 2].set_title("yaw(deg)")

    fig.legend()
    fig.tight_layout()
    save_path = "/home/ldd/msckf_vio/src/msckf_vio/path/" + save_name
    plt.savefig(save_path, dpi=300)
    plt.show()
  
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
