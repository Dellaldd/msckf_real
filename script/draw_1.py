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
    save_name = "record_2_1.png"
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
    print(max(data[:,9]))
    print(min(data[:,9]))
    
    
    
    ax[0][0].plot(data[first_index:,0], data[first_index:,1], 'b-', label = "msckf")
    ax[0][1].plot(data[first_index:,0], data[first_index:,2], 'b-')
    ax[0][2].plot(data[first_index:,0], -data[first_index:,3], 'b-')

    ax[0][0].plot(data[first_index:,0], data[first_index:,7]-data[first_index,7], 'r-', label = "opti")
    ax[0][1].plot(data[first_index:,0], data[first_index:,8]-data[first_index,8], 'r-')
    ax[0][2].plot(data[first_index:,0], data[first_index:,9]-data[first_index,9], 'r-')
    
    ax[1][0].plot(data[first_index:,0], data[first_index:,6], 'b-')
    ax[1][1].plot(data[first_index:,0], -data[first_index:,5], 'b-')
    ax[1][2].plot(data[first_index:,0], -data[first_index:,4], 'b-')

    ax[1][0].plot(data[first_index:,0], data[first_index:,12]-data[first_index,12], 'r-')
    ax[1][1].plot(data[first_index:,0], data[first_index:,11]-data[first_index,11], 'r-')
    ax[1][2].plot(data[first_index:,0], data[first_index:,10]-data[first_index,10], 'r-')
    
    ax[0, 0].set_title("position x")
    ax[0, 1].set_title("position y")
    ax[0, 2].set_title("position z")
    ax[1, 0].set_title("roll")
    ax[1, 1].set_title("pitch")
    ax[1, 2].set_title("yaw")

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
