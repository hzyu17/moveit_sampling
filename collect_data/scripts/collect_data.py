import rospy
from sensor_msgs.msg import JointState
import csv
import numpy as np
import pickle


new_trj = True
first_data = True

l_data = np.zeros(7)
total_data = l_data
n_data_point = 10000
i_traj = 0

def callback(data):
    global new_trj, first_data, l_data, total_data, i_traj

    if not new_trj:
        if (data.position[0] != l_data[0]):
            l_data = np.asarray(data.position[0:7])
            total_data = np.vstack([total_data, l_data])
            first_data = False
        else:
            if not first_data:
                file_name = 'trajectory_'+str(i_traj)+'.pickle'
                print("total trajectory")
                print(total_data)
                with open(file_name, 'wb') as handle:
                    pickle.dump(total_data, handle, protocol=pickle.HIGHEST_PROTOCOL)
                i_traj += 1
                new_trj = True
    else:
        l_data = np.asarray(data.position[0:7])
        total_data = l_data
        new_trj = False
        first_data = True

def listener():
    rospy.init_node('joint state listener')
    rospy.Subscriber("/joint_states", JointState, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
