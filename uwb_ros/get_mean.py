#!/usr/bin/env python  
from __future__ import print_function
from six.moves import input
import rospy
from geometry_msgs.msg import PoseStamped 
import numpy as np

import matplotlib.pyplot as plt

# [ 3.45387187  4.33125991]
# [ 3.58550932  4.76105366]
# [ 3.62404104  4.77026679]
# [3.625, 4.70]
# [ 3.89389802  4.69525672]



def get_tag2():
    loc_msg = rospy.wait_for_message("UWBTag2", PoseStamped)
    current_x = loc_msg.pose.position.x
    current_y = loc_msg.pose.position.y
    return current_x, current_y

if __name__=="__main__":
    rospy.init_node('get_mean', anonymous=True)
    pos_array = []
    pos_x = []
    pos_y = []
    SAMPLE_NUM = 500
    for i in range(SAMPLE_NUM):
        pos = get_tag2()
        pos_x.append(pos[0])
        pos_y.append(pos[1])
        pos_array.append(pos)
        print(i)
    pos_array = np.array(pos_array)
    sum = np.sum(pos_array, axis=0)
    
    print(sum/SAMPLE_NUM)

    plt.scatter(pos_x, pos_y, color="r")
    plt.scatter(origin[0], origin[1], color="g")
    plt.show()