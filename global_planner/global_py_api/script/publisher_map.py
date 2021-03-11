#! /usr/bin/python2.7
# -*- coding: utf-8 -*-
'''
* Author: Arvin
* E-mail: 1711506@mail.nankai.edu.cn
* Update: 2020.7.16
* 做一个server，接收"global_path_py"的client，返回规划好的路径
'''

'''
py_flag.src:
geometry_msgs/PoseStamped start		#起始点
geometry_msgs/PoseStamped goal		#目标点
nav_msgs/OccupancyGrid map      #地图
---
nav_msgs/Path path
'''
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseActionGoal
import numpy as np




#接收到可规划指令后的callback
#首先接受map，current_pose，goal_pose，之后初始化整个系统；
#然后调用规划函数；
#然后调用minimum-snap；
#最终用topic的形式与move_base通信。


def mapCb(msg):
    """
    map callback
    """

    global map_old
    map_old = msg
    map_old.header.stamp = rospy.Time.now()

    width = msg.info.width
    height = msg.info.height
    temp_map = list(msg.data)
 
    for i in range(len(temp_map)):
        if temp_map[i]<=80 and temp_map[i]>=0:
            temp_map[i]=0
    
    temp_map=tuple(temp_map)
    map_old.data = temp_map
    # print map_old




if __name__ == '__main__':
    start_point = []
    target_point = []
    global_map = []
    original_x = -1.0
    original_y = -1.0
    resolute = 0.0500000007451
    rospy.init_node('state_receiver',anonymous=True)
    rate = rospy.Rate(50)  # 1秒50次
    # sub_map = rospy.Subscriber('/move_base/global_costmap/costmap',OccupancyGrid, mapCb)
    pub_map = rospy.Publisher('map_pytorch',OccupancyGrid,queue_size=10)
    map_old = None

    while not rospy.is_shutdown():
        data = rospy.wait_for_message('/move_base/global_costmap/costmap',OccupancyGrid)
        mapCb(data)
        pub_map.publish(map_old)
        rate.sleep()
        # if start_point!=[] and target_point!=[] and global_map!=[]:
        # print('global map',global_map)
        # print('start_point',start_point)
        # print('target_point',target_point)
        # print('-'*30)
