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
def startCb(msg):
    start_x = msg.pose.pose.position.x
    start_y = msg.pose.pose.position.y
    global start_point
    start_point = [start_x,start_y]
    start_point = world2map(start_point[0],start_point[1])
    print('start point:',start_point)



def targetCb(msg):
    target_x = msg.pose.position.x
    target_y = msg.pose.position.y
    global target_point
    target_point = [target_x,target_y]
    target_point = world2map(target_point[0],target_point[1])
    print('target point:',target_point)

def world2map(w_x,w_y):
    global original_x,original_y,resolute
    m_x = (w_x - original_x)/resolute
    m_y = (w_y - original_y)/resolute
    return [m_x,m_y]


def mapCb(msg):
    """
    map callback
    """
    width = msg.info.width
    height = msg.info.height
    temp_map = msg.data
    # set original position of map in world frame.
    # global original_x,original_y, resolute
    # original_x = msg.info.origin.position.x
    # original_y = msg.info.origin.position.y
    # resolute = msg.info.resolution
    # print original_x
    # print original_y
    # print resolute
    # save map
    global global_map
    global_map = np.zeros((width,height))
    for x in range(width):
        for y in range(height):
            global_map[x][y]=temp_map[x+y*width]

    if global_map!=[]:
        with open("map.txt","a") as f:
            for x in range(width):
                for y in range(height):
                    f.write('{},'.format(global_map[x][y]))
                f.write('\n')
        f.close()
    print('map saved')
    print msg.info.origin.position.x
    print msg.info.origin.position.y
    print msg.info.resolution

#转换轨迹格式，把数组转换成path
def trans_path(x, y, map_name):
    gui_path = Path()
    gui_path.header.frame_id = map_name
    for i in range(len(x)):
        pose = PoseStamped()
        pose.header.frame_id = map_name
        pose.pose.position.x = x[i]
        pose.pose.position.y = y[i]
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        gui_path.poses.append(pose)
    return gui_path


if __name__ == '__main__':
    start_point = []
    target_point = []
    global_map = []
    original_x = -1.0
    original_y = -12.2
    resolute = 0.05
    rospy.init_node('state_receiver',anonymous=True)
    rospy.Rate(50)  # 1秒50次
    sub_map = rospy.Subscriber('/move_base/global_costmap/costmap',OccupancyGrid, mapCb)
    sub_start = rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped, startCb)
    sub_target = rospy.Subscriber('/move_base_simple/goal',PoseStamped, targetCb)
    #while not rospy.is_shutdown():
    rospy.spin()
        # if start_point!=[] and target_point!=[] and global_map!=[]:
        # print('global map',global_map)
        # print('start_point',start_point)
        # print('target_point',target_point)
        # print('-'*30)
