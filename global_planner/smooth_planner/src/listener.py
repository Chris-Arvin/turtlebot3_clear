#! /usr/bin/python2.7
# -*- coding: utf-8 -*-
import rospy
import geometry_msgs
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf
from minimum_snap import *
from global_py_api.srv import py_flag,py_flagRequest,py_flagResponse

class preprocess:
    def __init__(self,gridmap,current,goal):
        self.gridmap=gridmap
        self.current=current
        self.goal=goal

        self.map_process()
        self.current_process()
        self.goal_process()

    #预处理地图
    def map_process(self):
        self.height=self.gridmap.info.height
        self.width=self.gridmap.info.width
        self.col_map=np.zeros((self.height,self.width))

        for x in range(self.width):
            for y in range(self.height):
                self.col_map[x][y]=self.gridmap.data[x+y*self.width]

        self.resolution=self.gridmap.info.resolution
        self.step_size = (self.height+self.width)/15
        self.end_lim = (self.height+self.width)/15
        self.is_collision = 90
        #地图坐标系左下角位置
        self.gridmap_position_x=self.gridmap.info.origin.position.x
        self.gridmap_position_y=self.gridmap.info.origin.position.y

    #预处理机器人当前位姿
    def current_process(self):
        current_x = self.current.pose.position.x
        current_y = self.current.pose.position.y
        self.current_x,self.current_y=self.world2map(current_x,current_y)
        _, _, self.current_yaw = tf.transformations.euler_from_quaternion(
            [self.current.pose.orientation.x, self.current.pose.orientation.y, self.current.pose.orientation.z,
             self.current.pose.orientation.w])

    #预处理机器人目标位姿
    def goal_process(self):
        goal_x = self.goal.pose.position.x
        goal_y = self.goal.pose.position.y
        self.goal_x,self.goal_y=self.world2map(goal_x,goal_y)
        _, _, self.goal_yaw = tf.transformations.euler_from_quaternion(
            [self.goal.pose.orientation.x, self.goal.pose.orientation.y, self.goal.pose.orientation.z,
             self.goal.pose.orientation.w])

    #world坐标系到map坐标系转换
    def world2map(self,w_x,w_y):
        m_x=0
        m_y=0
        if self.resolution!=0:
            m_x=(w_x-self.gridmap_position_x)/self.resolution
            m_y=(w_y-self.gridmap_position_y)/self.resolution
        return m_x,m_y

    #map坐标系到world坐标系转换
    def map2world(self,m_x,m_y):
        w_x=0
        w_y=0
        if self.resolution!=0:
            w_x=self.gridmap_position_x + m_x*self.resolution
            w_y=self.gridmap_position_y + m_y*self.resolution
        return w_x,w_y


#接收到可规划指令后的callback
#首先接受map，current_pose，goal_pose，之后初始化整个系统；
#然后调用规划函数；
#然后调用minimum-snap；
#最终用topic的形式与move_base通信。
def startCb(req):
    #receive request
    gridmap=req.map
    pose=req.start
    goal=req.goal

    #combined-RRT
    pre_obj=preprocess(gridmap,pose,goal)
    rrt_agent = rrt(pre_obj.col_map,pre_obj.is_collision,pre_obj.height,pre_obj.width,pre_obj.step_size,pre_obj.end_lim,pre_obj.current_x,pre_obj.current_y,pre_obj.goal_x,pre_obj.goal_y)
    list=rrt_agent.find_path()

    #minimum-snap
    Dis_num=10 #每个多项式离散的点的个数
    LEVEL=pre_obj.is_collision #障碍物分界线
    n = 5
    while True:
        k = len(list) - 1
        #TODO 添加yaw！！！！！！！！！！！！！！！！！！！！！！
        Fig_class=minimum_snap(list,pre_obj.col_map,Dis_num,LEVEL,k,n)
        is_success,list=Fig_class.figure_out()
        if is_success:
            break

    # print("拟合后的轨迹为：")
    # for i in range(len(Fig_class.x)):
        # print(Fig_class.x[i],Fig_class.y[i])
    path= trans_path(Fig_class.x,Fig_class.y,'map')
    return path


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
    rospy.init_node('global_py_listener',anonymous=True)
    rospy.Rate(50)  # 1秒50次
    res= rospy.Service('global_path_py', py_flag, startCb)
    while not rospy.is_shutdown():
        rospy.spin()
