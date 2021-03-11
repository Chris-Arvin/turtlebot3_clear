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
import geometry_msgs
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf
from rrt_line import *
from minimum_snap import *
from global_py_api.srv import py_flag,py_flagRequest,py_flagResponse

class preprocess:
    def __init__(self,gridmap,current,goal):
        #初始化信息
        self.gridmap=gridmap
        self.current=current
        self.goal=goal
        #调用预处理
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
    x_original = req.map.info.origin.position.x
    y_original = req.map.info.origin.position.y
    resolute = req.map.info.resolution
    x_now = (req.start.pose.position.x-x_original)/resolute
    y_now = (req.start.pose.position.y)/resolute
    x=[21.0, 21.856028432272684, 22.832767670227856, 23.703090886912303, 24.60607982347195, 25.511033314854195, 26.416788660745574, 27.322693534851307, 28.229179587258223, 29.13835863144468, 30.052487011073232, 30.97223861621847, 31.891957522603057, 32.80440535236171, 33.70236638872604, 34.28951528427768, 34.865125705170236, 35.427188913432644, 35.973696171093806, 36.50376501273476, 37.021018063145064, 37.53020421966638, 38.03607237964039, 38.543371440408734, 39.055971557622115, 39.57422792016722, 40.097616975239674, 40.62561517003522, 41.15769895174948, 41.6933132919084, 42.23177725935886, 42.77237844727795, 43.31440444884283, 43.85714285723058, 44.4000000000888, 44.942857142947034, 45.48571428580525, 46.02857142866349, 46.571428571521714, 47.114285714379946, 47.65714285723818, 48.20000000009639, 48.74285714295463, 49.28571428581286, 49.828952381052034, 50.37447619057694, 50.924571428673275, 51.76328571438924, 52.624380952486206, 53.505952381059394, 54.400380952489755, 55.30000000011061, 56.200000000112404, 57.10000000011421, 58.000000000115996, 58.89550000011779, 59.764000000119516, 60.57850000012116, 61.31466666678929, 61.97916666679063, 62.59866666679187, 63.199666666793064, 63.77866666679422, 64.28566666679524, 64.666666666796, 64.88116666679645, 64.94266666679654, 64.87816666679643, 64.71244444457386, 64.44444444457334, 64.05644444457256, 63.5308333334604, 62.87333333345908, 62.382222222346975, 61.86444444456818, 61.333333333456004, 60.8000000001216, 60.266666666787195, 59.7333333334528, 59.2000000001184, 58.666666666783996, 58.136888889005164, 57.628444444559705, 57.162666666780986, 56.590666666779846, 56.227555555668005, 56.055555555667674, 56.00355555566757, 56.000000000112, 56.000000000112, 56.000000000112, 56.000000000112, 56.000000000112, 56.000000000111996, 56.000000000112, 56.000000000111996, 56.00000000011201, 56.000000000112, 56.000000000112, 56.000000000112, 56.000000000112, 56.000000000112, 55.99550000011199, 55.96400000011192, 55.87850000011176, 55.713333333444766, 55.45833333344425, 55.11333333344356, 54.678500000109366, 54.16400000010833, 53.595500000107194, 53.000000000106, 52.397000000104796, 51.77600000010355, 51.11900000010224, 50.4097777778786, 49.65277777787709, 48.86577777787551, 48.066666666762806, 47.266666666761196, 46.4666666667596, 45.666666666758005, 44.878666666756416, 44.16266666675499, 43.444444444531335]

    y=[244.0, 243.99999978774446, 243.99999425641272, 243.99998061423412, 243.9999544325891, 243.99991626029558, 243.99986953144324, 243.9996510732304, 243.98909841409758, 243.94255016941085, 243.8330117147019, 243.63477244896387, 243.32725786503494, 242.89117747921497, 242.31048030852156, 241.8521971535988, 241.35278485539905, 240.82596471935614, 240.2854580509037, 239.74269640817644, 239.19995236011255, 238.65720872835112, 238.1144483345313, 237.5716540002922, 237.02881369885685, 236.48593600978384, 235.94303466421562, 235.4001233932947, 234.85721592816353, 234.31432312687642, 233.77144435513515, 233.22857610555312, 232.68571487074408, 232.1428571433214, 231.60000000046318, 231.057142857605, 230.51428571474673, 229.97142857188848, 229.42857142903026, 228.88571428617206, 228.3428571433138, 227.80000000045558, 227.25714285759736, 226.71428571473916, 226.1750476195, 225.65752380997515, 225.18342857187892, 224.6012142861635, 224.2316190480675, 224.05654761949575, 224.00361904806707, 224.00016666711468, 224.01066666711472, 224.05716666711479, 224.166666667115, 224.3616666671154, 224.64666666711594, 225.02166666711673, 225.48533333378433, 226.0208333337854, 226.60133333378653, 227.20016666712107, 227.81066666712232, 228.4571666671236, 229.16666666712499, 229.9571666671266, 230.81066666712826, 231.70016666713008, 232.5995555560208, 233.49305555602257, 234.37155555602433, 235.22616666713714, 236.0546666671388, 236.59644444491758, 237.13288888936313, 237.66666666714198, 238.2000000004764, 238.7333333338108, 239.2666666671452, 239.80000000047963, 240.333333333814, 240.86711111159286, 241.4035555560384, 241.9453333338172, 242.77383333381889, 243.6284444449317, 244.50694444493348, 245.40044444493526, 246.30000000049262, 247.20000000049444, 248.1000000004962, 249.000000000498, 249.90000000049977, 250.80000000050157, 251.7000000005034, 252.6000000005052, 253.50000000050701, 254.4000000005088, 255.3000000005106, 256.2000000005124, 257.10000000051417, 258.000000000516, 258.9000000005178, 259.80000000051956, 260.7000000005214, 261.59866666718983, 262.47916666719163, 263.3146666671933, 264.07850000052815, 264.76400000052956, 265.3955000005308, 266.000000000532, 266.5910000005332, 267.1280000005342, 267.55700000053514, 267.829333333869, 267.9583333338693, 267.99733333386934, 268.000000000536, 268.000000000536, 268.00000000053603, 268.000000000536, 268.00000000053603, 268.000000000536, 268.000000000536]

#    index = 0 
#    for i in range(len(x)):
#        if (x[i]-x_now)**2+(y[i]-y_now)**2<0.5**2:
#            index = i
#            break
#    x_new = []
#    y_new = []
#    for i in range(index,len(x)): 
#        x_new.append(x[i])
#        y_new.append(y[i])
#    path= trans_path(x_new,y_new,'map')

    path= trans_path(x,y,'map')
    return path

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
    rospy.init_node('global_py_listener',anonymous=True)
    rospy.Rate(50)  # 1秒50次
    res= rospy.Service('global_path_py', py_flag, startCb)
    while not rospy.is_shutdown():
        rospy.spin()
