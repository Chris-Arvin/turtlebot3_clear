#! /usr/bin/python2.7
# -*- coding: utf-8 -*-

'''
* Author: Arvin
* E-mail: 1711506@mail.nankai.edu.cn
* Update: 2020.7.16
* 路径规划器，输出为直线型的路径点
'''

'''
rrt_pot_v1.py___update log
updated from rrt_connect(bidirectional):
1. add mk_range_dir(), this function is to make a directionary to help spring in the fixed arc
2. change spring and extend to fit the new function

rrt_pot_v2.py___update log:
updated from rrt_pot:
1. gather the function into a class

rrt_pot_v3.py___update log:
updated from rrt_pot_v2:
1. change the arc_directinoary to just check whether the new sample is in the arc

rrt_pot_v4--v6.py___update log:
updated from rrt_pot_v3:
1. change the check collision methods and extend method(from 2 to 1)；
2. add the process when spring a new node, update the nearest nodes in a circle

rrt_pot_v7.py___update log:
updated from rrt_pot_v6:
1. fully change the bidirectional RRT/RRT

rrt_pot_v8.py___update log:
updated from rrt_pot v7:
combine 2 springs to 1, choose the tree with less nodes, and link derictly the other tree

rrt_pot_v9_without middle lines.py___update log:
updated from rrt_pot_v8:
1. just remove lines drawn in the process
2. figure out the checking collisions problem
3. add an optimal function
4. instructions: 迭代了50次，时间大概在0.5s，输入起终点，输出为途径的点；接下来可以在采样方式、update、运动约束、轨迹优化上搞一搞

rrt_pot_v10___update log:
updated from rrt_pot_v9_without_middle_lins.py:
1. now we try to draw collisions in the map
2. by randomly selecting lots of collisions, we can say the algorithm is right and highly efficiency

w_rrt_pot_v1___update log:
updated from rrt_pot_v10:
1. change how to check collisions
2. add a time limitation to shut down new extend in advance
3. problems:狭小空间的计算时间明显太长；

w_rrt_pot_v2___update log:
updated from w_rrt_pot_v1:
1. add an dilated collision map

w_rrt_pot_v3___update log:
updated from w_rrt_pot_v2:
1. change choose parent method,from comparing the distance between two nodes to compare the distance from the node to the start
2. add rewire
3. now our problems:采样方式的改进，运动约束，轨迹优化，局部最优解的问题

rrt_line___update log:
updated from w_rrt_pot_v3_no_pic:
1. do not make the window exist internally
2. link with rrt_crooked,we can draw collisions and see the crooked picture
'''

#注意：用tkinter画图时，是【x，y】，而我们在生成node时用的是【row，col】；有一个转置的问题
#cv中用的是【row，col】

import numpy as np
import time
#由于ROS系统的影响，cv2安装时会有一些#的处理，导致python3.5找不到他，加入下面这两行就可以解决了
#很奇怪，如果我本次开机后，没有启动过ros相关的文件，就需要注释掉； 如果启动过，就不用注释
# import sys
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')



# each node has varieties:row,col,father
class node:
    def __init__(self, r=0, c=0, f=None):
        self.row = r
        self.col = c
        self.father = f
        self.distance=0
        father=self.father
        while True:
            if father == None:
                break
            self.distance+=np.sqrt((r-father.row)**2+(c-father.col)**2)
            r=father.row
            c=father.col
            father=father.father

class rrt:
    # initial the start, end, map
    def __init__(self,map,is_collision,height,width,step_size,end_lim,start_x,start_y,end_x,end_y):
        # initialize start time
        self.t_s = time.time()
        # initial map & window
        self.height = height
        self.width = width
        # initial extend limitation and ede limitation
        self.step_size = step_size
        self.end_lim = end_lim
        self.is_collision=is_collision

        #costmap的地图是左下角坐标系，所以我们这里的row和col也是从左下角开始
        self.start = node(start_y, start_x, None)
        self.end = node(end_y, end_x, None)
        self.col_map=map
        # for x in range(self.width):
        #     for y in range(self.height):
        #         if self.col_map[x][y]>90:
        #             print (x,",",y,": ",self.col_map[x][y])

        # node list
        self.list1 = []
        self.list2 = []
        self.list1.append(self.start)
        self.list2.append(self.end)
        self.less_long_path = np.inf
        self.last_path_length = np.inf
        self.path_all = []


    # figure out the nearest node
    def spring(self, flag, mk_dir_flag=1):
        new_r = int(self.height * np.random.rand())
        new_c = int(self.width * np.random.rand())
        bias = 2
        while mk_dir_flag:
            if np.sqrt((new_r - self.start.row) ** 2 + (new_c - self.start.col) ** 2) + np.sqrt(
                    (new_r - self.end.row) ** 2 + (new_c - self.end.col) ** 2) <= self.path_length + bias:
                break
            new_r = int(self.height * np.random.rand())
            new_c = int(self.width * np.random.rand())

        if flag == 2:
            self.list1,self.list2=self.list2,self.list1
        # "Near". find rule:only the distance
        min_node = 1000000
        temp_node = node()
        for i in range(len(self.list1)):
            temp = self.list1[i]
            dis_r = temp.row - new_r
            dis_c = temp.col - new_c
            distance = dis_r ** 2 + dis_c ** 2

            if np.sqrt(distance) < min_node and distance > 0:
                temp_node = temp
                min_node = distance

        # "Steer" and "Edge". link nodes
        distance = np.sqrt(min_node)

        if distance <= self.step_size:
            new_node = node(new_r, new_c, temp_node)

        else:
            add_row = (new_r - temp_node.row) * self.step_size / distance + temp_node.row
            add_col = (new_c - temp_node.col) * self.step_size / distance + temp_node.col
            new_node = node(add_row, add_col, temp_node)

        # rewire
        '''
        for temp in self.list1:
            distance=np.sqrt((new_node.col-temp.col)**2+(new_node.row-temp.row)**2)
            if distance<int(self.step_size):
                if temp == new_node.father or temp == self.start or temp == self.end:
                    continue
                if distance+new_node.distance < temp.distance:
                    temp.father = new_node
                    temp.distance=distance+new_node.distance
        '''


        # check collision the second time: whether the path is in the collision!
        col = np.linspace(temp_node.col, new_node.col, int(self.step_size ), endpoint=True)
        row = np.linspace(temp_node.row, new_node.row, int(self.step_size ), endpoint=True)
        for j in range(min(len(col), len(row))):
            if self.col_map[int(col[j])][int(row[j])]>self.is_collision:
                # print("1",self.col_map[int(col[j])][int(row[j])])
                if flag == 2:
                    self.list1, self.list2 = self.list2, self.list1
                return False


        # add the new node into node list
        self.list1.append(new_node)

        # the tree birthed from the end node;
        # 在第一颗树和新节点作用完成后，去考虑另一个树，从原来的树开始一直往new node连接，一直到撞到障碍物或者连接到new node（搜索结束）
        min_node = 1000000
        temp_node = node()
        for i in range(len(self.list2)):
            temp = self.list2[i]
            dis_r = temp.row - new_node.row
            dis_c = temp.col - new_node.col
            distance = dis_r ** 2 + dis_c ** 2

            if distance < min_node and distance > 0:
                temp_node = temp
                min_node = distance

        # "Steer" and "Edge". link nodes
        distance = np.sqrt(min_node)
        if distance <= self.step_size:
            new_node2 = node(new_node.row, new_node.col, temp_node)
        else:
            add_row = (new_node.row - temp_node.row) * self.step_size / distance + temp_node.row
            add_col = (new_node.col - temp_node.col) * self.step_size / distance + temp_node.col
            new_node2 = node(add_row, add_col, temp_node)

        # check collision: whether the path is in the collision!
        col = np.linspace(temp_node.col, new_node2.col, int(self.step_size ), endpoint=True)
        row = np.linspace(temp_node.row, new_node2.row, int(self.step_size ), endpoint=True)
        for j in range(min(len(col), len(row))):
            if self.col_map[int(col[j])][int(row[j])]>self.is_collision:
                # print("2",self.col_map[int(col[j])][int(row[j])])
                if flag == 2:
                    self.list1, self.list2 = self.list2, self.list1
                return False



        # add the new node into node list
        self.list2.append(new_node2)

        # 如果走一步就到了新node，就直接退出了
        if new_node2 == new_node:
            if flag == 2:
                self.list1, self.list2 = self.list2, self.list1
            return True
        else:
            while True:
                distance = np.sqrt((new_node2.col - new_node.col) ** 2 + (new_node2.row - new_node.row) ** 2)
                if distance <= self.step_size:
                    new_node3 = node(new_node.row, new_node.col, new_node2)
                else:
                    add_row = (new_node.row - new_node2.row) * self.step_size / distance + new_node2.row
                    add_col = (new_node.col - new_node2.col) * self.step_size / distance + new_node2.col
                    new_node3 = node(add_row, add_col, new_node2)

                # check collision the second time: whether the path is in the collision!
                col = np.linspace(new_node2.col, new_node3.col, int(self.step_size ), endpoint=True)
                row = np.linspace(new_node2.row, new_node3.row, int(self.step_size ), endpoint=True)
                for j in range(min(len(col), len(row))):
                    if self.col_map[int(col[j])][int(row[j])] > self.is_collision:
                        # print(3,self.col_map[int(col[j])][int(row[j])])
                        if flag == 2:
                            self.list1, self.list2 = self.list2, self.list1
                        return False


                # add the new node into node list
                self.list2.append(new_node3)
                # 结束标志，同上
                if new_node3.row == new_node.row and new_node3.col == new_node.col:
                    if flag == 2:
                        self.list1, self.list2 = self.list2, self.list1
                    return True
                # 更换new_node2，进行迭代
                new_node2 = new_node3



    # end requirement,返回的是能连接两个tree，且使得总长度最小的两个点
    def end_limitation(self):
        #t1,t2是两个可连接的节点
        t1 = None
        t2 = None
        path_all_length = np.inf
        #list1和list2是两个tree
        for temp1 in self.list1:
            for temp2 in self.list2:
                dis = np.inf
                if (temp1.row - temp2.row) ** 2 + (temp1.col - temp2.col) ** 2 <= self.step_size ** 2:
                    # calculate the length of all path
                    temp_node = temp1
                    dis = 0
                    while True:
                        if temp_node == self.start:
                            break
                        dis += np.sqrt(
                            (temp_node.row - temp_node.father.row) ** 2 + (temp_node.col - temp_node.father.col) ** 2)
                        temp_node = temp_node.father
                    temp_node = temp2
                    while True:
                        if temp_node == self.end:
                            break
                        dis += np.sqrt(
                            (temp_node.row - temp_node.father.row) ** 2 + (temp_node.col - temp_node.father.col) ** 2)
                        temp_node = temp_node.father
                    dis += np.sqrt((temp1.row - temp2.row) ** 2 + (temp1.col - temp2.col) ** 2)
                if dis < path_all_length:
                    t1 = temp1
                    t2 = temp2
        if t1 == None:
            return False
        return t1, t2

    # expend nodes, flag is to figure whether to limit the new springed node's position
    def extend(self, flag=0):
        #如果extend的时间较大，大概率是因为此路径无法再优化了（椭圆内障碍物太多），这时直接退出就可以了;
        #如果前后两次路径的差值小于1，则已收敛了
        self.is_success=True
        while True:
            now=time.time()
            if now-self.t_s>10:
                # print('no path')
                exit()
            if abs(self.last_path_length - self.less_long_path) < 1 and len(
                    self.path_all) > 1 and self.last_path_length != self.less_long_path or now-self.t_s>0.5 and len(self.path_all)>0:
                self.is_success = False
                return 0
            # if now-self.t_s>0.5 and len(self.path_all)>0:
            #     self.is_success=False
            #     return 0
            # consistently spring up new node until meet end requirement
            # spring the tree first which has less nodes
            # print ("path length: ",len(self.list1)+len(self.list2))
            if len(self.list1)<=len(self.list2):
                is_success=self.spring(1, flag)
            else:
                is_success=self.spring(2, flag)
            if is_success:
                temp = self.end_limitation()
                if temp != False:
                    self.path = self.results(temp)
                    break
        # self.canvas.create_line(temp[0].col, temp[0].row, temp[1].col, temp[1].row, fill='black')
        # self.canvas.update()
        # num = len(self.path) - 2
        # print('路径上包含了%d个节点' % num)

        self.path_length = 0
        # calculate 2a=path_length
        for i in range(len(self.path)):
            if i == len(self.path) - 1:
                break
            self.path_length += np.sqrt(
                (self.path[i].row - self.path[i + 1].row) ** 2 + (self.path[i].col - self.path[i + 1].col) ** 2)

        self.last_path_length=self.path_length
        # 如果新生成的路径长度小于原来的长度，则绘出
        if self.path_length <= self.less_long_path:
            self.less_long_path = self.path_length
            self.path_all.append(self.path)


    # draw arcs to find the better path
    def update_path(self):
        # node list
        self.list1 = []
        self.list2 = []
        self.list1.append(self.start)
        self.list2.append(self.end)
        self.extend(flag=1)

    #optimal path
    def optim_path(self,path):
        if len(path)>3:
            t=0
            while True:
                flag=True
                temp1=path[t]
                temp3=path[t+2]
                # check collision the second time: whether the path is in the collision!
                col = np.linspace(temp1.col, temp3.col, int(self.step_size), endpoint=True)
                row = np.linspace(temp1.row, temp3.row, int(self.step_size), endpoint=True)
                for j in range(min(len(col), len(row))):
                    if self.col_map[int(col[j])][int(row[j])] > self.is_collision:
                        flag=False
                if flag:
                    path.pop(t+1)
                else:
                    t+=1
                if temp3 == path[-1]:
                    break
        return path


    # when make it, go back to find the relavently low cost path
    def results(self, temp_all):
        # create the path list from start node to temp_all[0]
        temp = temp_all[0]
        res2 = []
        res2.append(temp)
        while temp != self.start:
            temp = temp.father
            res2.append(temp)
        # reverse the results
        res = []
        l = len(res2) - 1
        for i in range(len(res2)):
            count = l - i
            res.append(res2[count])

        # create the path list from temp_all[1] to end node
        temp = temp_all[1]
        res.append(temp)
        while temp != self.end:
            temp = temp.father
            res.append(temp)
        # return the full path
        res=self.optim_path(res)
        return res

    #consistently extend
    def find_path(self):
        self.t_ss = time.time()
        self.t_s=time.time()
        self.extend()
        #终止条件为迭代100次
        #提前结束条件为：有成功路径且搜索时间超过1s/某次搜索的时间过长/路径长度收敛
        for i in range(100):
            if time.time()-self.t_ss>1 and len(self.path_all)>0:
                break
            if self.is_success==False:
                break
            # time.sleep(1)
            self.t_s = time.time()
            # self.init_map()
            self.update_path()
            self.t_e=time.time()
            # print('第%d次迭代的路径长度为：'%(i+1), self.path_length,'时间为：',self.t_e-self.t_s)
        # self.init_map()
        self.path_end = self.path_all[-1]
        self.path_xy=[]
        for i in self.path_end:
            self.path_xy.append([i.col,i.row])
        # print('最优路径长度为：', self.less_long_path)
        t_ee = time.time()
        # print('总时间为:', t_ee - self.t_ss)

        # print('results: ',self.path_xy)
        return self.path_xy


