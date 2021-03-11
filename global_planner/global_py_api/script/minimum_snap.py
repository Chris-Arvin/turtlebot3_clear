# -*- coding: utf-8 -*-

'''
* Author: Arvin
* E-mail: 1711506@mail.nankai.edu.cn
* Update: 2020.7.16
* minimum-snap 轨迹拟合+优化器，最基础版，讲路径点转变为曲线轨迹并进行离散化处理
'''

import numpy as np
from cvxopt import matrix, solvers
from rrt_line import *

class minimum_snap:
    def __init__(self,list,map,Dis_num,LEVEL,k,n):
        self.T=1   #总时间
        self.t_to=[]    #到每个node的时间
        self.list=list  #size = len(node)*2
        self.list_x=[x[0] for x in self.list]
        self.list_y=[y[1] for y in self.list]
        self.Q_all=[]
        self.p_x=[]
        self.p_y=[]
        self.M=[]

        self.col_map=map
        self.Dis_num=Dis_num
        self.is_collision=LEVEL
        self.k=k
        self.n=n

        self.cal_time()
        self.def_Q_all()
        self.p_constrain()
        self.M_constrain()

        #这是最终离散化的结果
        self.x=[]
        self.y=[]

    #计算到每一个node的时间self.t_to
    def cal_time(self):
        t_every = []
        Distance_all = 0
        for i in range(len(self.list) - 1):
            dis = np.sqrt((self.list[i + 1][1] - self.list[i][1]) ** 2 + (self.list[i + 1][0] - self.list[i][0]) ** 2)
            Distance_all += dis
        for i in range(len(self.list) - 1):
            dis = np.sqrt((self.list[i + 1][1] - self.list[i][1]) ** 2 + (self.list[i + 1][0] - self.list[i][0]) ** 2)
            t_every.append(dis / Distance_all * self.T)

        self.t_to.append(0)
        for i in range(len(t_every)):
            self.t_to.append(self.t_to[i] + t_every[i]) #这里实际上有一种错位的思想

    #计算当前index下的q
    def Q_temp(self,index,length=6):
        q_temp = np.ones([length, length]) * 0.0
        for i in range(length):
            for j in range(length):
                if i >= 3 and j >= 3:
                    q_temp[i][j] = (i + 1) * (i) * (i - 1) * (i - 2) * (j + 1) * (j) * (j - 1) * (j - 2) / (
                                i + j - 5) * (self.t_to[index] ** (i + j - 5) - self.t_to[index - 1] ** (i + j - 5))
        return q_temp

    #计算全部的Q(目标函数中的Q)
    def def_Q_all(self):
        self.Q_all = np.zeros([self.k * (self.n + 1), self.k * (self.n + 1)])
        for i in range(self.k):
            q_temp = self.Q_temp(index=i + 1, length=6)
            num_q = i * (self.n + 1)
            for j in range(self.n + 1):
                for u in range(self.n + 1):
                    self.Q_all[num_q + j][num_q + u] = q_temp[j][u]    #对角线那种添加
        # print('Q_all is done:')
        # print(self.Q_all.shape)
        # print('-' * 60)

    #起点终点的x v a的限制和中间点的x限制,这个是等式后面的部分.
    #前3+k-1+3是对应的值，剩下的部分是0
    def p_constrain(self, vx0=0, ax0=0, vxk=0, axk=0, vy0=0, ay0=0, vyk=0, ayk=0):
        # define constrains: firstly equal constrains
        # p_x
        self.p_x = np.zeros([4 * self.k + 2, 1])
        # start node
        self.p_x[0][0] = self.list_x[0]
        self.p_x[1][0] = vx0
        self.p_x[2][0] = ax0
        # middle position node
        for i in range(1, self.k):
            self.p_x[i + 2][0] = self.list_x[i]
        # end node
        self.p_x[self.k + 2][0] = self.list_x[self.k]
        self.p_x[self.k + 3][0] = vxk
        self.p_x[self.k + 4][0] = axk


        # p_y
        self.p_y = np.zeros([4 * self.k + 2, 1])
        # start node
        self.p_y[0][0] = self.list_y[0]
        self.p_y[1][0] = vy0
        self.p_y[2][0] = ay0
        # middle position node
        for i in range(1, self.k):
            self.p_y[i + 2][0] = self.list_y[i]
        # end node
        self.p_y[self.k + 2][0] = self.list_y[self.k]
        self.p_y[self.k + 3][0] = vyk
        self.p_y[self.k + 4][0] = ayk

        # print('p is done')
        # print('k+5的值:', self.k + 5)
        # print('px的总长度:', len(self.p_x), 'py的总长度:', len(self.p_y), '4k+2的值:', 4 * self.k + 2)
        # print(self.p_x.shape)
        # print('-' * 60)

    #和p相对应，建立x和y的M约束。注意实际上x和y的M是一样的
    def M_constrain(self):
        self.M= np.zeros([4 * self.k + 2, (self.n + 1) * self.k])
        # M1
        M1 = np.zeros([3 + self.k - 1 + 3, (self.n + 1) * self.k])
        # start node: p0,v0,a0
        for i in range(self.n + 1):
            M1[0][i] = self.t_to[0] ** i
            if i == self.n:
                continue
            M1[1][i + 1] = (i + 1) * self.t_to[0] ** i
            if i == self.n - 1:
                continue
            M1[2][i + 2] = (i + 2) * (i + 1) * self.t_to[0] ** i

        # middle node: posiition
        for j in range(3, self.k + 2):
            for i in range(self.n + 1):
                M1[j][i + (j - 2) * (self.n + 1)] = self.t_to[j - 2] ** i

        # end node: pk,vk,ak
        for i in range(self.n + 1):
            M1[self.k + 2][i + (self.k - 1) * (self.n + 1)] = self.t_to[-1] ** i
            if i == self.n:
                continue
            M1[self.k + 3][i + 1 + (self.k - 1) * (self.n + 1)] = (i + 1) * self.t_to[-1] ** i
            if i == self.n - 1:
                continue
            M1[self.k + 4][i + 2 + (self.k - 1) * (self.n + 1)] = (i + 2) * (i + 1) * self.t_to[-1] ** i

        # print('M1 is done')
        # print(M1.shape)
        # print('-' * 60)

        # define constrains: secondly equal constrains
        M2 = np.ones([3 * self.k - 3, (self.n + 1) * self.k]) * 0.0
        j = 0
        l = 0
        while l < 3 * (self.k - 1):
            for i in range(self.n + 1):
                M2[l][j * (self.n + 1) + i] = self.t_to[j + 1] ** i
                M2[l][j * (self.n + 1) + i + (self.n + 1)] = -self.t_to[j + 1] ** i
                if i == self.n:
                    continue
                M2[l + 1][j * (self.n + 1) + i + 1] = (i + 1) * self.t_to[j + 1] ** i
                M2[l + 1][j * (self.n + 1) + i + 1 + (self.n + 1)] = -(i + 1) * self.t_to[j + 1] ** i
                if i == self.n - 1:
                    continue
                M2[l + 2][j * (self.n + 1) + i + 2] = (i + 2) * (i + 1) * self.t_to[j + 1] ** i
                M2[l + 2][j * (self.n + 1) + i + 2 + (self.n + 1)] = - (i + 2) * (i + 1) * self.t_to[j + 1] ** i

            j += 1
            l += 3

        # print('M2 is done')
        # print(M2.shape)
        # print('-' * 60)

        # combine M1 and M2 to M
        # M=np.ones([4*k+2,(n+1)*k])*0.0
        for i in range(4 * self.k + 2):
            for j in range((self.n + 1) * self.k):
                if i < self.k + 5:
                    self.M[i][j] = M1[i][j]
                else:
                    self.M[i][j] = M2[i - self.k - 5][j]

        # print('M is done')
        # print(self.M.shape)
        # print('-' * 60)


    #计算结果
    def figure_out(self):
        q = np.ones([len(self.Q_all), 1])
        q = matrix(q)
        Q_all = matrix(self.Q_all)
        M = matrix(self.M)

        p_x = matrix(self.p_x)
        result_x = solvers.qp(P=Q_all, q=q, A=M, b=p_x)

        p_y = matrix(self.p_y)
        result_y = solvers.qp(P=Q_all, q=q, A=M, b=p_y)

        lama_x=result_x['x']
        lama_y=result_y['x']

        # 将每个片段离散成10个点，如果collision，则把两点的中点添加到list中
        for t_a in range(len(self.t_to) - 1):
            time_list = np.linspace(self.t_to[t_a], self.t_to[t_a + 1], self.Dis_num, endpoint=True)
            for j in time_list:
                m = np.zeros([len(lama_x), 1])
                m[0 + t_a * 6] = 1
                m[1 + t_a * 6] = j
                m[2 + t_a * 6] = j ** 2
                m[3 + t_a * 6] = j ** 3
                m[4 + t_a * 6] = j ** 4
                m[5 + t_a * 6] = j ** 5
                self.x.append(np.dot(np.transpose(m), lama_x)[0][0])
                self.y.append(np.dot(np.transpose(m), lama_y)[0][0])

                if self.col_map[int(np.dot(np.transpose(m), lama_x)[0][0])][int(np.dot(np.transpose(m), lama_y)[0][0])] > self.is_collision:
                    l=[]
                    for i in self.list:
                        l.append(i)
                    # l = list.copy()
                    #被插入的中间点
                    b1 = int((self.list[t_a][0] + self.list[t_a + 1][0]) / 2)
                    b2 = int((self.list[t_a][1] + self.list[t_a + 1][1]) / 2)
                    l.insert(t_a + 1, [b1, b2])
                    return False, l
        # print('-'*60)
        # print('x:')
        # print(self.x)
        # print('y:')
        # print(self.y)
        # print('-' * 60)
        return True,1



