#pragma once
/*********************************************************************
* Authors: Arvin
* E-mail: 1711506@mail.nankai.edu.cn
* Update date:2020.7.15
* 封装了rrt集成优化算法，在rrt的基础上增加了：rrt*, informed-rrt, skip-rrt, connect-rrt/Bi-rrt
*********************************************************************/


/*********************************************************************
coding notes:
* costmap->getCost获得的是char型的！要比较大小，转化成int呀！
* todo 这种基于采样的路径规划方式，好像不能在这里用。因为是实时规划的，所有机器人的当前姿态会一直变化。。卧底娘哟。。后续需要在move_base种用informed或者后滞规划
* LEVEL要写253,在这里卡了很久
* 直接用它的costmap采样就行，不一定非要换成左上角坐标系，这里之前一直没有想明白
*********************************************************************/
#include<iostream>
#include<math.h>
#include<ctime>
#include<time.h>
#include<vector>

using namespace std;

/**
* 创建节点
*/
class node
{
private:
	float row, col;
	float distance;
public:
	node(float r, float c, node* f);
	//~node();
	float get_row();
	float get_col();
	float get_distance();
	//用于改变节点信息
	void change(float, float, node*);
	//rrt*, 用于改变父亲节点和distance
	void rewire(node*, float);
	node* father;
};


//实现rrt算法
class rrt
{
private:
	costmap_2d::Costmap2D* collision_map;
	double least_long_path;
	double path_length;
	vector<node> tree1;		//begin from the start node
	vector<node> tree2;		//begin from the end node
	node start;
	node end;
	clock_t Start_Time, End_Time;	//count processing time
	node *n1, *n2;					//record end_limitation nodes
	int row_num;
	int col_num;
	float step_size;	//树的生长距离
	float rewire_size;	//rrt*的更新距离
	float end_lim;		//两棵树可连接的最大距离
	float now_row;
	float now_col;
	float end_row;
	float end_col;
	float LEVEL;		//障碍物与free的分界线
public:
	rrt(int, int, float, float, float, float, float, float, float, costmap_2d::Costmap2D*);
	//~rrt();
	bool spring(int, float, float);		//生成新点
	bool end_limitation();		//检测两棵树可否连接
	void extend();			//核心入口，调用spring、end_limitation、optim_path等
	vector<node> add_mid(vector<node>);		//添加中间点（离散化处理）
	vector<node> optim_path(vector<node>);		//跳点处理路径点
	vector<node> results();			//连接两个树
	vector<node> least_path;		//最短路径
};
