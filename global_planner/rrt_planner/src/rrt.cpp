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


//define details of node
#include "rrt_planner/rrt.h"
node::node(float r = 0, float c = 0, node* f = NULL)
{
	this->row = r;
	this->col = c;
	this->father = f;
	this->distance = 0;
	node *fa = this->father;
	while (true)	//递归计算distance
	{
		if (fa == NULL)
			break;
		this->distance += sqrt(pow((r - fa->row), 2) + pow((c - fa->col), 2));
		r = fa->row;
		c = fa->col;
		fa = fa->father;
	}
}

//和初始化类似
void node::change(float r, float c, node *f = NULL)
{
	this->row = r;
	this->col = c;
	this->father = f;
	node *fa = this->father;
	while (true)
	{
		if (fa == NULL)
			break;
		this->distance += sqrt(pow((r - fa->row), 2) + pow((c - fa->col), 2));
		r = fa->row;
		c = fa->col;
		fa = fa->father;
	}
}

void node::rewire(node *t, float d)
{
	this->father = t;
	this->distance = d;
}

float node::get_row()
{
	return this->row;
}
float node::get_col()
{
	return this->col;
}
float node::get_distance()
{
	return this->distance;
}






//define details of rrt
rrt::rrt(int row_num, int col_num, float step_size, float rewire_size, float end_lim, float now_row, float now_col, float end_row, float end_col, costmap_2d::Costmap2D* map)
{
	//obstacle value
	this->LEVEL=252;
	this->row_num = row_num;
	this->col_num = col_num;
	this->collision_map = map;
	//parameters
	this->step_size = step_size;
	this->rewire_size = rewire_size;
	this->end_lim = end_lim;
	this->now_row = now_row;
	this->now_col = now_col;
	this->end_row = end_row;
	this->end_col = end_col;
	//start and end nodes
	this->start = node(this->now_row, this->now_col, NULL);
	this->end = node(this->end_row, this->end_col, NULL);
	//initialize start node & end node & map; they come from the sensor.
	this->least_long_path = INFINITY;
	this->path_length = INFINITY;
	this->n1 = NULL;
	this->n2 = NULL;

	this->tree1.reserve(1000);
	this->tree2.reserve(1000);
	this->tree1.push_back(this->start);
	this->tree2.push_back(this->end);
	//start time
	Start_Time = clock();
}





/**
* 输入：flag标志生长在哪颗树上（起点/终点），new_rr和new_cc是新node的行和列，随机生成的
*/
bool rrt::spring(int flag, float new_rr, float new_cc)
{
	if (flag == 2)
		this->tree1.swap(this->tree2);

	//new node
	float new_r = new_rr;
	float new_c = new_cc;
	// if ((int)this->collision_map->getCost((unsigned int)new_c, (unsigned int)(this->row_num - 1 - new_r))>this->LEVEL)
	if ((int)this->collision_map->getCost((unsigned int)new_c, (unsigned int)new_r)>this->LEVEL)
	{
		return false;
	}

	float min_dis = INFINITY;
	node *temp_node = &this->tree1[0];
	node *new_node = new node;
	//record the nodes whose distance is less than rewire_size with new node
	vector<int> tree1_rewire;
	vector<int> tree2_rewire;
	/*  the first tree  */

	for (int i = 0; i < this->tree1.size(); i++)
	{
		float dis_r = this->tree1[i].get_row() - new_r;
		float dis_c = this->tree1[i].get_col() - new_c;
		float dis_pow = pow(dis_r, 2) + pow(dis_c, 2);
		//find the nearest node
		if (dis_pow < min_dis && dis_pow>0)
		{
			temp_node = &this->tree1[i];
			min_dis = dis_pow;
		}
		//find the near node
		if (dis_pow <= pow(this->rewire_size, 2))
			tree1_rewire.push_back(i);
	}
	min_dis = sqrt(min_dis);
	if (min_dis < this->step_size)
		new_node->change((int)new_r, (int)new_c, temp_node);
	else
	{
		float add_row = (new_r - temp_node->get_row()) * this->step_size / min_dis + temp_node->get_row();
		float add_col = (new_c - temp_node->get_col())* this->step_size / min_dis + temp_node->get_col();
		new_node->change((int)add_row, (int)add_col, temp_node);
	}
	//check collision
	float r_delta = (new_node->get_row() - temp_node->get_row()) / this->step_size;
	float c_delta = (new_node->get_col() - temp_node->get_col()) / this->step_size;
	float r = temp_node->get_row();
	float c = temp_node->get_col();
	for (int i = 0; i <= this->step_size; i++)
	{
		// if ((int)this->collision_map->getCost((unsigned int)c, (unsigned int)(this->row_num - 1 - r))>this->LEVEL)
		if ((int)this->collision_map->getCost((unsigned int)c, (unsigned int)r)>this->LEVEL)
		{
			if (flag == 2)
				this->tree1.swap(this->tree2);
			delete new_node;
			return false;
		}
		r += r_delta;
		c += c_delta;
	}
	//append
	this->tree1.push_back(*new_node);
	//rrt*
	float dis_new = INFINITY;
	for (vector<int>::iterator it = tree1_rewire.begin(); it != tree1_rewire.end(); it++)
	{
		dis_new = new_node->get_distance() + sqrt(pow(new_node->get_row() - this->tree1[*it].get_row(), 2) + pow(new_node->get_col() - this->tree1[*it].get_col(), 2));
		if (dis_new < this->tree1[*it].get_distance())
			this->tree1[*it].rewire(&this->tree1[this->tree1.size() - 1], dis_new);
	}



	/*  the second tree  */
	//go forward to the new node
	min_dis = INFINITY;
	node *new_node2 = new node;
	for (int i = 0; i < this->tree2.size(); i++)
	{
		float dis_r = this->tree2[i].get_row() - new_r;
		float dis_c = this->tree2[i].get_col() - new_c;
		float dis_pow = pow(dis_r, 2) + pow(dis_c, 2);
		//find the nearest node
		if (dis_pow < min_dis && dis_pow>0)
		{
			temp_node = &this->tree2[i];
			min_dis = dis_pow;
		}
		//find the near node
		if (dis_pow <= pow(this->rewire_size, 2))
			tree2_rewire.push_back(i);
	}
	min_dis = sqrt(min_dis);
	if (min_dis < this->step_size)
		new_node2->change((int)new_r, (int)new_c, temp_node);
	else
	{
		float add_row = (new_r - temp_node->get_row()) * this->step_size / min_dis + temp_node->get_row();
		float add_col = (new_c - temp_node->get_col())* this->step_size / min_dis + temp_node->get_col();
		new_node2->change((int)add_row, (int)add_col, temp_node);
	}
	//check collision
	r_delta = (new_node2->get_row() - temp_node->get_row()) / this->step_size;
	c_delta = (new_node2->get_col() - temp_node->get_col()) / this->step_size;
	r = temp_node->get_row();
	c = temp_node->get_col();
	for (int i = 0; i <= this->step_size; i++)
	{
		// if ((int)this->collision_map->getCost((unsigned int)c, (unsigned int)(this->row_num - 1 - r))>this->LEVEL)
		if ((int)this->collision_map->getCost((unsigned int)c, (unsigned int)r)>this->LEVEL)
		{
			if (flag == 2)
				this->tree1.swap(this->tree2);
			delete new_node2;
			return false;
		}
		r += r_delta;
		c += c_delta;
	}
	//append
	this->tree2.push_back(*new_node2);
	//rrt*
	dis_new = INFINITY;
	for (vector<int>::iterator it = tree2_rewire.begin(); it != tree2_rewire.end(); it++)
	{
		dis_new = new_node2->get_distance() + sqrt(pow(new_node2->get_row() - this->tree2[*it].get_row(), 2) + pow(new_node2->get_col() - this->tree2[*it].get_col(), 2));
		if (dis_new < this->tree2[*it].get_distance())
			this->tree2[*it].rewire(&this->tree2[this->tree2.size() - 1], dis_new);
	}


	//go again and again
	int mm = 0;
	while (true)
	{
		node *new_node3 = new node;
		mm++;
		temp_node = &this->tree2[this->tree2.size() - 1];
		min_dis = sqrt(pow((temp_node->get_row() - new_node->get_row()), 2) + pow((temp_node->get_col() - new_node->get_col()), 2));
		if (min_dis < this->end_lim)
		{
			if (flag == 2)
				this->tree1.swap(this->tree2);
			return true;
		}
		else
		{
			//There is something wrong!!!!!!!
			float add_row = (new_node->get_row() - temp_node->get_row()) * this->step_size / min_dis + temp_node->get_row();
			float add_col = (new_node->get_col() - temp_node->get_col())* this->step_size / min_dis + temp_node->get_col();
			new_node3->change((int)add_row, (int)add_col, temp_node);
		}

		//check collision
		float r_delta = (new_node3->get_row() - temp_node->get_row()) / this->step_size;
		float c_delta = (new_node3->get_col() - temp_node->get_col()) / this->step_size;
		float r = temp_node->get_row();
		float c = temp_node->get_col();
		for (int i = 0; i <= this->step_size; i++)
		{
			// if ((int)this->collision_map->getCost((unsigned int)c, (unsigned int)(this->row_num - 1 - r))>this->LEVEL)
			if ((int)this->collision_map->getCost((unsigned int)c, (unsigned int)r)>this->LEVEL)
			{
				if (flag == 2)
					this->tree1.swap(this->tree2);
				return false;
			}
			r += r_delta;
			c += c_delta;
		}
		this->tree2.push_back(*new_node3);
		//rrt*
		float dis_new = INFINITY;
		for (int i = 0; i < this->tree2.size() - 1; i++)
		{
			dis_new = new_node3->get_distance() + sqrt(pow(new_node3->get_row() - this->tree2[i].get_row(), 2) + pow(new_node3->get_col() - this->tree2[i].get_col(), 2));
			if (dis_new < this->tree2[i].get_distance())
				this->tree2[i].rewire(&this->tree2[this->tree2.size() - 1], dis_new);
		}
	}
}


/**
* 核心函数：
	先生成点，在spring内部做一个可否有附近点的bool检测；
	如果true，调用end_limitation做检测；
	调用results进行连接；
	调用optim_path进行剪枝；
	调用add_mid进行离散化。
*/
void rrt::extend()
{
	int z = 0;
	//only the two trees are connected, successful.
	srand((unsigned int)time(NULL));
	float new_r = rand() % (this->row_num);
	float new_c = rand() % (this->col_num);
	bool is_success = true;
	is_success = spring(1, new_r, new_c);
	while (true)
	{
		z++;
		//很有意思的是，这个rand写在这里，然后给spring就没问题，如果实在spring内部生成的时候，他会在dt内生成同一个点
		new_r = rand() % (this->row_num);
		new_c = rand() % (this->col_num);
		if (z % 2 == 0)
			//if (this->tree1.size() < this->tree2.size())
			is_success = this->spring(1, new_r, new_c);
		else
			is_success = this->spring(2, new_r, new_c);
		//cout << is_success << endl;
		//this->pri();
		//we can link directly
		if (is_success)
		{
			bool is_final = this->end_limitation();
			if (!is_final)
				continue;
			vector<node> temp = this->optim_path(this->results());
			if (this->path_length<this->least_long_path)
			{
				this->least_path = this->add_mid(temp);
				this->least_long_path = this->path_length;
			}


			/*
			cout << "path:" << endl;
			for (vector<node>::iterator it = this->least_path.begin(); it != this->least_path.end(); it++)
			{
			cout << it->get_row() << "," << it->get_col() << endl;
			}
			*/
			


			break;
		}
		/*
		bool judge = 1;
		if (!is_success)
		judge = this->end_limitation();
		if (judge == 1)
		this->results();
		*/
	}
}


/**
* 本质上，遍历两棵树的所有节点，尝试进行连接，来找到使得总距离最短的两个节点，指针赋值给n1和n2
*/
bool rrt::end_limitation()
{
	float old_length = this->path_length;
	int i = -1;
	for (vector<node>::iterator it1 = this->tree1.begin(); it1 != this->tree1.end(); it1++)
	{
		i += 1;
		int j = -1;
		for (vector<node>::iterator it2 = this->tree2.begin(); it2 != this->tree2.end(); it2++)
		{
			j += 1;
			float dis = INFINITY;
			if (pow(it1->get_row() - it2->get_row(), 2) + pow(it1->get_col() - it2->get_col(), 2) <= pow(this->step_size, 2))
			{
				//check collision
				bool flag = 1;
				float r_delta = (it1->get_row() - it2->get_row()) / this->step_size;
				float c_delta = (it1->get_col() - it2->get_col()) / this->step_size;
				float r = it2->get_row();
				float c = it2->get_col();
				for (int n = 0; n <= this->step_size; n++)
				{
					// if ((int)this->collision_map->getCost((unsigned int)c, (unsigned int)(this->row_num - 1 - r))>this->LEVEL)
					if ((int)this->collision_map->getCost((unsigned int)c, (unsigned int)r)>this->LEVEL)
					{
						flag = 0;
						break;
					}
					r += r_delta;
					c += c_delta;
				}
				if (flag == 0)
					continue;
				
				//emmm。。有点问题，下面这两个while，其实好像可以直接用node的distance代替。。
				node *temp_node = &this->tree1[i];
				dis = 0;
				//针对tree1，递归求temp_node到根节点的距离
				while (true)
				{
					if (temp_node == &this->tree1[0])
						break;
					dis += sqrt(pow(temp_node->get_row() - temp_node->father->get_row(), 2) + pow(temp_node->get_col() - temp_node->father->get_col(), 2));
					temp_node = temp_node->father;
				}
				temp_node = &this->tree2[j];
				//针对tree2，递归求temp_node到根节点的距离
				while (true)
				{
					if (temp_node == &this->tree2[0])
						break;
					dis += sqrt(pow(temp_node->get_row() - temp_node->father->get_row(), 2) + pow(temp_node->get_col() - temp_node->father->get_col(), 2));
					temp_node = temp_node->father;
				}
			}
			//有一点问题。。没有考虑两个点间的距离。。
			if (dis < this->path_length)
			{
				this->path_length = dis;
				this->n1 = &this->tree1[i];
				this->n2 = &this->tree2[j];
			}

		}
	}
	if (old_length != this->path_length)
		return true;
	return false;
}

/**
* 离散化，添加中间点
*/
vector<node> rrt::add_mid(vector<node> path)
{
	int z = 0;
	while (true)
	{
		if (abs(path[z].get_col() - path[z + 1].get_col()) + abs(path[z].get_row() - path[z + 1].get_row()) > 4)
		{
			float r_mines = path[z + 1].get_row() - path[z].get_row();
			float c_mines = path[z + 1].get_col() - path[z].get_col();
			float r_delta = r_mines / sqrt(pow(r_mines, 2) + pow(c_mines, 2));
			float c_delta = c_mines / sqrt(pow(r_mines, 2) + pow(c_mines, 2));
			node temp_node(path[z].get_row() + r_delta, path[z].get_col() + c_delta);
			path.insert(path.begin() + z + 1, temp_node);
		}
		z++;
		if (z == path.size() - 1)
			break;
	}
	return path;
}

/*
* n1和n2为开始点，以起点为根的树通过insert添加，以终点为根的树通过push_back添加
* 建立两棵树联通后的总路径
*/
vector<node> rrt::results()
{
	node* temp1 = this->n1;
	node* temp2 = this->n2;
	vector<node> temp_path;
	while (true)
	{
		if (temp1 == NULL)
			break;
		temp_path.insert(temp_path.begin(), *temp1);
		temp1 = temp1->father;
	}
	while (true)
	{
		if (temp2 == NULL)
			break;
		temp_path.push_back(*temp2);
		temp2 = temp2->father;
	}
	return temp_path;
}

/**
* 路径优化，进行剪枝，跳过一些无意义的点
*/
vector<node> rrt::optim_path(vector<node> temp)
{
	if (temp.size()>3)
	{
		int t = 0;
		while (true)
		{
			bool flag = true;
			node temp1 = temp[t];
			node temp3 = temp[t + 2];
			//check collision
			float r_delta = (temp1.get_row() - temp3.get_row()) / this->step_size;
			float c_delta = (temp1.get_col() - temp3.get_col()) / this->step_size;
			float r = temp3.get_row();
			float c = temp3.get_col();
			for (int i = 0; i <= this->step_size; i++)
			{
				// if ((int)this->collision_map->getCost((unsigned int)c, (unsigned int)(this->row_num - 1 - r))>this->LEVEL)
				if ((int)this->collision_map->getCost((unsigned int)c, (unsigned int)r)>this->LEVEL)
				{
					flag = false;
					break;
				}
				r += r_delta;
				c += c_delta;
			}
			if (flag)
				temp.erase(temp.begin() + t + 1);
			else
				t += 1;
			if (t + 2 == temp.size())
				break;
		}
	}
	return temp;

}





