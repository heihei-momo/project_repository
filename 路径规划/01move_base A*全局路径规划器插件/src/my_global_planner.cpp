#include "my_global_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <ros/ros.h>
#include <vector>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include <algorithm> 
#include <nav_msgs/Path.h>  
//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(global_planner::GlobalPlanner,nav_core::BaseGlobalPlanner)
using namespace std;
//Default Constructor
namespace global_planner {
    GlobalPlanner::GlobalPlanner ()
        {}
    GlobalPlanner::GlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        initialize(name, costmap_ros);
    }

    void GlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        costmap_ros_ = costmap_ros;//保存原始代价地图的指针
        costmap_ = costmap_ros_->getCostmap();//获取Costmap2D对象，提取底层的Costmap2D对象
        originX = costmap_->getOriginX();
        originY = costmap_->getOriginY();//获取地图原点坐标
        width = costmap_->getSizeInCellsX();
        height = costmap_->getSizeInCellsY();//获取地图尺寸
        resolution = costmap_->getResolution();//获取每个栅格对应的物理尺寸（单位：米/栅格）
        for(int iy = 0; iy<height; iy++)
        {
            vector<int> temp_v;
            vector<int> temp_c;
            for(int ix = 0; ix<width; ix++)
            {
                int temp = static_cast<int>(costmap_->getCost(ix, iy));//提取当前珊格的带价值
                if(temp>=1 && temp<=254)//0-自由空间 1-252不同级别的障碍物代价 253膨胀后的障碍物内边界 254致命障碍物 255未知区域
                {
                    temp_v.push_back(1);//1代表障碍物
                    // temp_c.push_back(0);
                }
                else
                {
                    temp_v.push_back(0);
                    // temp_c.push_back(0);
                }
            }
            map_v.push_back(temp_v);//vector<vector<int>>，将当前这一行数组添加到此二维矩阵的末尾,二维数组
            // pathMap.push_back(temp_c);
        }
            ROS_INFO("map Ctrls over");

        plan_pub_ = nh.advertise<nav_msgs::Path>("/move_base/GlobalPlanner/plan", 1);
        
    }
    // void GlobalPlanner::map_fresh(costmap_2d::Costmap2D* costmap_map){
    //     originX = costmap_map->getOriginX();
    //     originY = costmap_map->getOriginY();//获取地图原点坐标
    //     width = costmap_map->getSizeInCellsX();//返回的是代价地图在 x 方向（通常是水平方向）的网格单元数量
    //     height = costmap_map->getSizeInCellsY();//获取地图尺寸
    //     resolution = costmap_map->getResolution();//获取每个栅格对应的物理尺寸（单位：米/栅格）
    //     map_v.clear();
    //     pathMap.clear();
    //     for(int iy = 0; iy<height; iy++)
    //     {
    //         // vector<int> temp_v;
    //         // vector<int> temp_c;
    //         vector<int> temp_v(width, 0);
    //         vector<int> temp_c(width, 0);
    //         for(int ix = 0; ix<width; ix++)
    //         {
    //             int temp = static_cast<int>(costmap_map->getCost(ix, iy));//提取当前珊格的代价值
    //             if(temp>=1 && temp<=254)//0-自由空间 1-252不同级别的障碍物代价 253膨胀后的障碍物内边界 254致命障碍物 255未知区域
    //             {
    //                 temp_v.push_back(1);//1代表障碍物
    //                 temp_c.push_back(0);
    //             }
    //             else
    //             {
    //                 temp_v.push_back(0);
    //                 temp_c.push_back(0);
    //             }
    //         }
    //         map_v.push_back(temp_v);//vector<vector<int>>，将当前这一行数组添加到此二维矩阵的末尾,二维数组
    //         pathMap.push_back(temp_c);
    //     }
    // }
    std::vector<std::vector<int>> dynamic_obstacle_map_;  // 存储动态障碍物
    std::vector<std::vector<int>> visited_map_;           // 存储访问状态
    void GlobalPlanner::map_fresh(costmap_2d::Costmap2D* costmap_map) {
        // 获取最新地图参数
        originX = costmap_map->getOriginX();
        originY = costmap_map->getOriginY();
        width = costmap_map->getSizeInCellsX();
        height = costmap_map->getSizeInCellsY();
        resolution = costmap_map->getResolution();
        
        // 调整地图大小（如果尺寸变化）
        if (dynamic_obstacle_map_.size() != height) {
            dynamic_obstacle_map_.resize(height);
        }
        if (visited_map_.size() != height) {
            visited_map_.resize(height);
        }
        
        // 更新动态障碍物地图和访问状态
        for (int y = 0; y < height; y++) {
            // 调整内部vector大小
            if (dynamic_obstacle_map_[y].size() != width) {
                dynamic_obstacle_map_[y].resize(width, 0);
            }
            if (visited_map_[y].size() != width) {
                visited_map_[y].resize(width, 0);
            }
            for (int x = 0; x < width; x++) {
                // 获取最新代价值
                unsigned char cost = costmap_map->getCost(x, y);
                
                // 存储动态障碍物状态 (1=障碍物, 0=自由空间)
                dynamic_obstacle_map_[y][x] = (cost >= 1 && cost <= 254) ? 1 : 0;
                
                // 重置访问状态
                visited_map_[y][x] = 0;
            }
        }
        ROS_WARN("Dynamic map refreshed. Size: %dx%d", width, height);
}

    #define ZXDJ 10
    #define XXDJ 14

    struct Point{
        int x,y;
        int f,g,h;//量化评估
    };
    struct treeNode{
        Point pos;
        vector<treeNode*> child;
        treeNode* pParent;
    };
    //创建树节点的函数
    treeNode* createTreeNode(int t_x,int t_y){
        treeNode* pNew=new treeNode;
        memset(pNew,0,sizeof(treeNode));//全部赋值为0 NULL
        pNew->pos.x=t_x;
        pNew->pos.y=t_y;
        return pNew;
    }
    enum dirent{p_up,p_down,p_left,p_right,p_lup,p_ldown,p_rup,p_rdown};

    int getH(Point Current_pos,Point end_pos){
        int level=abs(end_pos.x-Current_pos.x);
        int vertical=abs(end_pos.y-Current_pos.y);
        // ROS_INFO("曼哈顿距离代价：%d",ZXDJ*(level+vertical));
        return ZXDJ*(level+vertical);
    }

    void GlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
        
        nav_msgs::Path path;
        path.header.frame_id = plan[0].header.frame_id;  // 使用第一个点的坐标系
        path.header.stamp = ros::Time::now();
        path.poses = plan;
        plan_pub_.publish(path);
        }

    bool GlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan ){
    
        plan.clear();
        costmap_ros_->updateMap();
        costmap_2d::Costmap2D* costmap_map = costmap_ros_->getCostmap();
        map_fresh(costmap_map);
    
    
        plan.push_back(start);//在start和goal参数分别⽤于获得初始位置和⽬标位置，start⼀般为机器⼈⾃定位位置，goal为机器⼈收到的⽬标位置信息
        float start_P_x = start.pose.position.x;
        float start_P_y = start.pose.position.y;
        float end_P_x = goal.pose.position.x;
        float end_P_y = goal.pose.position.y;
        // 起点和终点
        int start_sg_x = (start_P_x- originX)/resolution;
        int start_sg_y=(start_P_y-originY)/resolution;
        int end_sg_x = (end_P_x- originX)/resolution;
        int end_sg_y=(end_P_y-originY)/resolution;
        

        Point startpos={start_sg_x,start_sg_y};
        Point endpos={end_sg_x,end_sg_y};

        //准备一棵树，起点为树的根节点
        treeNode* pRoot=createTreeNode(start_sg_x,start_sg_y);
        //标记走过的
        visited_map_[start_sg_y][start_sg_x]=1;//走过的是1
        //从起点开始
        treeNode* pCurrent=pRoot;//当前点
        treeNode* pChild=NULL;//当前点的儿子
        //准备个数组，储存周围八个点的信息
        vector<treeNode*> buff;
        //便于比较而创建
        vector<treeNode*>::iterator it;
        vector<treeNode*>::iterator itMin;//记录f最小

        bool isFindEnd=false;
        ROS_WARN("1");
        while(1)//循环寻路
        {
            //1.一个点找出周围能走的点
            for (int i=0; i<8; i++)
            {
                pChild=createTreeNode(pCurrent->pos.x,pCurrent->pos.y);
                switch (i)
                {
                case p_up:    pChild->pos.y--; pChild->pos.g += ZXDJ; break;  // 上：行减少
                case p_down:  pChild->pos.y++; pChild->pos.g += ZXDJ; break;  // 下：行增加
                case p_left:  pChild->pos.x--; pChild->pos.g += ZXDJ; break;  // 左：列减少
                case p_right: pChild->pos.x++; pChild->pos.g += ZXDJ; break;  // 右：列增加
                case p_lup:   pChild->pos.x--; pChild->pos.y--; pChild->pos.g += XXDJ; break;  // 左上
                case p_rup:   pChild->pos.x++; pChild->pos.y--; pChild->pos.g += XXDJ; break;  // 右上
                case p_ldown: pChild->pos.x--; pChild->pos.y++; pChild->pos.g += XXDJ; break;  // 左下
                case p_rdown: pChild->pos.x++; pChild->pos.y++; pChild->pos.g += XXDJ; break;  // 右下
                }
                //检查pChild是否能走
                // ROS_INFO("1.vi  %d 2.dy  %d",visited_map_[pChild->pos.y][pChild->pos.x],dynamic_obstacle_map_[pChild->pos.y][pChild->pos.x]);
                // if((map_v[pChild->pos.y][pChild->pos.x]==0)&&(dynamic_obstacle_map_[pChild->pos.y][pChild->pos.x]==0))//能走
                if(dynamic_obstacle_map_[pChild->pos.y][pChild->pos.x]==0&&visited_map_[pChild->pos.y][pChild->pos.x]==0)
                {
                    //2.入树 入数组
                    //计算h值
                    pChild->pos.h=getH(pChild->pos,endpos);
                    //计算f值       
                    pChild->pos.f=pChild->pos.g+pChild->pos.h;
                    //入树
                    pCurrent->child.push_back(pChild);//新节点成为老节点的孩子
                    pChild->pParent=pCurrent;//老节点成为新节点的爹
                    //入数组
                    buff.push_back(pChild);
                    ROS_INFO("%d,%d",pChild->pos.x,pChild->pos.y);
                    // buff[i]=pChild;

                }
                else//不能走
                {
                    delete pChild;
                }
            }//for循环后，八个点都拿出来了
            //3.从数组中挑出f最小的点

            // it=buff.begin();
            // itMin=buff.begin();
            // for (;it!=buff.end();it++)
            // {
            //     itMin=(((*itMin)->pos.f < (*it)->pos.f)? itMin:it);
            // }
            // pCurrent=*itMin;
            // visited_map_[pCurrent->pos.y][pCurrent->pos.x] = 1;
            // buff.erase(itMin);
            //删掉走过的点   被删元素及之后的迭代器失效
            // 替换原始迭代器代码
            size_t min_index = 0;
            treeNode* min_node = buff[0];
            int min_f = min_node->pos.f;

            for (size_t i = 1; i < buff.size(); i++) {
                if (buff[i]->pos.f < min_f) {
                    min_f = buff[i]->pos.f;
                    min_node = buff[i];
                    min_index = i;
                }
            }

            ROS_WARN("2.1 - Min node at (%d,%d) f=%d", 
                    min_node->pos.x, min_node->pos.y, min_f);

            pCurrent = min_node;
            visited_map_[pCurrent->pos.y][pCurrent->pos.x] = 1;

            buff.erase(buff.begin() + min_index);
//------------------------------------------------------------------------------
            if(endpos.x==pCurrent->pos.x && endpos.y==pCurrent->pos.y){
                isFindEnd=true;
                break;
            }
            if(buff.size()==0){
                break;
            }
            
            // ROS_WARN("2.2");
        }
        // ROS_WARN("3");
        std::vector<Point> pathPoints;
        if(isFindEnd==true){
            ROS_INFO("I Find End,path is");
            while(pCurrent){
                ROS_INFO("(%d,%d)",pCurrent->pos.x,pCurrent->pos.y);
                pathPoints.push_back({pCurrent->pos.x, pCurrent->pos.y});
                pCurrent=pCurrent->pParent;
            }
        }
        else{
            ROS_INFO("I Not Find End");
        }
        std::reverse(pathPoints.begin(),pathPoints.end());//反转存储的路径
        
        for(int p=1;p< pathPoints.size()-1;p++) { //跳过起点和终点（已添加）
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = start.header.frame_id; //使用相同的frame_id
            pose.header.stamp = ros::Time::now();
            
            // 转换为世界坐标
            pose.pose.position.x = pathPoints[p].x * resolution + originX;
            pose.pose.position.y = pathPoints[p].y * resolution + originY;
            pose.pose.position.z = 0.0;
            
            // 计算朝向（指向下一个点）
            if(p < pathPoints.size() - 1) {
                double dx = pathPoints[p+1].x - pathPoints[p].x;
                double dy = pathPoints[p+1].y - pathPoints[p].y;
                double yaw = atan2(dy, dx);
                pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
            } else {
                // 最后一个点使用目标点的朝向
                pose.pose.orientation = goal.pose.orientation;
            }
            plan.push_back(pose);
        }
        ROS_WARN("4");
        plan.push_back(goal);

        // 发布路径到 ROS 话题
        publishPlan(plan);
        ROS_INFO("publish success");

        // 步骤1：逐行释放内层vector
        for (auto& innerVec : dynamic_obstacle_map_) {
        // 使用swap技巧彻底释放内存
        vector<int>().swap(innerVec);
        }
        // 步骤2：释放外层vector
        vector<vector<int>>().swap(dynamic_obstacle_map_);
        for (auto& innerVe : visited_map_) {
        vector<int>().swap(innerVe);
        }
        vector<vector<int>>().swap(visited_map_);
       
        return true;
        
}
};
