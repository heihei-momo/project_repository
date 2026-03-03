#ifndef MY_PLANNER_H_
#define MY_PLANNER_H_

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>//始祖类

namespace my_planner
{
    class MyPlanner:public nav_core::BaseLocalPlanner
    {
        public:
            MyPlanner();//对外公开的接口函数---规划器诞生后会调用的函数
            ~MyPlanner();//析构函数--规划器被销毁后会调用的函数
            //局部规划器的固定接口
            void initialize(std::string name,tf2_ros::Buffer* tf,costmap_2d::Costmap2DROS* costmap_ros);
            bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);//接收来自全局规划器的全局导航路线函数，plan存储全局导航路线
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);//用于计算速度，对cmd_vel进行赋值，但不用发布cmd_vel，发送在move_base里面
            //computeVelocityCommands会被不停的调用
            bool isGoalReached();//用于向move_base提交结果

    };
}

#endif