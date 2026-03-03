#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>//上面两个是核心库和必要位置信息

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>//上面两个是costmap的头文件，可以直接调用initialize()函数中的地图信息进行路径规划

#include <nav_core/base_global_planner.h>//插件必备的始祖
#include <vector>

// #include <geometry_msgs/PoseStamped.h>
namespace global_planner{

    class GlobalPlanner : public nav_core::BaseGlobalPlanner{
        ////以公共成员类型继承nav_core::BaseGlobalPlanner
        public:
            GlobalPlanner();//用于无参构造，通常需后续显式调用 initialize() 完成初始化
            GlobalPlanner(std::string name,costmap_2d::Costmap2DROS* costmap_ros);//直接完成规划器的初始化，通常被ROS的插件系统调用。
            void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);//initialize函数可以调⽤地图信息，可以完成地图的初始化等⼯作，该处不能阻塞。
            bool makePlan(const geometry_msgs::PoseStamped& start,const geometry_msgs::PoseStamped& goal,std::vector<geometry_msgs::PoseStamped>& plan);
            //start	geometry_msgs::PoseStamped	起始位姿（带时间戳的坐标）
            // goal	geometry_msgs::PoseStamped	目标位姿
            // plan	std::vector<PoseStamped>	输出生成的路径点序列
            //定义⽗类中的虚函数，并重写⽗类中的虚函数，这步必不可少
            //最终计划将存储在⽅法的参数std::vector<geometry_msgs::PoseStamped>& plan
            void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
            void map_fresh(costmap_2d::Costmap2D* costmap_map);
    };
    float originX;
    float originY;
    float resolution;
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    int width;
    int height;
    std::vector<std::vector<int>> map_v;
    // std::vector<std::vector<int>> pathMap;
    ros::NodeHandle nh;
    ros::Publisher plan_pub_;
}