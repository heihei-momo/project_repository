#include "my_planner.h"
#include <pluginlib/class_list_macros.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

PLUGINLIB_EXPORT_CLASS(my_planner::MyPlanner,nav_core::BaseLocalPlanner)//把MyPlanner注册成一个插件
//pid控制系数
double Kp=4.5;//比例系数
double Ki=0.0;//积分系数
double Kd=0.5;//微分系数
//pid所需的变量
double angular_error=0;//当前误差
double last_error=0;//上一次误差
double error_sum=0;//误差累积
double error_diff=0;//误差变化率
double output=0;//pid输出值

namespace my_planner
{
    MyPlanner::MyPlanner()
    {
        setlocale(LC_ALL,"");
    }
    MyPlanner::~MyPlanner()
    {}

    tf::TransformListener* tf_listener_;
    costmap_2d::Costmap2DROS* costmap_ros_;
    void MyPlanner::initialize(std::string name,tf2_ros::Buffer* tf,costmap_2d::Costmap2DROS* costmap_ros)
    {
        tf_listener_=new tf::TransformListener();
        costmap_ros_=costmap_ros;
        ROS_WARN("局部规划器！");
    }

    std::vector<geometry_msgs::PoseStamped> global_plan_;
    int target_index_;
    bool pose_adjusting_;
    bool goal_reached_;
    bool MyPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
    {
        target_index_=0;//每次产生新的导航路线时，这个标记点被重置为0
        global_plan_=plan;
        pose_adjusting_=false;
        goal_reached_=false;
        return true;
    }
    bool MyPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)//全局路径规划的插件接受到的cotmap是全局代价地图，而局部路径规划接收到的代价地图是局部代价地图
    {
        //cv下展示局部代价地图
        costmap_2d::Costmap2D* costmap=costmap_ros_->getCostmap();//yaml文件的global是哪个坐标系就是基于哪个取出的地图数据，规划的全局路径也是按照该坐标系来的
        unsigned char* map_data=costmap->getCharMap();//获取代价地图的珊格值数组，返回的栅格数据是基于Costmap2DROS的global_frame坐标系存储
        unsigned int size_x=costmap->getSizeInCellsX();//获取珊格的列数--对应宽度
        unsigned int size_y=costmap->getSizeInCellsY();//获取珊格的行数--对应高度
        
        cv::Mat map_image(size_y,size_x,CV_8UC3,cv::Scalar(128,128,128));//统一的灰色
        for (unsigned int y=0;y<size_y;y++)
        {
            for (unsigned int x=0;x<size_x;x++)
            {
                int map_index=y*size_x+x;//计算一维数组中对应的下标值
                unsigned char cost=map_data[map_index];//读取一维数组中对应的下标的代价值
                cv::Vec3b& pixel=map_image.at<cv::Vec3b>(map_index);//获取图片中x，y坐标对应的像素地址，等价于map_image.at<cv::Vec3b>(y, x)
                if (cost==0)//可通行区域
                    pixel=cv::Vec3b(128,128,128);
                else if(cost==254)//障碍物
                    pixel=cv::Vec3b(0,0,0);
                else if(cost==253)//禁行区域
                    pixel=cv::Vec3b(255,255,0);
                else
                {
                    //根据灰度值显示红色到蓝色的渐变
                    unsigned char blue=255-cost;
                    unsigned char red=cost;
                    pixel=cv::Vec3b(blue,0,red);
                }
            }
        }

        //插入导航路线点的绘制代码
        for (int i=0;i<global_plan_.size();i++)
        {
            geometry_msgs::PoseStamped pose_odom;
            global_plan_[i].header.stamp=ros::Time(0);
            tf_listener_->transformPose("odom",global_plan_[i],pose_odom);//将路径点的地图坐标系转换为机器人坐标系
            double odom_x=pose_odom.pose.position.x;
            double odom_y=pose_odom.pose.position.y;//转换后的路径点坐标值，基准点为机器人的base_link,存在pose_base
            
            double origin_x=costmap->getOriginX();//因为local_costmap:global_frame: odom，所以取的是odom坐标系下的局部代价地图原点坐标值的点，原点看机器人xy坐标系的方向，和那个坐标系位姿保持一致
            double origin_y=costmap->getOriginY();
            double local_x=odom_x-origin_x;//局部代价地图下的 全局路径的坐标点（用局部代价地图坐标系）
            double local_y=odom_y-origin_y;//取出的坐标值单位还是m，没有变成珊格的形式

            int x=local_x/costmap->getResolution();
            int y=local_y/costmap->getResolution();
            cv::circle(map_image,cv::Point(x,y),0,cv::Scalar(255,0,255));//cv中显示导航路径坐标点，半径为0，就是画了一个点，紫色
            
            //检测前方路径点是否在禁行区域内
            if ((i>=target_index_)&&(i<target_index_+10))
            {
                cv::circle(map_image,cv::Point(x,y),0,cv::Scalar(0,255,255));
                int map_index=y*size_x+x;
                unsigned char cost=map_data[map_index];
                if (cost>=253)
                    // ros::Duration(1.0).sleep();
                    return false;//这边return false，就会再规划一条全局路径
                
            }
            
        }



        map_image.at<cv::Vec3b>(size_y/2,size_x/2)=cv::Vec3b(0,255,0);//标记机器人位置
        //翻转地图
        cv::Mat flipped_image(size_x,size_y,CV_8UC3,cv::Scalar(128,128,128));//把宽和高反了一下
        for (unsigned int y=0;y<size_y;y++)
        {
            for (unsigned int x=0;x<size_x;x++)
            {
                cv::Vec3b& pixel=map_image.at<cv::Vec3b>(y,x);
                flipped_image.at<cv::Vec3b>((size_x-1-x),(size_y-1-y))=pixel;
            }
        }
        map_image=flipped_image;

        cv::namedWindow("Map");
        cv::resize(map_image,map_image,cv::Size(size_y*5,size_x*5),0,0,cv::INTER_NEAREST);//图像map_image进行缩放
        cv::resizeWindow("Map",size_y*5,size_x*5);
        cv::imshow("Map",map_image);

        //-----------------------终点位姿处理----------------------
        int final_index=global_plan_.size()-1;//路径终点的标号
        geometry_msgs::PoseStamped pose_final;
        global_plan_[final_index].header.stamp=ros::Time(0);
        tf_listener_->transformPose("base_link",global_plan_[final_index],pose_final);//将终点姿态数据的坐标系转换到机器人坐标系下面来
        if(pose_adjusting_==false)//对终点姿态的调整
        {
            double dx=pose_final.pose.position.x;
            double dy=pose_final.pose.position.y;
            double dist=std::sqrt(dx*dx+dy*dy);
            if (dist<0.1)//距离小于0.5m
            {
                pose_adjusting_=true;//把机器人锁定在拟合姿态的分支
            }
        }
        if (pose_adjusting_==true)
        {
            double final_yaw=tf::getYaw(pose_final.pose.orientation);//得到当前姿态与目标位姿的偏差角
            ROS_WARN("开始调整最终姿态,final=%.2f",final_yaw);
            cmd_vel.linear.x=pose_final.pose.position.x*1.5;
            cmd_vel.angular.z=final_yaw*0.9;
            if (abs(final_yaw)<0.1)
            {
                goal_reached_=true;
                ROS_WARN("到达终点");
                cmd_vel.linear.x=0;
                cmd_vel.angular.z=0;
            }
            return true;//将小车的锁定在调整位姿状态
        }
        //结束对终点姿态的调整
        //-------------对全局路径的跟踪-----------------
        geometry_msgs::PoseStamped target_pose;
        for (int i=target_index_;i<global_plan_.size();i++)
        {
            geometry_msgs::PoseStamped pose_base;
            global_plan_[i].header.stamp=ros::Time(0);
            tf_listener_->transformPose("base_link",global_plan_[i],pose_base);//将路径点的地图坐标系转换为机器人坐标系
            double dx=pose_base.pose.position.x;
            double dy=pose_base.pose.position.y;//转换后的路径点坐标值，基准点为机器人的base_link,存在pose_base
            double dist=std::sqrt(dx*dx+dy*dy);//目标点和机器人之间的距离
            if (dist>0.2)
            {
                target_pose=pose_base;
                target_index_=i;
                ROS_WARN("选择第%d个路径点作为临时目标,距离为%0.2f",target_index_,dist);
                break;
            }

            if (i==global_plan_.size()-1)//用于处理最后一个点，可能只剩最后一个，还不满足追踪的距离条件
            {
                target_pose=pose_base;
            }
        }//结束遍历路径点
        //比例控制
        // cmd_vel.linear.x=target_pose.pose.position.x*1.5;//target_pose,机器人追踪的目标点,也是以机器人为坐标原点的
        cmd_vel.linear.x=target_pose.pose.position.x*1.5;
        angular_error=target_pose.pose.position.y;//目标点与机器人原点y方向的偏差，即左右偏差
        error_sum+=angular_error;//积分项
        error_diff=angular_error-last_error;//微分项
        output=Kp*angular_error+Ki*error_sum+Kd*error_diff;//pid输出
//Kp*angular_error  比例控制，根据横向偏差来改变转向速度    Ki*error_sum积分控制，将连续测量到的偏差值累加来改变转向  Kd*error_diff微分控制，将这一次的偏差量和上一次比较看看有没有变大，若变大就加大回调速度，避免超调振荡
                                //         x
                                //         |
                                //         |
                                //y<-------
        cmd_vel.angular.z=output;//输出速度

        last_error=angular_error;
        if (cmd_vel.linear.x>=0.2)
        {
            cmd_vel.linear.x=0.2;
        }
        else if (cmd_vel.linear.x<=-0.2)
        {
            cmd_vel.linear.x=-0.2;
        }
        
        if (cmd_vel.angular.z>=0.5)
        {
            cmd_vel.angular.z=0.5;
        }
        else if (cmd_vel.angular.z<=-0.5)
        {
            cmd_vel.angular.z=-0.5;
        }

        
        
        //--------------cv显示路线相对于机器人坐标系的情况----------------
        cv::Mat plan_image(600,600,CV_8UC3,cv::Scalar(0,0,0));//初始颜色为黑色
        for (int i=0;i<global_plan_.size();i++)
        {
            geometry_msgs::PoseStamped pose_base;
            global_plan_[i].header.stamp=ros::Time(0);
            tf_listener_->transformPose("base_link",global_plan_[i],pose_base);
            int cv_x=300-pose_base.pose.position.y*100;
            int cv_y=300-pose_base.pose.position.x*100;//计算cv坐标系下的坐标
            cv::circle(plan_image,cv::Point(cv_x,cv_y),1,cv::Scalar(255,0,255));//绘制路径点

        }
        cv::circle(plan_image,cv::Point(300,300),15,cv::Scalar(0,255,0));
        cv::line(plan_image,cv::Point(65,300),cv::Point(510,300),cv::Scalar(0,255,0),1);
        cv::line(plan_image,cv::Point(300,45),cv::Point(300,555),cv::Scalar(0,255,0),1);
        // cv::namedWindow("Plan");
        // cv::imshow("Plan",plan_image);
        cv::waitKey(1);
        
        
        return true;
    }
    bool MyPlanner::isGoalReached()
    {
        return goal_reached_;
    }
}