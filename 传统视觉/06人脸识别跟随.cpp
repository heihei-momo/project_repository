#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <geometry_msgs/Twist.h>

pcl::PointXYZRGB pt;//pt.z是离目标点的z轴距离
cv::Point center_picture;//储存图片中心的坐标
cv::Point center_lian;//储存脸中心的坐标
bool flag=false;

geometry_msgs::Twist vel_cmd;//速度消息包
ros::Publisher vel_pub;//速度发布对象
class FaceDepthDetector {
public:
    //构造函数
    FaceDepthDetector() : it_(nh_) {//具体成员初始化
        // 加载人脸检测模型
        if (!face_cascade.load("/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt.xml")) {
            ROS_ERROR("Failed to load face cascade");
            ros::shutdown();
        }

        // 订阅RGB图像和点云
        //FaceDepthDetector::：限定作用域，表明是类的成员
        //imageCallback：目标成员函数名
        // &：取地址运算符（必须显式使用，不同于普通函数可省略）
        //成员函数指针必须绑定到特定类的实例才能调用（隐含this参数）
        cloud_sub_ = nh_.subscribe("/camera/depth/color/points", 1,&FaceDepthDetector::cloudCallback, this);
        image_sub_ = it_.subscribe("/camera/color/image_raw", 1,&FaceDepthDetector::imageCallback, this);//this代表参数回调给FaceDepthDetector()
        //特性	        |  it_.subscribe() (ImageTransport)	              |   nh_.subscribe() (常规NodeHandle)
        // 优化目标 	 |      图像数据（高频率、大体积）	                    |        通用数据（任意消息类型）
        // 典型数据类型	  |sensor_msgs/Image 或 sensor_msgs/CompressedImage	|  任意ROS消息（如 sensor_msgs/PointCloud2）
        // 底层协议 	 |支持多种图像传输协议（RAW/JPEG/PNG/Theora）	       |     仅支持原生ROS序列化
        
        ROS_INFO("Face Depth Detector initialized");
    }
    //cv_bridge::CvImagePtr cv_ptr;类型
    void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);//cv_bridge::toCvCopy() 是一个关键函数，用于将ROS图像消息 (sensor_msgs/Image) 转换为OpenCV格式，msg：ROS图像消息指针
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("CV bridge error: %s", e.what());
            return;
        }
        processFrame();
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        current_cloud_ = msg;//sensor_msgs::PointCloud2ConstPtr current_cloud_;
    }

    void processFrame() {
        if (cv_ptr && current_cloud_) {//如果指针和点云，非空
            cv::Mat frame = cv_ptr->image;//取出图像
            cv::Mat gray;
            cvtColor(frame, gray, cv::COLOR_BGR2GRAY);//将BGR格式的彩色图像转换为单通道灰度图像
            equalizeHist(gray, gray);//对灰度图像进行直方图均衡化，增强对比度。

            // 人脸检测
            std::vector<cv::Rect> faces;//int x;       // 矩形左上角x坐标（像素）
                                                        // int y;       // 矩形左上角y坐标（像素）
                                                        // int width;   // 矩形宽度（像素）
                                                        // int height; 
            face_cascade.detectMultiScale(gray, faces, 1.1, 3, 0, cv::Size(30, 30));//face_cascade.detectMultiScale()是OpenCV中Haar级联分类器用于目标检测（人脸检测）的核心函数
            // void CascadeClassifier::detectMultiScale(
            //     InputArray image,                // 输入图像（灰度）
            //     std::vector<Rect>& objects,      // 输出检测结果矩形框
            //     double scaleFactor = 1.1,        // 图像缩放比例因子
            //     int minNeighbors = 3,            // 最小邻居数（过滤误检）
            //     int flags = 0,                   // 兼容旧版本的标志位
            //     Size minSize = Size(100，100),           // 最小目标尺寸
            //     Size maxSize = Size()，未注明            // 最大目标尺寸
            // );
            // 转换点云数据
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);//// 创建PCL点云智能指针
            pcl::fromROSMsg(*current_cloud_, *cloud);//ROS到PCL的转换

            for (const auto& face : faces) {
                // const	表示face是只读的，防止在循环中意外修改元素
                // auto&	自动类型推导为引用，避免拷贝元素（尤其对cv::Rect等非基本类型很重要）
                // face	循环变量名，代表当前迭代的元素
                // faces	要遍历的容器（需支持begin()和end()迭代器）
                //类似for (auto it = faces.begin(); it != faces.end(); ++it) {
                //     const auto& face = *it;  // 解引用迭代器
                //     // 循环体
                // }
                // 绘制人脸框
                cv::rectangle(frame, face, cv::Scalar(0, 255, 0), 2);
                //frame	cv::Mat	输入/输出的BGR图像矩阵
                // face	cv::Rect	人脸矩形区域（x,y,width,height）
                // Scalar(0,255,0)	cv::Scalar	BGR颜色值（绿色）
                // 2	int	线宽（像素）

                // 计算人脸中心点
                cv::Point center_face(face.x + face.width/2.0, face.y + face.height/2.0);
                center_lian=center_face;
                cv::circle(frame, center_face, 5, cv::Scalar(255, 0, 255), -1);
                cv::line(frame, cv::Point(center_picture.x,0), cv::Point(center_picture.x,frame.rows),cv::Scalar(0, 0, 255), 1);
                // 画人脸中心线（绿色）
                cv::line(frame, cv::Point(center_lian.x,0 ), cv::Point(center_lian.x,frame.rows), cv::Scalar(66, 255, 0), 1);
                // ROS_INFO("center is (%d,%d)",center_face.x,center_face.y );
                //计算图像中心
                cv::Point center_p(frame.cols/2, frame.rows/2);
                center_picture=center_p;
                // ROS_INFO("center_picture.y=%d,center_lian.y=%d",center_picture.y,center_lian.y);
                // 获取深度值
                // ROS_INFO("width is %d , height is %d",cloud->width,cloud->height );
                if ((center_face.x >= 0) && (center_face.x < cloud->width)&&(center_face.y >= 0) && (center_face.y < cloud->height)) {
                    pt = cloud->at(center_face.x, center_face.y);//深度值获取，通过at()方法访问点云中特定位置的像素
                    // ROS_INFO("distance is %.2f",pt.z);
                    if (!std::isnan(pt.z)) {//检查深度值是否为无效值
                        // 显示距离信息
                        char dist_text[50];
                        sprintf(dist_text, "%.2f meters", pt.z);
                        cv::putText(frame, dist_text,cv::Point(face.x, face.y-10),cv::FONT_HERSHEY_SIMPLEX, 1.0,cv::Scalar(0, 255, 0), 2);
                        // ROS_INFO("Detected face at %.2f meters",pt.z);
                        flag=true;
                    }
                }
                // else
                // {
                //     flag=false;
                // }
            }
            //跟踪人脸
        if(flag==true)
        {
            float fVelFoward=(pt.z-0.6)*0.4;
            float fVelTurn=(center_picture.x-center_lian.x)*0.05;
            vel_cmd.linear.x=fVelFoward;
            vel_cmd.linear.y=0;
            vel_cmd.linear.z=0;
            vel_cmd.angular.x=0;
            vel_cmd.angular.y=0;
            // ROS_INFO("center_picture.y=%.2f,center_lian.y=%.2f",center_picture.y,center_lian.y);
            vel_cmd.angular.z=fVelTurn;
            flag=false;
        }
        else{
            // ROS_INFO("目标消失......");
            vel_cmd.linear.x=0;
            vel_cmd.linear.y=0;
            vel_cmd.linear.z=0;
            vel_cmd.angular.x=0;
            vel_cmd.angular.y=0;
            vel_cmd.angular.z=0;
        }
        if (vel_cmd.linear.x>=0.2)
        {
            vel_cmd.linear.x=0.2;
        }
        else if (vel_cmd.linear.x<=-0.2)
        {
            vel_cmd.linear.x=-0.2;
        }
        
        if (vel_cmd.angular.z>=0.1)
        {
            vel_cmd.angular.z=0.1;
        }
        else if (vel_cmd.angular.z<=-0.1)
        {
            vel_cmd.angular.z=-0.1;
        }
        
        
        // vel_pub.publish(vel_cmd);
        // ROS_INFO("机器人运动:linear.x=%.2f,angular.z=%.2f",vel_cmd.linear.x,vel_cmd.angular.z);

            // 显示结果
            cv::imshow("Face Detection", frame);
            cv::waitKey(1);
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber cloud_sub_;
    sensor_msgs::PointCloud2ConstPtr current_cloud_;
    cv_bridge::CvImagePtr cv_ptr;
    cv::CascadeClassifier face_cascade;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "face_depth_detector");
    ros::NodeHandle nf;
    setlocale(LC_ALL,"");
    vel_pub=nf.advertise<geometry_msgs::Twist>("/cmd_vel",20);
    FaceDepthDetector fdd;
    ros::spin();  // 处理回调
    return 0;
}
