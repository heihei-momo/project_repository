#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>

using namespace cv;
using namespace std;

static int iLowH=8;
static int iHighH=19;

static int iLowS=229;
static int iHighS=255;

static int iLowV=59;
static int iHighV=255;

geometry_msgs::Twist vel_cmd;//速度消息包
ros::Publisher vel_pub;//速度发布对象

void Cam_RGB_Callback(const sensor_msgs::Image msg){
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr =cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);//图像数据转换
    }
    catch(const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exeception:%s",e.what());
        return;
    }
    Mat imgOriginl =cv_ptr->image;//获取转换后的相机图像
    Mat imgHSV;
    cvtColor(imgOriginl,imgHSV,COLOR_BGR2HSV);//将RGB转换为HSV格式，并存储到imgHSV
    //将HSV空间做直方图均衡化，不然可调节范围太小了
    vector<Mat> hsvSplit;//定义一个 vector<Mat> 容器 hsvSplit，用于存储 HSV 图像的 3 个通道（H、S、V）
    split(imgHSV,hsvSplit);//split() 函数将输入的 imgHSV（HSV 图像）分离成 3 个单通道矩阵，并存入 hsvSplit：
                           // hsvSplit[0]：Hue（色调） 通道（0-180，因为 OpenCV 默认用 uint8 表示 H 通道时，范围是 0-180）。
                            // hsvSplit[1]：Saturation（饱和度） 通道（0-255）。
                            // hsvSplit[2]：Value（亮度） 通道（0-255）。
    equalizeHist(hsvSplit[2],hsvSplit[2]);//equalizeHist() 对 V 通道（亮度） 进行直方图均衡化
    merge(hsvSplit,imgHSV);//分离并处理后的 3 个通道（H、S、V）重新合并为完整的 HSV 图像 imgHSV
    //设定阈值，将图像二值化
    Mat imgThresholded;
    inRange(imgHSV,Scalar(iLowH,iLowS,iLowV),Scalar(iHighH,iHighS,iHighV),imgThresholded);
    // HSV 图像 (imgHSV) 进行颜色阈值分割，提取符合特定颜色范围的区域，结果存储在 imgThresholded

    //开操作，去除一些噪点，腐蚀一些
    Mat element=getStructuringElement(MORPH_RECT,Size(5,5));
    morphologyEx(imgThresholded,imgThresholded,MORPH_OPEN,element);
    //闭操作,填充小孔，连接邻近物体
    morphologyEx(imgThresholded,imgThresholded,MORPH_CLOSE,element);
    //遍历二值化后的图像数据
    int nTargetx=0;
    int nTargety=0;//存储目标物的横坐标，与纵坐标
    int nPixCount=0;//记录目标物所占据的像素点数量
    int nImgWidth=imgThresholded.cols;//读取整个图像的宽度
    int nImgHeight=imgThresholded.rows;//读取整个图像的高度
    int nImgChannels=imgThresholded.channels();//读取目标结果图像的通道数--通道：每个像素占用几个字节

    for (int y= 0;y<nImgHeight;y++)//遍历，步长为单位长度
    {
        for (int x= 0;x<nImgWidth;x++)
        {
            if(imgThresholded.data[y*nImgWidth+x]==255){
                nTargetx+=x;
                nTargety+=y;
                nPixCount++;
            }
        }
        
    }
    if(nPixCount>0){                //图像存在目标物
        nTargetx/=nPixCount;
        nTargety/=nPixCount;
        ROS_INFO("颜色质心坐标为：(%d,%d),点数：%d",nTargetx,nTargety,nPixCount);
        //画坐标
        Point line_begin=Point(nTargetx-10,nTargety);
        Point line_end=Point(nTargetx+10,nTargety);
        line(imgOriginl,line_begin,line_end,Scalar(255,0,0),3);//图像上画线，目标点左侧十个像素点到目标点右侧十个像素点，颜色为蓝色,线条宽度（3像素）
        line_begin.x=nTargetx;
        line_begin.y=nTargety-10;
        line_end.x=nTargetx;
        line_end.y=nTargety+10;
        line(imgOriginl,line_begin,line_end,Scalar(255,0,0),3);//划线，目标点上侧十个像素点到目标点下侧十个像素点，颜色为蓝色,线条宽度（3像素）

        //计算机器人跟随速度值
        float fVelFoward=(nImgHeight/2-nTargety)*0.002*3;
        float fVelTurn=(nImgWidth/2-nTargetx)*0.003;
        vel_cmd.linear.x=fVelFoward;
        vel_cmd.linear.y=0;
        vel_cmd.linear.z=0;
        vel_cmd.angular.x=0;
        vel_cmd.angular.y=0;
        vel_cmd.angular.z=fVelTurn;
    }
    else{
        ROS_INFO("目标颜色消失......");
        vel_cmd.linear.x=0;
        vel_cmd.linear.y=0;
        vel_cmd.linear.z=0;
        vel_cmd.angular.x=0;
        vel_cmd.angular.y=0;
        vel_cmd.angular.z=0;
    }
    vel_pub.publish(vel_cmd);
    ROS_INFO("机器人运动:linear.x=%.2f,angular.z=%.2f",vel_cmd.linear.x,vel_cmd.angular.z);

    imshow("RGB",imgOriginl);
    imshow("Result",imgThresholded);

    waitKey(1);//延迟1ms
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"cv_image_node");
    ros::NodeHandle nh;
    ros::Subscriber rgb_sub = nh.subscribe("/camera/image_raw",1,Cam_RGB_Callback);
    vel_pub=nh.advertise<geometry_msgs::Twist>("/cmd_vel",30);
    //生成图像显示和参数调节的窗口
    namedWindow("Threshold",WINDOW_AUTOSIZE);

    createTrackbar("LowH","Threshold",&iLowH,179);//滑杆名称，隶属窗口，变量，范围（0-179）
    createTrackbar("HighH","Threshold",&iHighH,179);

    createTrackbar("LowS","Threshold",&iLowS,255);
    createTrackbar("HighS","Threshold",&iHighS,255);

    createTrackbar("LowV","Threshold",&iLowV,255);
    createTrackbar("V","Threshold",&iHighV,255);



    namedWindow("RGB");
    namedWindow("Result");

    ros::Rate loop_rate(30);//循环频率，30HZ
    while(ros::ok){
        ros::spinOnce();
        loop_rate.sleep();
    }
}