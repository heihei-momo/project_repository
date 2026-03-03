#include <ros/ros.h>
#include <iostream>
#include <string>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;
using namespace std;

static int iLowH=15;
static int iHighH=40;

static int iLowS=40;
static int iHighS=255;

static int iLowV=40;
static int iHighV=255;
Mat roi;//橙黄颜色的区域，其他都黑的
Mat imgThresholded;//原图中橙黄颜色的区域的二值图

void getcolor(Mat imgOriginl){
    
    Mat imgHSV;
    cvtColor(imgOriginl,imgHSV,COLOR_BGR2HSV);//将RGB转换为HSV格式，并存储到imgHSV
    //将HSV空间做直方图均衡化，不然可调节范围太小了
    vector<Mat> hsvSplit;//定义一个 vector<Mat> 容器 hsvSplit，用于存储 HSV 图像的 3 个通道（H、S、V）
    split(imgHSV,hsvSplit);//split() 函数将输入的 imgHSV（HSV 图像）分离成 3 个单通道矩阵，并存入 hsvSplit
    equalizeHist(hsvSplit[2],hsvSplit[2]);//equalizeHist() 对 V 通道（亮度） 进行直方图均衡化
    merge(hsvSplit,imgHSV);//分离并处理后的 3 个通道（H、S、V）重新合并为完整的 HSV 图像 imgHSV

    //设定阈值，将图像二值化
    inRange(imgHSV,Scalar(iLowH,iLowS,iLowV),Scalar(iHighH,iHighS,iHighV),imgThresholded);
    //HSV图像(imgHSV) 进行颜色阈值分割，提取符合特定颜色范围的区域，结果存储在 imgThresholded
    //开操作，去除一些噪点，腐蚀一些
    Mat element=getStructuringElement(MORPH_RECT,Size(17,17));
    morphologyEx(imgThresholded,imgThresholded,MORPH_OPEN,element);
    //闭操作,填充小孔，连接邻近物体
    morphologyEx(imgThresholded,imgThresholded,MORPH_CLOSE,element);
    // 提取原图中符合颜色范围的区域
    imgOriginl.copyTo(roi, imgThresholded);  //roi中非掩膜区域为黑色
}

void getContours(Mat frame,Mat imgDil, Mat img) {//img就是roi

	vector<vector<Point>> contours;//存储检测到的轮廓（contours）。
	vector<Vec4i> hierarchy;//存储轮廓之间的层级关系（父子关系）。
	findContours(imgDil, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	vector<vector<Point>> conPoly(contours.size());	// 逼近的多边形曲线，接近contours轮廓多边形		
	vector<Rect> boundRect(contours.size());	// contours轮廓多边形的边界包围盒

	// 遍历每一个轮廓多边形
	for (int i = 0; i < contours.size(); i++)
	{
		int area = contourArea(contours[i]);	// 计算轮廓的面积
		string objectType;
		if (area > 1100)	// 过滤那些面积特别小的轮廓，消除噪声
		{
			float peri = arcLength(contours[i], true);	// 计算轮廓周长(封闭的或者非封闭的)或曲线长度
			// approxPolyDP(contours[i], conPoly[i], 0.02 * peri, true);	// 以指定精度逼近多边形曲线
			approxPolyDP(contours[i], conPoly[i],0.04 * peri, true);
			boundRect[i] = boundingRect(conPoly[i]);	// 计算顶点集合或灰度图像的非零像素的右上边界矩形，获取边界包围盒（计算外接矩形）
			int objCor = (int)conPoly[i].size();	// 轮廓多边形的角落（顶点）个数
            ROS_INFO("图形 %d,有 %d 个顶点",i+1,objCor);
			drawContours(img, conPoly, i, Scalar(255, 0, 255), 25);	// 绘制轮廓或填充轮廓，颜色为粉色
			rectangle(img, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 7);	// 绘制边界包围盒（矩形），颜色为绿色（BGR）
			//标记质心
			Moments M;
			for (int c= 0;c<contours.size(); c++)
			{
				M = moments(contours[c]);
				double cX = double(M.m10 / M.m00);
				double cY = double(M.m01 / M.m00);//计算轮廓中心点
				circle(img, Point2d(cX, cY), 5, Scalar(0, 255, 0), 2, 8);
			}
					Point left_side[2];
					Point right_side[2];
					if(objCor==4){
				
							circle(roi, conPoly[i][0], 5, Scalar(0, 0, 0), -1);  // 第一点为黑色
							circle(roi, conPoly[i][1], 5, Scalar(0, 255, 0), -1);//第二点为绿色
							circle(roi, conPoly[i][2], 5, Scalar(255, 0, 0), -1);//第二点为蓝色
							circle(roi, conPoly[i][3], 5, Scalar(255, 255, 255), -1);//第二点为白色
							//标记顶点和坐标
							roi.copyTo(roi, imgThresholded);// 将处理后的roi合并回原图,仅更新掩膜区域
							for (int j= 0;j< conPoly[i].size();j++) {
								//conPoly[i][j]是point类型
								Point pt=conPoly[i][j];
								//坐标文本
								string coordText = "(" + to_string(pt.x) + "," + to_string(pt.y) + ")";
								//坐标文本位置
								Point textPos;
								if(j<2){
									textPos = pt+Point(0, 200);
									right_side[j]=pt;
								}
								else{
									textPos = pt+Point(400,0);//左边两个点
									left_side[j-2]=pt;
								}
								// putText(frame,coordText,textPos,FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 0, 0), 5);
								ROS_INFO("图形%d的第%d角的坐标为(%d,%d)",i+1,j+1,pt.x,pt.y);
							}
					//-----------------------------------
					//计算左右两条边斜率
						Point textPos_xie_left=Point(800, 100+i*100);
						Point textPos_xie_right=Point(800, 150+i*100);
						if(left_side[1].x!=left_side[0].x){
							float tilt_left=(float)(left_side[1].y-left_side[0].y)/(left_side[1].x-left_side[0].x);
							string coordText_left = "xie_left:"+to_string(tilt_left)+"-shape"+to_string(i+1);
							putText(frame,coordText_left,textPos_xie_left,FONT_HERSHEY_SIMPLEX,0.9, Scalar(66, 0, 0), 5);
						}
						else{
							string coordText_left ="xie_left:infinity-shape"+to_string(i+1);
							putText(frame,coordText_left,textPos_xie_left,FONT_HERSHEY_SIMPLEX,0.9, Scalar(66, 0, 0), 5);
						}
						if(right_side[0].x!=right_side[1].x){
							float tilt_right=(float)(right_side[0].y-right_side[1].y)/(right_side[0].x-right_side[1].x);
							string coordText_right = "xie_right:"+to_string(tilt_right)+"-shape"+to_string(i+1);
							putText(frame,coordText_right,textPos_xie_right,FONT_HERSHEY_SIMPLEX,0.9, Scalar(66, 0, 0), 5);
						}
						else{
							string coordText_right ="xie_right:infinity-shape"+to_string(i+1);
							putText(frame,coordText_right,textPos_xie_right,FONT_HERSHEY_SIMPLEX,0.9, Scalar(66, 0, 0), 5);
						}

					}
		}
	}
}

   
void getHough(Mat imgThresholded,Mat frame){
	vector<cv::Vec4i> lines;
    HoughLinesP(imgThresholded, lines,1,CV_PI/180,50,50,10); 
	/*void HoughLinesP(
    InputArray image,       // 输入图像（二值图，如Canny边缘检测结果）
    OutputArray lines,      // 输出线段向量（每条线用4个值表示：x1, y1, x2, y2）
    double rho,             // 距离分辨率（像素单位）
    double theta,           // 角度分辨率（弧度单位）
    int threshold,          // 累加器阈值（投票阈值）
    double minLineLength=0, // 最小线段长度（小于此值的线段被丢弃）
    double maxLineGap=0     // 最大允许的线段间距（同一行上的点之间最大间隙）
	);*/         
	// 绘制线段
    Mat color_dst;
    cvtColor(imgThresholded, color_dst, cv::COLOR_GRAY2BGR); // 转为彩色图
    for (const auto& line : lines) {    //这是一个基于范围的for循环，用于遍历lines容器中的每一个元素。
        cv::line(color_dst, Point(line[0], line[1]),Point(line[2], line[3]),Scalar(110, 0, 255), 2);    
    }

	//合并所有直线的端点
    std::vector<cv::Point> all_points;
	for (const auto& line : lines) {
		all_points.emplace_back(line[0], line[1]); // 起点
		all_points.emplace_back(line[2], line[3]); // 终点
	}

	if (all_points.size()>=1) {
		//拟合一条直线
		Vec4f fitted_line;
		fitLine(all_points, fitted_line, cv::DIST_L2, 0, 0.01, 0.01);
		/*cv::fitLine 输出的 fitted_line 是一个 cv::Vec4f 类型，包含4个浮点数：
		[vx, vy, x0, y0]
		(vx, vy)：直线的单位方向向量（归一化后的斜率方向）。
		满足 \(v_x^2 + v_y^2 = 1\)。
		(x0, y0)：直线上的一个参考点（通常是点集的几何中心附近）。*/

		//计算直线的两个端点（从图像顶部到底部）
		float vx = fitted_line[0], vy = fitted_line[1];
		float x0 = fitted_line[2], y0 = fitted_line[3];
		int y1 = 0;                     // 图像顶部
		int x1 = cvRound(x0 + (y1 - y0) * vx / vy);
		int y2 = color_dst.rows;             // 图像底部
		int x2 = cvRound(x0 + (y2 - y0) * vx / vy);

		//绘制拟合的直线
		line(frame, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(0, 255, 0), 2);
		float xie=vy/vx;
		string coordText="slope:"+to_string(xie);
		putText(frame,coordText,Point(720,700),FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 0, 0), 5);
	}


	namedWindow("霍夫变换",WINDOW_NORMAL);//显示窗口命名;
    resizeWindow("霍夫变换", 800, 600);    
	imshow("霍夫变换",color_dst);

}

int main(int argc, const char** argv)
{
    setlocale(LC_ALL,"");
    // Mat frame = imread("/home/yyh/study_opencv/src/open_lannent/meshes/车道线.jpeg");
	VideoCapture cap("/home/yyh/study_opencv/src/open_lannent/meshes/lannent.mp4");  // 支持 MP4、AVI 等常见格式
    if (!cap.isOpened()) {  // 检查是否成功打开
    cerr << "Error: Could not open video file." << endl;
    return -1;
    }
	
	
    while (true) {
		Mat frame;
        cap >> frame;  // 逐帧读取
        if (frame.empty())
			return 0;
		
		// Mat frame_copy=frame.clone();

        getcolor(frame);//getcolor后面才有roi
		Mat imgGray,imgBlur,imgCanny,imgDil;

		cvtColor(roi, imgGray, COLOR_BGR2GRAY);	// 灰度图;
		GaussianBlur(imgGray, imgBlur, Size(3, 3), 3, 0);	// 高斯模糊处理
		Canny(imgBlur, imgCanny, 25, 75);	// Canny边缘检测算法
		Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));//创建了一个3×3像素的矩形结构元素，每个元素是1
		dilate(imgCanny, imgDil, kernel);	// 膨胀图 是 OpenCV 中的膨胀（Dilation）操作

		getContours(frame,imgDil,roi);
        roi.copyTo(frame,imgThresholded);// 将处理后的roi合并回原图,仅更新掩膜区域
		getHough(imgThresholded,frame);

		namedWindow("颜色二值图",WINDOW_NORMAL);
		namedWindow("边缘二值图",WINDOW_NORMAL);
		namedWindow("颜色提取原图",WINDOW_NORMAL);
		namedWindow("原图识别",WINDOW_NORMAL);//显示窗口命名;



        resizeWindow("颜色二值图", 800, 600);    
		imshow("颜色二值图",imgThresholded);
        resizeWindow("边缘二值图", 800, 600);    
		imshow("边缘二值图",imgCanny);
		resizeWindow("颜色提取原图", 800, 600);
        imshow("颜色提取原图",roi);
        resizeWindow("原图识别", 800, 600);
        imshow("原图识别",frame);


		roi.release();

		if (waitKey(30) == 27) break;  // 按ESC退出
    }
		cap.release();

	return 0;
}