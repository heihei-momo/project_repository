#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <boost/filesystem.hpp>
#include <iomanip>
#include <fstream>
#include <chrono>

namespace fs = boost::filesystem;

cv::Mat handleLowContrast(const cv::Mat& input) {
    cv::Mat processed;
    
    // 直接转换为灰度图
    if (input.channels() > 1) {
        cv::cvtColor(input, processed, cv::COLOR_BGR2GRAY);
    } else {
        processed = input.clone();
    }
    
    // 自适应直方图均衡化 (CLAHE)
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(2.0);
    clahe->apply(processed, processed);
    
    return processed;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "camera_calibration_node");
    ros::NodeHandle nh("~");
    
    // 从参数服务器获取参数
    std::string image_dir = "/home/yyh/study_opencv/src/cv_pkg/meshes/calibrater";
    int board_width = 8;    // 棋盘格每行的内角点数
    int board_height = 6;   // 棋盘格每列的内角点数
    double square_size = 0.025; // 每个方格的实际大小（单位：米）
    
    // 添加参数解析
    nh.param("image_dir", image_dir, image_dir);
    nh.param("board_width", board_width, board_width);
    nh.param("board_height", board_height, board_height);
    nh.param("square_size", square_size, square_size);
    
    ROS_INFO("Starting camera calibration...");
    ROS_INFO("Image directory: %s", image_dir.c_str());
    ROS_INFO("Board size: %d x %d", board_width, board_height);
    ROS_INFO("Square size: %.3f m", square_size);
    
    // 检查目录是否存在
    if (!fs::exists(image_dir)) {
        ROS_ERROR("Directory does not exist: %s", image_dir.c_str());
        return 1;
    }
    
    // 准备世界坐标系点
    std::vector<cv::Point3f> object_points;
    for (int i = 0; i < board_height; i++) {
        for (int j = 0; j < board_width; j++) {
            object_points.push_back(cv::Point3f(j * square_size, i * square_size, 0.0f));
        }
    }
    
    // 存储检测到的点
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<std::vector<cv::Point3f>> object_points_vec;
    cv::Size board_size(board_width, board_height);
    cv::Size image_size(0, 0);
    
    // 收集所有图像路径
    std::vector<std::string> image_paths;
    for (const auto& entry : fs::directory_iterator(image_dir)) {
        if (fs::is_regular_file(entry)) {
            image_paths.push_back(entry.path().string());
        }
    }
    
    // 按文件名排序，确保处理顺序一致
    std::sort(image_paths.begin(), image_paths.end());
    
    int valid_images = 0;
    auto total_start = std::chrono::high_resolution_clock::now();
    
    // 多尺度检测参数
    const int max_image_size = 2000; // 最大处理尺寸
    const int min_image_size = 800;  // 最小处理尺寸
    
    for (const auto& image_path : image_paths) {
        auto start = std::chrono::high_resolution_clock::now();
        
        cv::Mat image = cv::imread(image_path, cv::IMREAD_COLOR);
        if (image.empty()) {
            ROS_WARN("Failed to read image: %s", image_path.c_str());
            continue;
        }
        
        // 记录第一张有效图像的尺寸
        if (image_size.width == 0) {
            image_size = image.size();
            ROS_INFO("Image size: %d x %d", image_size.width, image_size.height);
        }
        
        // 1. 预处理：优化图像处理流程
        cv::Mat processed = handleLowContrast(image);
        
        // 2. 多尺度检测：根据图像尺寸选择处理策略
        cv::Mat gray;
        double scale_factor = 1.0;
        int max_dim = std::max(image.cols, image.rows);
        
        // 大尺寸图像降采样处理
        if (max_dim > max_image_size) {
            scale_factor = static_cast<double>(max_image_size) / max_dim;
            cv::resize(processed, gray, cv::Size(), scale_factor, scale_factor, cv::INTER_AREA);
            ROS_DEBUG("Downscaled image to %.0f%% for processing", scale_factor * 100);
        } 
        // 小尺寸图像上采样处理
        else if (max_dim < min_image_size) {
            scale_factor = static_cast<double>(min_image_size) / max_dim;
            cv::resize(processed, gray, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);
            ROS_DEBUG("Upscaled image to %.0f%% for processing", scale_factor * 100);
        } 
        else {
            gray = processed;
        }
        
        // 3. 优化棋盘格检测
        std::vector<cv::Point2f> corners;
        bool found = false;
        
        // 尝试使用优化标志检测
        found = cv::findChessboardCorners(
            gray, board_size, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | 
            cv::CALIB_CB_FAST_CHECK | 
            cv::CALIB_CB_NORMALIZE_IMAGE
        );
        
        // 如果首次检测失败，尝试其他方法
        if (!found) {
            // 方法1：增加滤波
            cv::Mat blurred;
            cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.5);
            
            // 方法2：调整参数再次尝试
            found = cv::findChessboardCorners(
                blurred, board_size, corners,
                cv::CALIB_CB_ADAPTIVE_THRESH | 
                cv::CALIB_CB_NORMALIZE_IMAGE
            );
            
            // 方法3：如果仍失败，尝试在原始图像上检测
            if (!found) {
                found = cv::findChessboardCorners(
                    gray, board_size, corners,
                    cv::CALIB_CB_ADAPTIVE_THRESH
                );
            }
        }
        
        if (found) {
            // 4. 优化亚像素级角点精确化
            cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 50, 0.001);
            
            // 在原始分辨率或适当缩放图像上进行优化
            cv::Mat refine_img;
            if (scale_factor != 1.0) {
                cv::resize(processed, refine_img, cv::Size(), scale_factor, scale_factor, cv::INTER_LINEAR);
            } else {
                refine_img = processed;
            }
            
            cv::cornerSubPix(refine_img, corners, cv::Size(15, 15), cv::Size(-1, -1), criteria);
            
            // 如果进行了缩放，将角点映射回原始坐标
            if (scale_factor != 1.0) {
                for (auto& corner : corners) {
                    corner.x /= scale_factor;
                    corner.y /= scale_factor;
                }
            }
            
            // 在原始图像上最终优化
            cv::cornerSubPix(processed, corners, cv::Size(21, 21), cv::Size(-1, -1), criteria);
            
            image_points.push_back(corners);
            object_points_vec.push_back(object_points);
            valid_images++;
            
            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            ROS_INFO("Chessboard found in: %s (%.2f ms)", image_path.c_str(), duration.count());
            
            // 可视化检测结果 (可选)
            // cv::Mat display_img = image.clone();
            // cv::drawChessboardCorners(display_img, board_size, corners, found);
            // cv::imshow("Detected Corners", display_img);
            // cv::waitKey(100);
        } else {
            ROS_WARN("Chessboard not found in: %s", image_path.c_str());
        }
    }
    
    auto total_end = std::chrono::high_resolution_clock::now();
    auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(total_end - total_start);
    ROS_INFO("Image processing completed in %.2f seconds", total_duration.count() / 1000.0);
    
    if (valid_images < 5) {
        ROS_ERROR("Insufficient valid images (%d) for calibration. Need at least 5.", valid_images);
        return 1;
    }
    
    ROS_INFO("Found chessboards in %d images. Performing calibration...", valid_images);
    
    // 执行相机标定 - 移除k3
    cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, CV_64F);  // 只保留k1,k2,p1,p2
    std::vector<cv::Mat> rvecs, tvecs;
    
    // 优化标定参数 - 固定k3
    int calibration_flags = cv::CALIB_FIX_K3 |  // 固定k3
                           cv::CALIB_FIX_K4 | 
                           cv::CALIB_FIX_K5 | 
                           cv::CALIB_USE_LU;
    
    double rms = cv::calibrateCamera(object_points_vec, image_points, image_size, 
                                    camera_matrix, dist_coeffs, rvecs, tvecs,
                                    calibration_flags);
    
    // 计算重投影误差
    double total_error = 0.0;
    std::vector<float> perViewErrors(object_points_vec.size());
    std::vector<int> bad_images;
    
    for (size_t i = 0; i < object_points_vec.size(); i++) {
        std::vector<cv::Point2f> projected_points;
        cv::projectPoints(object_points_vec[i], rvecs[i], tvecs[i], 
                         camera_matrix, dist_coeffs, projected_points);
        
        double err = cv::norm(image_points[i], projected_points, cv::NORM_L2);
        size_t n = object_points_vec[i].size();
        perViewErrors[i] = static_cast<float>(std::sqrt(err * err / n));
        total_error += err * err;
        
        // 标记高误差图像
        if (perViewErrors[i] > 2.0) {
            bad_images.push_back(i);
        }
    }
    
    total_error = std::sqrt(total_error / (object_points_vec.size() * object_points_vec[0].size()));
    
    // 提取内参
    double fx = camera_matrix.at<double>(0, 0);
    double fy = camera_matrix.at<double>(1, 1);
    double cx = camera_matrix.at<double>(0, 2);
    double cy = camera_matrix.at<double>(1, 2);
    
    // 打印结果
    ROS_INFO("\n=== CALIBRATION RESULTS ===");
    ROS_INFO("Image size: %d x %d", image_size.width, image_size.height);
    ROS_INFO("RMS reprojection error: %.4f pixels", rms);
    ROS_INFO("Total reprojection error: %.4f pixels", total_error);
    
    ROS_INFO("\nCamera intrinsic parameters:");
    ROS_INFO("fx (x-axis focal length): %.2f pixels", fx);
    ROS_INFO("fy (y-axis focal length): %.2f pixels", fy);
    ROS_INFO("cx (principal point x): %.2f pixels", cx);
    ROS_INFO("cy (principal point y): %.2f pixels", cy);
    
    // 只输出4个畸变系数 (k1, k2, p1, p2)
    ROS_INFO("\nDistortion coefficients (k3 fixed to 0):");
    ROS_INFO("k1: %.6f", dist_coeffs.at<double>(0));
    ROS_INFO("k2: %.6f", dist_coeffs.at<double>(1));
    ROS_INFO("p1: %.6f", dist_coeffs.at<double>(2));
    ROS_INFO("p2: %.6f", dist_coeffs.at<double>(3));
    
    // 输出高误差图像
    if (!bad_images.empty()) {
        ROS_WARN("High reprojection error in %ld images:", bad_images.size());
        for (int idx : bad_images) {
            ROS_WARN("  Image %d: %.2f pixels", idx, perViewErrors[idx]);
        }
    }
    
    // 保存结果到文件
    std::string output_file = "/home/yyh/study_opencv/src/cv_pkg/meshes/calibrater/calibration_results.txt";
    std::ofstream out(output_file);
    if (out.is_open()) {
        out << "Camera Calibration Results\n";
        out << "=========================\n\n";
        out << "Image size: " << image_size.width << " x " << image_size.height << "\n";
        out << "Board size: " << board_width << " x " << board_height << "\n";
        out << "Square size: " << square_size << " m\n";
        out << "Valid images: " << valid_images << "\n";
        out << "RMS reprojection error: " << rms << " pixels\n";
        out << "Total reprojection error: " << total_error << " pixels\n\n";
        
        out << "Intrinsic Parameters:\n";
        out << "fx: " << fx << " pixels\n";
        out << "fy: " << fy << " pixels\n";
        out << "cx: " << cx << " pixels\n";
        out << "cy: " << cy << " pixels\n\n";
        
        out << "Distortion Coefficients (k3 fixed to 0):\n";
        out << "k1: " << dist_coeffs.at<double>(0) << "\n";
        out << "k2: " << dist_coeffs.at<double>(1) << "\n";
        out << "p1: " << dist_coeffs.at<double>(2) << "\n";
        out << "p2: " << dist_coeffs.at<double>(3) << "\n";
        
        // 保存高误差图像列表
        if (!bad_images.empty()) {
            out << "\nImages with high reprojection error:\n";
            for (int idx : bad_images) {
                out << "  Image " << idx << ": " << perViewErrors[idx] << " pixels\n";
            }
        }
        
        out.close();
        ROS_INFO("Results saved to: %s", output_file.c_str());
    } else {
        ROS_WARN("Failed to save results to file: %s", output_file.c_str());
    }
    
    // 验证标定结果
    if (!image_paths.empty()) {
        cv::Mat test_img = cv::imread(image_paths[0]);
        if (!test_img.empty()) {
            cv::Mat undistorted;
            cv::undistort(test_img, undistorted, camera_matrix, dist_coeffs);
            
            cv::Mat comparison;
            cv::hconcat(test_img, undistorted, comparison);
            cv::resize(comparison, comparison, cv::Size(), 0.5, 0.5);
            cv::imshow("Calibration Validation", comparison);
            cv::waitKey(3000);
        }
    }
    
    ROS_INFO("Calibration completed successfully!");
    return 0;
}