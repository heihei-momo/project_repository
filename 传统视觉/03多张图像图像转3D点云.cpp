#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <boost/format.hpp>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>

using namespace std;
typedef vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> TrajectoryType;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

void savePointCloudToPLY(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud, const string &filename) {
    ofstream plyFile(filename, ios::binary);
    if (!plyFile) {
        cerr << "无法打开 PLY 文件: " << filename << endl;
        return;
    }

    // PLY 文件头
    plyFile << "ply\n";
    plyFile << "format binary_little_endian 1.0\n";
    plyFile << "element vertex " << pointcloud.size() << "\n";
    plyFile << "property float x\n";
    plyFile << "property float y\n";
    plyFile << "property float z\n";
    plyFile << "property uchar red\n";
    plyFile << "property uchar green\n";
    plyFile << "property uchar blue\n";
    plyFile << "end_header\n";

    // 写入点云数据（二进制）
    for (const auto &p : pointcloud) {
        float x = static_cast<float>(p[0]);
        float y = static_cast<float>(p[1]);
        float z = static_cast<float>(p[2]);
        plyFile.write(reinterpret_cast<const char*>(&x), sizeof(float));
        plyFile.write(reinterpret_cast<const char*>(&y), sizeof(float));
        plyFile.write(reinterpret_cast<const char*>(&z), sizeof(float));
        uint8_t r = static_cast<uint8_t>(p[3]);
        uint8_t g = static_cast<uint8_t>(p[4]);
        uint8_t b = static_cast<uint8_t>(p[5]);
        plyFile.write(reinterpret_cast<const char*>(&r), sizeof(uint8_t));
        plyFile.write(reinterpret_cast<const char*>(&g), sizeof(uint8_t));
        plyFile.write(reinterpret_cast<const char*>(&b), sizeof(uint8_t));
    }

    plyFile.close();
    cout << "点云已保存到 PLY 文件: " << filename << endl;
}

// 保存点云到 TXT 文件（纯文本格式）
void savePointCloudToTXT(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud, const string &filename) {
    ofstream txtFile(filename);
    if (!txtFile) {
        cerr << "无法打开 TXT 文件: " << filename << endl;
        return;
    }

    // 写入点云数据（文本格式：x y z r g b）
    for (const auto &p : pointcloud) {
        txtFile << p[0] << " " << p[1] << " " << p[2] << " "
                << static_cast<int>(p[3]) << " " << static_cast<int>(p[4]) << " " << static_cast<int>(p[5]) << "\n";
    }

    txtFile.close();
    cout << "点云已保存到 TXT 文件: " << filename << endl;
}

// 在pangolin中显示点云（原函数未修改）
void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud) {
    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
        .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p : pointcloud) {
            glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);
    }
}

int main(int argc, char **argv) {
    vector<cv::Mat> colorImgs, depthImgs;
    TrajectoryType poses;

    ifstream fin("/home/yyh/study_opencv/src/cv_pkg/meshes/point_cloud_ping/pose.txt");// 5 个相机位姿，tx ty tz qx qy qz qw
    if (!fin) {
        cerr << "请在有pose.txt的目录下运行此程序" << endl;
        return 1;
    }

    for (int i = 0; i < 5; i++) {
        boost::format fmt("/home/yyh/study_opencv/src/cv_pkg/meshes/point_cloud_ping/%s/%d.%s");
        colorImgs.push_back(cv::imread((fmt % "color" % (i + 1) % "png").str()));
        depthImgs.push_back(cv::imread((fmt % "depth" % (i + 1) % "pgm").str(), -1));

        double data[7] = {0};
        for (auto &d : data) fin >> d;
        Sophus::SE3d pose(Eigen::Quaterniond(data[6], data[3], data[4], data[5]),// 四元数(qw, qx, qy, qz)
                          Eigen::Vector3d(data[0], data[1], data[2]));// 平移(tx, ty, tz)
        poses.push_back(pose);//最终将位姿存入poses向量
    }

    // 计算点云
    double cx = 325.5, cy = 253.5, fx = 518.0, fy = 519.0, depthScale = 1000.0;
    vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
    pointcloud.reserve(1000000);
    for (int i = 0; i < 5; i++) {
        cout << "转换图像中: " << i + 1 << endl;
        cv::Mat color = colorImgs[i];
        cv::Mat depth = depthImgs[i];
        Sophus::SE3d T = poses[i];
        for (int v = 0; v < color.rows; v++) {
            for (int u = 0; u < color.cols; u++) {
                unsigned int d = depth.ptr<unsigned short>(v)[u];
                if (d == 0) continue;
                Eigen::Vector3d point;
                point[2] = double(d) / depthScale;//depthScale：将传感器原始数据转换为物理单位（通常1000对应毫米→米转换）
                point[0] = (u - cx) * point[2] / fx;//X = (u-cx)*Z/fx
                point[1] = (v - cy) * point[2] / fy;// Y = (v-cy)*Z/fy  从像素坐标(u,v)到3D点(x,y,z)：
                Eigen::Vector3d pointWorld = T * point;//坐标系变换
                //运算本质：pointWorld = R * point + t
                Vector6d p;//颜色数据提取  [x, y, z, r, g, b]
                p.head<3>() = pointWorld;
                p[5] = color.data[v * color.step + u * color.channels()];     // B
                p[4] = color.data[v * color.step + u * color.channels() + 1]; // G
                p[3] = color.data[v * color.step + u * color.channels() + 2]; // R
                pointcloud.push_back(p);
            }
        }
    }

    cout << "点云共有" << pointcloud.size() << "个点." << endl;

    // 保存点云到文件
    savePointCloudToPLY(pointcloud, "/home/yyh/study_opencv/src/cv_pkg/meshes/pointcloud/output.ply");  // 二进制 PLY 格式

    return 0;
}