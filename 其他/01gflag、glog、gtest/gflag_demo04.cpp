#include <gflags/gflags.h>
#include <glog/logging.h>
#include <iostream>

DEFINE_string(host, "localhost", "Server host address");

int main(int argc, char** argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    

    // 设置日志目录
    FLAGS_log_dir = "/home/yyh/study_files/glog_gflag_gtest_study/src/gflag_ws/log";
    // FLAGS_logtostderr = true;  // 输出所有日志到终端

    LOG(INFO) << "Program started";
    LOG(WARNING) << "This is a warning";
    LOG(ERROR) << "This is an error";
    //glog 默认行为：每个日志文件包含 当前级别及以上的日志，不是严格只记录该级别：
    //3 个是真实日志文件 + 3 个符号链接
    std::cerr << "This goes to stderr" << std::endl;
    
    std::cout << "Host: " << FLAGS_host << std::endl;

    google::ShutdownGoogleLogging();
    return 0;
}

