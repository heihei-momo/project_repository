#include <cstdio>
#include <glog/logging.h>

int main(int argc, char* argv[]) {
    // Initialize Google’s logging library.
    google::InitGoogleLogging(argv[0]);//初始化 glog
    // FLAGS_logtostderr = true;
    std::printf("Hello google glog!\n");
    FLAGS_log_dir = "/home/yyh/study_files/glog_gflag_gtest_study/src/gflag_ws/log";
    //如果没有指定，只有error会打印在终端上
    int num_cookies = 1234;
    LOG(INFO) << "Found " << num_cookies << " cookies";
    std::printf("INFO!\n");

    LOG(WARNING) << "Found " << ++num_cookies << " cookies";
    std::printf("Warning!\n");

    LOG(ERROR) << "Found " << ++num_cookies << " cookies";
    std::printf("Error!\n");

    // LOG(FATAL) << "Found " << ++num_cookies << " cookies";//FATAL 日志会 直接终止程序
    // std::printf("FATAL!\n");


    return 0;
}

