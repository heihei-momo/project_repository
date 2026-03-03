#include <string>
#include <iostream>
#include "glog/logging.h"   // glog 头文件
#include "glog/raw_logging.h"

int main(int argc, char** argv){
    // FLAGS_log_dir=".";   //设置log目录  没有指定则输出到控制台
    // FLAGS_logtostderr = 1;  //输出到控制台  true
    google::InitGoogleLogging(argv[0]);    // 初始化
    std::string test = "this is test";
    int i = 2, number = 8;

    LOG(INFO) << "it is info";     // 打印log：“hello glog.  类似于C++ stream。
    LOG_IF(INFO, number > 10) << "number >  10"; //条件log
    LOG_IF(INFO, number < 10) << "number <  10";
    for(i=0; i<20 ;i++){
        LOG_EVERY_N(INFO, 5) << "log i = " << i;//每隔 5 次打印一次日志
    }

    LOG(WARNING) << "It is error info"; 
    LOG(ERROR) << "It is error info"; 
    //DLOG → 仅在 Debug 模式（NDEBUG 未定义）才输出。
    DLOG(INFO) << "it is debug mode";
    DLOG_IF(INFO, number > 10) << "debug number > 10";  
    // DLOG_EVERY_N(INFO, 10) << "log i = " << i;
    RAW_LOG(INFO, "it is pthread log");//直接输出原始日志，不受 LOG 宏的延迟或缓冲机制影响。常用于多线程或 pthread 调试

    //catkin_make 默认 Debug 模式
    //如果用catkin_make -DCMAKE_BUILD_TYPE=Release编译，就用Release编译了

    return 0;
}
