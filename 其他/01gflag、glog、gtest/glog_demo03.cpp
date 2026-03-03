#include <glog/logging.h>
#include <iostream>
using namespace std;
int main() {
  FLAGS_log_dir = "/home/yyh/study_files/glog_gflag_gtest_study/src/gflag_ws/log/";//指定日志文件的存放目录
  FLAGS_alsologtostderr = true;//同时输出日志到终端
  FLAGS_colorlogtostderr = true;//终端输出彩色日志，区分 INFO/WARNING/ERROR 等级
  FLAGS_stderrthreshold = google::GLOG_INFO;  //设置对应级别及以上级别标准输出，设置 级别阈值，大于等于该级别的日志输出到 stderr（终端）。这里 INFO → INFO/WARNING/ERROR/FATAL 都输出终端
  FLAGS_max_log_size = 10;                    //单个日志文件最大 10MB
  google::InitGoogleLogging("myapp");//初始化glog库，必须调用。"myapp" → 程序名，用于生成日志文件名

  //SetLogDestination(level, path_prefix)
    // 为每个日志等级设置文件前缀。
    // 例如
    // INFO → ./log/info_
    // WARNING → ./log/warning_
    // ERROR → ./log/error_
    // FATAL → ./log/fatal_
    // 实际文件名会在前缀后加时间戳和进程 ID，例如：
  google::SetLogDestination(google::GLOG_INFO,std::string(FLAGS_log_dir + "info_").c_str());//FLAGS_log_dir + "info_"路径
  google::SetLogDestination(google::GLOG_WARNING,std::string(FLAGS_log_dir + "warning_").c_str());
  google::SetLogDestination(google::GLOG_ERROR,std::string(FLAGS_log_dir + "error_").c_str());//已经带了类似 error_ 的前缀，glog 就会直接用它，不再额外拼接 severity（.ERROR）和程序名
//   google::SetLogDestination(google::GLOG_FATAL,std::string(FLAGS_log_dir + "fatal_").c_str());
  google::SetLogFilenameExtension(".log");
  LOG(ERROR) << "yes";
  cout << "yes" << endl;
  return 0;
}
