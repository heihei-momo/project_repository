#include <iostream>
#include <gflags/gflags.h>
DEFINE_string(host, "localhost", "Server host address.");
DEFINE_int32(port, 8080, "Server port.");

static bool ValidatePort(const char *flag, gflags::int32 value)
{
if (value > 0 && value < 32768)
return true;
std::cerr << "Invalid value for --" << flag << ": " << value << std::endl;
return false;
}

static const bool port_dummy = gflags::RegisterFlagValidator(&FLAGS_port, &ValidatePort);

int main(int argc, char **argv)
{
gflags::SetVersionString("1.1.0");
gflags::SetUsageMessage("./gflags");
//--version 时会输出 1.1.0
// --help 时会显示 Usage: ./gflags 以及参数帮助。
gflags::ParseCommandLineFlags(&argc, &argv, true);
// std::cout << gflags::SetCommandLineOption("port", "999") << std::endl;//运行时修改参数：SetCommandLineOption("port", "999")把 FLAGS_port 改成 999返回修改后的值（这里就是 "999"）
std::cout << "Got '" << FLAGS_host << ":" << FLAGS_port << "'" << std::endl;
return 0;
}
//rosrun gflag_ws gflag_demo03 --flagfile=/home/yyh/study_files/glog_gflag_gtest_study/src/gflag_ws/flags/user.flags
//读取文件里面的参数