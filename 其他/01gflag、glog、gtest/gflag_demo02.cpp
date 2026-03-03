#include <iostream>
#include <gflags/gflags.h>
DEFINE_string(host, "localhost", "Server host address.");
DEFINE_int32(port, 8080, "Server port.");
DEFINE_int32(susuzhuzhu, 12345, "Server port.");

static bool ValidatePort(const char *flag, gflags::int32 value)//定义校验函数
{
if (value > 0 && value < 32768)
return true;
std::cerr << "Invalid value for --" << flag << ": " << value << std::endl;
return false;
}
//注册校验器
//在程序加载阶段（全局静态变量初始化时），这行代码会执行一次。它调用 RegisterFlagValidator，告诉 gflags：
// “以后每次解析 FLAGS_port 的值时，都要调用 ValidatePort 来检查。”
// 变量 port_dummy 本身没用，只是用来“触发”这句注册代码
static const bool port_dummy = gflags::RegisterFlagValidator(&FLAGS_susuzhuzhu, &ValidatePort);//把上面写的 ValidatePort 绑定到 --port 参数上

int main(int argc, char **argv)
{
gflags::ParseCommandLineFlags(&argc, &argv, true);
//true → gflags 在解析后会移除已经识别的参数，只保留没被 gflags 识别的
//false → gflags 会保留所有参数，即便识别了
std::cout << "Got '" << FLAGS_host << ":" << FLAGS_port << "'"":" << FLAGS_susuzhuzhu << std::endl;
return 0;
}
