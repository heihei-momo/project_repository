#include <iostream>
#include <gflags/gflags.h>

DEFINE_string(host, "localhost", "Server host address");//命令行参数名，默认值，帮助信息   gflags 会生成一个全局变量 FLAGS_host，可以直接在代码中使用
DEFINE_int32(port, 8080, "Server port");
int main(int argc, char **argv)
{
gflags::ParseCommandLineFlags(&argc, &argv, true); //解析命令行参数，把 --host 和 --port 的值写入 FLAGS_host、FLAGS_port
std::cout << "Got '" << FLAGS_host << ":" << FLAGS_port << "'." << std::endl;
return 0;
}
//rosrun gflag_ws gflag_demo01 --host=11 --port=1111
//Got '11:1111'.