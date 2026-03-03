#include <iostream>
#include <gtest/gtest.h>
using namespace std;
int Add(int x, int y)
{
    return x + y;
}
TEST(testadd, test1)
{
    ASSERT_EQ(Add(10, 20), 30);//ASSERT_EQ(a, b)：断言 a == b，否则测试失败并终止此用例，这里验证 Add(10,20) 是否等于 30 → ✅ 成功
}
TEST(testadd, test2)
{
    ASSERT_LT(Add(10, 20), 40);//断言add(10,20)小于40
    ASSERT_GT(Add(10, 20), 40);//断言add(10,20)大于40
}
int main(int argc, char *argv[])
{
    // 将命令行参数传递给 gtest
    testing::InitGoogleTest(&argc, argv);//初始化
    // 运行所有测试案例
    return RUN_ALL_TESTS();
}