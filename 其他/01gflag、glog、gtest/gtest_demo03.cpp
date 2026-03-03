#include <iostream>
#include <gtest/gtest.h>
// TestCase:测试用例的单元测试,即针对每一个测试用例都使用独立的测试环境数据进行测试
// 概念:它是针对测试用例进行环境配置的一种事件机制
// 用法:先定义环境类,继承于 testing::Test 基类, 在环境类内重写SetUp/TearDown 接口
class HashTestEnv2 : public testing::Test
{
public:
    static void SetUpTestCase()//-----为整个测试套件（Test Fixture）做一次性的环境初始化。只会在第一次使用HashTestEnv2初始化一次，后面不管几个测试都不会执行了
    {
        std::cout << "环境 2 第一个 TEST 之前被调用,进行总体环境配置\n ";
    }
    static void TearDownTestCase()
    {
        std::cout << "环境 2 最后一个 TEST 之后被调用,进行总体环境清理\n ";
    }
    //在第一个测试用例前 / 最后一个测试用例后 执行一次
    virtual void SetUp() override//---------为每个测试用例 提供独立的初始化环境，每次测试前都会使用
    {
        std::cout << "环境 2 测试前:提前准备数据!!\n";
        dict.insert(std::make_pair("bye", "再见"));
        dict.insert(std::make_pair("see you", "再见"));
    }
    virtual void TearDown() override
    {
        std::cout << "环境 2 测试结束后:清理数据!!\n";
        dict.clear();
    }
    //虚函数 SetUp/TearDown
    // 每个 TEST_F 测试用例执行前调用 SetUp() → 准备数据。
    // 每个 TEST_F 测试用例执行后调用 TearDown() → 清理数据。
 
public:
    std::unordered_map<std::string, std::string> dict;//默认构造函数初始化，每次测试都会刷新构造函数
};
TEST_F(HashTestEnv2, insert_test)
{
    std::cout << "环境 2,中间测试\n";
    dict.insert(std::make_pair("hello", "你好"));//插入的一个hello
    ASSERT_EQ(dict.size(), 3);//断言检查 dict.size()为3
}
TEST_F(HashTestEnv2, size_test)
{
    std::cout << "环境 2,中间 size 测试\n";
    auto it = dict.find("hello");
    ASSERT_EQ(it, dict.end());//断言it是dict.end()
    ASSERT_EQ(dict.size(), 2);//断言 dict.size() 等于 2
}
int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    RUN_ALL_TESTS();
    return 0;
    //ASSERT是严格检查，EXPECT是宽松检查
}