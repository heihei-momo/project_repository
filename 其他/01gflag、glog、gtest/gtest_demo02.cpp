#include <iostream>
#include <gtest/gtest.h>
// TestSuite:测试套件/集合进行单元测试,即,将多个相关测试归入一组的方式进行测试, 为这组测试用例进行环境配置和清理
// 概念: 对一个功能的验证往往需要很多测试用例，测试套件就是针对一组相关测试用例进行环境配置的事件机制
// 用法: 先定义环境类,继承于 testing::Test 基类, 重写两个静态函数 SetUpTestCase / TearDownTestCase 进行环境的配置和清理
class HashTestEnv1 : public testing::Test//环境类定义 HashTestEnv1 继承自 testing::Test，就是一个测试套件类。
{
public:
    static void SetUpTestCase()
    {
        std::cout << "环境 1 第一个 TEST 之前调用\n";
    }
    static void TearDownTestCase()
    {
        std::cout << "环境 1 最后一个 TEST 之后调用\n";
    }
    //SetUpTestCase()：在 第一个测试用例运行前 只调用一次。
    // TearDownTestCase()：在 最后一个测试用例运行后 只调用一次。
    // dict：测试用例共享的成员变量（一个哈希表）。
    public:
        std::unordered_map<std::string, std::string> dict;//哈希表
};
// 注意,测试套件使用的不是 TEST 了,而是 TEST_F, 而第一个参数名称就是测试套件环境类名称
// main 函数中不需要再注册环境了,而是在 TEST_F 中可以直接访问类的成员变量和成员函数
TEST_F(HashTestEnv1, insert_test)//insert_test是给测试用例起的名字
{
    std::cout << "环境 1,中间 insert 测试\n";
    dict.insert(std::make_pair("Hello", "你好"));
    dict.insert(std::make_pair("雷吼", "你好"));
    //std::make_pair("Hello", "你好")
    // 创建一个 std::pair<std::string, std::string>，内容是：
    // first = "Hello"
    // second = "你好"
    // dict.insert(...)
    // 把这个键值对插入到哈希表 dict 中。
    auto it = dict.find("Hello");//dict.find("Hello") 会在 unordered_map 里查找 key = "Hello"。返回值是 迭代器：如果找到了 → 迭代器指向该元素。如果没找到 → 返回 dict.end()
    ASSERT_NE(it, dict.end());//是 gtest 提供的断言，检查 a != b
}
TEST_F(HashTestEnv1, sizeof)//每执行一次 TEST_F，gtest 都会新建一个 新的 HashTestEnv1 对象，dict 在这里是一个新的空 unordered_map，sizeof是给测试用例起的名字
{
    std::cout << "环境 1,中间 size 测试\n";
    ASSERT_GT(dict.size(), 0);//要求 dict.size() > 0，所以会断言失败
}
int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);//初始化
    return RUN_ALL_TESTS();
}