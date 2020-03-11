#include <mutex>
#include <thread>
#include <iostream>
 
int counter = 0;
std::mutex mtx; // 创建互斥量
 
void add()
{
    for (int i = 0; i < 1000000; i++)
    {
        std::lock_guard<std::mutex> lock(mtx); // 析构，自动解锁。
        std::cout << "add" << std::endl;
        counter = counter + 1;
    }
}
 
void sub()
{
    for (int i = 0; i < 1000000; i++)
    {
        std::lock_guard<std::mutex> lock(mtx); // 析构，自动解锁。
        std::cout << "sub" << std::endl;
        counter = counter - 1;
    }
}
 
int main()
{
    std::thread t1(add);
    std::thread t2(sub);
 
    t1.join();
    t2.join();
 
    std::cout << "counter:\t" << counter << std::endl;
}