#ifndef __TIME_COUNTER_HPP__
#define __TIME_COUNTER_HPP__

#include <iostream>
#include <thread>
#include <chrono> // 适用于多种平台
#include <vector>

using namespace std;

struct timeCounter
{
    float all_the_fucking_time = 0.0f; // 总时间
    int callTimes = 0;
    string description;

    timeCounter(string description = "default") : all_the_fucking_time(0.0f), callTimes(0), description(description)
    {}    
    ~timeCounter()
    {
        // std::cout << "Total time: " << all_the_fucking_time << " ms" << std::endl;
        std::cout << description <<": Average time per call: " << (callTimes > 0 ? all_the_fucking_time / callTimes : 0) << " ms" << std::endl;
    }
};

struct Timer {
    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
    std::chrono::duration<float> duration;

    timeCounter& counter;
    Timer(timeCounter& counter) : counter(counter)
    {
        start = std::chrono::high_resolution_clock::now();
    }
    ~Timer()
    {
        end = std::chrono::high_resolution_clock::now();
        duration = end - start;
        counter.all_the_fucking_time += (duration.count() * 1000); // 转换为毫秒
        counter.callTimes++;
    }
};

#endif // __TIME_COUNTER_HPP__