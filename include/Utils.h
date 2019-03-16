#ifndef UTILS_H_
#define UTILS_H_

#include <tf/transform_datatypes.h>
#include <opencv2/core/core.hpp>

// #define ENABLE_PROFILING

#ifdef ENABLE_PROFILING
#include <chrono>
#include <iostream>
#define PROFILER_BEGIN(name) auto PROFILE_MAIN_BEGIN_VAR_##name = std::chrono::system_clock::now()
#define PROFILER_END(name)                                                                        \
    {                                                                                             \
        auto PROFILE_MAIN_END_VAR_##name = std::chrono::system_clock::now();                      \
        auto PERIOD_##name = PROFILE_MAIN_END_VAR_##name - PROFILE_MAIN_BEGIN_VAR_##name;         \
        std::cout << "----- " << #name << " took: "                                               \
                  << std::chrono::duration_cast<std::chrono::milliseconds>(PERIOD_##name).count() \
                  << "ms" << std::endl;                                                           \
    }
#else
#define PROFILER_BEGIN(name)
#define PROFILER_END(name)
#endif // ENABLE_PROFILING

namespace ORB_SLAM2
{
void quat2Mat(const tf::Quaternion &q, cv::Mat &rot);
cv::Mat quat2Mat(const tf::Quaternion &q);
cv::Mat quat2RPY(const tf::Quaternion &q);
} // namespace ORB_SLAM2

#endif // UTILS_H_