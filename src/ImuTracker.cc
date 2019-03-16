#include "ImuTracker.h"
#include <iostream>
#include <iomanip>

using namespace std;

namespace ORB_SLAM2
{
void ImuTracker::init(const Eigen::Quaterniond &data)
{
    mInitQuat = data;
    mInit = true;
}

Eigen::Quaterniond ImuTracker::track(double x, double y, double z, double w)
{
    Eigen::Quaterniond data(w, x, y, z);
    if (!mInit)
    {
        init(data);
        return Eigen::Quaterniond::Identity();
    }
    else
    {
        return mInitQuat.inverse() * data;
    }
}
} // namespace ORB_SLAM2