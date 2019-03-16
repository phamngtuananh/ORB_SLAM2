#include "ImuTracker.h"
#include <iostream>
#include <iomanip>

using namespace std;

namespace ORB_SLAM2
{
void ImuTracker::init(const tf::Quaternion &data)
{
    mInitQuat = data;
}

tf::Quaternion ImuTracker::track(const tf::Quaternion &data)
{
    if (!mInit)
    {
        init(data);
        return tf::Quaternion();
    }
    else
    {
        return mInitQuat.inverse() * data;
    }
}

void printQuat(const tf::Quaternion &q)
{
    cout << quat2RPY(q) << endl;
}
} // namespace ORB_SLAM2