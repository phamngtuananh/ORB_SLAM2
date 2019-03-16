#ifndef IMUTRACKER_H_
#define IMUTRACKER_H_

#include <vector>
#include <deque>

#include <opencv2/core/core.hpp>
#include <tf/transform_datatypes.h>
#include <ros/time.h>

#include "Utils.h"

namespace ORB_SLAM2
{
class ImuTracker
{
  public:
    ImuTracker() : mInit(false) {}
    void init(const tf::Quaternion &data);
    tf::Quaternion track(const tf::Quaternion &data); // compute the current imu orientation

  private:
    // eImuState mState;
    bool mInit;
    tf::Quaternion mInitQuat;
};
} // namespace ORB_SLAM2
#endif // IMUTRACKER_H_