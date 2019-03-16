#ifndef IMUTRACKER_H_
#define IMUTRACKER_H_

#include <Eigen/Dense>

namespace ORB_SLAM2
{
class ImuTracker
{
  public:
    ImuTracker() : mInit(false) {}
    void init(const Eigen::Quaterniond &data);
    Eigen::Quaterniond track(double x, double y, double z, double w);

  private:
    bool mInit;
    Eigen::Quaterniond mInitQuat;
};
} // namespace ORB_SLAM2
#endif // IMUTRACKER_H_