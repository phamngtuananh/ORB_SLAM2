#include "Utils.h"

namespace ORB_SLAM2
{
void quat2Mat(const tf::Quaternion &q, cv::Mat &rot)
{
    rot = cv::Mat(3, 3, CV_32F);
    double d = q.length2();
    double s = 2.0 / d;
    double xs = q.x() * s, ys = q.y() * s, zs = q.z() * s;
    double wx = q.w() * xs, wy = q.w() * ys, wz = q.w() * zs;
    double xx = q.x() * xs, xy = q.x() * ys, xz = q.x() * zs;
    double yy = q.y() * ys, yz = q.y() * zs, zz = q.z() * zs;
    rot.at<float>(0, 0) = 1.0 - (yy + zz);
    rot.at<float>(0, 1) = xy - wz;
    rot.at<float>(0, 2) = xz + wy;
    rot.at<float>(1, 0) = xy + wz;
    rot.at<float>(1, 1) = 1.0 - (xx + zz);
    rot.at<float>(1, 2) = yz - wx;
    rot.at<float>(2, 0) = xz - wy;
    rot.at<float>(2, 1) = yz + wx;
    rot.at<float>(2, 2) = 1.0 - (xx + yy);
}

cv::Mat quat2Mat(const tf::Quaternion &q)
{
    cv::Mat rot = cv::Mat(3, 3, CV_32F);
    double d = q.length2();
    double s = 2.0 / d;
    double xs = q.x() * s, ys = q.y() * s, zs = q.z() * s;
    double wx = q.w() * xs, wy = q.w() * ys, wz = q.w() * zs;
    double xx = q.x() * xs, xy = q.x() * ys, xz = q.x() * zs;
    double yy = q.y() * ys, yz = q.y() * zs, zz = q.z() * zs;
    rot.at<float>(0, 0) = 1.0 - (yy + zz);
    rot.at<float>(0, 1) = xy - wz;
    rot.at<float>(0, 2) = xz + wy;
    rot.at<float>(1, 0) = xy + wz;
    rot.at<float>(1, 1) = 1.0 - (xx + zz);
    rot.at<float>(1, 2) = yz - wx;
    rot.at<float>(2, 0) = xz - wy;
    rot.at<float>(2, 1) = yz + wx;
    rot.at<float>(2, 2) = 1.0 - (xx + yy);
    return rot;
}

cv::Mat quat2RPY(const tf::Quaternion &q)
{
    cv::Mat rpy(1, 3, CV_32F);
    double r, p, y;
    tf::Matrix3x3(q).getRPY(r, p, y);
    rpy.at<float>(0, 0) = r;
    rpy.at<float>(0, 1) = p;
    rpy.at<float>(0, 2) = y;
    return rpy.clone();
}
} // namespace ORB_SLAM2