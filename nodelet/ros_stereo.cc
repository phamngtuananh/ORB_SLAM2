/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <opencv2/core/core.hpp>

#include "ImuTracker.h"
#include "Converter.h"
#include "System.h"
#include "Utils.h"

using namespace std;

class ImageGrabber
{
  public:
    ImageGrabber(ORB_SLAM2::System *pSLAM) : mpSLAM(pSLAM) {}

    void GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight);
    void imuCallback(const sensor_msgs::ImuConstPtr &imu);
    void publishCameraPose(const cv::Mat &pose, const ros::Time &ros_time);
    void publishImu(const sensor_msgs::Imu::ConstPtr &imu);

    // Camera tracking
    ORB_SLAM2::System *mpSLAM;
    bool do_rectify;
    cv::Mat M1l, M2l, M1r, M2r;

    // Imu tracking
    bool cam_tracked = false;
    ORB_SLAM2::ImuTracker imutr;

    // Publishers
    ros::Publisher camera_pose_pub, imu_pub, imu_pose_pub;
    tf::TransformBroadcaster *br;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vslam_stereo_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string orb_vocab_file, config_file;
    std::string imu_topic, left_camera_topic, right_camera_topic;
    pnh.getParam("orb_vocab_file", orb_vocab_file);
    pnh.getParam("config_file", config_file);
    pnh.getParam("imu_topic", imu_topic);
    pnh.getParam("left_camera_topic", left_camera_topic);
    pnh.getParam("right_camera_topic", right_camera_topic);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(orb_vocab_file, config_file, ORB_SLAM2::System::STEREO, true);

    ImageGrabber igb(&SLAM);
    pnh.getParam("do_rectify", igb.do_rectify);

    if (igb.do_rectify)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, igb.M1l, igb.M2l);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, igb.M1r, igb.M2r);
    }

    ros::Subscriber imu_sub = nh.subscribe(imu_topic, 1000, &ImageGrabber::imuCallback, &igb);
    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, left_camera_topic, 10);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, right_camera_topic, 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));

    igb.camera_pose_pub = nh.advertise<nav_msgs::Odometry>("orb_slam2/odom", 10);
    igb.imu_pub = nh.advertise<sensor_msgs::Imu>("orb_slam2/imu", 10);
    igb.imu_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("orb_slam2/imu_pose", 10);
    igb.br = new tf::TransformBroadcaster();

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight)
{
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Get rotational distance derived from IMU data
    cv::Mat imuRotDiff; // disabled

    PROFILER_BEGIN(imageCallback);
    cv::Mat Tcw;
    if (do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
        Tcw = mpSLAM->TrackStereo(imLeft, imRight, cv_ptrLeft->header.stamp.toSec(), imuRotDiff);
    }
    else
    {
        Tcw = mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec(), imuRotDiff);
    }

    if (!Tcw.empty())
    {
        cam_tracked = true;
        publishCameraPose(Tcw.inv(), ros::Time(cv_ptrLeft->header.stamp));
    }
    // else
    // {
    //     cout << "Empty pose" << endl;
    // }
    PROFILER_END(imageCallback);
}

ostream &operator<<(ostream &os, Eigen::Quaterniond &q)
{
    os << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
    return os;
}

void ImageGrabber::imuCallback(const sensor_msgs::ImuConstPtr &imu)
{
    // Wait until camera is tracked and the coordinates are init first
    if (!cam_tracked)
        return;

    // Convert quaternion to matrix pose format
    ros::Time imuTime(imu->header.stamp);
    Eigen::Quaterniond imuq(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
    Eigen::Quaterniond imuq_processed = imutr.track(imu->orientation.x, imu->orientation.y, imu->orientation.z, imu->orientation.w);
    sensor_msgs::Imu::Ptr imu_processed(new sensor_msgs::Imu(*imu));
    imu_processed->orientation.x = imuq_processed.x();
    imu_processed->orientation.y = imuq_processed.y();
    imu_processed->orientation.z = imuq_processed.z();
    imu_processed->orientation.w = imuq_processed.w();
    publishImu(imu_processed);
}

void ImageGrabber::publishCameraPose(const cv::Mat &Tcw, const ros::Time &ros_time)
{
    static float _val[] = {0, -1, 0, 0,
                           0, 0, -1, 0,
                           1, 0, 0, 0,
                           0, 0, 0, 1};
    static const cv::Mat Tbc = cv::Mat(4, 4, CV_32F, _val);
    static const cv::Mat Tcb = Tbc.inv();
    const cv::Mat Tbw = Tcb * (Tcw * Tbc);

    nav_msgs::Odometry msg;
    msg.header.stamp = ros_time;
    msg.header.frame_id = "odom";
    msg.child_frame_id = "camera";
    msg.pose.pose.position.x = Tbw.at<float>(0, 3);
    msg.pose.pose.position.y = Tbw.at<float>(1, 3);
    msg.pose.pose.position.z = Tbw.at<float>(2, 3);

    vector<float> r = ORB_SLAM2::Converter::toQuaternion(Tbw.rowRange(0, 3).colRange(0, 3));
    tf::Quaternion q(r[0], r[1], r[2], r[3]);
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();
    camera_pose_pub.publish(msg);

    tf::Transform trf;
    trf.setOrigin(tf::Vector3(Tbw.at<float>(0, 3), Tbw.at<float>(1, 3), Tbw.at<float>(2, 3)));
    trf.setRotation(q);
    br->sendTransform(tf::StampedTransform(trf.inverse(), ros_time, "camera", "map"));
}

void ImageGrabber::publishImu(const sensor_msgs::Imu::ConstPtr &imu)
{
    imu_pub.publish(imu);

    br->sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(imu->orientation.x,
                                                                        imu->orientation.y,
                                                                        imu->orientation.z,
                                                                        imu->orientation.w),
                                                         tf::Vector3(0.0, 0.0, 0.0)),
                                           ros::Time(imu->header.stamp),
                                           "map",
                                           "imu"));
}