#include <fstream>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

std::ofstream osfile;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vlam_pose_recorder");
    ros::NodeHandle nh;

    osfile.open("filename.txt");
    ros::Subscriber pose_sub = nh.subscribe("", 1000, &poseCallback);
    ros::spin();
    return 0;
}