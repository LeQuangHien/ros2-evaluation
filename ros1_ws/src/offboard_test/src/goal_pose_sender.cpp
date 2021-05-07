//
// Created by hien on 27.01.21.
//

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "offboard_test/GoalPose.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_pose_sender");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<offboard_test::GoalPose>("goal_pose", 1000);

    ros::Rate loop_rate(1000.0);

    offboard_test::GoalPose pose;
    pose.x = 0;
    pose.y = 1;
    pose.z = 2;
    pose.header.frame_id = "goal_pose";

    int count = 0;
    while (ros::ok())
    {

        pose.y = 1 + 0.01*count;
        pose.header.stamp = ros::Time::now();

        ROS_INFO("Goal pose sending: x=%.1f, y=%.1f, z=%.1f", pose.x, pose.y, pose.z);

        chatter_pub.publish(pose);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


    return 0;
}