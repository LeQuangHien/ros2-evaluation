/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <offboard_test/GoalPose.h>


mavros_msgs::State current_state;
ros::Subscriber state_sub;
ros::Publisher local_pos_pub;
ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;
ros::Subscriber goal_pose_sub;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void poseCallback(const offboard_test::GoalPose &goalPose)
{
    ros::Duration timeOffset = ros::Time::now() - goalPose.header.stamp;
    ROS_INFO("Goal pose sequence %d received: x=%.1f, y=%.1f, z=%.1f", goalPose.header.seq, goalPose.x, goalPose.y, goalPose.z);
    ROS_INFO("Time pose sequence %d received: sec: %d, nsec: %d", goalPose.header.seq, timeOffset.sec, timeOffset.nsec);
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = goalPose.x;
    pose.pose.position.y = goalPose.y;
    pose.pose.position.z = goalPose.z;
    local_pos_pub.publish(pose);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "goal_pose_receiver");
    ros::NodeHandle nh;

    state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    goal_pose_sub = nh.subscribe("goal_pose", 1000, poseCallback);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(1000.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 2;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

