// #include <casadi/core/calculus.hpp>
#include <casadi/core/sx_fwd.hpp>
#include<iostream>
#include<vector>
#include <casadi/casadi.hpp>
#include "mpc_ctl.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/GetModelState.h>
#include <nav_msgs/Odometry.h>

using namespace std;
using namespace casadi;

mavros_msgs::State current_state;
ros::Publisher odometry_pub;
MPC_CTL MPC_Ctl;

// for realfly
// void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
// for sim
void local_pose_cb(const gazebo_msgs::ModelStates::ConstPtr& msg){
    double roll, pitch, yaw;
    // for realfly
    // geometry_msgs::PoseStamped current_odom = *msg;
    // for sim
    std::string model_name = msg->name[3];
    geometry_msgs::Pose model_pose = msg->pose[3];
    geometry_msgs::Twist model_twist = msg->twist[3];

    // for realfly
    // double w = current_odom.pose.orientation.w;
    // double x = current_odom.pose.orientation.x;
    // double y = current_odom.pose.orientation.y;
    // double z = current_odom.pose.orientation.z;
    // for sim
    double w = model_pose.orientation.w;
    double x = model_pose.orientation.x;
    double y = model_pose.orientation.y;
    double z = model_pose.orientation.z;
    double x_ = model_pose.position.x;
    double y_ = model_pose.position.y;
    double z_ = model_pose.position.z;
    double v_x = model_twist.linear.x;
    double v_y = model_twist.linear.y;
    double v_z = model_twist.linear.z;
    double w_x = model_twist.angular.x;
    double w_y = model_twist.angular.y;
    double w_z = model_twist.angular.z;

    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    roll = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (w * y - z * x);
    if (abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // 使用 90 度或 -90 度
    else
        pitch = asin(sinp);

    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    yaw = atan2(siny_cosp, cosy_cosp);

    MPC_Ctl.updatex0(roll, pitch, yaw, w_x, w_y, w_z, 
                        x_, y_, z_, v_x, v_y, v_z);
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;
    
    MPC_Ctl.init(nh);
    // for realfly
    // ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    // ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
    //                                 "mavros/local_position/pose", 10, local_pose_cb);
    // for sim
    ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 10, local_pose_cb);

    ros::Rate rate(50.0);

    // for realfly
    // while(ros::ok() && !current_state.connected){
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    // MPC_Ctl.solve();
    while(ros::ok()) {
        MPC_Ctl.solve();

        ros::spinOnce();
        rate.sleep();
    }

}