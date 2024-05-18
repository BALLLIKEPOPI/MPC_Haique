// #include <casadi/core/calculus.hpp>
#include <casadi/core/sx_fwd.hpp>
#include<iostream>
#include<vector>
#include <casadi/casadi.hpp>
#include "mpc_ctl.h"
#include "ros/ros.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>


using namespace std;
using namespace casadi;

mavros_msgs::State current_state;
MPC_CTL MPC_Ctl;

void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    double roll, pitch, yaw;

    geometry_msgs::PoseStamped current_pose = *msg;
    double w = current_pose.pose.orientation.w;
    double x = current_pose.pose.orientation.x;
    double y = current_pose.pose.orientation.y;
    double z = current_pose.pose.orientation.z;

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

    MPC_Ctl.updatex0x_dot(roll, pitch, yaw);
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
                                    "mavros/local_position/pose", 10, local_pose_cb);
    ros::Rate rate(50.0);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    // MPC_Ctl.solve();
    while(ros::ok()) {
        MPC_Ctl.solve();

        ros::spinOnce();
        rate.sleep();
    }

}