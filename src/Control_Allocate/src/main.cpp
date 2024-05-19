// #include <casadi/core/calculus.hpp>
#include <casadi/core/sx_fwd.hpp>
#include<iostream>
#include<vector>
#include <casadi/casadi.hpp>
#include "Control_Allocate.h"
#include "ros/ros.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>


using namespace std;
using namespace casadi;

mavros_msgs::State current_state;
CTL_ALC CTL_Alc;

// callback get tx and T2
void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    double tx, T2, yaw;

    geometry_msgs::PoseStamped current_pose = *msg;

    CTL_Alc.updatex0(tx, T2);
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
    // CTL_Alc.solve();
    while(ros::ok()) {
        CTL_Alc.solve();

        ros::spinOnce();
        rate.sleep();
    }

}