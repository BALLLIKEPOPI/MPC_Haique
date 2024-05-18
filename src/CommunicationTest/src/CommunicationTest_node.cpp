#include <ros/ros.h>
#include "ros/subscriber.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>

using namespace std;

geometry_msgs::PoseStamped current_pose;
mavros_msgs::State current_state;

void local_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
    cout << current_pose << endl;
}

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "CommunicationTest_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
                                    "mavros/local_position/pose", 10, local_pose_cb);
    ros::Rate rate(20.0);

    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }
    cout << "ACFly connected" << endl;

    while(ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}