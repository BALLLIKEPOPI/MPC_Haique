#include "devel/include/mpc_control/controlPub.h"
#include "mpc_ctl.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/ros.h"
#include "ros/subscriber.h"
// #include "

void conSub_cb(const mpc_control::controlPub &msg){

}

int main(int argc, char **argv){
    ros::init(argc, argv, "thrustPub");
    ros::NodeHandle nh;

    ros::Publisher thrustPub = nh.advertise<>()
    ros::Subscriber conSub = nh.subscribe<mpc_control::controlPub>("/mpc_ctl", 10, conSub_cb); 
}