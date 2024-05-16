#include <casadi/core/sx_fwd.hpp>
#include <cstddef>
#include<iostream>
#include<vector>
#include <casadi/casadi.hpp>
#include "mpc_ctl.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/ros.h"

using namespace std;
using namespace casadi;

int main(int argc, char **argv){
    ros::init(argc, argv, "main");
    ros::NodeHandle nh;

    MPC_CTL MPC_Ctl;
    MPC_Ctl.solve();
    MPC_Ctl.getFirstCon();

}