#ifndef __CTL_ALC_H__
#define __CTL_ALC_H__

#include "casadi/casadi.hpp"
#include "eigen3/Eigen/Eigen"
#include <vector>

using namespace std;
using namespace casadi;

class CTL_ALC{
    public:
    CTL_ALC();
    ~CTL_ALC();

    SX Dyna_Func(SX state, SX control){
        return SX::vertcat({-u_r*l*k_r*cos(control(4))*(pow(control(1), 2)+pow(control(3), 2)) - 
                                2*l*k_r*sin(control(4))*(pow(control(1), 2)+pow(control(3), 2)) +
                                u_r*l*k_r*cos(control(5))*(pow(control(0), 2)+pow(control(2), 2)) + 
                                2*l*k_r*sin(control(5))*(pow(control(0), 2)+pow(control(2), 2)),
                                2*k_r*cos(control(5))*(control(0)+control(2)) - 
                                u_r*k_r*sin(control(5))*(pow(control(0), 2)+pow(control(2), 2)) +
                                2*k_r*cos(control(4))*(control(1)+control(3)) - 
                                u_r*k_r*sin(control(4))*(pow(control(1), 2)+pow(control(3), 2))});
    }

    void solve();
    void updatePara();
    void getFirstCon();
    void updatex0(double tx, double T2);
    void updatexs();

    private:

    float h = 0.02; // step[s]
    int N = 10; // prediction horizon
    float I1 = 0.103; float I2 = 0.104; float I3 = 0.161; 
    float m = 4.8; // kg
    float V = 0.0285; // m3
    float rou = 1000; // kg/m3
    float G = 9.8; // m/s2
    float l = 0.6; // m
    float c1 = 0.01; float c2 = 0.01; float c3 = 0.01;
    float k_r = 1;
    float u_r = 1;

    Slice all;
    // states
    SX tx = SX::sym("tx"); 
    SX T2 = SX::sym("T2"); 
    SX state = DM::vertcat({tx, T2});
    int n_state = 2;
    
    // controls
    SX w2 = SX::sym("w2");
    SX w4 = SX::sym("w4");
    SX w6 = SX::sym("w6");
    SX w8 = SX::sym("w8");
    SX alpha = SX::sym("alpha");
    SX beta = SX::sym("beta");
    SX control = DM::vertcat({w2, w4, w6, w8, alpha, beta});
    int n_control = 6;

    SX U = SX::sym("U", n_control, N); // Decision variables (controls)
    // parameters (which include the initial state and the reference state)
    SX P = SX::sym("P", 2*n_state); 
    // A vector that represents the states over the optimization problem.
    SX X = SX::sym("X", n_state, (N+1));
    // initialization of the states decision variables
    SX X0 = SX::sym("X0", 1, N+1);
    // 3 control inputs for each robot
    SX u0 = SX::zeros(N, 6);

    SX obj = 0; // objective function
    SX g = SX::sym("g", 2*(N+1)+3*N); // onstraints vector

    SX Q = SX::zeros(2, 2); // weighing matrices (states)
    SX R = SX::zeros(6, 6); // weighing matrices (controls)
    SX st = SX::sym("st", n_state); // initial state
    SX st_next = SX::sym("st_next", n_state);
    SX con = SX::sym("con", n_control);
    SX OPT_variables = SX::sym("OPT_variables", 2*(N+1)+6*N);

    // RK4
    SX k1 = SX::sym("k1");
    SX k2 = SX::sym("k2");
    SX k3 = SX::sym("k3");
    SX k4 = SX::sym("k4");
    SX st_next_RK4 = SX::sym("st_next_RK4", n_state);

    // Initial guess and bounds for the optimization variables
    vector<double> x0;
    vector<double> x_last;
    vector<double> xs;
    vector<double> x_dot;
    vector<double> para;
    vector<double> lbx;
    vector<double> ubx;
    // Nonlinear bounds
    vector<double> lbg;
    vector<double> ubg;
    std::map<std::string, DM> arg, res;
    Function solver;

    // output
    vector<double> u_f;
};

#endif