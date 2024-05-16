#ifndef __MPC_CTL_H__
#define __MPC_CTL_H__

#include "casadi/casadi.hpp"
#include "eigen3/Eigen/Eigen"
#include <vector>

using namespace std;
using namespace casadi;

class MPC_CTL{
    public:
    MPC_CTL();
    ~MPC_CTL();

    SX Dyna_Func(SX state, SX control, SX abs_states_dot){
        return SX::vertcat({(control(0) - (I3-I2)*state(1)*state(2) - rou*c1*abs_states_dot(0)*state(0))/I1,
                            (control(1) - (I1-I3)*state(0)*state(2) - rou*c2*abs_states_dot(1)*state(1))/I2,
                            (control(2) - (I2-I1)*state(0)*state(1) - rou*c3*abs_states_dot(2)*state(2))/I3});
    }

    void solve();
    void updatePara();
    void getFirstCon();

    private:

    float h = 0.2; // step[s]
    int N = 15; // prediction horizon
    float I1 = 0.103; float I2 = 0.104; float I3 = 0.161; 
    float m = 4.8; // kg
    float V = 0.0285; // m3
    float rou = 1000; // kg/m3
    float G = 9.8; // m/s2
    float l = 0.6; // m
    float c1 = 0.01; float c2 = 0.01; float c3 = 0.01;

    Slice all;
    // states
    SX theta = SX::sym("theta"); 
    SX phi = SX::sym("phi"); 
    SX psi = SX::sym("psi"); 
    SX state = DM::vertcat({theta, phi, psi});
    int n_state = 3;
    
    // controls
    SX t_theta = SX::sym("t_theta");
    SX t_phi = SX::sym("t_phi");
    SX t_psi = SX::sym("t_psi");
    SX control = DM::vertcat({t_theta, t_phi, t_psi});
    int n_control = 3;

    // parameter
    SX abs_theta_dot = SX::sym("abs_theta_dot");
    SX abs_phi_dot = SX::sym("abs_phi_dot");
    SX abs_psi_dot = SX::sym("abs_psi_dot");
    SX abs_states_dot = DM::vertcat({abs_theta_dot, abs_phi_dot, abs_psi_dot});

    SX U = SX::sym("U", n_control, N); // Decision variables (controls)
    // parameters (which include the initial state and the reference state)
    SX P = SX::sym("P", 3*n_state); 
    // A vector that represents the states over the optimization problem.
    SX X = SX::sym("X", n_state, (N+1));
    // initialization of the states decision variables
    SX X0 = SX::sym("X0", 1, N+1);
    // 3 control inputs for each robot
    SX u0 = SX::zeros(N, 3);

    SX obj = 0; // objective function
    SX g = SX::sym("g", 3*(N+1)); // onstraints vector

    SX Q = SX::zeros(3, 3); // weighing matrices (states)
    SX R = SX::zeros(3, 3); // weighing matrices (controls)
    SX st = SX::sym("st", n_state); // initial state
    SX st_next = SX::sym("st_next", n_state);
    SX con = SX::sym("con", n_control);
    SX abs_st_dot = SX::sym("abs_st_dot", 3);
    SX OPT_variables = SX::sym("OPT_variables", 3*(N+1)+3*N);

    // RK4
    SX k1 = SX::sym("k1");
    SX k2 = SX::sym("k2");
    SX k3 = SX::sym("k3");
    SX k4 = SX::sym("k4");
    SX st_next_RK4 = SX::sym("st_next_RK4", n_state);

    // Initial guess and bounds for the optimization variables
    vector<double> x0;
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