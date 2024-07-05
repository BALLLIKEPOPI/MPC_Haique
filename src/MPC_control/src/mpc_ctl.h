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

    SX Dyna_Func(SX state, SX con){
        SX M1 = c*con(0)/k;     SX M2 = c*con(1)/k;     
        SX M3 = c*con(2)/k;     SX M4 = c*con(3)/k; 
        SX M5 = c*con(4)/k;     SX M6 = c*con(5)/k; 
        SX M7 = c*con(6)/k;     SX M8 = c*con(7)/k; 

        SX Mx = (-M2+M6)*cos(con(9))+(-M4+M8)*cos(con(8))+
                    l*((con(1)+con(5))*sin(con(9))-(con(3)+con(7))*sin(con(8)));
        SX My = l*(-con(0)-con(4)+con(2)+con(6));
        SX Mz = M1+M3-M2-M4+M6+M8-M5-M7+(-M2+M6)*sin(con(9))+(-M4+M8)*sin(con(8));

        SX Tx = (con(1)+con(5))*cos(con(9))+(con(3)+con(7))*cos(con(8));
        SX Ty = 0;
        SX Tz = con(0)+con(2)+con(4)+con(6)+(con(1)+con(5))*sin(con(9))+
                    (con(3)+con(7))*sin(con(8));
        return SX::vertcat({state(3),
                            state(4),
                            state(5),
                            (Mx - (I3-I2)*state(4)*state(5))/I1,
                            (My - (I1-I3)*state(3)*state(5))/I2,
                            (Mz - (I2-I1)*state(3)*state(4))/I3,
                            state(9),
                            state(10),
                            state(11),
                            1/m*(cos(state(1))*cos(state(2))*Tx+(cos(state(2))*sin(state(1))*cos(state(0))+sin(state(2))*sin(state(0)))*Tz),
                            1/m*(cos(state(0))*sin(state(2))*Tx+(sin(state(2))*sin(state(1))*cos(state(0))-cos(state(2))*sin(state(0))*Tz)),
                            1/m*(-sin(state(1))*Tx+cos(state(0))*cos(state(1))*Tz+(rou*V-m)*G)});
    }

    void solve();
    void updatePara();
    void getFirstCon();
    void updatex0(double roll, double pitch, double yaw);
    void updatexs();

    private:

    float h = 0.3; // step[s]
    int N = 10; // prediction horizon
    float I1 = 0.103; float I2 = 0.104; float I3 = 0.161; 
    float m = 4.8; // kg
    float V = 0.00285; // m3
    float rou = 1000; // kg/m3
    float G = 9.8; // m/s2
    float l = 0.6; // m
    float c1 = 0.01; float c2 = 0.01; float c3 = 0.01;
    float k = 0.0000005; // 推力系数
    float c = 0.0000001; // 反扭系数

    Slice all;
    // states
    SX phi = SX::sym("phi");        SX p = SX::sym("p");
    SX theta = SX::sym("theta");    SX q = SX::sym("q");
    SX psi = SX::sym("psi");        SX r = SX::sym("r");
    SX x = SX::sym("x");            SX u = SX::sym("u");
    SX y = SX::sym("y");            SX v = SX::sym("v");
    SX z = SX::sym("z");            SX w = SX::sym("w");
    SX state = DM::vertcat({phi, theta, psi, p, q, r, 
                                    x, y, z, u, v, w});
    int n_state = 12;
    
    // controls
    SX T1 = SX::sym("T1");          SX T4 = SX::sym("T4");
    SX T2 = SX::sym("T2");          SX T5 = SX::sym("T5");
    SX T3 = SX::sym("T3");          SX T6 = SX::sym("T6");
    SX T7 = SX::sym("T7");          SX T8 = SX::sym("T8");
    SX alpha = SX::sym("alpha");    SX beta = SX::sym("beta");
    SX control = DM::vertcat({T1, T2, T3, T4, T5, T6, 
                                    T7, T8, alpha, beta});
    int n_control = 10;

    // parameter


    SX U = SX::sym("U", n_control, N); // Decision variables (controls)
    // parameters (which include the initial state and the reference state)
    SX P = SX::sym("P", 2*n_state); 
    // A vector that represents the states over the optimization problem.
    SX X = SX::sym("X", n_state, (N+1));
    // initialization of the states decision variables
    SX X0 = SX::sym("X0", 1, N+1);
    // 3 control inputs for each robot
    SX u0 = SX::zeros(N, 10);

    SX obj = 0; // objective function
    SX g = SX::sym("g", n_state*(N+1)+3*N); // constraints vector

    SX Q = SX::zeros(n_state, n_state); // weighing matrices (states)
    SX R = SX::zeros(n_control, n_control); // weighing matrices (controls)
    SX st = SX::sym("st", n_state); // initial state
    SX st_next = SX::sym("st_next", n_state);
    SX con = SX::sym("con", n_control);
    SX OPT_variables = SX::sym("OPT_variables", 12*(N+1)+10*N);

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
    vector<double> state_upper_bound;
    vector<double> state_lower_bound;
    vector<double> con_upper_bound;
    vector<double> con_lower_bound;
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