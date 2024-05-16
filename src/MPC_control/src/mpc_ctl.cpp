#include "mpc_ctl.h"
#include <asm-generic/errno.h>
#include <casadi/core/function.hpp>
#include <casadi/core/generic_matrix.hpp>
#include <casadi/core/mx.hpp>
#include <casadi/core/nlpsol.hpp>
#include <casadi/core/optistack.hpp>
#include <casadi/core/slice.hpp>
#include <casadi/core/sparsity_interface.hpp>
#include <casadi/core/sx_fwd.hpp>
#include <iterator>
#include <vector>

MPC_CTL::MPC_CTL(){

    Q(0, 0) = 10; R(0, 0) = 0;
    Q(1, 1) = 10; R(1, 1) = 0;
    Q(2, 2) = 10; R(2, 2) = 0;

    st = X(all, 0);
    abs_st_dot = abs(P(Slice(6, 9)));
    g(Slice(0, 3)) = st - P(Slice(0, 3));

    for(int i = 0; i < N; i++){
        st = X(all, i);
        con = U(all, i);
        obj += SX::mtimes({(st-P(Slice(3, 6))).T(), Q, (st-P(Slice(3, 6)))}) +
                SX::mtimes({con.T(), R, con});
        st_next = X(all, i+1);
        // RK4
        k1 = Dyna_Func(st, con, abs_st_dot);
        k2 = Dyna_Func(st + h/2*k1, con, abs_st_dot);
        k3 = Dyna_Func(st + h/2*k2, con, abs_st_dot);
        k4 = Dyna_Func(st + h*k3, con, abs_st_dot);
        st_next_RK4 = st + h/6*(k1+k2+k3+k4);
        abs_st_dot = SX::abs(st_next - st)/h;
        // compute constraints 
        g(Slice((i+1)*3, (i+2)*3)) = st_next - st_next_RK4;
    }
    // make the decision variable one column vector
    OPT_variables = SX::vertcat({SX::reshape(X, 3*(N+1), 1), SX::reshape(U, 3*N, 1)});
    SXDict nlp = {{"x", OPT_variables}, {"f", obj}, {"g", g}, {"p", P}};
    solver = nlpsol("solver", "ipopt", nlp);

    // Initial guess and bounds for the optimization variables
    x0 = {0.0, 0.0, 0.0};
    xs = {0.0, 0.0, 0.0};
    x_dot = {0.0, 0.0, 0.0};
    X0 = SX::repmat(x0, 1, N+1);
    
    lbx.insert(lbx.begin(), 3*(N+1), -pi/2);
    lbx.insert((lbx.begin()+3*(N+1)), 3*N, -15);
    ubx.insert(ubx.begin(), 3*(N+1), pi/2);
    ubx.insert((ubx.begin()+3*(N+1)), 3*N, 15);
    lbg.insert(lbg.begin(), 3*(N+1), 0);
    ubg.insert(ubg.begin(), 3*(N+1), 0);

    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    arg["lbg"] = lbg;
    arg["ubg"] = ubg;
}

MPC_CTL::~MPC_CTL(){
}

void MPC_CTL::solve(){
    updatePara(); // set the values of the parameters vector
    arg["p"] = para;
    // initial value of the optimization variables
    arg["x0"] = SX::vertcat({SX::reshape(X0.T(), 3*(N+1), 1),
                                SX::reshape(u0.T(), 3*N, 1)});
    res = solver(arg);
    cout << res << endl;
}

void MPC_CTL::updatePara(){
    para.clear();
    para.insert(para.end(), x0.begin(), x0.end());
    para.insert(para.end(), xs.begin(), xs.end());
    para.insert(para.end(), x_dot.begin(), x_dot.end());
}