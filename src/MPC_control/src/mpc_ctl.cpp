#include "mpc_ctl.h"
#include <asm-generic/errno.h>
#include <casadi/core/function.hpp>
#include <casadi/core/nlpsol.hpp>
#include <casadi/core/slice.hpp>
#include <casadi/core/sx_fwd.hpp>
#include <vector>

MPC_CTL::MPC_CTL(){
    Q(0, 0) = 10; R(0, 0) = 0;
    Q(1, 1) = 10; R(1, 1) = 0;
    Q(2, 2) = 10; R(2, 2) = 0;

    st(Slice(0, n_state-1)) = X(Slice(0, n_state-1), 0);
    abs_st_dot = DM::abs(P(Slice(6, 8)));
    g(Slice(0, 2)) = st - P(Slice(0, 2));

    for(int i = 0; i < N; i++){
        st(Slice((i+1)*n_state, (i+2)*n_state-1)) = X(Slice(0, n_state-1), i);
        con(Slice((i+1)*n_control, (i+2)*n_control-1)) = U(Slice(0, n_state-1), i);
        obj = obj + (st-P(Slice(3, 5))).T()*Q*(st-P(Slice(3, 5))) +
                con.T()*R*con;
        st_next = X(Slice(0, n_state-1), i+1);
        // RK4
        k1 = Dyna_Func(st, con, abs_st_dot);
        k2 = Dyna_Func(st + h/2*k1, con, abs_st_dot);
        k3 = Dyna_Func(st + h/2*k2, con, abs_st_dot);
        k4 = Dyna_Func(st + h*k3, con, abs_st_dot);
        st_next_RK4 = st + h/6*(k1+k2+k3+k4);
        abs_st_dot = DM::abs(st_next - st)/h;
        // compute constraints 
        g(Slice((i+1)*3, (i+2)*3-1)) = st_next - st_next_RK4;
    }
    // make the decision variable one column vector
    OPT_variables = DM::vertcat({DM::reshape(X, 3*(N+1), 1), DM::reshape(U, 3*N, 1)});
    SXDict nlp = {{"x", OPT_variables}, {"f", obj}, {"g", g}};
    Function slover = nlpsol("solver", "ipopt", nlp);

    // Initial guess and bounds for the optimization variables
    x0 = {0.0, 0.0, 0.0};
    for(int i = 0; i < 3*(N+1)-1; i++){
        lbg.push_back(0);
        ubg.push_back(0);
    }
    for(int i = 0; i < 3*(N+1)-1; i++){
        if(i % 2 == 0)
            lbx.push_back(-pi/2);
        else
            lbx.push_back(pi/2);
    }
    for(int i = 0; i < 3*N-1; i++){
        if(i % 2 == 0)
            lbx.push_back(-15);
        else
            lbx.push_back(15);
    }
}

MPC_CTL::~MPC_CTL(){
}
