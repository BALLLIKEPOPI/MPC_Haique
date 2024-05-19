#include "Control_Allocate.h"
#include <asm-generic/errno.h>
#include <casadi/core/function.hpp>
#include <casadi/core/generic_matrix.hpp>
#include <casadi/core/mx.hpp>
#include <casadi/core/nlpsol.hpp>
#include <casadi/core/optistack.hpp>
#include <casadi/core/slice.hpp>
#include <casadi/core/sparsity_interface.hpp>
#include <casadi/core/sx_fwd.hpp>
#include <vector>

CTL_ALC::CTL_ALC(){

    Q(0, 0) = 5; Q(1, 1) = 5; 
    R(0, 0) = 0; R(1, 1) = 0;
    R(2, 2) = 5; R(3, 3) = 0;
    R(4, 4) = 5; R(5, 5) = 0;

    st = X(all, 0);
    g(Slice(0, 2)) = st - P(Slice(0, 2));

    for(int i = 0; i < N; i++){
        st = X(all, i);
        con = U(all, i);
        obj += SX::mtimes({(st-P(Slice(2, 4))).T(), Q, (st-P(Slice(2, 4)))}) +
                SX::mtimes({con.T(), R, con});
        st_next = X(all, i+1);
        // RK4
        k1 = Dyna_Func(st, con);
        k2 = Dyna_Func(st + h/2*k1, con);
        k3 = Dyna_Func(st + h/2*k2, con);
        k4 = Dyna_Func(st + h*k3, con);
        st_next_RK4 = st + h/6*(k1+k2+k3+k4);
        // compute constraints 
        g(Slice((i+1)*5-3, (i+1)*5-1)) = st_next - st_next_RK4;
        g((i+1)*5-1) = con(0) - con(2);
        g((i+1)*5) = con(1) - con(3);
        g((i+1)*5+1) = con(4)*con(5);
    }
    // make the decision variable one column vector
    OPT_variables = SX::vertcat({SX::reshape(X, 2*(N+1), 1), SX::reshape(U, 6*N, 1)});
    SXDict nlp = {{"x", OPT_variables}, {"f", obj}, {"g", g}, {"p", P}};
    Dict opts = { {"ipopt.max_iter", 99}, {"ipopt.print_level", 0}, {"print_time", 1}, 
                    {"ipopt.acceptable_tol", 1e-8}, {"ipopt.acceptable_obj_change_tol", 1e-6}};
    solver = nlpsol("solver", "ipopt", nlp, opts);

    // Initial guess and bounds for the optimization variables
    x0 = {0.0, 0.0}; // initial state
    xs = {0.0, 0.0}; // desire state
    X0 = SX::repmat(x0, 1, N+1);
    
    for(int i = 0; i < 2*(N+1); i++){
        if(i % 2 == 0){
            lbx.push_back(-15); // state tx lower bound
            ubx.push_back(15); // state tx upper bound
        }
        else{
            lbx.push_back(0); // state T2 lower bound
            ubx.push_back(100); // state T2 upper bound
        }
    }
    for(int i = 0; i < 6*N; i++){
        if((i % 6 == 4) || (i % 6 == 5)){
            lbx.push_back(0); // alpha beta lower bound
            ubx.push_back(pi/2); // alpha beta upper bound
        }
        else{
            lbx.push_back(-300); // w2 w4 w6 w8 lower bound
            ubx.push_back(300); // w2 w4 w6 w8 upper bound
        }
    }
    lbg.insert(lbg.begin(), 5*(N+1)-3, 0);
    ubg.insert(ubg.begin(), 5*(N+1)-3, 0);

    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    arg["lbg"] = lbg;
    arg["ubg"] = ubg;
}

CTL_ALC::~CTL_ALC(){
}

void CTL_ALC::solve(){
    updatePara(); // set the values of the parameters vector
    arg["p"] = para;
    // initial value of the optimization variables
    arg["x0"] = SX::vertcat({SX::reshape(X0.T(), 2*(N+1), 1),
                                SX::reshape(u0.T(), 6*N, 1)});
    res = solver(arg);
    getFirstCon();
}

void CTL_ALC::updatePara(){
    para.clear();
    para.insert(para.end(), x0.begin(), x0.end());
    para.insert(para.end(), xs.begin(), xs.end());
}

void CTL_ALC::getFirstCon(){
    u_f.clear();
    vector<double> u_f_ = res.at("x").get_elements();
    u_f.insert(u_f.begin(), u_f_.begin()+2*(N+1), u_f_.begin()+6+2*(N+1));
    cout << "uf0: " << u_f[0] 
        << "  uf1: " << u_f[1] 
        << "  uf2: " << u_f[2] 
        << "  uf3: " << u_f[3] 
        << "  uf4: " << u_f[4] 
        << "  uf5: " << u_f[5] 
        << endl;
}

void CTL_ALC::updatex0(double tx, double T2){
    x_last = x0;
    x0.clear();
    x0.push_back(tx);
    x0.push_back(T2);
}

void CTL_ALC::updatexs(){

}