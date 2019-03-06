/* Produced by CVXGEN, 2019-02-27 17:57:27 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H
/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif
namespace iiwa_ik_cvxgen {
    typedef struct Params_t {
        double damping[7];
        double qref[7];
        double slack[6];
        double J_1[7];
        double dx[6];
        double J_2[7];
        double J_3[7];
        double J_4[7];
        double J_5[7];
        double J_6[7];
        double qlow[7];
        double qup[7];
        double* J[7];
    } Params;

    typedef struct Vars_t {
        double* dq; /* 7 rows. */
        double* delta; /* 6 rows. */
    } Vars;

    typedef struct Workspace_t {
        double h[14];
        double s_inv[14];
        double s_inv_z[14];
        double b[6];
        double q[13];
        double rhs[47];
        double x[47];
        double* s;
        double* z;
        double* y;
        double lhs_aff[47];
        double lhs_cc[47];
        double buffer[47];
        double buffer2[47];
        double KKT[117];
        double L[91];
        double d[47];
        double v[47];
        double d_inv[47];
        double gap;
        double optval;
        double ineq_resid_squared;
        double eq_resid_squared;
        double block_33[1];
        /* Pre-op symbols. */
        double quad_568441778176[1];
        int converged;
    } Workspace;

    typedef struct Settings_t {
        double resid_tol;
        double eps;
        int max_iters;
        int refine_steps;
        int better_start;
        /* Better start obviates the need for s_init and z_init. */
        double s_init;
        double z_init;
        int verbose;
        /* Show extra details of the iterative refinement steps. */
        int verbose_refinement;
        int debug;
        /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
        double kkt_reg;
    } Settings;

    extern Vars vars;
    extern Params params;
    extern Workspace work;
    extern Settings settings;

    /* Function definitions in ldl.c: */
    void ldl_solve(double* target, double* var);
    void ldl_factor(void);
    double check_factorization(void);
    void matrix_multiply(double* result, double* source);
    double check_residual(double* target, double* multiplicand);
    void fill_KKT(void);

    /* Function definitions in matrix_support.c: */
    void multbymA(double* lhs, double* rhs);
    void multbymAT(double* lhs, double* rhs);
    void multbymG(double* lhs, double* rhs);
    void multbymGT(double* lhs, double* rhs);
    void multbyP(double* lhs, double* rhs);
    void fillq(void);
    void fillh(void);
    void fillb(void);
    void pre_ops(void);

    /* Function definitions in solver.c: */
    double eval_gap(void);
    void set_defaults(void);
    void setup_pointers(void);
    void setup_indexed_params(void);
    void setup_indexing(void);
    void set_start(void);
    double eval_objv(void);
    void fillrhs_aff(void);
    void fillrhs_cc(void);
    void refine(double* target, double* var);
    double calc_ineq_resid_squared(void);
    double calc_eq_resid_squared(void);
    void better_start(void);
    void fillrhs_start(void);
    long solve(void);

    /* Function definitions in util.c: */
    void tic(void);
    float toc(void);
    float tocq(void);
    void printmatrix(char* name, double* A, int m, int n, int sparse);
    double unif(double lower, double upper);
    float ran1(long* idum, int reset);
    float randn_internal(long* idum, int reset);
    double randn(void);
    void reset_rand(void);
} // namespace iiwa_ik_cvxgen

#endif
