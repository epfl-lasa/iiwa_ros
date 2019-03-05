/* Produced by CVXGEN, 2019-02-27 17:57:27 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
namespace iiwa_ik_cvxgen {
    void multbymA(double* lhs, double* rhs)
    {
        lhs[0] = -rhs[0] * (-1) - rhs[6] * (params.J_1[0]) - rhs[7] * (params.J_1[1]) - rhs[8] * (params.J_1[2]) - rhs[9] * (params.J_1[3]) - rhs[10] * (params.J_1[4]) - rhs[11] * (params.J_1[5]) - rhs[12] * (params.J_1[6]);
        lhs[1] = -rhs[1] * (-1) - rhs[6] * (params.J_2[0]) - rhs[7] * (params.J_2[1]) - rhs[8] * (params.J_2[2]) - rhs[9] * (params.J_2[3]) - rhs[10] * (params.J_2[4]) - rhs[11] * (params.J_2[5]) - rhs[12] * (params.J_2[6]);
        lhs[2] = -rhs[2] * (-1) - rhs[6] * (params.J_3[0]) - rhs[7] * (params.J_3[1]) - rhs[8] * (params.J_3[2]) - rhs[9] * (params.J_3[3]) - rhs[10] * (params.J_3[4]) - rhs[11] * (params.J_3[5]) - rhs[12] * (params.J_3[6]);
        lhs[3] = -rhs[3] * (-1) - rhs[6] * (params.J_4[0]) - rhs[7] * (params.J_4[1]) - rhs[8] * (params.J_4[2]) - rhs[9] * (params.J_4[3]) - rhs[10] * (params.J_4[4]) - rhs[11] * (params.J_4[5]) - rhs[12] * (params.J_4[6]);
        lhs[4] = -rhs[4] * (-1) - rhs[6] * (params.J_5[0]) - rhs[7] * (params.J_5[1]) - rhs[8] * (params.J_5[2]) - rhs[9] * (params.J_5[3]) - rhs[10] * (params.J_5[4]) - rhs[11] * (params.J_5[5]) - rhs[12] * (params.J_5[6]);
        lhs[5] = -rhs[5] * (-1) - rhs[6] * (params.J_6[0]) - rhs[7] * (params.J_6[1]) - rhs[8] * (params.J_6[2]) - rhs[9] * (params.J_6[3]) - rhs[10] * (params.J_6[4]) - rhs[11] * (params.J_6[5]) - rhs[12] * (params.J_6[6]);
    }

    void multbymAT(double* lhs, double* rhs)
    {
        lhs[0] = -rhs[0] * (-1);
        lhs[1] = -rhs[1] * (-1);
        lhs[2] = -rhs[2] * (-1);
        lhs[3] = -rhs[3] * (-1);
        lhs[4] = -rhs[4] * (-1);
        lhs[5] = -rhs[5] * (-1);
        lhs[6] = -rhs[0] * (params.J_1[0]) - rhs[1] * (params.J_2[0]) - rhs[2] * (params.J_3[0]) - rhs[3] * (params.J_4[0]) - rhs[4] * (params.J_5[0]) - rhs[5] * (params.J_6[0]);
        lhs[7] = -rhs[0] * (params.J_1[1]) - rhs[1] * (params.J_2[1]) - rhs[2] * (params.J_3[1]) - rhs[3] * (params.J_4[1]) - rhs[4] * (params.J_5[1]) - rhs[5] * (params.J_6[1]);
        lhs[8] = -rhs[0] * (params.J_1[2]) - rhs[1] * (params.J_2[2]) - rhs[2] * (params.J_3[2]) - rhs[3] * (params.J_4[2]) - rhs[4] * (params.J_5[2]) - rhs[5] * (params.J_6[2]);
        lhs[9] = -rhs[0] * (params.J_1[3]) - rhs[1] * (params.J_2[3]) - rhs[2] * (params.J_3[3]) - rhs[3] * (params.J_4[3]) - rhs[4] * (params.J_5[3]) - rhs[5] * (params.J_6[3]);
        lhs[10] = -rhs[0] * (params.J_1[4]) - rhs[1] * (params.J_2[4]) - rhs[2] * (params.J_3[4]) - rhs[3] * (params.J_4[4]) - rhs[4] * (params.J_5[4]) - rhs[5] * (params.J_6[4]);
        lhs[11] = -rhs[0] * (params.J_1[5]) - rhs[1] * (params.J_2[5]) - rhs[2] * (params.J_3[5]) - rhs[3] * (params.J_4[5]) - rhs[4] * (params.J_5[5]) - rhs[5] * (params.J_6[5]);
        lhs[12] = -rhs[0] * (params.J_1[6]) - rhs[1] * (params.J_2[6]) - rhs[2] * (params.J_3[6]) - rhs[3] * (params.J_4[6]) - rhs[4] * (params.J_5[6]) - rhs[5] * (params.J_6[6]);
    }

    void multbymG(double* lhs, double* rhs)
    {
        lhs[0] = -rhs[6] * (-1);
        lhs[1] = -rhs[7] * (-1);
        lhs[2] = -rhs[8] * (-1);
        lhs[3] = -rhs[9] * (-1);
        lhs[4] = -rhs[10] * (-1);
        lhs[5] = -rhs[11] * (-1);
        lhs[6] = -rhs[12] * (-1);
        lhs[7] = -rhs[6] * (1);
        lhs[8] = -rhs[7] * (1);
        lhs[9] = -rhs[8] * (1);
        lhs[10] = -rhs[9] * (1);
        lhs[11] = -rhs[10] * (1);
        lhs[12] = -rhs[11] * (1);
        lhs[13] = -rhs[12] * (1);
    }

    void multbymGT(double* lhs, double* rhs)
    {
        lhs[0] = 0;
        lhs[1] = 0;
        lhs[2] = 0;
        lhs[3] = 0;
        lhs[4] = 0;
        lhs[5] = 0;
        lhs[6] = -rhs[0] * (-1) - rhs[7] * (1);
        lhs[7] = -rhs[1] * (-1) - rhs[8] * (1);
        lhs[8] = -rhs[2] * (-1) - rhs[9] * (1);
        lhs[9] = -rhs[3] * (-1) - rhs[10] * (1);
        lhs[10] = -rhs[4] * (-1) - rhs[11] * (1);
        lhs[11] = -rhs[5] * (-1) - rhs[12] * (1);
        lhs[12] = -rhs[6] * (-1) - rhs[13] * (1);
    }

    void multbyP(double* lhs, double* rhs)
    {
        /* TODO use the fact that P is symmetric? */
        /* TODO check doubling / half factor etc. */
        lhs[0] = rhs[0] * (2 * params.slack[0]);
        lhs[1] = rhs[1] * (2 * params.slack[1]);
        lhs[2] = rhs[2] * (2 * params.slack[2]);
        lhs[3] = rhs[3] * (2 * params.slack[3]);
        lhs[4] = rhs[4] * (2 * params.slack[4]);
        lhs[5] = rhs[5] * (2 * params.slack[5]);
        lhs[6] = rhs[6] * (2 * params.damping[0]);
        lhs[7] = rhs[7] * (2 * params.damping[1]);
        lhs[8] = rhs[8] * (2 * params.damping[2]);
        lhs[9] = rhs[9] * (2 * params.damping[3]);
        lhs[10] = rhs[10] * (2 * params.damping[4]);
        lhs[11] = rhs[11] * (2 * params.damping[5]);
        lhs[12] = rhs[12] * (2 * params.damping[6]);
    }

    void fillq(void)
    {
        work.q[0] = 0;
        work.q[1] = 0;
        work.q[2] = 0;
        work.q[3] = 0;
        work.q[4] = 0;
        work.q[5] = 0;
        work.q[6] = 2 * params.damping[0] * params.qref[0];
        work.q[7] = 2 * params.damping[1] * params.qref[1];
        work.q[8] = 2 * params.damping[2] * params.qref[2];
        work.q[9] = 2 * params.damping[3] * params.qref[3];
        work.q[10] = 2 * params.damping[4] * params.qref[4];
        work.q[11] = 2 * params.damping[5] * params.qref[5];
        work.q[12] = 2 * params.damping[6] * params.qref[6];
    }

    void fillh(void)
    {
        work.h[0] = -(params.qlow[0] - params.qref[0]);
        work.h[1] = -(params.qlow[1] - params.qref[1]);
        work.h[2] = -(params.qlow[2] - params.qref[2]);
        work.h[3] = -(params.qlow[3] - params.qref[3]);
        work.h[4] = -(params.qlow[4] - params.qref[4]);
        work.h[5] = -(params.qlow[5] - params.qref[5]);
        work.h[6] = -(params.qlow[6] - params.qref[6]);
        work.h[7] = -(params.qref[0] - params.qup[0]);
        work.h[8] = -(params.qref[1] - params.qup[1]);
        work.h[9] = -(params.qref[2] - params.qup[2]);
        work.h[10] = -(params.qref[3] - params.qup[3]);
        work.h[11] = -(params.qref[4] - params.qup[4]);
        work.h[12] = -(params.qref[5] - params.qup[5]);
        work.h[13] = -(params.qref[6] - params.qup[6]);
    }

    void fillb(void)
    {
        work.b[0] = params.dx[0];
        work.b[1] = params.dx[1];
        work.b[2] = params.dx[2];
        work.b[3] = params.dx[3];
        work.b[4] = params.dx[4];
        work.b[5] = params.dx[5];
    }

    void pre_ops(void)
    {
        work.quad_568441778176[0] = params.qref[0] * params.damping[0] * params.qref[0] + params.qref[1] * params.damping[1] * params.qref[1] + params.qref[2] * params.damping[2] * params.qref[2] + params.qref[3] * params.damping[3] * params.qref[3] + params.qref[4] * params.damping[4] * params.qref[4] + params.qref[5] * params.damping[5] * params.qref[5] + params.qref[6] * params.damping[6] * params.qref[6];
    }
} // namespace iiwa_ik_cvxgen
