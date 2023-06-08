//|
//|    Copyright (C) 2019-2022 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Authors:  Konstantinos Chatzilygeroudis (maintainer)
//|              Matthias Mayr
//|              Bernardo Fichera
//|    email:    costashatz@gmail.com
//|              matthias.mayr@cs.lth.se
//|              bernardo.fichera@epfl.ch
//|    Other contributors:
//|              Yoan Mollard (yoan@aubrune.eu)
//|              Walid Amanhoud (walid.amanhoud@epfl.ch)
//|    website:  lasa.epfl.ch
//|
//|    This file is part of iiwa_ros.
//|
//|    iiwa_ros is free software: you can redistribute it and/or modify
//|    it under the terms of the GNU General Public License as published by
//|    the Free Software Foundation, either version 3 of the License, or
//|    (at your option) any later version.
//|
//|    iiwa_ros is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|
#ifndef IIWA_TOOLS_IIWA_TOOLS_CVXGEN_IIWA_IK_CVXGEN_HPP
#define IIWA_TOOLS_IIWA_TOOLS_CVXGEN_IIWA_IK_CVXGEN_HPP

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define IA 16807
#define IM 2147483647
#define AM (1.0 / IM)
#define IQ 127773
#define IR 2836
#define NTAB 32
#define NDIV (1 + (IM - 1) / NTAB)
#define EPS 1.2e-7
#define RNMX (1.0 - EPS)

namespace iiwa_ik_cvxgen {

    struct Params {
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
    };

    struct Vars {
        double* dq; /* 7 rows. */
        double* delta; /* 6 rows. */
    };

    struct Workspace {
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
    };

    struct Settings {
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
    };

    class Solver {
    public:
        Params params;
        Vars vars;
        Workspace work;
        Settings settings;

        long global_seed = 1;

        void ldl_solve(double* target, double* var)
        {
            int i;
            /* Find var = (L*diag(work.d)*L') \ target, then unpermute. */
            /* Answer goes into var. */
            /* Forward substitution. */
            /* Include permutation as we retrieve from target. Use v so we can unpermute */
            /* later. */
            work.v[0] = target[13];
            work.v[1] = target[14];
            work.v[2] = target[15];
            work.v[3] = target[16];
            work.v[4] = target[17];
            work.v[5] = target[18];
            work.v[6] = target[19];
            work.v[7] = target[20];
            work.v[8] = target[21];
            work.v[9] = target[22];
            work.v[10] = target[23];
            work.v[11] = target[24];
            work.v[12] = target[25];
            work.v[13] = target[26];
            work.v[14] = target[27] - work.L[0] * work.v[0];
            work.v[15] = target[28] - work.L[1] * work.v[1];
            work.v[16] = target[29] - work.L[2] * work.v[2];
            work.v[17] = target[30] - work.L[3] * work.v[3];
            work.v[18] = target[31] - work.L[4] * work.v[4];
            work.v[19] = target[32] - work.L[5] * work.v[5];
            work.v[20] = target[33] - work.L[6] * work.v[6];
            work.v[21] = target[34] - work.L[7] * work.v[7];
            work.v[22] = target[35] - work.L[8] * work.v[8];
            work.v[23] = target[36] - work.L[9] * work.v[9];
            work.v[24] = target[37] - work.L[10] * work.v[10];
            work.v[25] = target[38] - work.L[11] * work.v[11];
            work.v[26] = target[39] - work.L[12] * work.v[12];
            work.v[27] = target[40] - work.L[13] * work.v[13];
            work.v[28] = target[0];
            work.v[29] = target[1];
            work.v[30] = target[2];
            work.v[31] = target[3];
            work.v[32] = target[4];
            work.v[33] = target[5];
            work.v[34] = target[6] - work.L[14] * work.v[14] - work.L[15] * work.v[21];
            work.v[35] = target[7] - work.L[16] * work.v[15] - work.L[17] * work.v[22];
            work.v[36] = target[8] - work.L[18] * work.v[16] - work.L[19] * work.v[23];
            work.v[37] = target[9] - work.L[20] * work.v[17] - work.L[21] * work.v[24];
            work.v[38] = target[10] - work.L[22] * work.v[18] - work.L[23] * work.v[25];
            work.v[39] = target[11] - work.L[24] * work.v[19] - work.L[25] * work.v[26];
            work.v[40] = target[12] - work.L[26] * work.v[20] - work.L[27] * work.v[27];
            work.v[41] = target[41] - work.L[28] * work.v[28] - work.L[29] * work.v[34] - work.L[30] * work.v[35] - work.L[31] * work.v[36] - work.L[32] * work.v[37] - work.L[33] * work.v[38] - work.L[34] * work.v[39] - work.L[35] * work.v[40];
            work.v[42] = target[42] - work.L[36] * work.v[29] - work.L[37] * work.v[34] - work.L[38] * work.v[35] - work.L[39] * work.v[36] - work.L[40] * work.v[37] - work.L[41] * work.v[38] - work.L[42] * work.v[39] - work.L[43] * work.v[40] - work.L[44] * work.v[41];
            work.v[43] = target[43] - work.L[45] * work.v[30] - work.L[46] * work.v[34] - work.L[47] * work.v[35] - work.L[48] * work.v[36] - work.L[49] * work.v[37] - work.L[50] * work.v[38] - work.L[51] * work.v[39] - work.L[52] * work.v[40] - work.L[53] * work.v[41] - work.L[54] * work.v[42];
            work.v[44] = target[44] - work.L[55] * work.v[31] - work.L[56] * work.v[34] - work.L[57] * work.v[35] - work.L[58] * work.v[36] - work.L[59] * work.v[37] - work.L[60] * work.v[38] - work.L[61] * work.v[39] - work.L[62] * work.v[40] - work.L[63] * work.v[41] - work.L[64] * work.v[42] - work.L[65] * work.v[43];
            work.v[45] = target[45] - work.L[66] * work.v[32] - work.L[67] * work.v[34] - work.L[68] * work.v[35] - work.L[69] * work.v[36] - work.L[70] * work.v[37] - work.L[71] * work.v[38] - work.L[72] * work.v[39] - work.L[73] * work.v[40] - work.L[74] * work.v[41] - work.L[75] * work.v[42] - work.L[76] * work.v[43] - work.L[77] * work.v[44];
            work.v[46] = target[46] - work.L[78] * work.v[33] - work.L[79] * work.v[34] - work.L[80] * work.v[35] - work.L[81] * work.v[36] - work.L[82] * work.v[37] - work.L[83] * work.v[38] - work.L[84] * work.v[39] - work.L[85] * work.v[40] - work.L[86] * work.v[41] - work.L[87] * work.v[42] - work.L[88] * work.v[43] - work.L[89] * work.v[44] - work.L[90] * work.v[45];
            /* Diagonal scaling. Assume correctness of work.d_inv. */
            for (i = 0; i < 47; i++)
                work.v[i] *= work.d_inv[i];
            /* Back substitution */
            work.v[45] -= work.L[90] * work.v[46];
            work.v[44] -= work.L[77] * work.v[45] + work.L[89] * work.v[46];
            work.v[43] -= work.L[65] * work.v[44] + work.L[76] * work.v[45] + work.L[88] * work.v[46];
            work.v[42] -= work.L[54] * work.v[43] + work.L[64] * work.v[44] + work.L[75] * work.v[45] + work.L[87] * work.v[46];
            work.v[41] -= work.L[44] * work.v[42] + work.L[53] * work.v[43] + work.L[63] * work.v[44] + work.L[74] * work.v[45] + work.L[86] * work.v[46];
            work.v[40] -= work.L[35] * work.v[41] + work.L[43] * work.v[42] + work.L[52] * work.v[43] + work.L[62] * work.v[44] + work.L[73] * work.v[45] + work.L[85] * work.v[46];
            work.v[39] -= work.L[34] * work.v[41] + work.L[42] * work.v[42] + work.L[51] * work.v[43] + work.L[61] * work.v[44] + work.L[72] * work.v[45] + work.L[84] * work.v[46];
            work.v[38] -= work.L[33] * work.v[41] + work.L[41] * work.v[42] + work.L[50] * work.v[43] + work.L[60] * work.v[44] + work.L[71] * work.v[45] + work.L[83] * work.v[46];
            work.v[37] -= work.L[32] * work.v[41] + work.L[40] * work.v[42] + work.L[49] * work.v[43] + work.L[59] * work.v[44] + work.L[70] * work.v[45] + work.L[82] * work.v[46];
            work.v[36] -= work.L[31] * work.v[41] + work.L[39] * work.v[42] + work.L[48] * work.v[43] + work.L[58] * work.v[44] + work.L[69] * work.v[45] + work.L[81] * work.v[46];
            work.v[35] -= work.L[30] * work.v[41] + work.L[38] * work.v[42] + work.L[47] * work.v[43] + work.L[57] * work.v[44] + work.L[68] * work.v[45] + work.L[80] * work.v[46];
            work.v[34] -= work.L[29] * work.v[41] + work.L[37] * work.v[42] + work.L[46] * work.v[43] + work.L[56] * work.v[44] + work.L[67] * work.v[45] + work.L[79] * work.v[46];
            work.v[33] -= work.L[78] * work.v[46];
            work.v[32] -= work.L[66] * work.v[45];
            work.v[31] -= work.L[55] * work.v[44];
            work.v[30] -= work.L[45] * work.v[43];
            work.v[29] -= work.L[36] * work.v[42];
            work.v[28] -= work.L[28] * work.v[41];
            work.v[27] -= work.L[27] * work.v[40];
            work.v[26] -= work.L[25] * work.v[39];
            work.v[25] -= work.L[23] * work.v[38];
            work.v[24] -= work.L[21] * work.v[37];
            work.v[23] -= work.L[19] * work.v[36];
            work.v[22] -= work.L[17] * work.v[35];
            work.v[21] -= work.L[15] * work.v[34];
            work.v[20] -= work.L[26] * work.v[40];
            work.v[19] -= work.L[24] * work.v[39];
            work.v[18] -= work.L[22] * work.v[38];
            work.v[17] -= work.L[20] * work.v[37];
            work.v[16] -= work.L[18] * work.v[36];
            work.v[15] -= work.L[16] * work.v[35];
            work.v[14] -= work.L[14] * work.v[34];
            work.v[13] -= work.L[13] * work.v[27];
            work.v[12] -= work.L[12] * work.v[26];
            work.v[11] -= work.L[11] * work.v[25];
            work.v[10] -= work.L[10] * work.v[24];
            work.v[9] -= work.L[9] * work.v[23];
            work.v[8] -= work.L[8] * work.v[22];
            work.v[7] -= work.L[7] * work.v[21];
            work.v[6] -= work.L[6] * work.v[20];
            work.v[5] -= work.L[5] * work.v[19];
            work.v[4] -= work.L[4] * work.v[18];
            work.v[3] -= work.L[3] * work.v[17];
            work.v[2] -= work.L[2] * work.v[16];
            work.v[1] -= work.L[1] * work.v[15];
            work.v[0] -= work.L[0] * work.v[14];
            /* Unpermute the result, from v to var. */
            var[0] = work.v[28];
            var[1] = work.v[29];
            var[2] = work.v[30];
            var[3] = work.v[31];
            var[4] = work.v[32];
            var[5] = work.v[33];
            var[6] = work.v[34];
            var[7] = work.v[35];
            var[8] = work.v[36];
            var[9] = work.v[37];
            var[10] = work.v[38];
            var[11] = work.v[39];
            var[12] = work.v[40];
            var[13] = work.v[0];
            var[14] = work.v[1];
            var[15] = work.v[2];
            var[16] = work.v[3];
            var[17] = work.v[4];
            var[18] = work.v[5];
            var[19] = work.v[6];
            var[20] = work.v[7];
            var[21] = work.v[8];
            var[22] = work.v[9];
            var[23] = work.v[10];
            var[24] = work.v[11];
            var[25] = work.v[12];
            var[26] = work.v[13];
            var[27] = work.v[14];
            var[28] = work.v[15];
            var[29] = work.v[16];
            var[30] = work.v[17];
            var[31] = work.v[18];
            var[32] = work.v[19];
            var[33] = work.v[20];
            var[34] = work.v[21];
            var[35] = work.v[22];
            var[36] = work.v[23];
            var[37] = work.v[24];
            var[38] = work.v[25];
            var[39] = work.v[26];
            var[40] = work.v[27];
            var[41] = work.v[41];
            var[42] = work.v[42];
            var[43] = work.v[43];
            var[44] = work.v[44];
            var[45] = work.v[45];
            var[46] = work.v[46];
#ifndef ZERO_LIBRARY_MODE
            if (settings.debug) {
                printf("Squared norm for solution is %.8g.\n", check_residual(target, var));
            }
#endif
        }

        void ldl_factor(void)
        {
            work.d[0] = work.KKT[0];
            if (work.d[0] < 0)
                work.d[0] = settings.kkt_reg;
            else
                work.d[0] += settings.kkt_reg;
            work.d_inv[0] = 1 / work.d[0];
            work.L[0] = work.KKT[1] * work.d_inv[0];
            work.v[1] = work.KKT[2];
            work.d[1] = work.v[1];
            if (work.d[1] < 0)
                work.d[1] = settings.kkt_reg;
            else
                work.d[1] += settings.kkt_reg;
            work.d_inv[1] = 1 / work.d[1];
            work.L[1] = (work.KKT[3]) * work.d_inv[1];
            work.v[2] = work.KKT[4];
            work.d[2] = work.v[2];
            if (work.d[2] < 0)
                work.d[2] = settings.kkt_reg;
            else
                work.d[2] += settings.kkt_reg;
            work.d_inv[2] = 1 / work.d[2];
            work.L[2] = (work.KKT[5]) * work.d_inv[2];
            work.v[3] = work.KKT[6];
            work.d[3] = work.v[3];
            if (work.d[3] < 0)
                work.d[3] = settings.kkt_reg;
            else
                work.d[3] += settings.kkt_reg;
            work.d_inv[3] = 1 / work.d[3];
            work.L[3] = (work.KKT[7]) * work.d_inv[3];
            work.v[4] = work.KKT[8];
            work.d[4] = work.v[4];
            if (work.d[4] < 0)
                work.d[4] = settings.kkt_reg;
            else
                work.d[4] += settings.kkt_reg;
            work.d_inv[4] = 1 / work.d[4];
            work.L[4] = (work.KKT[9]) * work.d_inv[4];
            work.v[5] = work.KKT[10];
            work.d[5] = work.v[5];
            if (work.d[5] < 0)
                work.d[5] = settings.kkt_reg;
            else
                work.d[5] += settings.kkt_reg;
            work.d_inv[5] = 1 / work.d[5];
            work.L[5] = (work.KKT[11]) * work.d_inv[5];
            work.v[6] = work.KKT[12];
            work.d[6] = work.v[6];
            if (work.d[6] < 0)
                work.d[6] = settings.kkt_reg;
            else
                work.d[6] += settings.kkt_reg;
            work.d_inv[6] = 1 / work.d[6];
            work.L[6] = (work.KKT[13]) * work.d_inv[6];
            work.v[7] = work.KKT[14];
            work.d[7] = work.v[7];
            if (work.d[7] < 0)
                work.d[7] = settings.kkt_reg;
            else
                work.d[7] += settings.kkt_reg;
            work.d_inv[7] = 1 / work.d[7];
            work.L[7] = (work.KKT[15]) * work.d_inv[7];
            work.v[8] = work.KKT[16];
            work.d[8] = work.v[8];
            if (work.d[8] < 0)
                work.d[8] = settings.kkt_reg;
            else
                work.d[8] += settings.kkt_reg;
            work.d_inv[8] = 1 / work.d[8];
            work.L[8] = (work.KKT[17]) * work.d_inv[8];
            work.v[9] = work.KKT[18];
            work.d[9] = work.v[9];
            if (work.d[9] < 0)
                work.d[9] = settings.kkt_reg;
            else
                work.d[9] += settings.kkt_reg;
            work.d_inv[9] = 1 / work.d[9];
            work.L[9] = (work.KKT[19]) * work.d_inv[9];
            work.v[10] = work.KKT[20];
            work.d[10] = work.v[10];
            if (work.d[10] < 0)
                work.d[10] = settings.kkt_reg;
            else
                work.d[10] += settings.kkt_reg;
            work.d_inv[10] = 1 / work.d[10];
            work.L[10] = (work.KKT[21]) * work.d_inv[10];
            work.v[11] = work.KKT[22];
            work.d[11] = work.v[11];
            if (work.d[11] < 0)
                work.d[11] = settings.kkt_reg;
            else
                work.d[11] += settings.kkt_reg;
            work.d_inv[11] = 1 / work.d[11];
            work.L[11] = (work.KKT[23]) * work.d_inv[11];
            work.v[12] = work.KKT[24];
            work.d[12] = work.v[12];
            if (work.d[12] < 0)
                work.d[12] = settings.kkt_reg;
            else
                work.d[12] += settings.kkt_reg;
            work.d_inv[12] = 1 / work.d[12];
            work.L[12] = (work.KKT[25]) * work.d_inv[12];
            work.v[13] = work.KKT[26];
            work.d[13] = work.v[13];
            if (work.d[13] < 0)
                work.d[13] = settings.kkt_reg;
            else
                work.d[13] += settings.kkt_reg;
            work.d_inv[13] = 1 / work.d[13];
            work.L[13] = (work.KKT[27]) * work.d_inv[13];
            work.v[0] = work.L[0] * work.d[0];
            work.v[14] = work.KKT[28] - work.L[0] * work.v[0];
            work.d[14] = work.v[14];
            if (work.d[14] > 0)
                work.d[14] = -settings.kkt_reg;
            else
                work.d[14] -= settings.kkt_reg;
            work.d_inv[14] = 1 / work.d[14];
            work.L[14] = (work.KKT[29]) * work.d_inv[14];
            work.v[1] = work.L[1] * work.d[1];
            work.v[15] = work.KKT[30] - work.L[1] * work.v[1];
            work.d[15] = work.v[15];
            if (work.d[15] > 0)
                work.d[15] = -settings.kkt_reg;
            else
                work.d[15] -= settings.kkt_reg;
            work.d_inv[15] = 1 / work.d[15];
            work.L[16] = (work.KKT[31]) * work.d_inv[15];
            work.v[2] = work.L[2] * work.d[2];
            work.v[16] = work.KKT[32] - work.L[2] * work.v[2];
            work.d[16] = work.v[16];
            if (work.d[16] > 0)
                work.d[16] = -settings.kkt_reg;
            else
                work.d[16] -= settings.kkt_reg;
            work.d_inv[16] = 1 / work.d[16];
            work.L[18] = (work.KKT[33]) * work.d_inv[16];
            work.v[3] = work.L[3] * work.d[3];
            work.v[17] = work.KKT[34] - work.L[3] * work.v[3];
            work.d[17] = work.v[17];
            if (work.d[17] > 0)
                work.d[17] = -settings.kkt_reg;
            else
                work.d[17] -= settings.kkt_reg;
            work.d_inv[17] = 1 / work.d[17];
            work.L[20] = (work.KKT[35]) * work.d_inv[17];
            work.v[4] = work.L[4] * work.d[4];
            work.v[18] = work.KKT[36] - work.L[4] * work.v[4];
            work.d[18] = work.v[18];
            if (work.d[18] > 0)
                work.d[18] = -settings.kkt_reg;
            else
                work.d[18] -= settings.kkt_reg;
            work.d_inv[18] = 1 / work.d[18];
            work.L[22] = (work.KKT[37]) * work.d_inv[18];
            work.v[5] = work.L[5] * work.d[5];
            work.v[19] = work.KKT[38] - work.L[5] * work.v[5];
            work.d[19] = work.v[19];
            if (work.d[19] > 0)
                work.d[19] = -settings.kkt_reg;
            else
                work.d[19] -= settings.kkt_reg;
            work.d_inv[19] = 1 / work.d[19];
            work.L[24] = (work.KKT[39]) * work.d_inv[19];
            work.v[6] = work.L[6] * work.d[6];
            work.v[20] = work.KKT[40] - work.L[6] * work.v[6];
            work.d[20] = work.v[20];
            if (work.d[20] > 0)
                work.d[20] = -settings.kkt_reg;
            else
                work.d[20] -= settings.kkt_reg;
            work.d_inv[20] = 1 / work.d[20];
            work.L[26] = (work.KKT[41]) * work.d_inv[20];
            work.v[7] = work.L[7] * work.d[7];
            work.v[21] = work.KKT[42] - work.L[7] * work.v[7];
            work.d[21] = work.v[21];
            if (work.d[21] > 0)
                work.d[21] = -settings.kkt_reg;
            else
                work.d[21] -= settings.kkt_reg;
            work.d_inv[21] = 1 / work.d[21];
            work.L[15] = (work.KKT[43]) * work.d_inv[21];
            work.v[8] = work.L[8] * work.d[8];
            work.v[22] = work.KKT[44] - work.L[8] * work.v[8];
            work.d[22] = work.v[22];
            if (work.d[22] > 0)
                work.d[22] = -settings.kkt_reg;
            else
                work.d[22] -= settings.kkt_reg;
            work.d_inv[22] = 1 / work.d[22];
            work.L[17] = (work.KKT[45]) * work.d_inv[22];
            work.v[9] = work.L[9] * work.d[9];
            work.v[23] = work.KKT[46] - work.L[9] * work.v[9];
            work.d[23] = work.v[23];
            if (work.d[23] > 0)
                work.d[23] = -settings.kkt_reg;
            else
                work.d[23] -= settings.kkt_reg;
            work.d_inv[23] = 1 / work.d[23];
            work.L[19] = (work.KKT[47]) * work.d_inv[23];
            work.v[10] = work.L[10] * work.d[10];
            work.v[24] = work.KKT[48] - work.L[10] * work.v[10];
            work.d[24] = work.v[24];
            if (work.d[24] > 0)
                work.d[24] = -settings.kkt_reg;
            else
                work.d[24] -= settings.kkt_reg;
            work.d_inv[24] = 1 / work.d[24];
            work.L[21] = (work.KKT[49]) * work.d_inv[24];
            work.v[11] = work.L[11] * work.d[11];
            work.v[25] = work.KKT[50] - work.L[11] * work.v[11];
            work.d[25] = work.v[25];
            if (work.d[25] > 0)
                work.d[25] = -settings.kkt_reg;
            else
                work.d[25] -= settings.kkt_reg;
            work.d_inv[25] = 1 / work.d[25];
            work.L[23] = (work.KKT[51]) * work.d_inv[25];
            work.v[12] = work.L[12] * work.d[12];
            work.v[26] = work.KKT[52] - work.L[12] * work.v[12];
            work.d[26] = work.v[26];
            if (work.d[26] > 0)
                work.d[26] = -settings.kkt_reg;
            else
                work.d[26] -= settings.kkt_reg;
            work.d_inv[26] = 1 / work.d[26];
            work.L[25] = (work.KKT[53]) * work.d_inv[26];
            work.v[13] = work.L[13] * work.d[13];
            work.v[27] = work.KKT[54] - work.L[13] * work.v[13];
            work.d[27] = work.v[27];
            if (work.d[27] > 0)
                work.d[27] = -settings.kkt_reg;
            else
                work.d[27] -= settings.kkt_reg;
            work.d_inv[27] = 1 / work.d[27];
            work.L[27] = (work.KKT[55]) * work.d_inv[27];
            work.v[28] = work.KKT[56];
            work.d[28] = work.v[28];
            if (work.d[28] < 0)
                work.d[28] = settings.kkt_reg;
            else
                work.d[28] += settings.kkt_reg;
            work.d_inv[28] = 1 / work.d[28];
            work.L[28] = (work.KKT[57]) * work.d_inv[28];
            work.v[29] = work.KKT[58];
            work.d[29] = work.v[29];
            if (work.d[29] < 0)
                work.d[29] = settings.kkt_reg;
            else
                work.d[29] += settings.kkt_reg;
            work.d_inv[29] = 1 / work.d[29];
            work.L[36] = (work.KKT[59]) * work.d_inv[29];
            work.v[30] = work.KKT[60];
            work.d[30] = work.v[30];
            if (work.d[30] < 0)
                work.d[30] = settings.kkt_reg;
            else
                work.d[30] += settings.kkt_reg;
            work.d_inv[30] = 1 / work.d[30];
            work.L[45] = (work.KKT[61]) * work.d_inv[30];
            work.v[31] = work.KKT[62];
            work.d[31] = work.v[31];
            if (work.d[31] < 0)
                work.d[31] = settings.kkt_reg;
            else
                work.d[31] += settings.kkt_reg;
            work.d_inv[31] = 1 / work.d[31];
            work.L[55] = (work.KKT[63]) * work.d_inv[31];
            work.v[32] = work.KKT[64];
            work.d[32] = work.v[32];
            if (work.d[32] < 0)
                work.d[32] = settings.kkt_reg;
            else
                work.d[32] += settings.kkt_reg;
            work.d_inv[32] = 1 / work.d[32];
            work.L[66] = (work.KKT[65]) * work.d_inv[32];
            work.v[33] = work.KKT[66];
            work.d[33] = work.v[33];
            if (work.d[33] < 0)
                work.d[33] = settings.kkt_reg;
            else
                work.d[33] += settings.kkt_reg;
            work.d_inv[33] = 1 / work.d[33];
            work.L[78] = (work.KKT[67]) * work.d_inv[33];
            work.v[14] = work.L[14] * work.d[14];
            work.v[21] = work.L[15] * work.d[21];
            work.v[34] = work.KKT[68] - work.L[14] * work.v[14] - work.L[15] * work.v[21];
            work.d[34] = work.v[34];
            if (work.d[34] < 0)
                work.d[34] = settings.kkt_reg;
            else
                work.d[34] += settings.kkt_reg;
            work.d_inv[34] = 1 / work.d[34];
            work.L[29] = (work.KKT[69]) * work.d_inv[34];
            work.L[37] = (work.KKT[70]) * work.d_inv[34];
            work.L[46] = (work.KKT[71]) * work.d_inv[34];
            work.L[56] = (work.KKT[72]) * work.d_inv[34];
            work.L[67] = (work.KKT[73]) * work.d_inv[34];
            work.L[79] = (work.KKT[74]) * work.d_inv[34];
            work.v[15] = work.L[16] * work.d[15];
            work.v[22] = work.L[17] * work.d[22];
            work.v[35] = work.KKT[75] - work.L[16] * work.v[15] - work.L[17] * work.v[22];
            work.d[35] = work.v[35];
            if (work.d[35] < 0)
                work.d[35] = settings.kkt_reg;
            else
                work.d[35] += settings.kkt_reg;
            work.d_inv[35] = 1 / work.d[35];
            work.L[30] = (work.KKT[76]) * work.d_inv[35];
            work.L[38] = (work.KKT[77]) * work.d_inv[35];
            work.L[47] = (work.KKT[78]) * work.d_inv[35];
            work.L[57] = (work.KKT[79]) * work.d_inv[35];
            work.L[68] = (work.KKT[80]) * work.d_inv[35];
            work.L[80] = (work.KKT[81]) * work.d_inv[35];
            work.v[16] = work.L[18] * work.d[16];
            work.v[23] = work.L[19] * work.d[23];
            work.v[36] = work.KKT[82] - work.L[18] * work.v[16] - work.L[19] * work.v[23];
            work.d[36] = work.v[36];
            if (work.d[36] < 0)
                work.d[36] = settings.kkt_reg;
            else
                work.d[36] += settings.kkt_reg;
            work.d_inv[36] = 1 / work.d[36];
            work.L[31] = (work.KKT[83]) * work.d_inv[36];
            work.L[39] = (work.KKT[84]) * work.d_inv[36];
            work.L[48] = (work.KKT[85]) * work.d_inv[36];
            work.L[58] = (work.KKT[86]) * work.d_inv[36];
            work.L[69] = (work.KKT[87]) * work.d_inv[36];
            work.L[81] = (work.KKT[88]) * work.d_inv[36];
            work.v[17] = work.L[20] * work.d[17];
            work.v[24] = work.L[21] * work.d[24];
            work.v[37] = work.KKT[89] - work.L[20] * work.v[17] - work.L[21] * work.v[24];
            work.d[37] = work.v[37];
            if (work.d[37] < 0)
                work.d[37] = settings.kkt_reg;
            else
                work.d[37] += settings.kkt_reg;
            work.d_inv[37] = 1 / work.d[37];
            work.L[32] = (work.KKT[90]) * work.d_inv[37];
            work.L[40] = (work.KKT[91]) * work.d_inv[37];
            work.L[49] = (work.KKT[92]) * work.d_inv[37];
            work.L[59] = (work.KKT[93]) * work.d_inv[37];
            work.L[70] = (work.KKT[94]) * work.d_inv[37];
            work.L[82] = (work.KKT[95]) * work.d_inv[37];
            work.v[18] = work.L[22] * work.d[18];
            work.v[25] = work.L[23] * work.d[25];
            work.v[38] = work.KKT[96] - work.L[22] * work.v[18] - work.L[23] * work.v[25];
            work.d[38] = work.v[38];
            if (work.d[38] < 0)
                work.d[38] = settings.kkt_reg;
            else
                work.d[38] += settings.kkt_reg;
            work.d_inv[38] = 1 / work.d[38];
            work.L[33] = (work.KKT[97]) * work.d_inv[38];
            work.L[41] = (work.KKT[98]) * work.d_inv[38];
            work.L[50] = (work.KKT[99]) * work.d_inv[38];
            work.L[60] = (work.KKT[100]) * work.d_inv[38];
            work.L[71] = (work.KKT[101]) * work.d_inv[38];
            work.L[83] = (work.KKT[102]) * work.d_inv[38];
            work.v[19] = work.L[24] * work.d[19];
            work.v[26] = work.L[25] * work.d[26];
            work.v[39] = work.KKT[103] - work.L[24] * work.v[19] - work.L[25] * work.v[26];
            work.d[39] = work.v[39];
            if (work.d[39] < 0)
                work.d[39] = settings.kkt_reg;
            else
                work.d[39] += settings.kkt_reg;
            work.d_inv[39] = 1 / work.d[39];
            work.L[34] = (work.KKT[104]) * work.d_inv[39];
            work.L[42] = (work.KKT[105]) * work.d_inv[39];
            work.L[51] = (work.KKT[106]) * work.d_inv[39];
            work.L[61] = (work.KKT[107]) * work.d_inv[39];
            work.L[72] = (work.KKT[108]) * work.d_inv[39];
            work.L[84] = (work.KKT[109]) * work.d_inv[39];
            work.v[20] = work.L[26] * work.d[20];
            work.v[27] = work.L[27] * work.d[27];
            work.v[40] = work.KKT[110] - work.L[26] * work.v[20] - work.L[27] * work.v[27];
            work.d[40] = work.v[40];
            if (work.d[40] < 0)
                work.d[40] = settings.kkt_reg;
            else
                work.d[40] += settings.kkt_reg;
            work.d_inv[40] = 1 / work.d[40];
            work.L[35] = (work.KKT[111]) * work.d_inv[40];
            work.L[43] = (work.KKT[112]) * work.d_inv[40];
            work.L[52] = (work.KKT[113]) * work.d_inv[40];
            work.L[62] = (work.KKT[114]) * work.d_inv[40];
            work.L[73] = (work.KKT[115]) * work.d_inv[40];
            work.L[85] = (work.KKT[116]) * work.d_inv[40];
            work.v[28] = work.L[28] * work.d[28];
            work.v[34] = work.L[29] * work.d[34];
            work.v[35] = work.L[30] * work.d[35];
            work.v[36] = work.L[31] * work.d[36];
            work.v[37] = work.L[32] * work.d[37];
            work.v[38] = work.L[33] * work.d[38];
            work.v[39] = work.L[34] * work.d[39];
            work.v[40] = work.L[35] * work.d[40];
            work.v[41] = 0 - work.L[28] * work.v[28] - work.L[29] * work.v[34] - work.L[30] * work.v[35] - work.L[31] * work.v[36] - work.L[32] * work.v[37] - work.L[33] * work.v[38] - work.L[34] * work.v[39] - work.L[35] * work.v[40];
            work.d[41] = work.v[41];
            if (work.d[41] > 0)
                work.d[41] = -settings.kkt_reg;
            else
                work.d[41] -= settings.kkt_reg;
            work.d_inv[41] = 1 / work.d[41];
            work.L[44] = (-work.L[37] * work.v[34] - work.L[38] * work.v[35] - work.L[39] * work.v[36] - work.L[40] * work.v[37] - work.L[41] * work.v[38] - work.L[42] * work.v[39] - work.L[43] * work.v[40]) * work.d_inv[41];
            work.L[53] = (-work.L[46] * work.v[34] - work.L[47] * work.v[35] - work.L[48] * work.v[36] - work.L[49] * work.v[37] - work.L[50] * work.v[38] - work.L[51] * work.v[39] - work.L[52] * work.v[40]) * work.d_inv[41];
            work.L[63] = (-work.L[56] * work.v[34] - work.L[57] * work.v[35] - work.L[58] * work.v[36] - work.L[59] * work.v[37] - work.L[60] * work.v[38] - work.L[61] * work.v[39] - work.L[62] * work.v[40]) * work.d_inv[41];
            work.L[74] = (-work.L[67] * work.v[34] - work.L[68] * work.v[35] - work.L[69] * work.v[36] - work.L[70] * work.v[37] - work.L[71] * work.v[38] - work.L[72] * work.v[39] - work.L[73] * work.v[40]) * work.d_inv[41];
            work.L[86] = (-work.L[79] * work.v[34] - work.L[80] * work.v[35] - work.L[81] * work.v[36] - work.L[82] * work.v[37] - work.L[83] * work.v[38] - work.L[84] * work.v[39] - work.L[85] * work.v[40]) * work.d_inv[41];
            work.v[29] = work.L[36] * work.d[29];
            work.v[34] = work.L[37] * work.d[34];
            work.v[35] = work.L[38] * work.d[35];
            work.v[36] = work.L[39] * work.d[36];
            work.v[37] = work.L[40] * work.d[37];
            work.v[38] = work.L[41] * work.d[38];
            work.v[39] = work.L[42] * work.d[39];
            work.v[40] = work.L[43] * work.d[40];
            work.v[41] = work.L[44] * work.d[41];
            work.v[42] = 0 - work.L[36] * work.v[29] - work.L[37] * work.v[34] - work.L[38] * work.v[35] - work.L[39] * work.v[36] - work.L[40] * work.v[37] - work.L[41] * work.v[38] - work.L[42] * work.v[39] - work.L[43] * work.v[40] - work.L[44] * work.v[41];
            work.d[42] = work.v[42];
            if (work.d[42] > 0)
                work.d[42] = -settings.kkt_reg;
            else
                work.d[42] -= settings.kkt_reg;
            work.d_inv[42] = 1 / work.d[42];
            work.L[54] = (-work.L[46] * work.v[34] - work.L[47] * work.v[35] - work.L[48] * work.v[36] - work.L[49] * work.v[37] - work.L[50] * work.v[38] - work.L[51] * work.v[39] - work.L[52] * work.v[40] - work.L[53] * work.v[41]) * work.d_inv[42];
            work.L[64] = (-work.L[56] * work.v[34] - work.L[57] * work.v[35] - work.L[58] * work.v[36] - work.L[59] * work.v[37] - work.L[60] * work.v[38] - work.L[61] * work.v[39] - work.L[62] * work.v[40] - work.L[63] * work.v[41]) * work.d_inv[42];
            work.L[75] = (-work.L[67] * work.v[34] - work.L[68] * work.v[35] - work.L[69] * work.v[36] - work.L[70] * work.v[37] - work.L[71] * work.v[38] - work.L[72] * work.v[39] - work.L[73] * work.v[40] - work.L[74] * work.v[41]) * work.d_inv[42];
            work.L[87] = (-work.L[79] * work.v[34] - work.L[80] * work.v[35] - work.L[81] * work.v[36] - work.L[82] * work.v[37] - work.L[83] * work.v[38] - work.L[84] * work.v[39] - work.L[85] * work.v[40] - work.L[86] * work.v[41]) * work.d_inv[42];
            work.v[30] = work.L[45] * work.d[30];
            work.v[34] = work.L[46] * work.d[34];
            work.v[35] = work.L[47] * work.d[35];
            work.v[36] = work.L[48] * work.d[36];
            work.v[37] = work.L[49] * work.d[37];
            work.v[38] = work.L[50] * work.d[38];
            work.v[39] = work.L[51] * work.d[39];
            work.v[40] = work.L[52] * work.d[40];
            work.v[41] = work.L[53] * work.d[41];
            work.v[42] = work.L[54] * work.d[42];
            work.v[43] = 0 - work.L[45] * work.v[30] - work.L[46] * work.v[34] - work.L[47] * work.v[35] - work.L[48] * work.v[36] - work.L[49] * work.v[37] - work.L[50] * work.v[38] - work.L[51] * work.v[39] - work.L[52] * work.v[40] - work.L[53] * work.v[41] - work.L[54] * work.v[42];
            work.d[43] = work.v[43];
            if (work.d[43] > 0)
                work.d[43] = -settings.kkt_reg;
            else
                work.d[43] -= settings.kkt_reg;
            work.d_inv[43] = 1 / work.d[43];
            work.L[65] = (-work.L[56] * work.v[34] - work.L[57] * work.v[35] - work.L[58] * work.v[36] - work.L[59] * work.v[37] - work.L[60] * work.v[38] - work.L[61] * work.v[39] - work.L[62] * work.v[40] - work.L[63] * work.v[41] - work.L[64] * work.v[42]) * work.d_inv[43];
            work.L[76] = (-work.L[67] * work.v[34] - work.L[68] * work.v[35] - work.L[69] * work.v[36] - work.L[70] * work.v[37] - work.L[71] * work.v[38] - work.L[72] * work.v[39] - work.L[73] * work.v[40] - work.L[74] * work.v[41] - work.L[75] * work.v[42]) * work.d_inv[43];
            work.L[88] = (-work.L[79] * work.v[34] - work.L[80] * work.v[35] - work.L[81] * work.v[36] - work.L[82] * work.v[37] - work.L[83] * work.v[38] - work.L[84] * work.v[39] - work.L[85] * work.v[40] - work.L[86] * work.v[41] - work.L[87] * work.v[42]) * work.d_inv[43];
            work.v[31] = work.L[55] * work.d[31];
            work.v[34] = work.L[56] * work.d[34];
            work.v[35] = work.L[57] * work.d[35];
            work.v[36] = work.L[58] * work.d[36];
            work.v[37] = work.L[59] * work.d[37];
            work.v[38] = work.L[60] * work.d[38];
            work.v[39] = work.L[61] * work.d[39];
            work.v[40] = work.L[62] * work.d[40];
            work.v[41] = work.L[63] * work.d[41];
            work.v[42] = work.L[64] * work.d[42];
            work.v[43] = work.L[65] * work.d[43];
            work.v[44] = 0 - work.L[55] * work.v[31] - work.L[56] * work.v[34] - work.L[57] * work.v[35] - work.L[58] * work.v[36] - work.L[59] * work.v[37] - work.L[60] * work.v[38] - work.L[61] * work.v[39] - work.L[62] * work.v[40] - work.L[63] * work.v[41] - work.L[64] * work.v[42] - work.L[65] * work.v[43];
            work.d[44] = work.v[44];
            if (work.d[44] > 0)
                work.d[44] = -settings.kkt_reg;
            else
                work.d[44] -= settings.kkt_reg;
            work.d_inv[44] = 1 / work.d[44];
            work.L[77] = (-work.L[67] * work.v[34] - work.L[68] * work.v[35] - work.L[69] * work.v[36] - work.L[70] * work.v[37] - work.L[71] * work.v[38] - work.L[72] * work.v[39] - work.L[73] * work.v[40] - work.L[74] * work.v[41] - work.L[75] * work.v[42] - work.L[76] * work.v[43]) * work.d_inv[44];
            work.L[89] = (-work.L[79] * work.v[34] - work.L[80] * work.v[35] - work.L[81] * work.v[36] - work.L[82] * work.v[37] - work.L[83] * work.v[38] - work.L[84] * work.v[39] - work.L[85] * work.v[40] - work.L[86] * work.v[41] - work.L[87] * work.v[42] - work.L[88] * work.v[43]) * work.d_inv[44];
            work.v[32] = work.L[66] * work.d[32];
            work.v[34] = work.L[67] * work.d[34];
            work.v[35] = work.L[68] * work.d[35];
            work.v[36] = work.L[69] * work.d[36];
            work.v[37] = work.L[70] * work.d[37];
            work.v[38] = work.L[71] * work.d[38];
            work.v[39] = work.L[72] * work.d[39];
            work.v[40] = work.L[73] * work.d[40];
            work.v[41] = work.L[74] * work.d[41];
            work.v[42] = work.L[75] * work.d[42];
            work.v[43] = work.L[76] * work.d[43];
            work.v[44] = work.L[77] * work.d[44];
            work.v[45] = 0 - work.L[66] * work.v[32] - work.L[67] * work.v[34] - work.L[68] * work.v[35] - work.L[69] * work.v[36] - work.L[70] * work.v[37] - work.L[71] * work.v[38] - work.L[72] * work.v[39] - work.L[73] * work.v[40] - work.L[74] * work.v[41] - work.L[75] * work.v[42] - work.L[76] * work.v[43] - work.L[77] * work.v[44];
            work.d[45] = work.v[45];
            if (work.d[45] > 0)
                work.d[45] = -settings.kkt_reg;
            else
                work.d[45] -= settings.kkt_reg;
            work.d_inv[45] = 1 / work.d[45];
            work.L[90] = (-work.L[79] * work.v[34] - work.L[80] * work.v[35] - work.L[81] * work.v[36] - work.L[82] * work.v[37] - work.L[83] * work.v[38] - work.L[84] * work.v[39] - work.L[85] * work.v[40] - work.L[86] * work.v[41] - work.L[87] * work.v[42] - work.L[88] * work.v[43] - work.L[89] * work.v[44]) * work.d_inv[45];
            work.v[33] = work.L[78] * work.d[33];
            work.v[34] = work.L[79] * work.d[34];
            work.v[35] = work.L[80] * work.d[35];
            work.v[36] = work.L[81] * work.d[36];
            work.v[37] = work.L[82] * work.d[37];
            work.v[38] = work.L[83] * work.d[38];
            work.v[39] = work.L[84] * work.d[39];
            work.v[40] = work.L[85] * work.d[40];
            work.v[41] = work.L[86] * work.d[41];
            work.v[42] = work.L[87] * work.d[42];
            work.v[43] = work.L[88] * work.d[43];
            work.v[44] = work.L[89] * work.d[44];
            work.v[45] = work.L[90] * work.d[45];
            work.v[46] = 0 - work.L[78] * work.v[33] - work.L[79] * work.v[34] - work.L[80] * work.v[35] - work.L[81] * work.v[36] - work.L[82] * work.v[37] - work.L[83] * work.v[38] - work.L[84] * work.v[39] - work.L[85] * work.v[40] - work.L[86] * work.v[41] - work.L[87] * work.v[42] - work.L[88] * work.v[43] - work.L[89] * work.v[44] - work.L[90] * work.v[45];
            work.d[46] = work.v[46];
            if (work.d[46] > 0)
                work.d[46] = -settings.kkt_reg;
            else
                work.d[46] -= settings.kkt_reg;
            work.d_inv[46] = 1 / work.d[46];
#ifndef ZERO_LIBRARY_MODE
            if (settings.debug) {
                printf("Squared Frobenius for factorization is %.8g.\n", check_factorization());
            }
#endif
        }

        double check_factorization(void)
        {
            /* Returns the squared Frobenius norm of A - L*D*L'. */
            double temp, residual;
            /* Only check the lower triangle. */
            residual = 0;
            temp = work.KKT[56] - 1 * work.d[28] * 1;
            residual += temp * temp;
            temp = work.KKT[58] - 1 * work.d[29] * 1;
            residual += temp * temp;
            temp = work.KKT[60] - 1 * work.d[30] * 1;
            residual += temp * temp;
            temp = work.KKT[62] - 1 * work.d[31] * 1;
            residual += temp * temp;
            temp = work.KKT[64] - 1 * work.d[32] * 1;
            residual += temp * temp;
            temp = work.KKT[66] - 1 * work.d[33] * 1;
            residual += temp * temp;
            temp = work.KKT[68] - 1 * work.d[34] * 1 - work.L[14] * work.d[14] * work.L[14] - work.L[15] * work.d[21] * work.L[15];
            residual += temp * temp;
            temp = work.KKT[75] - 1 * work.d[35] * 1 - work.L[16] * work.d[15] * work.L[16] - work.L[17] * work.d[22] * work.L[17];
            residual += temp * temp;
            temp = work.KKT[82] - 1 * work.d[36] * 1 - work.L[18] * work.d[16] * work.L[18] - work.L[19] * work.d[23] * work.L[19];
            residual += temp * temp;
            temp = work.KKT[89] - 1 * work.d[37] * 1 - work.L[20] * work.d[17] * work.L[20] - work.L[21] * work.d[24] * work.L[21];
            residual += temp * temp;
            temp = work.KKT[96] - 1 * work.d[38] * 1 - work.L[22] * work.d[18] * work.L[22] - work.L[23] * work.d[25] * work.L[23];
            residual += temp * temp;
            temp = work.KKT[103] - 1 * work.d[39] * 1 - work.L[24] * work.d[19] * work.L[24] - work.L[25] * work.d[26] * work.L[25];
            residual += temp * temp;
            temp = work.KKT[110] - 1 * work.d[40] * 1 - work.L[26] * work.d[20] * work.L[26] - work.L[27] * work.d[27] * work.L[27];
            residual += temp * temp;
            temp = work.KKT[0] - 1 * work.d[0] * 1;
            residual += temp * temp;
            temp = work.KKT[2] - 1 * work.d[1] * 1;
            residual += temp * temp;
            temp = work.KKT[4] - 1 * work.d[2] * 1;
            residual += temp * temp;
            temp = work.KKT[6] - 1 * work.d[3] * 1;
            residual += temp * temp;
            temp = work.KKT[8] - 1 * work.d[4] * 1;
            residual += temp * temp;
            temp = work.KKT[10] - 1 * work.d[5] * 1;
            residual += temp * temp;
            temp = work.KKT[12] - 1 * work.d[6] * 1;
            residual += temp * temp;
            temp = work.KKT[14] - 1 * work.d[7] * 1;
            residual += temp * temp;
            temp = work.KKT[16] - 1 * work.d[8] * 1;
            residual += temp * temp;
            temp = work.KKT[18] - 1 * work.d[9] * 1;
            residual += temp * temp;
            temp = work.KKT[20] - 1 * work.d[10] * 1;
            residual += temp * temp;
            temp = work.KKT[22] - 1 * work.d[11] * 1;
            residual += temp * temp;
            temp = work.KKT[24] - 1 * work.d[12] * 1;
            residual += temp * temp;
            temp = work.KKT[26] - 1 * work.d[13] * 1;
            residual += temp * temp;
            temp = work.KKT[1] - work.L[0] * work.d[0] * 1;
            residual += temp * temp;
            temp = work.KKT[3] - work.L[1] * work.d[1] * 1;
            residual += temp * temp;
            temp = work.KKT[5] - work.L[2] * work.d[2] * 1;
            residual += temp * temp;
            temp = work.KKT[7] - work.L[3] * work.d[3] * 1;
            residual += temp * temp;
            temp = work.KKT[9] - work.L[4] * work.d[4] * 1;
            residual += temp * temp;
            temp = work.KKT[11] - work.L[5] * work.d[5] * 1;
            residual += temp * temp;
            temp = work.KKT[13] - work.L[6] * work.d[6] * 1;
            residual += temp * temp;
            temp = work.KKT[15] - work.L[7] * work.d[7] * 1;
            residual += temp * temp;
            temp = work.KKT[17] - work.L[8] * work.d[8] * 1;
            residual += temp * temp;
            temp = work.KKT[19] - work.L[9] * work.d[9] * 1;
            residual += temp * temp;
            temp = work.KKT[21] - work.L[10] * work.d[10] * 1;
            residual += temp * temp;
            temp = work.KKT[23] - work.L[11] * work.d[11] * 1;
            residual += temp * temp;
            temp = work.KKT[25] - work.L[12] * work.d[12] * 1;
            residual += temp * temp;
            temp = work.KKT[27] - work.L[13] * work.d[13] * 1;
            residual += temp * temp;
            temp = work.KKT[28] - work.L[0] * work.d[0] * work.L[0] - 1 * work.d[14] * 1;
            residual += temp * temp;
            temp = work.KKT[30] - work.L[1] * work.d[1] * work.L[1] - 1 * work.d[15] * 1;
            residual += temp * temp;
            temp = work.KKT[32] - work.L[2] * work.d[2] * work.L[2] - 1 * work.d[16] * 1;
            residual += temp * temp;
            temp = work.KKT[34] - work.L[3] * work.d[3] * work.L[3] - 1 * work.d[17] * 1;
            residual += temp * temp;
            temp = work.KKT[36] - work.L[4] * work.d[4] * work.L[4] - 1 * work.d[18] * 1;
            residual += temp * temp;
            temp = work.KKT[38] - work.L[5] * work.d[5] * work.L[5] - 1 * work.d[19] * 1;
            residual += temp * temp;
            temp = work.KKT[40] - work.L[6] * work.d[6] * work.L[6] - 1 * work.d[20] * 1;
            residual += temp * temp;
            temp = work.KKT[42] - work.L[7] * work.d[7] * work.L[7] - 1 * work.d[21] * 1;
            residual += temp * temp;
            temp = work.KKT[44] - work.L[8] * work.d[8] * work.L[8] - 1 * work.d[22] * 1;
            residual += temp * temp;
            temp = work.KKT[46] - work.L[9] * work.d[9] * work.L[9] - 1 * work.d[23] * 1;
            residual += temp * temp;
            temp = work.KKT[48] - work.L[10] * work.d[10] * work.L[10] - 1 * work.d[24] * 1;
            residual += temp * temp;
            temp = work.KKT[50] - work.L[11] * work.d[11] * work.L[11] - 1 * work.d[25] * 1;
            residual += temp * temp;
            temp = work.KKT[52] - work.L[12] * work.d[12] * work.L[12] - 1 * work.d[26] * 1;
            residual += temp * temp;
            temp = work.KKT[54] - work.L[13] * work.d[13] * work.L[13] - 1 * work.d[27] * 1;
            residual += temp * temp;
            temp = work.KKT[29] - 1 * work.d[14] * work.L[14];
            residual += temp * temp;
            temp = work.KKT[31] - 1 * work.d[15] * work.L[16];
            residual += temp * temp;
            temp = work.KKT[33] - 1 * work.d[16] * work.L[18];
            residual += temp * temp;
            temp = work.KKT[35] - 1 * work.d[17] * work.L[20];
            residual += temp * temp;
            temp = work.KKT[37] - 1 * work.d[18] * work.L[22];
            residual += temp * temp;
            temp = work.KKT[39] - 1 * work.d[19] * work.L[24];
            residual += temp * temp;
            temp = work.KKT[41] - 1 * work.d[20] * work.L[26];
            residual += temp * temp;
            temp = work.KKT[43] - 1 * work.d[21] * work.L[15];
            residual += temp * temp;
            temp = work.KKT[45] - 1 * work.d[22] * work.L[17];
            residual += temp * temp;
            temp = work.KKT[47] - 1 * work.d[23] * work.L[19];
            residual += temp * temp;
            temp = work.KKT[49] - 1 * work.d[24] * work.L[21];
            residual += temp * temp;
            temp = work.KKT[51] - 1 * work.d[25] * work.L[23];
            residual += temp * temp;
            temp = work.KKT[53] - 1 * work.d[26] * work.L[25];
            residual += temp * temp;
            temp = work.KKT[55] - 1 * work.d[27] * work.L[27];
            residual += temp * temp;
            temp = work.KKT[57] - work.L[28] * work.d[28] * 1;
            residual += temp * temp;
            temp = work.KKT[69] - work.L[29] * work.d[34] * 1;
            residual += temp * temp;
            temp = work.KKT[76] - work.L[30] * work.d[35] * 1;
            residual += temp * temp;
            temp = work.KKT[83] - work.L[31] * work.d[36] * 1;
            residual += temp * temp;
            temp = work.KKT[90] - work.L[32] * work.d[37] * 1;
            residual += temp * temp;
            temp = work.KKT[97] - work.L[33] * work.d[38] * 1;
            residual += temp * temp;
            temp = work.KKT[104] - work.L[34] * work.d[39] * 1;
            residual += temp * temp;
            temp = work.KKT[111] - work.L[35] * work.d[40] * 1;
            residual += temp * temp;
            temp = work.KKT[59] - work.L[36] * work.d[29] * 1;
            residual += temp * temp;
            temp = work.KKT[70] - work.L[37] * work.d[34] * 1;
            residual += temp * temp;
            temp = work.KKT[77] - work.L[38] * work.d[35] * 1;
            residual += temp * temp;
            temp = work.KKT[84] - work.L[39] * work.d[36] * 1;
            residual += temp * temp;
            temp = work.KKT[91] - work.L[40] * work.d[37] * 1;
            residual += temp * temp;
            temp = work.KKT[98] - work.L[41] * work.d[38] * 1;
            residual += temp * temp;
            temp = work.KKT[105] - work.L[42] * work.d[39] * 1;
            residual += temp * temp;
            temp = work.KKT[112] - work.L[43] * work.d[40] * 1;
            residual += temp * temp;
            temp = work.KKT[61] - work.L[45] * work.d[30] * 1;
            residual += temp * temp;
            temp = work.KKT[71] - work.L[46] * work.d[34] * 1;
            residual += temp * temp;
            temp = work.KKT[78] - work.L[47] * work.d[35] * 1;
            residual += temp * temp;
            temp = work.KKT[85] - work.L[48] * work.d[36] * 1;
            residual += temp * temp;
            temp = work.KKT[92] - work.L[49] * work.d[37] * 1;
            residual += temp * temp;
            temp = work.KKT[99] - work.L[50] * work.d[38] * 1;
            residual += temp * temp;
            temp = work.KKT[106] - work.L[51] * work.d[39] * 1;
            residual += temp * temp;
            temp = work.KKT[113] - work.L[52] * work.d[40] * 1;
            residual += temp * temp;
            temp = work.KKT[63] - work.L[55] * work.d[31] * 1;
            residual += temp * temp;
            temp = work.KKT[72] - work.L[56] * work.d[34] * 1;
            residual += temp * temp;
            temp = work.KKT[79] - work.L[57] * work.d[35] * 1;
            residual += temp * temp;
            temp = work.KKT[86] - work.L[58] * work.d[36] * 1;
            residual += temp * temp;
            temp = work.KKT[93] - work.L[59] * work.d[37] * 1;
            residual += temp * temp;
            temp = work.KKT[100] - work.L[60] * work.d[38] * 1;
            residual += temp * temp;
            temp = work.KKT[107] - work.L[61] * work.d[39] * 1;
            residual += temp * temp;
            temp = work.KKT[114] - work.L[62] * work.d[40] * 1;
            residual += temp * temp;
            temp = work.KKT[65] - work.L[66] * work.d[32] * 1;
            residual += temp * temp;
            temp = work.KKT[73] - work.L[67] * work.d[34] * 1;
            residual += temp * temp;
            temp = work.KKT[80] - work.L[68] * work.d[35] * 1;
            residual += temp * temp;
            temp = work.KKT[87] - work.L[69] * work.d[36] * 1;
            residual += temp * temp;
            temp = work.KKT[94] - work.L[70] * work.d[37] * 1;
            residual += temp * temp;
            temp = work.KKT[101] - work.L[71] * work.d[38] * 1;
            residual += temp * temp;
            temp = work.KKT[108] - work.L[72] * work.d[39] * 1;
            residual += temp * temp;
            temp = work.KKT[115] - work.L[73] * work.d[40] * 1;
            residual += temp * temp;
            temp = work.KKT[67] - work.L[78] * work.d[33] * 1;
            residual += temp * temp;
            temp = work.KKT[74] - work.L[79] * work.d[34] * 1;
            residual += temp * temp;
            temp = work.KKT[81] - work.L[80] * work.d[35] * 1;
            residual += temp * temp;
            temp = work.KKT[88] - work.L[81] * work.d[36] * 1;
            residual += temp * temp;
            temp = work.KKT[95] - work.L[82] * work.d[37] * 1;
            residual += temp * temp;
            temp = work.KKT[102] - work.L[83] * work.d[38] * 1;
            residual += temp * temp;
            temp = work.KKT[109] - work.L[84] * work.d[39] * 1;
            residual += temp * temp;
            temp = work.KKT[116] - work.L[85] * work.d[40] * 1;
            residual += temp * temp;
            return residual;
        }

        void matrix_multiply(double* result, double* source)
        {
            /* Finds result = A*source. */
            result[0] = work.KKT[56] * source[0] + work.KKT[57] * source[41];
            result[1] = work.KKT[58] * source[1] + work.KKT[59] * source[42];
            result[2] = work.KKT[60] * source[2] + work.KKT[61] * source[43];
            result[3] = work.KKT[62] * source[3] + work.KKT[63] * source[44];
            result[4] = work.KKT[64] * source[4] + work.KKT[65] * source[45];
            result[5] = work.KKT[66] * source[5] + work.KKT[67] * source[46];
            result[6] = work.KKT[68] * source[6] + work.KKT[29] * source[27] + work.KKT[43] * source[34] + work.KKT[69] * source[41] + work.KKT[70] * source[42] + work.KKT[71] * source[43] + work.KKT[72] * source[44] + work.KKT[73] * source[45] + work.KKT[74] * source[46];
            result[7] = work.KKT[75] * source[7] + work.KKT[31] * source[28] + work.KKT[45] * source[35] + work.KKT[76] * source[41] + work.KKT[77] * source[42] + work.KKT[78] * source[43] + work.KKT[79] * source[44] + work.KKT[80] * source[45] + work.KKT[81] * source[46];
            result[8] = work.KKT[82] * source[8] + work.KKT[33] * source[29] + work.KKT[47] * source[36] + work.KKT[83] * source[41] + work.KKT[84] * source[42] + work.KKT[85] * source[43] + work.KKT[86] * source[44] + work.KKT[87] * source[45] + work.KKT[88] * source[46];
            result[9] = work.KKT[89] * source[9] + work.KKT[35] * source[30] + work.KKT[49] * source[37] + work.KKT[90] * source[41] + work.KKT[91] * source[42] + work.KKT[92] * source[43] + work.KKT[93] * source[44] + work.KKT[94] * source[45] + work.KKT[95] * source[46];
            result[10] = work.KKT[96] * source[10] + work.KKT[37] * source[31] + work.KKT[51] * source[38] + work.KKT[97] * source[41] + work.KKT[98] * source[42] + work.KKT[99] * source[43] + work.KKT[100] * source[44] + work.KKT[101] * source[45] + work.KKT[102] * source[46];
            result[11] = work.KKT[103] * source[11] + work.KKT[39] * source[32] + work.KKT[53] * source[39] + work.KKT[104] * source[41] + work.KKT[105] * source[42] + work.KKT[106] * source[43] + work.KKT[107] * source[44] + work.KKT[108] * source[45] + work.KKT[109] * source[46];
            result[12] = work.KKT[110] * source[12] + work.KKT[41] * source[33] + work.KKT[55] * source[40] + work.KKT[111] * source[41] + work.KKT[112] * source[42] + work.KKT[113] * source[43] + work.KKT[114] * source[44] + work.KKT[115] * source[45] + work.KKT[116] * source[46];
            result[13] = work.KKT[0] * source[13] + work.KKT[1] * source[27];
            result[14] = work.KKT[2] * source[14] + work.KKT[3] * source[28];
            result[15] = work.KKT[4] * source[15] + work.KKT[5] * source[29];
            result[16] = work.KKT[6] * source[16] + work.KKT[7] * source[30];
            result[17] = work.KKT[8] * source[17] + work.KKT[9] * source[31];
            result[18] = work.KKT[10] * source[18] + work.KKT[11] * source[32];
            result[19] = work.KKT[12] * source[19] + work.KKT[13] * source[33];
            result[20] = work.KKT[14] * source[20] + work.KKT[15] * source[34];
            result[21] = work.KKT[16] * source[21] + work.KKT[17] * source[35];
            result[22] = work.KKT[18] * source[22] + work.KKT[19] * source[36];
            result[23] = work.KKT[20] * source[23] + work.KKT[21] * source[37];
            result[24] = work.KKT[22] * source[24] + work.KKT[23] * source[38];
            result[25] = work.KKT[24] * source[25] + work.KKT[25] * source[39];
            result[26] = work.KKT[26] * source[26] + work.KKT[27] * source[40];
            result[27] = work.KKT[1] * source[13] + work.KKT[28] * source[27] + work.KKT[29] * source[6];
            result[28] = work.KKT[3] * source[14] + work.KKT[30] * source[28] + work.KKT[31] * source[7];
            result[29] = work.KKT[5] * source[15] + work.KKT[32] * source[29] + work.KKT[33] * source[8];
            result[30] = work.KKT[7] * source[16] + work.KKT[34] * source[30] + work.KKT[35] * source[9];
            result[31] = work.KKT[9] * source[17] + work.KKT[36] * source[31] + work.KKT[37] * source[10];
            result[32] = work.KKT[11] * source[18] + work.KKT[38] * source[32] + work.KKT[39] * source[11];
            result[33] = work.KKT[13] * source[19] + work.KKT[40] * source[33] + work.KKT[41] * source[12];
            result[34] = work.KKT[15] * source[20] + work.KKT[42] * source[34] + work.KKT[43] * source[6];
            result[35] = work.KKT[17] * source[21] + work.KKT[44] * source[35] + work.KKT[45] * source[7];
            result[36] = work.KKT[19] * source[22] + work.KKT[46] * source[36] + work.KKT[47] * source[8];
            result[37] = work.KKT[21] * source[23] + work.KKT[48] * source[37] + work.KKT[49] * source[9];
            result[38] = work.KKT[23] * source[24] + work.KKT[50] * source[38] + work.KKT[51] * source[10];
            result[39] = work.KKT[25] * source[25] + work.KKT[52] * source[39] + work.KKT[53] * source[11];
            result[40] = work.KKT[27] * source[26] + work.KKT[54] * source[40] + work.KKT[55] * source[12];
            result[41] = work.KKT[57] * source[0] + work.KKT[69] * source[6] + work.KKT[76] * source[7] + work.KKT[83] * source[8] + work.KKT[90] * source[9] + work.KKT[97] * source[10] + work.KKT[104] * source[11] + work.KKT[111] * source[12];
            result[42] = work.KKT[59] * source[1] + work.KKT[70] * source[6] + work.KKT[77] * source[7] + work.KKT[84] * source[8] + work.KKT[91] * source[9] + work.KKT[98] * source[10] + work.KKT[105] * source[11] + work.KKT[112] * source[12];
            result[43] = work.KKT[61] * source[2] + work.KKT[71] * source[6] + work.KKT[78] * source[7] + work.KKT[85] * source[8] + work.KKT[92] * source[9] + work.KKT[99] * source[10] + work.KKT[106] * source[11] + work.KKT[113] * source[12];
            result[44] = work.KKT[63] * source[3] + work.KKT[72] * source[6] + work.KKT[79] * source[7] + work.KKT[86] * source[8] + work.KKT[93] * source[9] + work.KKT[100] * source[10] + work.KKT[107] * source[11] + work.KKT[114] * source[12];
            result[45] = work.KKT[65] * source[4] + work.KKT[73] * source[6] + work.KKT[80] * source[7] + work.KKT[87] * source[8] + work.KKT[94] * source[9] + work.KKT[101] * source[10] + work.KKT[108] * source[11] + work.KKT[115] * source[12];
            result[46] = work.KKT[67] * source[5] + work.KKT[74] * source[6] + work.KKT[81] * source[7] + work.KKT[88] * source[8] + work.KKT[95] * source[9] + work.KKT[102] * source[10] + work.KKT[109] * source[11] + work.KKT[116] * source[12];
        }

        double check_residual(double* target, double* multiplicand)
        {
            /* Returns the squared 2-norm of lhs - A*rhs. */
            /* Reuses v to find the residual. */
            int i;
            double residual;
            residual = 0;
            matrix_multiply(work.v, multiplicand);
            for (i = 0; i < 13; i++) {
                residual += (target[i] - work.v[i]) * (target[i] - work.v[i]);
            }
            return residual;
        }

        void fill_KKT(void)
        {
            work.KKT[56] = 2 * params.slack[0];
            work.KKT[58] = 2 * params.slack[1];
            work.KKT[60] = 2 * params.slack[2];
            work.KKT[62] = 2 * params.slack[3];
            work.KKT[64] = 2 * params.slack[4];
            work.KKT[66] = 2 * params.slack[5];
            work.KKT[68] = 2 * params.damping[0];
            work.KKT[75] = 2 * params.damping[1];
            work.KKT[82] = 2 * params.damping[2];
            work.KKT[89] = 2 * params.damping[3];
            work.KKT[96] = 2 * params.damping[4];
            work.KKT[103] = 2 * params.damping[5];
            work.KKT[110] = 2 * params.damping[6];
            work.KKT[0] = work.s_inv_z[0];
            work.KKT[2] = work.s_inv_z[1];
            work.KKT[4] = work.s_inv_z[2];
            work.KKT[6] = work.s_inv_z[3];
            work.KKT[8] = work.s_inv_z[4];
            work.KKT[10] = work.s_inv_z[5];
            work.KKT[12] = work.s_inv_z[6];
            work.KKT[14] = work.s_inv_z[7];
            work.KKT[16] = work.s_inv_z[8];
            work.KKT[18] = work.s_inv_z[9];
            work.KKT[20] = work.s_inv_z[10];
            work.KKT[22] = work.s_inv_z[11];
            work.KKT[24] = work.s_inv_z[12];
            work.KKT[26] = work.s_inv_z[13];
            work.KKT[1] = 1;
            work.KKT[3] = 1;
            work.KKT[5] = 1;
            work.KKT[7] = 1;
            work.KKT[9] = 1;
            work.KKT[11] = 1;
            work.KKT[13] = 1;
            work.KKT[15] = 1;
            work.KKT[17] = 1;
            work.KKT[19] = 1;
            work.KKT[21] = 1;
            work.KKT[23] = 1;
            work.KKT[25] = 1;
            work.KKT[27] = 1;
            work.KKT[28] = work.block_33[0];
            work.KKT[30] = work.block_33[0];
            work.KKT[32] = work.block_33[0];
            work.KKT[34] = work.block_33[0];
            work.KKT[36] = work.block_33[0];
            work.KKT[38] = work.block_33[0];
            work.KKT[40] = work.block_33[0];
            work.KKT[42] = work.block_33[0];
            work.KKT[44] = work.block_33[0];
            work.KKT[46] = work.block_33[0];
            work.KKT[48] = work.block_33[0];
            work.KKT[50] = work.block_33[0];
            work.KKT[52] = work.block_33[0];
            work.KKT[54] = work.block_33[0];
            work.KKT[29] = -1;
            work.KKT[31] = -1;
            work.KKT[33] = -1;
            work.KKT[35] = -1;
            work.KKT[37] = -1;
            work.KKT[39] = -1;
            work.KKT[41] = -1;
            work.KKT[43] = 1;
            work.KKT[45] = 1;
            work.KKT[47] = 1;
            work.KKT[49] = 1;
            work.KKT[51] = 1;
            work.KKT[53] = 1;
            work.KKT[55] = 1;
            work.KKT[57] = -1;
            work.KKT[69] = params.J_1[0];
            work.KKT[76] = params.J_1[1];
            work.KKT[83] = params.J_1[2];
            work.KKT[90] = params.J_1[3];
            work.KKT[97] = params.J_1[4];
            work.KKT[104] = params.J_1[5];
            work.KKT[111] = params.J_1[6];
            work.KKT[59] = -1;
            work.KKT[70] = params.J_2[0];
            work.KKT[77] = params.J_2[1];
            work.KKT[84] = params.J_2[2];
            work.KKT[91] = params.J_2[3];
            work.KKT[98] = params.J_2[4];
            work.KKT[105] = params.J_2[5];
            work.KKT[112] = params.J_2[6];
            work.KKT[61] = -1;
            work.KKT[71] = params.J_3[0];
            work.KKT[78] = params.J_3[1];
            work.KKT[85] = params.J_3[2];
            work.KKT[92] = params.J_3[3];
            work.KKT[99] = params.J_3[4];
            work.KKT[106] = params.J_3[5];
            work.KKT[113] = params.J_3[6];
            work.KKT[63] = -1;
            work.KKT[72] = params.J_4[0];
            work.KKT[79] = params.J_4[1];
            work.KKT[86] = params.J_4[2];
            work.KKT[93] = params.J_4[3];
            work.KKT[100] = params.J_4[4];
            work.KKT[107] = params.J_4[5];
            work.KKT[114] = params.J_4[6];
            work.KKT[65] = -1;
            work.KKT[73] = params.J_5[0];
            work.KKT[80] = params.J_5[1];
            work.KKT[87] = params.J_5[2];
            work.KKT[94] = params.J_5[3];
            work.KKT[101] = params.J_5[4];
            work.KKT[108] = params.J_5[5];
            work.KKT[115] = params.J_5[6];
            work.KKT[67] = -1;
            work.KKT[74] = params.J_6[0];
            work.KKT[81] = params.J_6[1];
            work.KKT[88] = params.J_6[2];
            work.KKT[95] = params.J_6[3];
            work.KKT[102] = params.J_6[4];
            work.KKT[109] = params.J_6[5];
            work.KKT[116] = params.J_6[6];
        }

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

        double eval_gap(void)
        {
            int i;
            double gap;
            gap = 0;
            for (i = 0; i < 14; i++)
                gap += work.z[i] * work.s[i];
            return gap;
        }

        void set_defaults(void)
        {
            settings.resid_tol = 1e-6;
            settings.eps = 1e-4;
            settings.max_iters = 25;
            settings.refine_steps = 1;
            settings.s_init = 1;
            settings.z_init = 1;
            settings.debug = 0;
            settings.verbose = 1;
            settings.verbose_refinement = 0;
            settings.better_start = 1;
            settings.kkt_reg = 1e-7;
        }

        void setup_pointers(void)
        {
            work.y = work.x + 13;
            work.s = work.x + 19;
            work.z = work.x + 33;
            vars.delta = work.x + 0;
            vars.dq = work.x + 6;
        }

        void setup_indexed_params(void)
        {
            /* In CVXGEN, you can say */
            /*   parameters */
            /*     A[i] (5,3), i=1..4 */
            /*   end */
            /* This function sets up A[2] to be a pointer to A_2, which is a length-15 */
            /* vector of doubles. */
            /* If you access parameters that you haven't defined in CVXGEN, the result */
            /* is undefined. */
            params.J[1] = params.J_1;
            params.J[2] = params.J_2;
            params.J[3] = params.J_3;
            params.J[4] = params.J_4;
            params.J[5] = params.J_5;
            params.J[6] = params.J_6;
        }

        void setup_indexing(void)
        {
            setup_pointers();
            setup_indexed_params();
        }

        void set_start(void)
        {
            int i;
            for (i = 0; i < 13; i++)
                work.x[i] = 0;
            for (i = 0; i < 6; i++)
                work.y[i] = 0;
            for (i = 0; i < 14; i++)
                work.s[i] = (work.h[i] > 0) ? work.h[i] : settings.s_init;
            for (i = 0; i < 14; i++)
                work.z[i] = settings.z_init;
        }

        double eval_objv(void)
        {
            int i;
            double objv;
            /* Borrow space in work.rhs. */
            multbyP(work.rhs, work.x);
            objv = 0;
            for (i = 0; i < 13; i++)
                objv += work.x[i] * work.rhs[i];
            objv *= 0.5;
            for (i = 0; i < 13; i++)
                objv += work.q[i] * work.x[i];
            objv += work.quad_568441778176[0];
            return objv;
        }

        void fillrhs_aff(void)
        {
            int i;
            double *r1, *r2, *r3, *r4;
            r1 = work.rhs;
            r2 = work.rhs + 13;
            r3 = work.rhs + 27;
            r4 = work.rhs + 41;
            /* r1 = -A^Ty - G^Tz - Px - q. */
            multbymAT(r1, work.y);
            multbymGT(work.buffer, work.z);
            for (i = 0; i < 13; i++)
                r1[i] += work.buffer[i];
            multbyP(work.buffer, work.x);
            for (i = 0; i < 13; i++)
                r1[i] -= work.buffer[i] + work.q[i];
            /* r2 = -z. */
            for (i = 0; i < 14; i++)
                r2[i] = -work.z[i];
            /* r3 = -Gx - s + h. */
            multbymG(r3, work.x);
            for (i = 0; i < 14; i++)
                r3[i] += -work.s[i] + work.h[i];
            /* r4 = -Ax + b. */
            multbymA(r4, work.x);
            for (i = 0; i < 6; i++)
                r4[i] += work.b[i];
        }

        void fillrhs_cc(void)
        {
            int i;
            double* r2;
            double *ds_aff, *dz_aff;
            double mu;
            double alpha;
            double sigma;
            double smu;
            double minval;
            r2 = work.rhs + 13;
            ds_aff = work.lhs_aff + 13;
            dz_aff = work.lhs_aff + 27;
            mu = 0;
            for (i = 0; i < 14; i++)
                mu += work.s[i] * work.z[i];
            /* Don't finish calculating mu quite yet. */
            /* Find min(min(ds./s), min(dz./z)). */
            minval = 0;
            for (i = 0; i < 14; i++)
                if (ds_aff[i] < minval * work.s[i])
                    minval = ds_aff[i] / work.s[i];
            for (i = 0; i < 14; i++)
                if (dz_aff[i] < minval * work.z[i])
                    minval = dz_aff[i] / work.z[i];
            /* Find alpha. */
            if (-1 < minval)
                alpha = 1;
            else
                alpha = -1 / minval;
            sigma = 0;
            for (i = 0; i < 14; i++)
                sigma += (work.s[i] + alpha * ds_aff[i]) * (work.z[i] + alpha * dz_aff[i]);
            sigma /= mu;
            sigma = sigma * sigma * sigma;
            /* Finish calculating mu now. */
            mu *= 0.07142857142857142;
            smu = sigma * mu;
            /* Fill-in the rhs. */
            for (i = 0; i < 13; i++)
                work.rhs[i] = 0;
            for (i = 27; i < 47; i++)
                work.rhs[i] = 0;
            for (i = 0; i < 14; i++)
                r2[i] = work.s_inv[i] * (smu - ds_aff[i] * dz_aff[i]);
        }

        void refine(double* target, double* var)
        {
            int i, j;
            double* residual = work.buffer;
            double norm2;
            double* new_var = work.buffer2;
            for (j = 0; j < settings.refine_steps; j++) {
                norm2 = 0;
                matrix_multiply(residual, var);
                for (i = 0; i < 47; i++) {
                    residual[i] = residual[i] - target[i];
                    norm2 += residual[i] * residual[i];
                }
#ifndef ZERO_LIBRARY_MODE
                if (settings.verbose_refinement) {
                    if (j == 0)
                        printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
                    else
                        printf("After refinement we get squared norm %.6g.\n", norm2);
                }
#endif
                /* Solve to find new_var = KKT \ (target - A*var). */
                ldl_solve(residual, new_var);
                /* Update var += new_var, or var += KKT \ (target - A*var). */
                for (i = 0; i < 47; i++) {
                    var[i] -= new_var[i];
                }
            }
#ifndef ZERO_LIBRARY_MODE
            if (settings.verbose_refinement) {
                /* Check the residual once more, but only if we're reporting it, since */
                /* it's expensive. */
                norm2 = 0;
                matrix_multiply(residual, var);
                for (i = 0; i < 47; i++) {
                    residual[i] = residual[i] - target[i];
                    norm2 += residual[i] * residual[i];
                }
                if (j == 0)
                    printf("Initial residual before refinement has norm squared %.6g.\n", norm2);
                else
                    printf("After refinement we get squared norm %.6g.\n", norm2);
            }
#endif
        }

        double calc_ineq_resid_squared(void)
        {
            /* Calculates the norm ||-Gx - s + h||. */
            double norm2_squared;
            int i;
            /* Find -Gx. */
            multbymG(work.buffer, work.x);
            /* Add -s + h. */
            for (i = 0; i < 14; i++)
                work.buffer[i] += -work.s[i] + work.h[i];
            /* Now find the squared norm. */
            norm2_squared = 0;
            for (i = 0; i < 14; i++)
                norm2_squared += work.buffer[i] * work.buffer[i];
            return norm2_squared;
        }

        double calc_eq_resid_squared(void)
        {
            /* Calculates the norm ||-Ax + b||. */
            double norm2_squared;
            int i;
            /* Find -Ax. */
            multbymA(work.buffer, work.x);
            /* Add +b. */
            for (i = 0; i < 6; i++)
                work.buffer[i] += work.b[i];
            /* Now find the squared norm. */
            norm2_squared = 0;
            for (i = 0; i < 6; i++)
                norm2_squared += work.buffer[i] * work.buffer[i];
            return norm2_squared;
        }

        void better_start(void)
        {
            /* Calculates a better starting point, using a similar approach to CVXOPT. */
            /* Not yet speed optimized. */
            int i;
            double *x, *s, *z, *y;
            double alpha;
            work.block_33[0] = -1;
            /* Make sure sinvz is 1 to make hijacked KKT system ok. */
            for (i = 0; i < 14; i++)
                work.s_inv_z[i] = 1;
            fill_KKT();
            ldl_factor();
            fillrhs_start();
            /* Borrow work.lhs_aff for the solution. */
            ldl_solve(work.rhs, work.lhs_aff);
            /* Don't do any refinement for now. Precision doesn't matter too much. */
            x = work.lhs_aff;
            s = work.lhs_aff + 13;
            z = work.lhs_aff + 27;
            y = work.lhs_aff + 41;
            /* Just set x and y as is. */
            for (i = 0; i < 13; i++)
                work.x[i] = x[i];
            for (i = 0; i < 6; i++)
                work.y[i] = y[i];
            /* Now complete the initialization. Start with s. */
            /* Must have alpha > max(z). */
            alpha = -1e99;
            for (i = 0; i < 14; i++)
                if (alpha < z[i])
                    alpha = z[i];
            if (alpha < 0) {
                for (i = 0; i < 14; i++)
                    work.s[i] = -z[i];
            }
            else {
                alpha += 1;
                for (i = 0; i < 14; i++)
                    work.s[i] = -z[i] + alpha;
            }
            /* Now initialize z. */
            /* Now must have alpha > max(-z). */
            alpha = -1e99;
            for (i = 0; i < 14; i++)
                if (alpha < -z[i])
                    alpha = -z[i];
            if (alpha < 0) {
                for (i = 0; i < 14; i++)
                    work.z[i] = z[i];
            }
            else {
                alpha += 1;
                for (i = 0; i < 14; i++)
                    work.z[i] = z[i] + alpha;
            }
        }

        void fillrhs_start(void)
        {
            /* Fill rhs with (-q, 0, h, b). */
            int i;
            double *r1, *r2, *r3, *r4;
            r1 = work.rhs;
            r2 = work.rhs + 13;
            r3 = work.rhs + 27;
            r4 = work.rhs + 41;
            for (i = 0; i < 13; i++)
                r1[i] = -work.q[i];
            for (i = 0; i < 14; i++)
                r2[i] = 0;
            for (i = 0; i < 14; i++)
                r3[i] = work.h[i];
            for (i = 0; i < 6; i++)
                r4[i] = work.b[i];
        }

        long solve(void)
        {
            int i;
            int iter;
            double *dx, *ds, *dy, *dz;
            double minval;
            double alpha;
            work.converged = 0;
            setup_pointers();
            pre_ops();
#ifndef ZERO_LIBRARY_MODE
            if (settings.verbose)
                printf("iter     objv        gap       |Ax-b|    |Gx+s-h|    step\n");
#endif
            fillq();
            fillh();
            fillb();
            if (settings.better_start)
                better_start();
            else
                set_start();
            for (iter = 0; iter < settings.max_iters; iter++) {
                for (i = 0; i < 14; i++) {
                    work.s_inv[i] = 1.0 / work.s[i];
                    work.s_inv_z[i] = work.s_inv[i] * work.z[i];
                }
                work.block_33[0] = 0;
                fill_KKT();
                ldl_factor();
                /* Affine scaling directions. */
                fillrhs_aff();
                ldl_solve(work.rhs, work.lhs_aff);
                refine(work.rhs, work.lhs_aff);
                /* Centering plus corrector directions. */
                fillrhs_cc();
                ldl_solve(work.rhs, work.lhs_cc);
                refine(work.rhs, work.lhs_cc);
                /* Add the two together and store in aff. */
                for (i = 0; i < 47; i++)
                    work.lhs_aff[i] += work.lhs_cc[i];
                /* Rename aff to reflect its new meaning. */
                dx = work.lhs_aff;
                ds = work.lhs_aff + 13;
                dz = work.lhs_aff + 27;
                dy = work.lhs_aff + 41;
                /* Find min(min(ds./s), min(dz./z)). */
                minval = 0;
                for (i = 0; i < 14; i++)
                    if (ds[i] < minval * work.s[i])
                        minval = ds[i] / work.s[i];
                for (i = 0; i < 14; i++)
                    if (dz[i] < minval * work.z[i])
                        minval = dz[i] / work.z[i];
                /* Find alpha. */
                if (-0.99 < minval)
                    alpha = 1;
                else
                    alpha = -0.99 / minval;
                /* Update the primal and dual variables. */
                for (i = 0; i < 13; i++)
                    work.x[i] += alpha * dx[i];
                for (i = 0; i < 14; i++)
                    work.s[i] += alpha * ds[i];
                for (i = 0; i < 14; i++)
                    work.z[i] += alpha * dz[i];
                for (i = 0; i < 6; i++)
                    work.y[i] += alpha * dy[i];
                work.gap = eval_gap();
                work.eq_resid_squared = calc_eq_resid_squared();
                work.ineq_resid_squared = calc_ineq_resid_squared();
#ifndef ZERO_LIBRARY_MODE
                if (settings.verbose) {
                    work.optval = eval_objv();
                    printf("%3d   %10.3e  %9.2e  %9.2e  %9.2e  % 6.4f\n",
                        iter + 1, work.optval, work.gap, sqrt(work.eq_resid_squared),
                        sqrt(work.ineq_resid_squared), alpha);
                }
#endif
                /* Test termination conditions. Requires optimality, and satisfied */
                /* constraints. */
                if ((work.gap < settings.eps)
                    && (work.eq_resid_squared <= settings.resid_tol * settings.resid_tol)
                    && (work.ineq_resid_squared <= settings.resid_tol * settings.resid_tol)) {
                    work.converged = 1;
                    work.optval = eval_objv();
                    return iter + 1;
                }
            }
            return iter;
        }

        void printmatrix(char* name, double* A, int m, int n, int sparse)
        {
            int i, j;
            printf("%s = [...\n", name);
            for (i = 0; i < m; i++) {
                for (j = 0; j < n; j++)
                    if ((sparse == 1) && (A[i + j * m] == 0))
                        printf("         0");
                    else
                        printf("  % 9.4f", A[i + j * m]);
                printf(",\n");
            }
            printf("];\n");
        }

        double unif(double lower, double upper)
        {
            return lower + ((upper - lower) * rand()) / RAND_MAX;
        }

        float ran1(long* idum, int reset)
        {
            int j;
            long k;
            static long iy = 0;
            static long iv[NTAB];
            float temp;
            if (reset) {
                iy = 0;
            }
            if (*idum <= 0 || !iy) {
                if (-(*idum) < 1)
                    *idum = 1;
                else
                    *idum = -(*idum);
                for (j = NTAB + 7; j >= 0; j--) {
                    k = (*idum) / IQ;
                    *idum = IA * (*idum - k * IQ) - IR * k;
                    if (*idum < 0)
                        *idum += IM;
                    if (j < NTAB)
                        iv[j] = *idum;
                }
                iy = iv[0];
            }
            k = (*idum) / IQ;
            *idum = IA * (*idum - k * IQ) - IR * k;
            if (*idum < 0)
                *idum += IM;
            j = iy / NDIV;
            iy = iv[j];
            iv[j] = *idum;
            if ((temp = AM * iy) > RNMX)
                return RNMX;
            else
                return temp;
        }

        float randn_internal(long* idum, int reset)
        {
            static int iset = 0;
            static float gset;
            float fac, rsq, v1, v2;
            if (reset) {
                iset = 0;
            }
            if (iset == 0) {
                do {
                    v1 = 2.0 * ran1(idum, reset) - 1.0;
                    v2 = 2.0 * ran1(idum, reset) - 1.0;
                    rsq = v1 * v1 + v2 * v2;
                } while (rsq >= 1.0 || rsq == 0.0);
                fac = sqrt(-2.0 * log(rsq) / rsq);
                gset = v1 * fac;
                iset = 1;
                return v2 * fac;
            }
            else {
                iset = 0;
                return gset;
            }
        }

        double randn(void)
        {
            return randn_internal(&global_seed, 0);
        }

        void reset_rand(void)
        {
            srand(15);
            global_seed = 1;
            randn_internal(&global_seed, 1);
        }
    };
} // namespace iiwa_ik_cvxgen

#endif