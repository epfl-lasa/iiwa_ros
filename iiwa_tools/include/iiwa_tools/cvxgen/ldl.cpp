/* Produced by CVXGEN, 2019-02-27 17:57:27 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: ldl.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
namespace iiwa_ik_cvxgen {
    /* Be sure to place ldl_solve first, so storage schemes are defined by it. */
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
} // namespace iiwa_ik_cvxgen
