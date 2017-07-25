#ifndef __KINE_H__
#define __KINE_H__

//rpy2r
void rpy2r(const double rpy[3],double r[3][3]);

//r2rpy
void r2rpy(const double r[3][3], double rpy[3]);

//LU decomposition
void LUMethod(const double* A, const int& order, double** LU);

//solve Ax=B by LU decomposition
void SolveLinearEquationByLU(double **LUMatrix, const double *b, const int &order, double *x);

//get center eigenvalue by inverse power method
void InversePowerMethod(const double *matrix, const int &order, const double &center, double &outEigenvalue, double *outEigenvector);

//ψ region of joint_2 or joint_6
void region_cos_type(double a, double b, double c,
        double joint_u, double joint_l, double* region);

//get the union of region_1 and region_2
void region_union(const double* region_1, const double* region_2, double* region);

//rotate of shoulder
void Rot_shoulder(const double shoulder[3], double R[3][3]);

//rotate of elbow
void Rot_elbow(double elbow, double R[3][3]);

//rotate of wrist
void Rot_wrist(const double wrist[3], double R[3][3]);

//forward kinematics
void Fkine(const double joint[7], double T[4][4]);
void Fkine_rpy(const double joint[7], double end[6]);

//inverse kinematics
void Ikine(const double T[4][4], double joint[7],double* psi_last);
void Ikine_rpy(const double end[6], double joint[7],double* psi_last);

//jacobian
void Jacob(const double joint[7], double J[6][7]);

//manipulability
double Manip(const double J[6][7]);

//Rl*Rr
inline void Mat_multi(const double Rl[3][3], const double Rr[3][3], double R[3][3])
{
        R[0][0] = Rl[0][0] * Rr[0][0] + Rl[0][1] * Rr[1][0] + Rl[0][2] * Rr[2][0];
        R[0][1] = Rl[0][0] * Rr[0][1] + Rl[0][1] * Rr[1][1] + Rl[0][2] * Rr[2][1];
        R[0][2] = Rl[0][0] * Rr[0][2] + Rl[0][1] * Rr[1][2] + Rl[0][2] * Rr[2][2];

        R[1][0] = Rl[1][0] * Rr[0][0] + Rl[1][1] * Rr[1][0] + Rl[1][2] * Rr[2][0];
        R[1][1] = Rl[1][0] * Rr[0][1] + Rl[1][1] * Rr[1][1] + Rl[1][2] * Rr[2][1];
        R[1][2] = Rl[1][0] * Rr[0][2] + Rl[1][1] * Rr[1][2] + Rl[1][2] * Rr[2][2];

        R[2][0] = Rl[2][0] * Rr[0][0] + Rl[2][1] * Rr[1][0] + Rl[2][2] * Rr[2][0];
        R[2][1] = Rl[2][0] * Rr[0][1] + Rl[2][1] * Rr[1][1] + Rl[2][2] * Rr[2][1];
        R[2][2] = Rl[2][0] * Rr[0][2] + Rl[2][1] * Rr[1][2] + Rl[2][2] * Rr[2][2];
}

//R.transports
inline void Mat_transpose(const double R[3][3], double Rt[3][3])
{
        for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
        {
                Rt[i][j] = R[j][i];
        }
}

#endif
