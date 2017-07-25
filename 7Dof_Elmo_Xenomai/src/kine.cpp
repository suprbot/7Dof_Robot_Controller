#include "kine.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
//#include <cmath>
//#include<cstdlib>

#define PI 3.1415926535897932384626433832795
const double EPS = 1e-6;
const double ZER = 1e-12;

//DH parameters
const double Dbs = 0.374;       //G1 0.374;     G2 0.3;     G3 0.31895;
const double Dse = 0.413;       //G1 0.413;     G2 0.4;     G3 0.400;
const double Dew = 0.3445;    //G1 0.3445;  	G2 0.4;     G3 0.400;
const double Dwt = 0.2405;     //G1 0.2405;   G2 0.245; G3 0.1428;

//joint limits
const double theta1_u = 180 / 180.0 * PI;
const double theta1_l = -180 / 180.0 * PI;
const double theta2_u = 120 / 180.0 * PI;
const double theta2_l = -120 / 180.0 * PI;
const double theta3_u = 180 / 180.0 * PI;
const double theta3_l = -180 / 180.0 * PI;
const double theta4_u = 120 / 180.0 * PI;
const double theta4_l = -120 / 180.0 * PI;
const double theta5_u = 180 / 180.0 * PI;
const double theta5_l = -180 / 180.0 * PI;
const double theta6_u = 120 / 180.0 * PI;
const double theta6_l = -120 / 180.0 * PI;
const double theta7_u = 180 / 180.0 * PI;
const double theta7_l = -180 / 180.0 * PI;

//rpy2r
void rpy2r(const double rpy[3],double r[3][3])
{
    r[0][0] = cos(rpy[1])*cos(rpy[2]);
    r[0][1] = -cos(rpy[1])*sin(rpy[2]);
    r[0][2] = sin(rpy[1]);
    r[1][0] = cos(rpy[0])*sin(rpy[2]) + cos(rpy[2])*sin(rpy[0])*sin(rpy[1]);
    r[1][1] = cos(rpy[0])*cos(rpy[2]) - sin(rpy[0])*sin(rpy[1])*sin(rpy[2]);
    r[1][2] = -cos(rpy[1])*sin(rpy[0]);
    r[2][0] = sin(rpy[0])*sin(rpy[2]) - cos(rpy[0])*cos(rpy[2])*sin(rpy[1]);
    r[2][1] = cos(rpy[2])*sin(rpy[0]) + cos(rpy[0])*sin(rpy[1])*sin(rpy[2]);
    r[2][2] = cos(rpy[0])*cos(rpy[1]);
}

//r2rpy
void r2rpy(const double r[3][3], double rpy[3])
{
    double sr,cr;
    if((fabs(r[2][2]) < 1E-8 && fabs(r[1][2]) < 1E-8))
    {
        // singularity
        rpy[0] = 0;  // roll is zero
        rpy[1] = atan2(r[0][2], r[2][2]);  // pitch
        rpy[2] = atan2(r[1][0], r[1][1]);  // yaw is sum of roll + yaw
    }
    else
    {
        rpy[0] = atan2(-r[1][2], r[2][2]);        // roll
        // compute sin / cos of roll angle
        sr = sin(rpy[0]);
        cr = cos(rpy[0]);
        rpy[1] = atan2(r[0][2], cr * r[2][2] - sr * r[1][2]);  // pitch
        rpy[2] = atan2(-r[0][1], r[0][0]);        // yaw
    }
}

//LU decomposition
void LUMethod(const double* A, const int& order, double** LU)
{
    for (int k = 0; k < order; k++)
    {
        for (int j = k; j < order; j++)
        {
            double sumForU = 0;
            for (int t = 0; t <= k - 1; t++)
            {
                sumForU += LU[k][t] * LU[t][j];
            }
            LU[k][j] = A[k*order + j] - sumForU;
        }
        for (int i = k + 1; i < order; i++)
        {
            double sumForL = 0;
            for (int t = 0; t <= k - 1; t++)
            {
                sumForL += LU[i][t] * LU[t][k];
            }
            LU[i][k] = (A[i*order + k] - sumForL) / LU[k][k];
        }
    }
}

//solve Ax=B by LU decomposition
void SolveLinearEquationByLU(double **LUMatrix, const double *b, const int &order, double *x)
{
    double *y = new double[order];
    for (int i = 0; i < order; i++)
    {
        double sumForY = 0;
        for (int t = 0; t <= i - 1; t++)
            sumForY += LUMatrix[i][t] * y[t];
        y[i] = b[i] - sumForY;
    }

    for (int i = order - 1; i >= 0; i--)
    {
        double sumForX = 0;
        for (int t = i + 1; t < order; t++)
            sumForX += LUMatrix[i][t] * x[t];
        x[i] = (y[i] - sumForX) / LUMatrix[i][i];
    }

    delete[] y;
}

//get center eigenvalue by inverse power method
void InversePowerMethod(const double *matrix, const int &order, const double &center, double &outEigenvalue, double *outEigenvector)
{
    //init some variables
    double *u0 = new double[order];
    double *u1 = new double[order];
    double *y0 = new double[order];
    double beta0 = 0, beta1 = 0;
    size_t size = sizeof(u0);
    memset(u0, 0, order*size);

    //init nonzero u0
    u0[0] = 1;

    //translation matrix
    double *matrix_trans = new double[order*order];
    for (int i = 0; i<order*order; i++)
    {
        matrix_trans[i] = matrix[i];
    }
    for (int i = 0; i<order; i++)
        matrix_trans[i*order + i] -= center;

    //matrix_trans LU decomposition
    double **LUMatrix = new double*[order];
    for (int i = 0; i<order; i++)
        LUMatrix[i] = new double[order];
    LUMethod(matrix_trans, order, LUMatrix);

    //****************************************************
    //main process of inverse power method
    do
    {
        beta0 = beta1;		//last beta
        memcpy(u0, u1, order*sizeof(double));


        double lengthOfU = 0.0;
        for (int i = 0; i < order; i++)
            lengthOfU += u0[i] * u0[i];
        lengthOfU = sqrt(lengthOfU);

        for (int i = 0; i<order; i++)
            y0[i] = u0[i] / lengthOfU;

        //A*u1 = y0,get u1
        SolveLinearEquationByLU(LUMatrix, y0, order, u1);

        beta1 = 0;
        for (int i = 0; i < order; i++)
            beta1 += y0[i] * u1[i];
    } while (fabs(beta1 - beta0) / fabs(beta1) > EPS);

    outEigenvalue = (double)1.0 / beta1 + center;
    memcpy(outEigenvector, y0, order*sizeof(double));
    //****************************************************

    delete[](u0, u1, y0);
    for (int i = 0; i<order; i++)
    {
        delete[] LUMatrix[i];
        LUMatrix[i] = 0;
    }
    delete[] LUMatrix;
}

//ψ region of joint_2 or joint_6
void region_cos_type(double a, double b, double c,
                     double joint_u, double joint_l, double* region)
{
    double joint_ul;
    if (joint_u > 0 && joint_l < 0)
    {
        joint_ul = fabs(joint_u) < fabs(joint_l) ? fabs(joint_u) : fabs(joint_l);
    }
    else
        return; //joint region error


    double cos_theta;
    cos_theta = cos(joint_ul);

    double psi_1, psi_2;
    double delta = pow(a, 2) + pow(b, 2) - pow(c - cos_theta, 2);

    if (delta < 0 || fabs(b - c + cos_theta) < ZER)
    {
        region[0] = 1;
        region[1] = -PI;
        region[2] = PI;
    }
    else
    {
        psi_1 = 2 * atan((a - sqrt(delta)) / (b - c + cos_theta));
        psi_2 = 2 * atan((a + sqrt(delta)) / (b - c + cos_theta));
        double psi_u, psi_l;
        psi_u = psi_1 > psi_2 ? psi_1 : psi_2;
        psi_l = psi_1 < psi_2 ? psi_1 : psi_2;

        double psi_mid = (psi_l + psi_u) / 2;
        double theta_at_mid = acos(a*sin(psi_mid) + b*cos(psi_mid) + c);
        if (theta_at_mid < joint_u)
            region[0] = 1;
        else
            region[0] = 2;

        region[1] = psi_l;
        region[2] = psi_u;
    }
}

//get the union of region_1 and region_2
void region_union(const double* region_1, const double* region_2, double* region)
{
    if (fabs(region_1[0] - 1) < ZER && fabs(region_2[0] - 1)<ZER)
    {
        region[0] = 1;
        region[1] = region_1[1] > region_2[1] ? region_1[1] : region_2[1];
        region[2] = region_1[2] < region_2[2] ? region_1[2] : region_2[2];
    }
    else if (fabs(region_1[0] - 2) < ZER && fabs(region_2[0] - 1) < ZER)
    {
        if ((region_2[2]<region_1[1]) || (region_2[1]>region_1[2]))
        {
            region[0] = 1;
            region[1] = region_2[1];
            region[2] = region_2[2];
        }
        else if ((region_2[1]<region_1[1]) && (region_2[2]<region_1[2]))
        {
            region[0] = 1;
            region[1] = region_2[1];
            region[2] = region_1[1];
        }
        else if ((region_2[1] > region_1[1]) && (region_2[2] > region_1[2]))
        {
            region[0] = 1;
            region[1] = region_1[2];
            region[2] = region_2[2];
        }
        else if ((region_2[1] < region_1[1]) && (region_2[2] > region_1[2]))
        {
            region[0] = 2;
            region[1] = region_2[1];
            region[2] = region_1[1];
            region[3] = region_2[2];
            region[4] = region_1[2];
        }
        else region[0] = 3;
    }
    else if (fabs(region_1[0] - 1) < ZER && fabs(region_2[0] - 2) < ZER)
    {
        if ((region_1[2]<region_2[1]) || (region_1[1]>region_2[2]))
        {
            region[0] = 1;
            region[1] = region_1[1];
            region[2] = region_1[2];
        }
        else if ((region_1[1]<region_2[1]) && (region_1[2]<region_2[2]))
        {
            region[0] = 1;
            region[1] = region_1[1];
            region[2] = region_2[1];
        }
        else if ((region_1[1] > region_2[1]) && (region_1[2] > region_2[2]))
        {
            region[0] = 1;
            region[1] = region_2[2];
            region[2] = region_1[2];
        }
        else if ((region_1[1] < region_2[1]) && (region_1[2] > region_2[2]))
        {
            region[0] = 2;
            region[1] = region_1[1];
            region[2] = region_2[1];
            region[3] = region_1[2];
            region[4] = region_2[2];
        }
        else region[0] = 3;
    }
}

//rotate of shoulder
void Rot_shoulder(const double shoulder[3], double R[3][3])
{
    R[0][0] = cos(shoulder[0])*cos(shoulder[1])*cos(shoulder[2]) - sin(shoulder[0])*sin(shoulder[2]);
    R[0][1] = -cos(shoulder[0])*sin(shoulder[1]);
    R[0][2] = -cos(shoulder[2])*sin(shoulder[0]) - cos(shoulder[0])*cos(shoulder[1])*sin(shoulder[2]);

    R[1][0] = cos(shoulder[0])*sin(shoulder[2]) + cos(shoulder[1])*cos(shoulder[2])*sin(shoulder[0]);
    R[1][1] = -sin(shoulder[0])*sin(shoulder[1]);
    R[1][2] = cos(shoulder[0])*cos(shoulder[2]) - cos(shoulder[1])*sin(shoulder[0])*sin(shoulder[2]);

    R[2][0] = -cos(shoulder[2])*sin(shoulder[1]);
    R[2][1] = -cos(shoulder[1]);
    R[2][2] = sin(shoulder[1])*sin(shoulder[2]);
}

//rotate of elbow
void Rot_elbow(double elbow, double R[3][3])
{
    R[0][0] = cos(elbow);	R[0][1] = 0;	R[0][2] = sin(elbow);
    R[1][0] = sin(elbow);	R[1][1] = 0;	R[1][2] = -cos(elbow);
    R[2][0] = 0;			R[2][1] = 1;	R[2][2] = 0;
}

//rotate of wrist
void Rot_wrist(const double wrist[3], double R[3][3])
{
    R[0][0] = cos(wrist[0])*cos(wrist[1])*cos(wrist[2]) - sin(wrist[0])*sin(wrist[2]);
    R[0][1] = -cos(wrist[2])*sin(wrist[0]) - cos(wrist[0])*cos(wrist[1])*sin(wrist[2]);
    R[0][2] = cos(wrist[0])*sin(wrist[1]);

    R[1][0] = cos(wrist[0])*sin(wrist[2]) + cos(wrist[1])*cos(wrist[2])*sin(wrist[0]);
    R[1][1] = cos(wrist[0])*cos(wrist[2]) - cos(wrist[1])*sin(wrist[0])*sin(wrist[2]);
    R[1][2] = sin(wrist[0])*sin(wrist[1]);

    R[2][0] = -cos(wrist[2])*sin(wrist[1]);
    R[2][1] = sin(wrist[1])*sin(wrist[2]);
    R[2][2] = cos(wrist[1]);
}

//forward kinematics
void Fkine(const double joint[7], double T[4][4])
{
    T[0][0] = cos(joint[6])*(sin(joint[5])*(sin(joint[3])*(sin(joint[0])*sin(joint[2]) - cos(joint[0])*cos(joint[1])*cos(joint[2])) - cos(joint[0])*cos(joint[3])*sin(joint[1])) - cos(joint[5])*(cos(joint[4])*(cos(joint[3])*(sin(joint[0])*sin(joint[2]) - cos(joint[0])*cos(joint[1])*cos(joint[2])) + cos(joint[0])*sin(joint[1])*sin(joint[3])) + sin(joint[4])*(cos(joint[2])*sin(joint[0]) + cos(joint[0])*cos(joint[1])*sin(joint[2])))) + sin(joint[6])*(sin(joint[4])*(cos(joint[3])*(sin(joint[0])*sin(joint[2]) - cos(joint[0])*cos(joint[1])*cos(joint[2])) + cos(joint[0])*sin(joint[1])*sin(joint[3])) - cos(joint[4])*(cos(joint[2])*sin(joint[0]) + cos(joint[0])*cos(joint[1])*sin(joint[2])));
    T[0][1] = cos(joint[6])*(sin(joint[4])*(cos(joint[3])*(sin(joint[0])*sin(joint[2]) - cos(joint[0])*cos(joint[1])*cos(joint[2])) + cos(joint[0])*sin(joint[1])*sin(joint[3])) - cos(joint[4])*(cos(joint[2])*sin(joint[0]) + cos(joint[0])*cos(joint[1])*sin(joint[2]))) - sin(joint[6])*(sin(joint[5])*(sin(joint[3])*(sin(joint[0])*sin(joint[2]) - cos(joint[0])*cos(joint[1])*cos(joint[2])) - cos(joint[0])*cos(joint[3])*sin(joint[1])) - cos(joint[5])*(cos(joint[4])*(cos(joint[3])*(sin(joint[0])*sin(joint[2]) - cos(joint[0])*cos(joint[1])*cos(joint[2])) + cos(joint[0])*sin(joint[1])*sin(joint[3])) + sin(joint[4])*(cos(joint[2])*sin(joint[0]) + cos(joint[0])*cos(joint[1])*sin(joint[2]))));
    T[0][2] = -cos(joint[5])*(sin(joint[3])*(sin(joint[0])*sin(joint[2]) - cos(joint[0])*cos(joint[1])*cos(joint[2])) - cos(joint[0])*cos(joint[3])*sin(joint[1])) - sin(joint[5])*(cos(joint[4])*(cos(joint[3])*(sin(joint[0])*sin(joint[2]) - cos(joint[0])*cos(joint[1])*cos(joint[2])) + cos(joint[0])*sin(joint[1])*sin(joint[3])) + sin(joint[4])*(cos(joint[2])*sin(joint[0]) + cos(joint[0])*cos(joint[1])*sin(joint[2])));
    T[0][3] = Dse*cos(joint[0])*sin(joint[1]) - Dwt*(cos(joint[5])*(sin(joint[3])*(sin(joint[0])*sin(joint[2]) - cos(joint[0])*cos(joint[1])*cos(joint[2])) - cos(joint[0])*cos(joint[3])*sin(joint[1])) + sin(joint[5])*(cos(joint[4])*(cos(joint[3])*(sin(joint[0])*sin(joint[2]) - cos(joint[0])*cos(joint[1])*cos(joint[2])) + cos(joint[0])*sin(joint[1])*sin(joint[3])) + sin(joint[4])*(cos(joint[2])*sin(joint[0]) + cos(joint[0])*cos(joint[1])*sin(joint[2])))) - Dew*(sin(joint[3])*(sin(joint[0])*sin(joint[2]) - cos(joint[0])*cos(joint[1])*cos(joint[2])) - cos(joint[0])*cos(joint[3])*sin(joint[1]));

    T[1][0] = -cos(joint[6])*(sin(joint[5])*(sin(joint[3])*(cos(joint[0])*sin(joint[2]) + cos(joint[1])*cos(joint[2])*sin(joint[0])) + cos(joint[3])*sin(joint[0])*sin(joint[1])) - cos(joint[5])*(cos(joint[4])*(cos(joint[3])*(cos(joint[0])*sin(joint[2]) + cos(joint[1])*cos(joint[2])*sin(joint[0])) - sin(joint[0])*sin(joint[1])*sin(joint[3])) + sin(joint[4])*(cos(joint[0])*cos(joint[2]) - cos(joint[1])*sin(joint[0])*sin(joint[2])))) - sin(joint[6])*(sin(joint[4])*(cos(joint[3])*(cos(joint[0])*sin(joint[2]) + cos(joint[1])*cos(joint[2])*sin(joint[0])) - sin(joint[0])*sin(joint[1])*sin(joint[3])) - cos(joint[4])*(cos(joint[0])*cos(joint[2]) - cos(joint[1])*sin(joint[0])*sin(joint[2])));
    T[1][1] = sin(joint[6])*(sin(joint[5])*(sin(joint[3])*(cos(joint[0])*sin(joint[2]) + cos(joint[1])*cos(joint[2])*sin(joint[0])) + cos(joint[3])*sin(joint[0])*sin(joint[1])) - cos(joint[5])*(cos(joint[4])*(cos(joint[3])*(cos(joint[0])*sin(joint[2]) + cos(joint[1])*cos(joint[2])*sin(joint[0])) - sin(joint[0])*sin(joint[1])*sin(joint[3])) + sin(joint[4])*(cos(joint[0])*cos(joint[2]) - cos(joint[1])*sin(joint[0])*sin(joint[2])))) - cos(joint[6])*(sin(joint[4])*(cos(joint[3])*(cos(joint[0])*sin(joint[2]) + cos(joint[1])*cos(joint[2])*sin(joint[0])) - sin(joint[0])*sin(joint[1])*sin(joint[3])) - cos(joint[4])*(cos(joint[0])*cos(joint[2]) - cos(joint[1])*sin(joint[0])*sin(joint[2])));
    T[1][2] = cos(joint[5])*(sin(joint[3])*(cos(joint[0])*sin(joint[2]) + cos(joint[1])*cos(joint[2])*sin(joint[0])) + cos(joint[3])*sin(joint[0])*sin(joint[1])) + sin(joint[5])*(cos(joint[4])*(cos(joint[3])*(cos(joint[0])*sin(joint[2]) + cos(joint[1])*cos(joint[2])*sin(joint[0])) - sin(joint[0])*sin(joint[1])*sin(joint[3])) + sin(joint[4])*(cos(joint[0])*cos(joint[2]) - cos(joint[1])*sin(joint[0])*sin(joint[2])));
    T[1][3] = Dew*(sin(joint[3])*(cos(joint[0])*sin(joint[2]) + cos(joint[1])*cos(joint[2])*sin(joint[0])) + cos(joint[3])*sin(joint[0])*sin(joint[1])) + Dwt*(cos(joint[5])*(sin(joint[3])*(cos(joint[0])*sin(joint[2]) + cos(joint[1])*cos(joint[2])*sin(joint[0])) + cos(joint[3])*sin(joint[0])*sin(joint[1])) + sin(joint[5])*(cos(joint[4])*(cos(joint[3])*(cos(joint[0])*sin(joint[2]) + cos(joint[1])*cos(joint[2])*sin(joint[0])) - sin(joint[0])*sin(joint[1])*sin(joint[3])) + sin(joint[4])*(cos(joint[0])*cos(joint[2]) - cos(joint[1])*sin(joint[0])*sin(joint[2])))) + Dse*sin(joint[0])*sin(joint[1]);

    T[2][0] = sin(joint[6])*(sin(joint[4])*(cos(joint[1])*sin(joint[3]) + cos(joint[2])*cos(joint[3])*sin(joint[1])) + cos(joint[4])*sin(joint[1])*sin(joint[2])) - cos(joint[6])*(cos(joint[5])*(cos(joint[4])*(cos(joint[1])*sin(joint[3]) + cos(joint[2])*cos(joint[3])*sin(joint[1])) - sin(joint[1])*sin(joint[2])*sin(joint[4])) + sin(joint[5])*(cos(joint[1])*cos(joint[3]) - cos(joint[2])*sin(joint[1])*sin(joint[3])));
    T[2][1] = cos(joint[6])*(sin(joint[4])*(cos(joint[1])*sin(joint[3]) + cos(joint[2])*cos(joint[3])*sin(joint[1])) + cos(joint[4])*sin(joint[1])*sin(joint[2])) + sin(joint[6])*(cos(joint[5])*(cos(joint[4])*(cos(joint[1])*sin(joint[3]) + cos(joint[2])*cos(joint[3])*sin(joint[1])) - sin(joint[1])*sin(joint[2])*sin(joint[4])) + sin(joint[5])*(cos(joint[1])*cos(joint[3]) - cos(joint[2])*sin(joint[1])*sin(joint[3])));
    T[2][2] = cos(joint[5])*(cos(joint[1])*cos(joint[3]) - cos(joint[2])*sin(joint[1])*sin(joint[3])) - sin(joint[5])*(cos(joint[4])*(cos(joint[1])*sin(joint[3]) + cos(joint[2])*cos(joint[3])*sin(joint[1])) - sin(joint[1])*sin(joint[2])*sin(joint[4]));
    T[2][3] = Dbs + Dew*(cos(joint[1])*cos(joint[3]) - cos(joint[2])*sin(joint[1])*sin(joint[3])) + Dse*cos(joint[1]) - Dwt*(sin(joint[5])*(cos(joint[4])*(cos(joint[1])*sin(joint[3]) + cos(joint[2])*cos(joint[3])*sin(joint[1])) - sin(joint[1])*sin(joint[2])*sin(joint[4])) - cos(joint[5])*(cos(joint[1])*cos(joint[3]) - cos(joint[2])*sin(joint[1])*sin(joint[3])));

    T[3][0] = 0;
    T[3][1] = 0;
    T[3][2] = 0;
    T[3][3] = 1;
}

void Fkine_rpy(const double joint[7], double end[6])
{

    double T[4][4],r[3][3],rpy[3];
    Fkine(joint,T);
    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            r[i][j]=T[i][j];
    r2rpy(r,rpy);
    end[0]=T[0][3];
    end[1]=T[1][3];
    end[2]=T[2][3];
    end[3]=rpy[0];
    end[4]=rpy[1];
    end[5]=rpy[2];
}

//inverse kinematics
void Ikine(const double T[4][4], double joint[7],double* psi_last)
{
//    if (fabs(joint[3])<0.035)   //+-2 angle
//        return;

//    if (fabs(joint[1])<1E-2 && fabs(joint[5])<1E-2)
//        return;

//    if (fabs(joint[0])>179 / 180.0*PI ||
//        fabs(joint[1])>109 / 180.0*PI ||
//        fabs(joint[2]) >179 / 180.0*PI ||
//        fabs(joint[3]) > 120 / 180.0*PI ||
//        fabs(joint[4]) > 179 / 180.0*PI ||
//        fabs(joint[5]) > 109 / 180.0*PI ||
//        fabs(joint[6]) > 179 / 180.0*PI)
//        return;



    //DH model
    double Lbs[3] = { 0, 0, Dbs };
    double Lse[3] = { 0, -Dse, 0 };
    double Lew[3] = { 0, 0, Dew };
    double Lwt[3] = { 0, 0, Dwt };

    //end: separate rotate and trans
    double end_r[3][3];
    double end_t[3];
    end_r[0][0] = T[0][0];	end_r[0][1] = T[0][1];	end_r[0][2] = T[0][2];	end_t[0] = T[0][3];
    end_r[1][0] = T[1][0];	end_r[1][1] = T[1][1];	end_r[1][2] = T[1][2];	end_t[1] = T[1][3];
    end_r[2][0] = T[2][0];	end_r[2][1] = T[2][1];	end_r[2][2] = T[2][2];	end_t[2] = T[2][3];


    //arm angle ��
    double w_norm, psi_cur, sintheta, costheta, sqrtcheck;

    double psi, psi_cache[2];
    double As_cache[2][3][3];
    double Bs_cache[2][3][3];
    double Cs_cache[2][3][3];
    double Aw_cache[2][3][3];
    double Bw_cache[2][3][3];
    double Cw_cache[2][3][3];

    //loops for 16/8 solutions at fixed ��
    int lp1 = 1;
    int lp2 = 1;
    int lp3 = 1;

    //save the 8 solutions
    double joint_temp[8][7];
    double joint_tar[7] = { 10, 10, 10, 10, 10, 10, 10 };

    double theta1, theta2, theta3, theta4, theta5, theta6, theta7;


    //joint angle when ��=0
    double theta1_0, theta2_0, theta3_0, theta4_0;
    theta3_0 = 0.0;
    double shoulder_0[3];
    double r43_0[3][3], r43_0_t[3][3];
    double r30_0[3][3];
    double Lsw[3];
    double Usw[3], Usw_m[3][3];
    double Uswx[3][3], Uswx_neg[3][3], Uswx_sq[3][3];
    double As[3][3], Bs[3][3], Cs[3][3], As_t[3][3], Bs_t[3][3], Cs_t[3][3];
    double Aw[3][3], Bw[3][3], Cw[3][3], Aw_temp[3][3], Bw_temp[3][3], Cw_temp[3][3];

    //best ��
    double r30_d[3][3], r74_d[3][3], r30_d_t[3][3], r74_d_t[3][3];
    double shoulder_d[3] = { 0.0, 0.0, 0.0 };
    double wrist_d[3] = { 0.0, 0.0, 0.0 };

    double AsR30dt[3][3], BsR30dt[3][3], CsR30dt[3][3], AwR74dt[3][3], BwR74dt[3][3], CwR74dt[3][3];
    double psi_best_1, psi_best_2, psi_best;
    double aaa, bbb, ccc;

    //region of ��
    double region_1[3], region_2[3], region[5]; //region_1&joint2   region2&joint6


    w_norm = sqrt(pow(Dse, 2) + pow(Dew, 2) + 2 * Dse*Dew*cos(joint[3]));
    sintheta = Dse*Dew*sin(joint[1])*sin(joint[2])*sin(joint[3]) / w_norm;
    costheta = -Dse / pow(w_norm, 2) * (-Dew * Dew * cos(joint[3]) * cos(joint[2]) * sin(joint[1]) * sin(joint[3]) + Dew * Dew * cos(joint[1]) * pow(cos(joint[3]), 2) - Dew * Dse * cos(joint[2]) * sin(joint[1]) * sin(joint[3]) + 2 * Dew * Dse * cos(joint[1]) * cos(joint[3]) + Dse * Dse * cos(joint[1]) - cos(joint[1]) * w_norm * w_norm);
    if (fabs(costheta) > 1.0)
        costheta = 1.0;

    if (fabs(sintheta)<ZER)			//psi_cur = ATAN2psi_cur_y, psi_cur_x);
    {
        if (costheta>0)
            psi_cur = 0.0;
        else
            psi_cur = PI;
    }
    else if (sintheta > 0)
        psi_cur = acos(costheta / sqrt(pow(sintheta, 2) + pow(costheta, 2)));
    else
        psi_cur = -acos(costheta / sqrt(pow(sintheta, 2) + pow(costheta, 2)));


    //Xsw
    double Xsw0[3];
    Xsw0[0] = end_t[0] - Lbs[0] - (end_r[0][0] * Lwt[0] + end_r[0][1] * Lwt[1] + end_r[0][2] * Lwt[2]);
    Xsw0[1] = end_t[1] - Lbs[1] - (end_r[1][0] * Lwt[0] + end_r[1][1] * Lwt[1] + end_r[1][2] * Lwt[2]);
    Xsw0[2] = end_t[2] - Lbs[2] - (end_r[2][0] * Lwt[0] + end_r[2][1] * Lwt[1] + end_r[2][2] * Lwt[2]);

    double Xsw0_L = sqrt(pow(Xsw0[0], 2) + pow(Xsw0[1], 2) + pow(Xsw0[2], 2));

    Usw[0] = Xsw0[0] / Xsw0_L; Usw[1] = Xsw0[1] / Xsw0_L; Usw[2] = Xsw0[2] / Xsw0_L;
    Uswx[0][0] = 0;			Uswx[0][1] = -Usw[2];	Uswx[0][2] = Usw[1];
    Uswx[1][0] = Usw[2];	Uswx[1][1] = 0;			Uswx[1][2] = -Usw[0];
    Uswx[2][0] = -Usw[1];	Uswx[2][1] = Usw[0];	Uswx[2][2] = 0;

    for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
    {
        Uswx_neg[i][j] = -Uswx[i][j];
    }

    //best ��
    Rot_shoulder(shoulder_d, r30_d);
    Rot_wrist(wrist_d, r74_d);


    /*************************start loop**************************/
    //loop1:			//for two solutions of theta4_0

    costheta = (pow(Xsw0_L, 2) - pow(Dse, 2) - pow(Dew, 2)) / (2 * Dse*Dew);
    if (fabs(costheta) > 1.0)
        costheta = 1.0;

    if (joint[3] > 0)
    {
        if (fabs(costheta) > 1)
            theta4_0 = 0;
        else
            theta4_0 = acos(costheta);
    }
    else
    {
        if (fabs(costheta) > 1)
            theta4_0 = 0;
        else
            theta4_0 = -acos(costheta);
    }

    theta4 = theta4_0;

    Rot_elbow(theta4_0, r43_0);
    Lsw[0] = Lse[0] + (r43_0[0][0] * Lew[0] + r43_0[0][1] * Lew[1] + r43_0[0][2] * Lew[2]);
    Lsw[1] = Lse[1] + (r43_0[1][0] * Lew[0] + r43_0[1][1] * Lew[1] + r43_0[1][2] * Lew[2]);
    Lsw[2] = Lse[2] + (r43_0[2][0] * Lew[0] + r43_0[2][1] * Lew[1] + r43_0[2][2] * Lew[2]);

loop1:				//for two solutions of theta2_0
    sqrtcheck = pow(Lsw[0], 2) + pow(Lsw[1], 2) - pow(Xsw0[2], 2);
    if (sqrtcheck < ZER)
        sqrtcheck = 0.0;

    if (1 == lp1)
    {
        theta2_0 = 2 * atan((Lsw[0] - sqrt(sqrtcheck)) / (Lsw[1] - Xsw0[2]));
        lp1 = 0;
    }
    else
    {
        theta2_0 = 2 * atan((Lsw[0] + sqrt(sqrtcheck)) / (Lsw[1] - Xsw0[2]));
        lp1 = 1;
    }



    sintheta = Xsw0[1] / (Lsw[0] * cos(theta2_0) - Lsw[1] * sin(theta2_0));
    costheta = Xsw0[0] / (Lsw[0] * cos(theta2_0) - Lsw[1] * sin(theta2_0));
    //theta1_0 = atan2(sintheta, costheta);     //calculated slower than calculation below
    if (fabs(costheta) > 1.0)
        costheta = 1.0;

    if (fabs(sintheta)<ZER)
    {
        if (costheta>0)
            theta1_0 = 0.0;
        else
            theta1_0 = PI;
    }
    else if (sintheta > 0)
        theta1_0 = acos(costheta);
    else
        theta1_0 = -acos(costheta);

    //As Bs Cs Aw Bw Cw
    shoulder_0[0] = theta1_0;
    shoulder_0[1] = theta2_0;
    shoulder_0[2] = theta3_0;
    Rot_shoulder(shoulder_0, r30_0);

    Mat_multi(Uswx, r30_0, As);
    Mat_multi(Uswx_neg, Uswx, Uswx_sq);
    Mat_multi(Uswx_sq, r30_0, Bs);

    Usw_m[0][0] = Usw[0] * Usw[0];	Usw_m[0][1] = Usw[0] * Usw[1];	Usw_m[0][2] = Usw[0] * Usw[2];
    Usw_m[1][0] = Usw[1] * Usw[0];	Usw_m[1][1] = Usw[1] * Usw[1];	Usw_m[1][2] = Usw[1] * Usw[2];
    Usw_m[2][0] = Usw[2] * Usw[0];	Usw_m[2][1] = Usw[2] * Usw[1];	Usw_m[2][2] = Usw[2] * Usw[2];
    Mat_multi(Usw_m, r30_0, Cs);

    Mat_transpose(r43_0, r43_0_t);
    Mat_transpose(As, As_t);
    Mat_transpose(Bs, Bs_t);
    Mat_transpose(Cs, Cs_t);
    Mat_multi(r43_0_t, As_t, Aw_temp);
    Mat_multi(Aw_temp, end_r, Aw);
    Mat_multi(r43_0_t, Bs_t, Bw_temp);
    Mat_multi(Bw_temp, end_r, Bw);
    Mat_multi(r43_0_t, Cs_t, Cw_temp);
    Mat_multi(Cw_temp, end_r, Cw);


    region_cos_type(-As[2][1], -Bs[2][1], -Cs[2][1], theta2_u, theta2_l, region_1);
    region_cos_type(Aw[2][2], Bw[2][2], Cw[2][2], theta6_u, theta6_l, region_2);
    region_union(region_1, region_2, region);

    Mat_transpose(r30_d, r30_d_t);
    Mat_transpose(r74_d, r74_d_t);
    Mat_multi(As, r30_d_t, AsR30dt);
    Mat_multi(Bs, r30_d_t, BsR30dt);
    Mat_multi(Cs, r30_d_t, CsR30dt);
    Mat_multi(Aw, r74_d_t, AwR74dt);
    Mat_multi(Bw, r74_d_t, BwR74dt);
    Mat_multi(Cw, r74_d_t, CwR74dt);

    aaa = (3 * (AsR30dt[0][0] + AsR30dt[1][1] + AsR30dt[2][2]) + 2 * (AwR74dt[0][0] + AwR74dt[1][1] + AwR74dt[2][2])) / 5;
    bbb = (3 * (BsR30dt[0][0] + BsR30dt[1][1] + BsR30dt[2][2]) + 2 * (BwR74dt[0][0] + BwR74dt[1][1] + BwR74dt[2][2])) / 5;
    ccc = (3 * (CsR30dt[0][0] + CsR30dt[1][1] + CsR30dt[2][2]) + 2 * (CwR74dt[0][0] + CwR74dt[1][1] + CwR74dt[2][2])) / 5;

    if (fabs(aaa)<ZER)
    {
        psi_best = 0;
    }
    else
    {
        psi_best_1 = 2 * atan((-bbb - sqrt(pow(aaa, 2) + pow(bbb, 2))) / aaa);
        psi_best_2 = 2 * atan((-bbb + sqrt(pow(aaa, 2) + pow(bbb, 2))) / aaa);
        psi_best = (aaa*sin(psi_best_1) + bbb*cos(psi_best_1) + ccc) > (aaa*sin(psi_best_2) + bbb*cos(psi_best_2) + ccc) ? psi_best_1 : psi_best_2;
    }

    if (fabs(region[0] - 1)<ZER)
    {
        if (psi_best>region[1] && psi_best < region[2])
        {
            psi = psi_best;
        }
        else if (psi_best<region[1])
        {
            psi = region[1];
        }
        else psi = region[2];
    }
    else
    {
        if ((psi_best>region[1] && psi_best < region[2]) || (psi_best>region[3] && psi_best < region[4]))
            psi = psi_best;
        else if (psi_best < region[1])
            psi = region[1];
        else if (psi_best > region[4])
            psi = region[4];
        else
            psi = fabs(region[2] - psi_best) < fabs(region[3] - psi_best) ? region[2] : region[3];
    }

    psi_cache[lp1] = psi;

    memcpy(As_cache + lp1 * 9 * sizeof(double), As, 9 * sizeof(double));
    memcpy(Bs_cache + lp1 * 9 * sizeof(double), Bs, 9 * sizeof(double));
    memcpy(Cs_cache + lp1 * 9 * sizeof(double), Cs, 9 * sizeof(double));
    memcpy(Aw_cache + lp1 * 9 * sizeof(double), Aw, 9 * sizeof(double));
    memcpy(Bw_cache + lp1 * 9 * sizeof(double), Bw, 9 * sizeof(double));
    memcpy(Cw_cache + lp1 * 9 * sizeof(double), Cw, 9 * sizeof(double));

    if (fabs(*psi_last - psi_cur) > 0.01)
        *psi_last = psi_cur;

    if (false)
        psi = *psi_last;
    else
    {
        if (fabs(psi - *psi_last) > 0.00001)
            psi = *psi_last + fabs(psi - *psi_last) / (psi - *psi_last)*0.00001;
        *psi_last = psi;
    }


loop2:				//for two solutions of theta2
    //theta2
    costheta = -As[2][1] * sin(psi) - Bs[2][1] * cos(psi) - Cs[2][1];

    if (fabs(costheta) > 1.0)
        costheta = fabs(costheta) / costheta;

    if (1 == lp2)
    {
        theta2 = acos(costheta);
        lp2 = 0;
    }
    else
    {
        theta2 = -acos(costheta);
        lp2 = 1;
    }

    //theta1
    sintheta = (-As[1][1] * sin(psi) - Bs[1][1] * cos(psi) - Cs[1][1]) / sin(theta2);
    costheta = (-As[0][1] * sin(psi) - Bs[0][1] * cos(psi) - Cs[0][1]) / sin(theta2);
    if (fabs(costheta) > 1.0)
        costheta = 1.0;

    if (fabs(sintheta)<ZER)
    {
        if (costheta>0)
            theta1 = 0.0;
        else
            theta1 = PI;
    }
    else if (sintheta > 0)
        theta1 = acos(costheta);
    else
        theta1 = -acos(costheta);

    //theta3
    sintheta = (As[2][2] * sin(psi) + Bs[2][2] * cos(psi) + Cs[2][2]) / sin(theta2);
    costheta = (-As[2][0] * sin(psi) - Bs[2][0] * cos(psi) - Cs[2][0]) / sin(theta2);
    if (fabs(costheta) > 1.0)
        costheta = 1.0;

    if (fabs(sintheta)<ZER)
    {
        if (costheta>0)
            theta3 = 0.0;
        else
            theta3 = PI;
    }
    else if (sintheta > 0)
        theta3 = acos(costheta);
    else
        theta3 = -acos(costheta);

loop3:				//for two solutions of theta6
    costheta = Aw[2][2] * sin(psi) + Bw[2][2] * cos(psi) + Cw[2][2];
    if (fabs(costheta) > 1.0)
        costheta = 1.0;
    //theta6
    if (1 == lp3)
    {
        theta6 = acos(costheta);
        lp3 = 0;
    }
    else
    {
        theta6 = -acos(costheta);
        lp3 = 1;
    }

    //theta5
    sintheta = (Aw[1][2] * sin(psi) + Bw[1][2] * cos(psi) + Cw[1][2]) / sin(theta6);
    costheta = (Aw[0][2] * sin(psi) + Bw[0][2] * cos(psi) + Cw[0][2]) / sin(theta6);

    if (fabs(costheta) > 1.0)
        costheta = 1.0;

    if (fabs(sintheta)<ZER)
    {
        if (costheta>0)
            theta5 = 0.0;
        else
            theta5 = PI;
    }
    else if (sintheta > 0)
        theta5 = acos(costheta);
    else
        theta5 = -acos(costheta);

    //theta7
    sintheta = (Aw[2][1] * sin(psi) + Bw[2][1] * cos(psi) + Cw[2][1]) / sin(theta6);
    costheta = (-Aw[2][0] * sin(psi) - Bw[2][0] * cos(psi) - Cw[2][0]) / sin(theta6);

    if (fabs(costheta) > 1.0)
        costheta = 1.0;

    if (fabs(sintheta)<ZER)
    {
        if (costheta>0)
            theta7 = 0.0;
        else
            theta7 = PI;
    }
    if (fabs(sintheta)<ZER)
        theta7 = 0.0;
    else if (sintheta>0)
        theta7 = acos(costheta);
    else
        theta7 = -acos(costheta);

    //save all solutions
    joint_temp[4 * lp1 + 2 * lp2 + lp3][0] = theta1;
    joint_temp[4 * lp1 + 2 * lp2 + lp3][1] = theta2;
    joint_temp[4 * lp1 + 2 * lp2 + lp3][2] = theta3;
    joint_temp[4 * lp1 + 2 * lp2 + lp3][3] = theta4;
    joint_temp[4 * lp1 + 2 * lp2 + lp3][4] = theta5;
    joint_temp[4 * lp1 + 2 * lp2 + lp3][5] = theta6;
    joint_temp[4 * lp1 + 2 * lp2 + lp3][6] = theta7;


    if (0 == lp3)	goto loop3;
    if (0 == lp2)	goto loop2;
    if (0 == lp1)	goto loop1;
    /************************end loop*************************/

    for (int i = 0; i < 8; i++)
    {
        if (fabs(joint_temp[i][0] - joint[0]) < 1E-1 &&
            fabs(joint_temp[i][1] - joint[1]) < 1E-1 &&
            fabs(joint_temp[i][2] - joint[2]) < 1E-1 &&
            fabs(joint_temp[i][3] - joint[3]) < 1E-1 &&
            fabs(joint_temp[i][4] - joint[4]) < 1E-1 &&
            fabs(joint_temp[i][5] - joint[5]) < 1E-1 &&
            fabs(joint_temp[i][6] - joint[6]) < 1E-1)
        {
            if (sqrt(7 * pow(joint_temp[i][0] - joint[0], 2) +
                6 * pow(joint_temp[i][1] - joint[1], 2) +
                5 * pow(joint_temp[i][2] - joint[2], 2) +
                4 * pow(joint_temp[i][3] - joint[3], 2) +
                3 * pow(joint_temp[i][4] - joint[4], 2) +
                2 * pow(joint_temp[i][5] - joint[5], 2) +
                1 * pow(joint_temp[i][6] - joint[6], 2)) <
                sqrt(7 * pow(joint_tar[0] - joint[0], 2) +
                6 * pow(joint_tar[1] - joint[1], 2) +
                5 * pow(joint_tar[2] - joint[2], 2) +
                4 * pow(joint_tar[3] - joint[3], 2) +
                3 * pow(joint_tar[4] - joint[4], 2) +
                2 * pow(joint_tar[5] - joint[5], 2) +
                1 * pow(joint_tar[6] - joint[6], 2)))
                joint_tar[0] = joint_temp[i][0];
            joint_tar[1] = joint_temp[i][1];
            joint_tar[2] = joint_temp[i][2];
            joint_tar[3] = joint_temp[i][3];
            joint_tar[4] = joint_temp[i][4];
            joint_tar[5] = joint_temp[i][5];
            joint_tar[6] = joint_temp[i][6];
        }
    }

    joint[0] = joint_tar[0];
    joint[1] = joint_tar[1];
    joint[2] = joint_tar[2];
    joint[3] = joint_tar[3];
    joint[4] = joint_tar[4];
    joint[5] = joint_tar[5];
    joint[6] = joint_tar[6];
}

void Ikine_rpy(const double end[6], double joint[7], double* psi_last)
{
    double rpy[3],r[3][3],T[4][4];
    rpy[0]=end[3];
    rpy[1]=end[4];
    rpy[2]=end[5];
    rpy2r(rpy,r);

    for(int i=0;i<3;i++)
        for(int j=0;j<3;j++)
            T[i][j]=r[i][j];

    T[0][3]=end[0];
    T[1][3]=end[1];
    T[2][3]=end[2];

    T[3][0]=0;
    T[3][1]=0;
    T[3][2]=0;
    T[3][3]=1;

    Ikine(T,joint,psi_last);
}

void Jacob(const double joint[7], double J[6][7])
{
    J[0][0] = -Dwt*sin(joint[5])*sin(joint[0])*cos(joint[1])*cos(joint[2])*cos(joint[3])*cos(joint[4]) + Dwt*sin(joint[5])*sin(joint[3])*sin(joint[0])*sin(joint[1])*cos(joint[4]) + Dwt*sin(joint[5])*sin(joint[0])*sin(joint[2])*cos(joint[1])*sin(joint[4]) - Dwt*sin(joint[5])*sin(joint[2])*cos(joint[0])*cos(joint[3])*cos(joint[4]) - Dwt*sin(joint[3])*sin(joint[0])*cos(joint[1])*cos(joint[2])*cos(joint[5]) - Dew*sin(joint[3])*sin(joint[0])*cos(joint[1])*cos(joint[2]) - Dwt*sin(joint[5])*cos(joint[0])*cos(joint[2])*sin(joint[4]) - Dwt*sin(joint[3])*sin(joint[2])*cos(joint[0])*cos(joint[5]) - Dwt*sin(joint[0])*cos(joint[3])*sin(joint[1])*cos(joint[5]) - Dew*sin(joint[3])*sin(joint[2])*cos(joint[0]) - Dew*sin(joint[0])*cos(joint[3])*sin(joint[1]) - Dse*sin(joint[0])*sin(joint[1]);
    J[0][1] = -cos(joint[0])*(Dwt*sin(joint[5])*cos(joint[2])*cos(joint[3])*sin(joint[1])*cos(joint[4]) + Dwt*sin(joint[5])*sin(joint[3])*cos(joint[1])*cos(joint[4]) - Dwt*sin(joint[5])*sin(joint[2])*sin(joint[1])*sin(joint[4]) + Dwt*sin(joint[3])*cos(joint[2])*sin(joint[1])*cos(joint[5]) + Dew*sin(joint[3])*cos(joint[2])*sin(joint[1]) - Dwt*cos(joint[1])*cos(joint[3])*cos(joint[5]) - Dew*cos(joint[1])*cos(joint[3]) - Dse*cos(joint[1]));
    J[0][2] = -Dwt*sin(joint[5])*sin(joint[2])*cos(joint[0])*cos(joint[1])*cos(joint[3])*cos(joint[4]) - Dwt*sin(joint[5])*sin(joint[0])*cos(joint[2])*cos(joint[3])*cos(joint[4]) - Dwt*sin(joint[5])*cos(joint[0])*cos(joint[1])*cos(joint[2])*sin(joint[4]) - Dwt*sin(joint[3])*sin(joint[2])*cos(joint[0])*cos(joint[1])*cos(joint[5]) - Dew*sin(joint[3])*sin(joint[2])*cos(joint[0])*cos(joint[1]) + Dwt*sin(joint[5])*sin(joint[0])*sin(joint[2])*sin(joint[4]) - Dwt*sin(joint[3])*sin(joint[0])*cos(joint[2])*cos(joint[5]) - Dew*sin(joint[3])*sin(joint[0])*cos(joint[2]);
    J[0][3] = -Dwt*sin(joint[5])*sin(joint[3])*cos(joint[0])*cos(joint[1])*cos(joint[2])*cos(joint[4]) + Dwt*sin(joint[5])*sin(joint[3])*sin(joint[0])*sin(joint[2])*cos(joint[4]) - Dwt*sin(joint[5])*cos(joint[0])*cos(joint[3])*sin(joint[1])*cos(joint[4]) + Dwt*cos(joint[0])*cos(joint[1])*cos(joint[2])*cos(joint[3])*cos(joint[5]) + Dew*cos(joint[0])*cos(joint[1])*cos(joint[2])*cos(joint[3]) - Dwt*sin(joint[3])*cos(joint[0])*sin(joint[1])*cos(joint[5]) - Dwt*sin(joint[0])*sin(joint[2])*cos(joint[3])*cos(joint[5]) - Dew*sin(joint[3])*cos(joint[0])*sin(joint[1]) - Dew*sin(joint[0])*sin(joint[2])*cos(joint[3]);
    J[0][4] = Dwt*sin(joint[5])*(-cos(joint[0])*cos(joint[1])*cos(joint[2])*cos(joint[3])*sin(joint[4]) + cos(joint[0])*sin(joint[1])*sin(joint[3])*sin(joint[4]) + cos(joint[3])*sin(joint[0])*sin(joint[2])*sin(joint[4]) - cos(joint[0])*cos(joint[1])*cos(joint[4])*sin(joint[2]) - cos(joint[2])*cos(joint[4])*sin(joint[0]));
    J[0][5] = Dwt*(cos(joint[0])*cos(joint[1])*cos(joint[2])*cos(joint[3])*cos(joint[4])*cos(joint[5]) - cos(joint[0])*cos(joint[1])*cos(joint[2])*sin(joint[3])*sin(joint[5]) - cos(joint[0])*cos(joint[4])*cos(joint[5])*sin(joint[1])*sin(joint[3]) - cos(joint[3])*cos(joint[4])*cos(joint[5])*sin(joint[0])*sin(joint[2]) - cos(joint[0])*cos(joint[1])*cos(joint[5])*sin(joint[2])*sin(joint[4]) + sin(joint[0])*sin(joint[2])*sin(joint[3])*sin(joint[5]) - cos(joint[0])*cos(joint[3])*sin(joint[1])*sin(joint[5]) - cos(joint[2])*cos(joint[5])*sin(joint[0])*sin(joint[4]));
    J[0][6] = 0;

    J[1][0] = Dwt*cos(joint[0])*cos(joint[1])*cos(joint[2])*cos(joint[3])*sin(joint[5])*cos(joint[4]) + Dwt*cos(joint[5])*sin(joint[3])*cos(joint[0])*cos(joint[1])*cos(joint[2]) - Dwt*sin(joint[3])*cos(joint[0])*sin(joint[1])*sin(joint[5])*cos(joint[4]) - Dwt*cos(joint[0])*sin(joint[2])*cos(joint[1])*sin(joint[5])*sin(joint[4]) - Dwt*sin(joint[2])*sin(joint[0])*cos(joint[3])*sin(joint[5])*cos(joint[4]) + Dew*sin(joint[3])*cos(joint[0])*cos(joint[1])*cos(joint[2]) - Dwt*cos(joint[5])*sin(joint[3])*sin(joint[2])*sin(joint[0]) + Dwt*cos(joint[5])*cos(joint[0])*cos(joint[3])*sin(joint[1]) - Dwt*cos(joint[2])*sin(joint[0])*sin(joint[5])*sin(joint[4]) - Dew*sin(joint[3])*sin(joint[2])*sin(joint[0]) + Dew*cos(joint[0])*cos(joint[3])*sin(joint[1]) + Dse*cos(joint[0])*sin(joint[1]);
    J[1][1] = -sin(joint[0])*(Dwt*cos(joint[2])*cos(joint[3])*sin(joint[1])*sin(joint[5])*cos(joint[4]) + Dwt*cos(joint[5])*sin(joint[3])*cos(joint[2])*sin(joint[1]) + Dwt*sin(joint[3])*cos(joint[1])*sin(joint[5])*cos(joint[4]) - Dwt*sin(joint[2])*sin(joint[1])*sin(joint[5])*sin(joint[4]) + Dew*sin(joint[3])*cos(joint[2])*sin(joint[1]) - Dwt*cos(joint[5])*cos(joint[1])*cos(joint[3]) - Dew*cos(joint[1])*cos(joint[3]) - Dse*cos(joint[1]));
    J[1][2] = -Dwt*sin(joint[2])*cos(joint[1])*sin(joint[0])*cos(joint[3])*sin(joint[5])*cos(joint[4]) - Dwt*cos(joint[5])*sin(joint[3])*sin(joint[2])*cos(joint[1])*sin(joint[0]) + Dwt*cos(joint[0])*cos(joint[2])*cos(joint[3])*sin(joint[5])*cos(joint[4]) - Dwt*cos(joint[1])*cos(joint[2])*sin(joint[0])*sin(joint[5])*sin(joint[4]) - Dew*sin(joint[3])*sin(joint[2])*cos(joint[1])*sin(joint[0]) + Dwt*cos(joint[5])*sin(joint[3])*cos(joint[0])*cos(joint[2]) - Dwt*cos(joint[0])*sin(joint[2])*sin(joint[5])*sin(joint[4]) + Dew*sin(joint[3])*cos(joint[0])*cos(joint[2]);
    J[1][3] = -Dwt*cos(joint[4])*sin(joint[5])*sin(joint[3])*cos(joint[1])*cos(joint[2])*sin(joint[0]) - Dwt*cos(joint[4])*sin(joint[5])*sin(joint[3])*cos(joint[0])*sin(joint[2]) - Dwt*cos(joint[4])*sin(joint[5])*sin(joint[0])*cos(joint[3])*sin(joint[1]) + Dwt*cos(joint[5])*cos(joint[1])*cos(joint[2])*sin(joint[0])*cos(joint[3]) + Dew*cos(joint[1])*cos(joint[2])*sin(joint[0])*cos(joint[3]) - Dwt*cos(joint[5])*sin(joint[3])*sin(joint[0])*sin(joint[1]) + Dwt*cos(joint[5])*cos(joint[0])*sin(joint[2])*cos(joint[3]) - Dew*sin(joint[3])*sin(joint[0])*sin(joint[1]) + Dew*cos(joint[0])*sin(joint[2])*cos(joint[3]);
    J[1][4] = Dwt*sin(joint[5])*(-cos(joint[1])*cos(joint[2])*cos(joint[3])*sin(joint[0])*sin(joint[4]) - cos(joint[1])*cos(joint[4])*sin(joint[0])*sin(joint[2]) + sin(joint[0])*sin(joint[1])*sin(joint[3])*sin(joint[4]) - cos(joint[0])*cos(joint[3])*sin(joint[2])*sin(joint[4]) + cos(joint[0])*cos(joint[2])*cos(joint[4]));
    J[1][5] = -Dwt*(cos(joint[3])*sin(joint[0])*sin(joint[1])*sin(joint[5]) - cos(joint[0])*cos(joint[2])*cos(joint[5])*sin(joint[4]) + cos(joint[0])*sin(joint[2])*sin(joint[3])*sin(joint[5]) - cos(joint[0])*cos(joint[3])*cos(joint[4])*cos(joint[5])*sin(joint[2]) + cos(joint[1])*cos(joint[2])*sin(joint[0])*sin(joint[3])*sin(joint[5]) + cos(joint[1])*cos(joint[5])*sin(joint[0])*sin(joint[2])*sin(joint[4]) + cos(joint[4])*cos(joint[5])*sin(joint[0])*sin(joint[1])*sin(joint[3]) - cos(joint[1])*cos(joint[2])*cos(joint[3])*cos(joint[4])*cos(joint[5])*sin(joint[0]));
    J[1][6] = 0;

    J[2][0] = 0;
    J[2][1] = -Dwt*cos(joint[1])*cos(joint[2])*cos(joint[3])*cos(joint[4])*sin(joint[5]) + Dwt*sin(joint[4])*cos(joint[1])*sin(joint[2])*sin(joint[5]) - Dwt*cos(joint[1])*sin(joint[3])*cos(joint[2])*cos(joint[5]) + Dwt*sin(joint[3])*sin(joint[1])*cos(joint[4])*sin(joint[5]) - Dew*cos(joint[1])*sin(joint[3])*cos(joint[2]) - Dwt*cos(joint[3])*sin(joint[1])*cos(joint[5]) - Dew*cos(joint[3])*sin(joint[1]) - Dse*sin(joint[1]);
    J[2][2] = sin(joint[1])*(Dwt*cos(joint[3])*cos(joint[4])*sin(joint[2])*sin(joint[5]) + Dwt*sin(joint[4])*cos(joint[2])*sin(joint[5]) + Dwt*sin(joint[3])*sin(joint[2])*cos(joint[5]) + Dew*sin(joint[3])*sin(joint[2]));
    J[2][3] = Dwt*sin(joint[3])*cos(joint[2])*sin(joint[1])*cos(joint[4])*sin(joint[5]) - Dwt*cos(joint[1])*cos(joint[3])*cos(joint[4])*sin(joint[5]) - Dwt*cos(joint[2])*cos(joint[3])*sin(joint[1])*cos(joint[5]) - Dew*cos(joint[2])*cos(joint[3])*sin(joint[1]) - Dwt*cos(joint[1])*sin(joint[3])*cos(joint[5]) - Dew*cos(joint[1])*sin(joint[3]);
    J[2][4] = Dwt*sin(joint[5])*(cos(joint[4])*sin(joint[1])*sin(joint[2]) + cos(joint[1])*sin(joint[3])*sin(joint[4]) + cos(joint[2])*cos(joint[3])*sin(joint[1])*sin(joint[4]));
    J[2][5] = Dwt*(-cos(joint[2])*cos(joint[3])*cos(joint[4])*cos(joint[5])*sin(joint[1]) + cos(joint[2])*sin(joint[1])*sin(joint[3])*sin(joint[5]) - cos(joint[1])*cos(joint[4])*cos(joint[5])*sin(joint[3]) + cos(joint[5])*sin(joint[1])*sin(joint[2])*sin(joint[4]) - cos(joint[1])*cos(joint[3])*sin(joint[5]));
    J[2][6] = 0;

    J[3][0] = 0;
    J[3][1] = -sin(joint[0]);
    J[3][2] = cos(joint[0])*sin(joint[1]);
    J[3][3] = -cos(joint[0])*cos(joint[1])*sin(joint[2]) - cos(joint[2])*sin(joint[0]);
    J[3][4] = sin(joint[3])*cos(joint[0])*cos(joint[1])*cos(joint[2]) - sin(joint[3])*sin(joint[0])*sin(joint[2]) + cos(joint[0])*cos(joint[3])*sin(joint[1]);
    J[3][5] = cos(joint[0])*sin(joint[1])*sin(joint[3])*sin(joint[4]) - cos(joint[0])*cos(joint[1])*cos(joint[4])*sin(joint[2]) - cos(joint[2])*cos(joint[4])*sin(joint[0]) + cos(joint[3])*sin(joint[0])*sin(joint[2])*sin(joint[4]) - cos(joint[0])*cos(joint[1])*cos(joint[2])*cos(joint[3])*sin(joint[4]);
    J[3][6] = sin(joint[5])*cos(joint[0])*cos(joint[1])*cos(joint[2])*cos(joint[3])*cos(joint[4]) - sin(joint[5])*sin(joint[3])*cos(joint[0])*sin(joint[1])*cos(joint[4]) - sin(joint[5])*sin(joint[0])*sin(joint[2])*cos(joint[3])*cos(joint[4]) - sin(joint[5])*sin(joint[2])*cos(joint[0])*cos(joint[1])*sin(joint[4]) + sin(joint[3])*cos(joint[0])*cos(joint[1])*cos(joint[2])*cos(joint[5]) - sin(joint[5])*sin(joint[0])*cos(joint[2])*sin(joint[4]) - sin(joint[3])*sin(joint[0])*sin(joint[2])*cos(joint[5]) + cos(joint[0])*cos(joint[3])*sin(joint[1])*cos(joint[5]);

    J[4][0] = 0;
    J[4][1] = cos(joint[0]);
    J[4][2] = sin(joint[0])*sin(joint[1]);
    J[4][3] = cos(joint[0])*cos(joint[2]) - cos(joint[1])*sin(joint[0])*sin(joint[2]);
    J[4][4] = sin(joint[3])*cos(joint[1])*cos(joint[2])*sin(joint[0]) + sin(joint[3])*cos(joint[0])*sin(joint[2]) + cos(joint[3])*sin(joint[0])*sin(joint[1]);
    J[4][5] = cos(joint[0])*cos(joint[2])*cos(joint[4]) - cos(joint[1])*cos(joint[4])*sin(joint[0])*sin(joint[2]) - cos(joint[0])*cos(joint[3])*sin(joint[2])*sin(joint[4]) + sin(joint[0])*sin(joint[1])*sin(joint[3])*sin(joint[4]) - cos(joint[1])*cos(joint[2])*cos(joint[3])*sin(joint[0])*sin(joint[4]);
    J[4][6] = cos(joint[2])*cos(joint[4])*cos(joint[1])*sin(joint[0])*cos(joint[3])*sin(joint[5]) + cos(joint[0])*cos(joint[4])*sin(joint[2])*cos(joint[3])*sin(joint[5]) + cos(joint[2])*cos(joint[1])*sin(joint[0])*sin(joint[3])*cos(joint[5]) - cos(joint[4])*sin(joint[0])*sin(joint[1])*sin(joint[3])*sin(joint[5]) - cos(joint[1])*sin(joint[0])*sin(joint[2])*sin(joint[4])*sin(joint[5]) + cos(joint[0])*cos(joint[2])*sin(joint[4])*sin(joint[5]) + cos(joint[0])*sin(joint[2])*sin(joint[3])*cos(joint[5]) + sin(joint[0])*cos(joint[3])*sin(joint[1])*cos(joint[5]);

    J[5][0] = 1;
    J[5][1] = 0;
    J[5][2] = cos(joint[1]);
    J[5][3] = sin(joint[1])*sin(joint[2]);
    J[5][4] = cos(joint[1])*cos(joint[3]) - cos(joint[2])*sin(joint[1])*sin(joint[3]);
    J[5][5] = cos(joint[4])*sin(joint[1])*sin(joint[2]) + cos(joint[1])*sin(joint[3])*sin(joint[4]) + cos(joint[2])*cos(joint[3])*sin(joint[1])*sin(joint[4]);
    J[5][6] = cos(joint[4])*sin(joint[1])*cos(joint[2])*cos(joint[3])*sin(joint[5]) - cos(joint[4])*cos(joint[1])*sin(joint[3])*sin(joint[5]) + sin(joint[1])*sin(joint[2])*sin(joint[4])*sin(joint[5]) - sin(joint[1])*sin(joint[3])*cos(joint[2])*cos(joint[5]) + cos(joint[1])*cos(joint[3])*cos(joint[5]);
}

//manipulability
double Manip(const double J[6][7])
{
    double JJT[6][6];
    for (int i = 0; i < 6; i++)
        for (int j = 0; j < 6; j++)
        {
            JJT[i][j] = J[i][0] * J[j][0] + J[i][1] * J[j][1] + J[i][2] * J[j][2] + J[i][3] * J[j][3] + J[i][4] * J[j][4] + J[i][5] * J[j][5] + J[i][6] * J[j][6];
        }

    double matrix[6 * 6];
    double center = 0.0;
    double eigenvalue = 0.0;
    double eigenvector[6];

    memcpy(matrix, JJT, 6 * 6 * sizeof(double));
    InversePowerMethod(matrix, 6, center, eigenvalue, eigenvector);
    return eigenvalue;

}
