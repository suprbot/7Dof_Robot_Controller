#ifndef __ROBOT_JOG__
#define __ROBOT_JOG__

#include "global_variables.h"
#include "elmo_ecat.h"
#include "mc_jog.h"
#include "jog_cp.h"

class robot_jog
{
public:
    robot_jog(elmo_ecat*_axis[]);
    ~robot_jog();

    void run( bool _bExec=false,int _jogMode=0,int _jogVel=0,int* jogJoint=NULL,int* jogEnd=NULL);

public:
    bool enable;
    bool busy;
    bool error;
    bool errorid;
    bool done;

private:
    elmo_ecat** axis;

    int nState;
    mc_jog joint_mcjog[7];
    bool joint_fw[7];
    bool joint_bw[7];
    //void actjog();

    int step;
    int cyc_temp;
    int cyc_relative;
    int cyc_done;
    double actpos;
    double tarpos;

    bool dir;
    double dVel;
    double dMaxAcc;
    double dJerk;
    double dCyc;
    double dT;
    double dW, dT1, dT2;
    double dTCyc ;
    double dPos1,dPos2;

    double endStartpos[6];
    double endTarpos[6];
    double jointIkpos[7];
    double endTarPosM[6];
    double jointIkposArc[7];

    jog_cp endJogcp[6];
    bool end_fw[6];
    bool end_bw[6];
};


#endif
