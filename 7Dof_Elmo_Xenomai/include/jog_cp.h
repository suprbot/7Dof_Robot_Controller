#ifndef __JOG_CP__
#define __JOG_CP__

#include "global_variables.h"

class jog_cp
{
public:
    jog_cp();
    ~jog_cp();

    double run(bool MovFw=false,bool MovBw=false,double _dVel=0, double _dMaxAcc=0,double _dJerk=0);

public:
    bool cping;
    bool error;
    bool bquit;

private:
    double moving();
    double quitting();


    int step;
    int cyc_temp;
    int cyc_relative;

    double cppos;

    double dVel;
    double dMaxAcc;
    double dJerk;
    double dCyc;
    double dT;

    double dT11, dT12, dT1;
    double dAccM;
    // Jerk,Acc,Vel,Pos
    double dJ,dA,dV,dP;
    // Record the current Time,Acc,Vel,Pos
    double dCurTime,dCurAcc,dCurVel,dCurPos;
    bool bStopped;
};


#endif
