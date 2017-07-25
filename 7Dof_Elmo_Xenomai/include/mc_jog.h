#ifndef __MC_JOG__
#define __MC_JOG__

#include "global_variables.h"
#include "elmo_ecat.h"

class mc_jog
{
public:
    mc_jog(elmo_ecat *_axis);
    ~mc_jog();

    void run(bool MovFw=false,bool MovBw=false,double _dVel=0, double _dMaxAcc=0,double _dJerk=0);

public:

    bool enable;
    bool busy;
    bool error;
    bool errorid;
    bool done;

private:
    double moving();
    double quitting();

    elmo_ecat *axis;
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

    double dT11, dT12, dT1;
    double dAccM;
    // Jerk,Acc,Vel,Pos
    double dJ,dA,dV,dP;
    // Record the current Time,Acc,Vel,Pos
    double dCurTime,dCurAcc,dCurVel,dCurPos;
    bool bStopped;
};


#endif
