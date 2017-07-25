#ifndef __MC_MOVEABSOLUTE__
#define __MC_MOVEABSOLUTE__

#include "global_variables.h"
#include "elmo_ecat.h"

class mc_moveabsolute
{
public:
    mc_moveabsolute(elmo_ecat *_axis);
    ~mc_moveabsolute();

    //void init(double _dPos, double _dVel, double _dMaxAcc);
    void run(bool exec=false,double _dPos=0, double _dVel=0, double _dMaxAcc=0);

public:

    bool enable;
    bool busy;
    bool error;
    bool errorid;
    bool done;

private:
    void planning();
    double moving();

    elmo_ecat *axis;
    int step;
    int cyc_temp;
    int cyc_relative;
    int cyc_done;
    double actpos;
    double tarpos;
    double actpos_bias;
    double tarpos_bias;

    bool dir;
    double dPos;
    double dVel;
    double dMaxAcc;
    double dCyc;
    double dW, dT1, dT2;
    double dTCyc ;
    double dPos1,dPos2;
};


#endif
