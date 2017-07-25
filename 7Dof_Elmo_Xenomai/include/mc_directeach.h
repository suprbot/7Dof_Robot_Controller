#ifndef __MC_DIRECTEACH__
#define __MC_DIRECTEACH__

#include "global_variables.h"
#include "elmo_ecat.h"

void gravitycomp(const double m_link[7],const double mc_link[7][3],const double m_load,const double mc_load[3],const double joint[7],double gravity[7]);

class mc_directeach
{
public:
    mc_directeach(elmo_ecat *_axis[]);
    ~mc_directeach();

    void run(bool bExec);

public:

    bool enable;
    bool busy;
    bool error;
    bool errorid;
    bool done;

private:
    elmo_ecat **axis;
    int step;
    int cyc_temp;
    int cyc_relative;
    int cyc_done;
    double actpos;

    //
    double axis2_StillOffset;
    double axis6_StillOffset;
    int axis_stopflag;
    bool reset;

    double axis_SmoothDirect[7];
    int axis_Direct[7];
    int axis_Dflag[7];
    int axis_Vflag[7];
    double axis_TorGravity[7];

    double jointpostemp[7];
    int axis4_Sflag;     					//为轴4加上一个弹簧阻尼作为限位。当这个标志位是1的时候就是超过限位，弹簧阻尼开始作用。
    int axis4_Sk;        					//轴4弹簧的刚度系数
    double axis4_PosError;
    int axis2_Sflag;
    double axis2_PosError;
    int axis2_Sk;

    double jointpos[7];
    double jointvelo[7];

    int axis_TargetTorque[7];
};


#endif
