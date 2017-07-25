#ifndef __MC_POWER_H__
#define __MC_POWER_H__

#include "global_variables.h"
#include "elmo_ecat.h"

class mc_power
{
public:
    mc_power(elmo_ecat *_axis);
    ~mc_power();

    void run(bool);
public:
    elmo_ecat *axis;
    bool enable;
    bool busy;
    bool error;
    bool errorid;
    bool done;

private:
    int step;
    int cyc_temp;
    int actpos;
};


#endif
