#include "mc_power.h"

mc_power::mc_power(elmo_ecat *_axis):
    axis(_axis),
    enable(false),
    busy(false),
    error(false),
    errorid(0),
    done(false),
    step(0)
{}

mc_power::~mc_power()
{}

void mc_power::run(bool exec)
{
    if(error) return;

    switch (step){
    case 0:
        if(exec)
        {
            busy=true;
            cyc_temp=cycle_counter;
            step++;
        }
        break;
    case 1:
        axis->cntlwd(0x80);
        if(axis->status()==592)
        {
            cyc_temp=cycle_counter;
            step++;
        }
        break;
    case 2:
        axis->cntlwd(0x06);
        if(axis->status()==561)
        {
            cyc_temp=cycle_counter;
            step++;
        }
        else
            if(cycle_counter-cyc_temp>100)
                error=true;
        break;
    case 3:
        axis->cntlwd(0x07);
        actpos=axis->actpos();
        axis->setbias(actpos);
        if(axis->status()==563)
        {
            cyc_temp=cycle_counter;
            step++;
        }
        else
            if(cycle_counter-cyc_temp>86)
                error=true;
        break;
    case 4:
        axis->cntlwd(0x0F);
        axis->tarpos(actpos);
        axis->setmaxpos_module(180);
        axis->setminpos_module(-180);
        if(axis->status()==4663)
        {
            cyc_temp=cycle_counter;
            step++;
        }
        else
            if(cycle_counter-cyc_temp>10)
                error=true;
        break;
    case 5:
        axis->cntlwd(0x1F);
        if(axis->status()==4663)
            step++;
        else
            if(cycle_counter-cyc_temp>10)
                error=true;
        break;
    case 6:
        busy=false;
        done=true;
        if(!exec)
        {
            axis->cntlwd(0x80);
            step=0;
            busy=false;
            error=false;
            errorid=0;
            done=false;
        }
        break;
    }
}
