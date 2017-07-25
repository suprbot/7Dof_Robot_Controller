#include <math.h>
#include <stdio.h>

#include "mc_moveabsolute.h"

mc_moveabsolute::mc_moveabsolute(elmo_ecat *_axis):
    axis(_axis),
    enable(false),
    busy(false),
    error(false),
    errorid(0),
    done(false),
    step(0)
{}

mc_moveabsolute::~mc_moveabsolute()
{}

void mc_moveabsolute::run(bool exec,double _dPos, double _dVel, double _dMaxAcc)
{
    if(error) return;
    switch (step){
    case 0:
        if(exec)
        {
            busy=true;
            actpos=axis->actpos_modulo();
            actpos_bias=axis->actpos_bias_modulo();
            dPos=_dPos-actpos_bias;
            dir=dPos>0?true:false;
            dPos=fabs(dPos);
            dVel=fabs(_dVel);
            dMaxAcc=fabs(_dMaxAcc);
            planning();

            cyc_temp=cycle_counter;
            step++;
        }
        break;
    case 1:
        cyc_relative=cycle_counter-cyc_temp;
        if(cyc_relative<=cyc_done)
        {
            tarpos=actpos+moving();
            axis->tarpos_modulo(tarpos);
        }
        else
            step++;
        break;
    case 2:
        busy=false;
        done=true;
        if(!exec)
        {
            step=0;
            busy=false;
            error=false;
            errorid=0;
            done=false;
        }
        break;
    }
}

void mc_moveabsolute::planning()
{
    // Interpolation cycle
    dCyc = CYC_TIME/1000.0;

    // Maximum velocity
    double dMaxVel = sqrt(2*dMaxAcc*dPos/PI);

    // Target velocity reachable or not
    if ( dVel <= dMaxVel)
    {
        dW = 2*dMaxAcc/dVel;
        dT1 = PI*dVel/(2*dMaxAcc);
        dT2 = dPos/dVel;
    }
    else
    {
        dVel = dMaxVel;
        dT1 = dPos/dVel;
        dT2 = dPos/dVel;
        dW =PI/dT1;
    }

    dPos1 = dMaxAcc*dT1/dW;
    dPos2 = dVel*(dT2-dT1);
    dTCyc = dT1+dT2;
    cyc_done  = floor(dT1/dCyc)+ floor(dT2/dCyc);
}

double mc_moveabsolute::moving()
{
    double dSNorm;
    double dS;

    double dT = cyc_relative*dCyc;
    if ( dT >= 0 && dT < dT1 )
        dSNorm = dMaxAcc*(dT-sin(dW*dT)/dW)/dW/dPos;
    else if ( dT >= dT1 && dT < dT2 )
        dSNorm = (dPos1+dVel*(dT-dT1))/dPos;
    else if ( dT >= dT2 && dT <= dTCyc)
        dSNorm = (dPos1+dPos2+dMaxAcc*((dT-dT2)+sin(dW*(dT-dT2))/dW)/dW)/dPos;

    dS = dSNorm*dPos;

    if(!dir)dS=-dS;
    return dS;
}
