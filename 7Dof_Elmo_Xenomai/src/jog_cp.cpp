#include <math.h>
#include <stdio.h>

#include "jog_cp.h"

jog_cp::jog_cp():
    step(0)
{}

jog_cp::~jog_cp()
{}

double jog_cp::run(bool MovFw, bool MovBw,double _dVel, double _dMaxAcc,double _dJerk)
{
    if(MovFw&&MovBw)error=true;
    if(error) return 0;

    switch (step){
    case 0:
        if(MovFw||MovBw)
        {
            cping=true;
            bStopped=false;
            bquit=false;

            dVel=fabs(_dVel);
            dMaxAcc=fabs(_dMaxAcc);
            dJerk=fabs(_dJerk);

            dAccM = sqrt(dJerk*dVel);           // Acc time
            dCyc = CYC_TIME/1000.0;
            cyc_temp=cycle_counter;
            if(MovFw)step=10;
            if(MovBw)step=20;
        }
        cppos=0;
        break;

    case 10:
        cyc_relative=cycle_counter-cyc_temp;
        dT = cyc_relative*dCyc;
        if(MovFw)
            cppos=moving();
        else if(!bStopped)
        {
            cppos=quitting();
            bquit=true;
        }
        else
            step=30;
        break;

    case 20:
        cyc_relative=cycle_counter-cyc_temp;
        dT = cyc_relative*dCyc;
        if(MovBw)
            cppos=-moving();
        else if(!bStopped)
        {
            cppos=-quitting();
            bquit=true;
        }
        else
            step=30;
        break;

    case 30:
        bquit=false;
        cping=false;
        bStopped=false;
        step=0;
        break;
    }

    return cppos;
}

double jog_cp::moving()
{
    if (dAccM<=dMaxAcc)
    {
        dT11 = sqrt(dVel/dJerk);
        dT12 = dT11;
        dT1 = dT11+dT12;

        if (dT>=0 && dT<dT11)
        {
            dA = dJerk*dT;
            dV = 1.0/2*dJerk*pow(dT,2);
            dP = 1.0/6*dJerk*pow(dT,3);
            dCurTime = dT;
            dCurAcc = dA;
            dCurVel = dV;
            dCurPos = dP;
        }
        else if (dT>=dT11 && dT<dT1)
        {
            dA = -dJerk*dT+2*sqrt(dJerk*dVel);
            dV = -1.0/2*dJerk*pow(dT,2)+2*sqrt(dJerk*dVel)*dT-dVel;
            dP = -1.0/6*dJerk*pow(dT,3)+sqrt(dJerk*dVel)*pow(dT,2)-dVel*dT+1.0/3*sqrt(pow(dVel,3)/dJerk);
            dCurTime = dT;
            dCurAcc = dA;
            dCurVel = dV;
            dCurPos = dP;
        }
        else if (dT>=dT1)
        {
            dA = 0;
            dV = dVel;
            dP = sqrt(pow(dVel,3)/dJerk)+dVel*(dT-dT1);
            dCurTime = dT;
            dCurAcc = dA;
            dCurVel = dV;
            dCurPos = dP;
        }
    }
    else
    {
        dT11 = dMaxAcc/dJerk;
        dT12 = dVel/dMaxAcc;
        dT1 = dT11+dT12;

        if (dT>=0 && dT<dT11)
        {
            dA = dJerk*dT;
            dV = 1.0/2*dJerk*pow(dT,2);
            dP = 1.0/6*dJerk*pow(dT,3);
            dCurTime = dT;
            dCurAcc = dA;
            dCurVel = dV;
            dCurPos = dP;
        }
        else if (dT>=dT11 && dT<dT12)
        {
            dA = dMaxAcc;
            dV = dMaxAcc*dT-1.0/2*pow(dMaxAcc,2)/dJerk;
            dP = 1.0/2*dMaxAcc*pow(dT,2)-1.0/2*pow(dMaxAcc,2)/dJerk*dT+1.0/6*pow(dMaxAcc,3)/pow(dJerk,2);
            dCurTime = dT;
            dCurAcc = dA;
            dCurVel = dV;
            dCurPos = dP;
        }
        else if (dT>=dT12 && dT<dT1)
        {
            dA = -dJerk*dT+dMaxAcc+dJerk*dVel/dMaxAcc;
            dV = -1.0/2*dJerk*pow(dT,2)+(dMaxAcc+dJerk*dVel/dMaxAcc)*dT-1.0/2*(pow(dMaxAcc,2)/dJerk+dJerk*pow(dVel,2)/pow(dMaxAcc,2));
            dP = -1.0/6*dJerk*pow(dT,3)+1.0/2*(dMaxAcc+dJerk*dVel/dMaxAcc)*pow(dT,2)-1.0/2*(pow(dMaxAcc,2)/dJerk+dJerk*pow(dVel,2)/pow(dMaxAcc,2))*dT+1.0/6*(pow(dMaxAcc,3)/pow(dJerk,2)+dJerk*pow(dVel,3)/pow(dMaxAcc,3));
            dCurTime = dT;
            dCurAcc = dA;
            dCurVel = dV;
            dCurPos = dP;
        }
        else if (dT>=dT1)
        {
            dA = 0;
            dV = dVel;
            dP = (-1.0/6*dJerk*pow(dT1,3)+1.0/2*(dMaxAcc+dJerk*dVel/dMaxAcc)*pow(dT1,2)-1.0/2*(pow(dMaxAcc,2)/dJerk+dJerk*pow(dVel,2)/pow(dMaxAcc,2))*dT1+1.0/6*(pow(dMaxAcc,3)/pow(dJerk,2)+dJerk*pow(dVel,3)/pow(dMaxAcc,3)))+(dVel*(dT-dT1));
            dCurTime = dT;
            dCurAcc = dA;
            dCurVel = dV;
            dCurPos = dP;
        }
    }
    return dP;
}

double jog_cp::quitting()
{
    if(fabs(dV)<1e0)bStopped=true;

    if (dAccM<=dMaxAcc)
    {
        dT11=sqrt(dVel/dJerk);
        dT12=dT11;
        dT1=dT11+dT12;
        if (dCurTime>=0 && dCurTime<dT11)
        {
            if (dT>=dCurTime && dT<dCurTime+2*dCurAcc/dJerk)
            {
                dJ = -dJerk;
                dA = dCurAcc-dJerk*(dT-dCurTime);
                dV = dCurVel+(-((dCurTime - dT)*(2*dCurAcc + dCurTime*dJerk - dJerk*dT))/2);
                dP = dCurPos+(-((dCurTime - dT)*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*dT - 3*dCurAcc*dCurTime - dJerk*pow(dT,2) + 3*dCurAcc*dT + 6*dCurVel))/6);
            }

            else if (dT>=dCurTime+2*dCurAcc/dJerk && dT<=dCurTime+3*dCurAcc/dJerk)
            {
                dJ = dJerk;
                dA = (dCurAcc-dJerk*((dCurTime+2*dCurAcc/dJerk)-dCurTime))+(dJerk*dT - dCurTime*dJerk - 2*dCurAcc);
                dV = (dCurVel+(-((dCurTime - (dCurTime+2*dCurAcc/dJerk))*(2*dCurAcc + dCurTime*dJerk - dJerk*(dCurTime+2*dCurAcc/dJerk)))/2))+(3*dCurAcc*dCurTime - 3*dCurAcc*dT + dJerk*(pow(dCurTime,2)/2 - dCurTime*dT + pow(dT,2)/2) + (4*pow(dCurAcc,2))/dJerk);
                dP = (dCurPos+(-((dCurTime - (dCurTime+2*dCurAcc/dJerk))*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*(dCurTime+2*dCurAcc/dJerk) - 3*dCurAcc*dCurTime - dJerk*pow(dCurTime+2*dCurAcc/dJerk,2) + 3*dCurAcc*(dCurTime+2*dCurAcc/dJerk) + 6*dCurVel))/6))+(-((2*dCurAcc + dCurTime*dJerk - dJerk*dT)*(10*pow(dCurAcc,2) + 7*dCurAcc*dCurTime*dJerk - 7*dCurAcc*dJerk*dT + pow(dCurTime,2)*pow(dJerk,2) - 2*dCurTime*pow(dJerk,2)*dT + pow(dJerk,2)*pow(dT,2) + 6*dCurVel*dJerk))/(6*pow(dJerk,2)));
            }
            else if (dT>dCurTime+3*dCurAcc/dJerk)
            {
                dJ = 0;
                dA = 0;
                dV = 0;
                dP = (dCurPos+(-((dCurTime - (dCurTime+2*dCurAcc/dJerk))*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*(dCurTime+2*dCurAcc/dJerk) - 3*dCurAcc*dCurTime - dJerk*pow(dCurTime+2*dCurAcc/dJerk,2) + 3*dCurAcc*(dCurTime+2*dCurAcc/dJerk) + 6*dCurVel))/6))+(-((2*dCurAcc + dCurTime*dJerk - dJerk*(dCurTime+3*dCurAcc/dJerk))*(10*pow(dCurAcc,2) + 7*dCurAcc*dCurTime*dJerk - 7*dCurAcc*dJerk*(dCurTime+3*dCurAcc/dJerk) + pow(dCurTime,2)*pow(dJerk,2) - 2*dCurTime*pow(dJerk,2)*(dCurTime+3*dCurAcc/dJerk) + pow(dJerk,2)*pow(dCurTime+3*dCurAcc/dJerk,2) + 6*dCurVel*dJerk))/(6*pow(dJerk,2)));
            }
        }
        else if (dCurTime>=dT11 && dCurTime<dT1)
        {
            if (dT>=dCurTime && dT<3*dT11)
            {
                dJ = -dJerk;
                dA = dCurAcc-dJerk*(dT-dCurTime);
                dV = dCurVel+(-((dCurTime - dT)*(2*dCurAcc + dCurTime*dJerk - dJerk*dT))/2);
                dP = dCurPos+(-((dCurTime - dT)*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*dT - 3*dCurAcc*dCurTime - dJerk*pow(dT,2) + 3*dCurAcc*dT + 6*dCurVel))/6);
            }
            else if (dT>=3*dT11 && dT<=4*dT11)
            {
                dJ = dJerk;
                dA = (dCurAcc-dJerk*(3*dT11-dCurTime))+(dJerk*(dT-3*dT11));
                dV = (dCurVel+(-((dCurTime - 3*dT11)*(2*dCurAcc + dCurTime*dJerk - dJerk*3*dT11))/2))+(((dT - 3*dT11)*(2*dCurAcc + 2*dCurTime*dJerk + dJerk*dT - 9*dJerk*dT11))/2);
                dP = (dCurPos+(-((dCurTime - 3*dT11)*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*3*dT11 - 3*dCurAcc*dCurTime - dJerk*pow(3*dT11,2) + 3*dCurAcc*3*dT11 + 6*dCurVel))/6))+(((dT - 3*dT11)*(- 3*dJerk*pow(dCurTime,2) + 3*dJerk*dCurTime*dT + 9*dJerk*dCurTime*dT11 - 6*dCurAcc*dCurTime + dJerk*pow(dT,2) - 15*dJerk*dT*dT11 + 3*dCurAcc*dT + 9*dJerk*pow(dT11,2) + 9*dCurAcc*dT11 + 6*dCurVel))/6);
            }
            else if (dT>4*dT11)
            {
                dJ = 0;
                dA = 0;
                dV = 0;
                dP = (dCurPos+(-((dCurTime - 3*dT11)*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*3*dT11 - 3*dCurAcc*dCurTime - dJerk*pow(3*dT11,2) + 3*dCurAcc*3*dT11 + 6*dCurVel))/6))+((((4*dT11) - 3*dT11)*(- 3*dJerk*pow(dCurTime,2) + 3*dJerk*dCurTime*(4*dT11) + 9*dJerk*dCurTime*dT11 - 6*dCurAcc*dCurTime + dJerk*pow(4*dT11,2) - 15*dJerk*(4*dT11)*dT11 + 3*dCurAcc*(4*dT11) + 9*dJerk*pow(dT11,2) + 9*dCurAcc*dT11 + 6*dCurVel))/6);
            }
        }
        else if (dCurTime>=dT1)
        {
            if (dT>=dCurTime && dT<dCurTime+dT11)
            {
                dJ = -dJerk;
                dA = -dJerk*(dT-dCurTime);
                dV = dVel+(-(dJerk*pow((dCurTime - dT),2))/2);
                dP = dCurPos+(-((dCurTime - dT)*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*dT - dJerk*pow(dT,2) + 6*dVel))/6);
            }
            else if (dT>=dCurTime+dT11 && dT<dCurTime+2*dT11)
            {
                dJ = dJerk;
                dA = (-dJerk*dT11)+(dJerk*(dT-dCurTime-dT11));
                dV = (dVel+(-(dJerk*pow((dCurTime - (dCurTime+dT11)),2))/2))+((dJerk*(pow(dCurTime,2) - 2*dCurTime*dT + 4*dCurTime*dT11 + pow(dT,2) - 4*dT*dT11 + 3*pow(dT11,2)))/2);
                dP = (dCurPos+(-((dCurTime - (dCurTime+dT11))*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*(dCurTime+dT11) - dJerk*pow(dCurTime+dT11,2) + 6*dVel))/6))+(-((dCurTime - dT + dT11)*(dJerk*pow(dCurTime,2) - 2*dJerk*dCurTime*dT + 5*dJerk*dCurTime*dT11 + dJerk*pow(dT,2) - 5*dJerk*dT*dT11 + dJerk*pow(dT11,2) + 6*dVel))/6);
            }
            else if (dT>=dCurTime+2*dT11)
            {
                dJ = 0;
                dA = 0;
                dV = 0;
                dP = (dCurPos+(-((dCurTime - (dCurTime+dT11))*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*(dCurTime+dT11) - dJerk*pow(dCurTime+dT11,2) + 6*dVel))/6))+(-((dCurTime - (dCurTime+2*dT11) + dT11)*(dJerk*pow(dCurTime,2) - 2*dJerk*dCurTime*(dCurTime+2*dT11) + 5*dJerk*dCurTime*dT11 + dJerk*pow(dCurTime+2*dT11,2) - 5*dJerk*(dCurTime+2*dT11)*dT11 + dJerk*pow(dT11,2) + 6*dVel))/6);
            }
        }
    }
    else
    {
        dT11=dMaxAcc/dJerk;
        dT12=dVel/dMaxAcc;
        dT1=dT11+dT12;
        if (dCurTime>=0 && dCurTime<dT11)
        {
            if (dT>=dCurTime && dT<dCurTime+2*dCurAcc/dJerk)
            {
                dA = dCurAcc-dJerk*(dT-dCurTime);
                dV = dCurVel+(-((dCurTime - dT)*(2*dCurAcc + dCurTime*dJerk - dJerk*dT))/2);
                dP = dCurPos+(-((dCurTime - dT)*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*dT - 3*dCurAcc*dCurTime - dJerk*pow(dT,2) + 3*dCurAcc*dT + 6*dCurVel))/6);
            }
            else if (dT>=dCurTime+2*dCurAcc/dJerk && dT<=dCurTime+3*dCurAcc/dJerk)
            {
                dJ = dJerk;
                dA = (dCurAcc-dJerk*((dCurTime+2*dCurAcc/dJerk)-dCurTime))+(dJerk*dT - dCurTime*dJerk - 2*dCurAcc);
                dV = (dCurVel+(-((dCurTime - (dCurTime+2*dCurAcc/dJerk))*(2*dCurAcc + dCurTime*dJerk - dJerk*(dCurTime+2*dCurAcc/dJerk)))/2))+(3*dCurAcc*dCurTime - 3*dCurAcc*dT + dJerk*(pow(dCurTime,2)/2 - dCurTime*dT + pow(dT,2)/2) + (4*pow(dCurAcc,2))/dJerk);
                dP = (dCurPos+(-((dCurTime - (dCurTime+2*dCurAcc/dJerk))*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*(dCurTime+2*dCurAcc/dJerk) - 3*dCurAcc*dCurTime - dJerk*pow(dCurTime+2*dCurAcc/dJerk,2) + 3*dCurAcc*(dCurTime+2*dCurAcc/dJerk) + 6*dCurVel))/6))+(-((2*dCurAcc + dCurTime*dJerk - dJerk*dT)*(10*pow(dCurAcc,2) + 7*dCurAcc*dCurTime*dJerk - 7*dCurAcc*dJerk*dT + pow(dCurTime,2)*pow(dJerk,2) - 2*dCurTime*pow(dJerk,2)*dT + pow(dJerk,2)*pow(dT,2) + 6*dCurVel*dJerk))/(6*pow(dJerk,2)));
            }
            else if (dT>dCurTime+3*dCurAcc/dJerk)
            {
                dJ = 0;
                dA = 0;
                dV = 0;
                dP = (dCurPos+(-((dCurTime - (dCurTime+2*dCurAcc/dJerk))*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*(dCurTime+2*dCurAcc/dJerk) - 3*dCurAcc*dCurTime - dJerk*pow(dCurTime+2*dCurAcc/dJerk,2) + 3*dCurAcc*(dCurTime+2*dCurAcc/dJerk) + 6*dCurVel))/6))+(-((2*dCurAcc + dCurTime*dJerk - dJerk*(dCurTime+3*dCurAcc/dJerk))*(10*pow(dCurAcc,2) + 7*dCurAcc*dCurTime*dJerk - 7*dCurAcc*dJerk*(dCurTime+3*dCurAcc/dJerk) + pow(dCurTime,2)*pow(dJerk,2) - 2*dCurTime*pow(dJerk,2)*(dCurTime+3*dCurAcc/dJerk) + pow(dJerk,2)*pow(dCurTime+3*dCurAcc/dJerk,2) + 6*dCurVel*dJerk))/(6*pow(dJerk,2)));
            }
        }

        else if (dCurTime>=dT11 && dCurTime<dT12)
        {
            if (dT>=dCurTime && dT<dCurTime+2*dT11)
            {
                dA = dCurAcc-dJerk*(dT-dCurTime);
                dV = dCurVel+(-((dCurTime - dT)*(2*dCurAcc + dCurTime*dJerk - dJerk*dT))/2);
                dP = dCurPos+(-((dCurTime - dT)*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*dT - 3*dCurAcc*dCurTime - dJerk*pow(dT,2) + 3*dCurAcc*dT + 6*dCurVel))/6);

            }
            else if (dT>=dCurTime+2*dT11 && dT<2*dCurTime+dT11)
            {
                dJ = 0;
                dA = -dMaxAcc;
                dV = (dCurVel+(-((dCurTime - (dCurTime+2*dT11))*(2*dCurAcc + dCurTime*dJerk - dJerk*(dCurTime+2*dT11)))/2))+(dMaxAcc*(dCurTime - dT + 2*dT11));
                dP = (dCurPos+(-((dCurTime - (dCurTime+2*dT11))*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*(dCurTime+2*dT11) - 3*dCurAcc*dCurTime - dJerk*pow(dCurTime+2*dT11,2) + 3*dCurAcc*(dCurTime+2*dT11) + 6*dCurVel))/6))+(-((dCurTime - dT + 2*dT11)*(2*dCurVel + dMaxAcc*dCurTime - dMaxAcc*dT + 2*dMaxAcc*dT11 + 4*dCurAcc*dT11 - 4*dJerk*pow(dT11,2)))/2);
            }
            else if (dT>=2*dCurTime+dT11 && dT<=2*dCurTime+2*dT11)
            {
                dJ = dJerk;
                dA = (-dMaxAcc)+(-dJerk*(2*dCurTime - dT + dP));
                dV = ((dCurVel+(-((dCurTime - (dCurTime+2*dT11))*(2*dCurAcc + dCurTime*dJerk - dJerk*(dCurTime+2*dT11)))/2))+(dMaxAcc*(dCurTime - (2*dCurTime+dT11) + 2*dT11)))+(((2*dCurTime - dT + dT11)*(2*dMaxAcc + 2*dCurTime*dJerk - dJerk*dT + dJerk*dT11))/2);
                dP = ((dCurPos+(-((dCurTime - (dCurTime+2*dT11))*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*(dCurTime+2*dT11) - 3*dCurAcc*dCurTime - dJerk*pow(dCurTime+2*dT11,2) + 3*dCurAcc*(dCurTime+2*dT11) + 6*dCurVel))/6))+(-((dCurTime - (2*dCurTime+dT11) + 2*dT11)*(2*dCurVel + dMaxAcc*dCurTime - dMaxAcc*(2*dCurTime+dT11) + 2*dMaxAcc*dT11 + 4*dCurAcc*dT11 - 4*dJerk*pow(dT11,2)))/2))+(dCurVel*dT - 2*dCurVel*dCurTime - dCurVel*dT11 - (4*pow(dCurTime,3)*dJerk)/3 - (dMaxAcc*pow(dT,2))/2 - (3*dMaxAcc*pow(dT11,2))/2 - 2*dCurAcc*pow(dT11,2) + (dJerk*pow(dT,3))/6 + (11*dJerk*pow(dT11,3))/6 - dCurTime*dJerk*pow(dT,2) + 2*pow(dCurTime,2)*dJerk*dT + 3*dCurTime*dJerk*pow(dT11,2) - 2*pow(dCurTime,2)*dJerk*dT11 - (3*dJerk*dT*pow(dT11,2))/2 - (dJerk*pow(dT,2)*dT11)/2 + dMaxAcc*dCurTime*dT - 3*dMaxAcc*dCurTime*dT11 - 4*dCurAcc*dCurTime*dT11 + 2*dMaxAcc*dT*dT11 + 2*dCurAcc*dT*dT11 + 2*dCurTime*dJerk*dT*dT11);
            }
            else if (dT>2*dCurTime+2*dT11)
            {
                dJ = 0;
                dA = 0;
                dV = 0;
                dP = ((dCurPos+(-((dCurTime - (dCurTime+2*dT11))*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*(dCurTime+2*dT11) - 3*dCurAcc*dCurTime - dJerk*pow(dCurTime+2*dT11,2) + 3*dCurAcc*(dCurTime+2*dT11) + 6*dCurVel))/6))+(-((dCurTime - (2*dCurTime+dT11) + 2*dT11)*(2*dCurVel + dMaxAcc*dCurTime - dMaxAcc*(2*dCurTime+dT11) + 2*dMaxAcc*dT11 + 4*dCurAcc*dT11 - 4*dJerk*pow(dT11,2)))/2))+(dCurVel*(2*dCurTime+2*dT11) - 2*dCurVel*dCurTime - dCurVel*dT11 - (4*pow(dCurTime,3)*dJerk)/3 - (dMaxAcc*pow(2*dCurTime+2*dT11,2))/2 - (3*dMaxAcc*pow(dT11,2))/2 - 2*dCurAcc*pow(dT11,2) + (dJerk*pow(2*dCurTime+2*dT11,3))/6 + (11*dJerk*pow(dT11,3))/6 - dCurTime*dJerk*pow(2*dCurTime+2*dT11,2) + 2*pow(dCurTime,2)*dJerk*(2*dCurTime+2*dT11) + 3*dCurTime*dJerk*pow(dT11,2) - 2*pow(dCurTime,2)*dJerk*dT11 - (3*dJerk*(2*dCurTime+2*dT11)*pow(dT11,2))/2 - (dJerk*pow(2*dCurTime+2*dT11,2)*dT11)/2 + dMaxAcc*dCurTime*(2*dCurTime+2*dT11) - 3*dMaxAcc*dCurTime*dT11 - 4*dCurAcc*dCurTime*dT11 + 2*dMaxAcc*(2*dCurTime+2*dT11)*dT11 + 2*dCurAcc*(2*dCurTime+2*dT11)*dT11 + 2*dCurTime*dJerk*(2*dCurTime+2*dT11)*dT11);
            }
        }

        else if (dCurTime>=dT12 && dCurTime<dT1)
        {
            if (dT>=dCurTime && dT<dT1+dT11)
            {
                dA = dCurAcc-dJerk*(dT-dCurTime);
                dV = dCurVel+(-((dCurTime - dT)*(2*dCurAcc + dCurTime*dJerk - dJerk*dT))/2);
                dP = dCurPos+(-((dCurTime - dT)*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*dT - 3*dCurAcc*dCurTime - dJerk*pow(dT,2) + 3*dCurAcc*dT + 6*dCurVel))/6);
            }
            else if (dT>=dT1+dT11 && dT<=dT11+2*dT12)
            {
                dA = -dMaxAcc;
                dV = (dCurVel+(-((dCurTime - (dT1+dT11))*(2*dCurAcc + dCurTime*dJerk - dJerk*dT))/2))+(dMaxAcc*(dCurTime - dT));
                dP = (dCurPos+(-((dCurTime - (dT1+dT11))*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*(dT1+dT11) - 3*dCurAcc*dCurTime - dJerk*pow((dT1+dT11),2) + 3*dCurAcc*(dT1+dT11) + 6*dCurVel))/6))+(-((dCurTime - dT)*(4*dCurVel + 4*dT1*dCurAcc + 2*dMaxAcc*dCurTime - 4*dCurAcc*dCurTime - 2*dMaxAcc*dT + 4*dCurAcc*dT11 - pow(dCurTime,2)*dJerk + dT1*dCurTime*dJerk - dT1*dJerk*dT + dCurTime*dJerk*dT + dCurTime*dJerk*dT11 - dJerk*dT*dT11))/4);
            }
            else if (dT>dT11+2*dT12 && dT<=2*dT1)
            {
                dA = -dCurAcc+dJerk*(dT-dT11-2*dT12);
                dV = ((dCurVel+(-((dCurTime - (dT1+dT11))*(2*dCurAcc + dCurTime*dJerk - dJerk*(dT11+2*dT12)))/2))+(dMaxAcc*(dCurTime - (dT11+2*dT12))))+(((dT11 - dT + 2*dT12)*(2*dCurAcc - dJerk*dT + dJerk*dT11 + 2*dJerk*dT12))/2);
                dP = ((dCurPos+(-((dCurTime - (dT1+dT11))*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*(dT1+dT11) - 3*dCurAcc*dCurTime - dJerk*pow((dT1+dT11),2) + 3*dCurAcc*(dT1+dT11) + 6*dCurVel))/6))+(-((dCurTime - (dT11+2*dT12))*(4*dCurVel + 4*dT1*dCurAcc + 2*dMaxAcc*dCurTime - 4*dCurAcc*dCurTime - 2*dMaxAcc*(dT11+2*dT12) + 4*dCurAcc*dT11 - pow(dCurTime,2)*dJerk + dT1*dCurTime*dJerk - dT1*dJerk*(dT11+2*dT12) + dCurTime*dJerk*(dT11+2*dT12) + dCurTime*dJerk*dT11 - dJerk*(dT11+2*dT12)*dT11))/4))+(((dT11 - dT + 2*dT12)*(6*dCurAcc*dCurTime - 6*dT1*dCurAcc - 6*dMaxAcc*dCurTime - 6*dCurVel + 6*dMaxAcc*dT11 + 12*dMaxAcc*dT12 + 3*dCurAcc*dT - 9*dCurAcc*dT11 - 6*dCurAcc*dT12 + 3*pow(dCurTime,2)*dJerk - dJerk*pow(dT,2) + 2*dJerk*pow(dT11,2) - 4*dJerk*pow(dT12,2) - 3*dT1*dCurTime*dJerk + 3*dT1*dJerk*dT11 + 6*dT1*dJerk*dT12 - 6*dCurTime*dJerk*dT11 - 6*dCurTime*dJerk*dT12 + 2*dJerk*dT*dT11 + 4*dJerk*dT*dT12 + 2*dJerk*dT11*dT12))/6);
            }
            else if (dT>2*dT1)
            {
                dA = 0;
                dV = 0;
                dP = ((dCurPos+(-((dCurTime - (dT1+dT11))*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*(dT1+dT11) - 3*dCurAcc*dCurTime - dJerk*pow((dT1+dT11),2) + 3*dCurAcc*(dT1+dT11) + 6*dCurVel))/6))+(-((dCurTime - (dT11+2*dT12))*(4*dCurVel + 4*dT1*dCurAcc + 2*dMaxAcc*dCurTime - 4*dCurAcc*dCurTime - 2*dMaxAcc*(dT11+2*dT12) + 4*dCurAcc*dT11 - pow(dCurTime,2)*dJerk + dT1*dCurTime*dJerk - dT1*dJerk*(dT11+2*dT12) + dCurTime*dJerk*(dT11+2*dT12) + dCurTime*dJerk*dT11 - dJerk*(dT11+2*dT12)*dT11))/4))+(((dT11 - (2*dT1) + 2*dT12)*(6*dCurAcc*dCurTime - 6*dT1*dCurAcc - 6*dMaxAcc*dCurTime - 6*dCurVel + 6*dMaxAcc*dT11 + 12*dMaxAcc*dT12 + 3*dCurAcc*(2*dT1) - 9*dCurAcc*dT11 - 6*dCurAcc*dT12 + 3*pow(dCurTime,2)*dJerk - dJerk*pow(2*dT1,2) + 2*dJerk*pow(dT11,2) - 4*dJerk*pow(dT12,2) - 3*dT1*dCurTime*dJerk + 3*dT1*dJerk*dT11 + 6*dT1*dJerk*dT12 - 6*dCurTime*dJerk*dT11 - 6*dCurTime*dJerk*dT12 + 2*dJerk*(2*dT1)*dT11 + 4*dJerk*(2*dT1)*dT12 + 2*dJerk*dT11*dT12))/6);
            }
        }

        else if (dCurTime>=dT1)
        {
            if (dT>=dCurTime && dT<dCurTime+dT11)
            {
                dA = dCurAcc-dJerk*(dT-dCurTime);
                dV = dCurVel+(-(dJerk*pow((dCurTime - dT),2))/2);
                dP = dCurPos+(-((dCurTime - dT)*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*dT - dJerk*pow(dT,2) + 6*dCurVel))/6);
            }
            else if (dT>=dCurTime+dT11 && dT<=dCurTime+dT12)
            {
                dA = -dMaxAcc;
                dV = (dCurVel+(-(dJerk*pow((dCurTime - (dCurTime+dT11)),2))/2))+(dMaxAcc*(dCurTime - dT + dT11));
                dP = (dCurPos+(-((dCurTime - (dCurTime+dT11))*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*(dCurTime+dT11) - dJerk*pow((dCurTime+dT11),2) + 6*dCurVel))/6))+(-((dCurTime - dT + dT11)*(- dJerk*pow(dT11,2) + dMaxAcc*dT11 + 2*dCurVel + dMaxAcc*dCurTime - dMaxAcc*dT))/2);
            }
            else if (dT>=dCurTime+dT12 && dT<dCurTime+dT1)
            {
                dA = -dMaxAcc+dJerk*(dT-dCurTime-dT12);
                dV = ((dCurVel+(-(dJerk*pow((dCurTime - (dCurTime+dT11)),2))/2))+(dMaxAcc*(dCurTime - (dCurTime+dT12) + dT11)))+(((dCurTime - dT + dT12)*(2*dMaxAcc + dCurTime*dJerk - dJerk*dT + dJerk*dT12))/2);
                dP = ((dCurPos+(-((dCurTime - (dCurTime+dT11))*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*(dCurTime+dT11) - dJerk*pow((dCurTime+dT11),2) + 6*dCurVel))/6))+(-((dCurTime - (dCurTime+dT12) + dT11)*(- dJerk*pow(dT11,2) + dMaxAcc*dT11 + 2*dCurVel + dMaxAcc*dCurTime - dMaxAcc*(dCurTime+dT12)))/2))+(-((dCurTime - dT + dT12)*(dJerk*pow(dCurTime,2) - 2*dJerk*dCurTime*dT + 2*dJerk*dCurTime*dT12 + 3*dMaxAcc*dCurTime + dJerk*pow(dT,2) - 2*dJerk*dT*dT12 - 3*dMaxAcc*dT - 3*dJerk*pow(dT11,2) + 6*dMaxAcc*dT11 + dJerk*pow(dT12,2) - 3*dMaxAcc*dT12 + 6*dCurVel))/6);
            }
            else if (dT>=dCurTime+dT1)
            {
                dA = 0;
                dV = 0;
                dP = ((dCurPos+(-((dCurTime - (dCurTime+dT11))*(- dJerk*pow(dCurTime,2) + 2*dJerk*dCurTime*(dCurTime+dT11) - dJerk*pow((dCurTime+dT11),2) + 6*dCurVel))/6))+(-((dCurTime - (dCurTime+dT12) + dT11)*(- dJerk*pow(dT11,2) + dMaxAcc*dT11 + 2*dCurVel + dMaxAcc*dCurTime - dMaxAcc*(dCurTime+dT12)))/2))+(-((dCurTime - (dCurTime+dT1) + dT12)*(dJerk*pow(dCurTime,2) - 2*dJerk*dCurTime*(dCurTime+dT1) + 2*dJerk*dCurTime*dT12 + 3*dMaxAcc*dCurTime + dJerk*pow(dCurTime+dT1,2) - 2*dJerk*(dCurTime+dT1)*dT12 - 3*dMaxAcc*(dCurTime+dT1) - 3*dJerk*pow(dT11,2) + 6*dMaxAcc*dT11 + dJerk*pow(dT12,2) - 3*dMaxAcc*dT12 + 6*dCurVel))/6);
            }
        }
    }

    return dP;
}
