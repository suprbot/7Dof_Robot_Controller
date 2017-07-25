#include <math.h>
#include <stdio.h>

#include "mc_directeach.h"

//DH parameters
const double Dbs = 0.374;       //G1 0.374;     G2 0.3;     G3 0.31895;
const double Dse = 0.413;       //G1 0.413;     G2 0.4;     G3 0.400;
const double Dew = 0.3445;    //G1 0.3445;  	G2 0.4;     G3 0.400;
const double Dwt = 0.2405;     //G1 0.2405;   G2 0.245; G3 0.1428;

//
const double m_link[7]={0,0,7.19,5.73,4.01,3.53,4.18};			//link: mass kg
const double mc_link[7][3]=
{0,0,0,
 0,0,0,
 0,-0.1891,0.0206,
 0.00248,0.02956,-0.01437,
 0,-0.13256,0.02153,
 0.00386,0.02221,-0.02215,
 0,-0.118,0.0091};													//link: center of gravity xyz
const double m_load=0;														//load: mass kg
const double mc_load[3]={0,0,0};

const double axis_Kt[7]={0,5.79,6,4.9,10.5,10.55,10.62};
const double axis_Dfriction[7]={120,128,58,46,75,65,70};
const double axis_Vfriction[7]={5,4.9,1.669,1.25,1.2,1.1,2};
const double axis_VeloLimit[7]={50,50,50,50,50,50,50};

void gravitycomp(const double m_link[7],const double mc_link[7][3],const double m_load,const double mc_load[3],const double joint[7],double gravity[7])
{
    gravity[0]=0.0;

    gravity[1]=9.8*m_link[3]*(mc_link[3][2]*sin(joint[1])+Dse*sin(joint[1])+mc_link[3][0]*cos(joint[1])*cos(joint[2])-mc_link[3][1]*cos(joint[1])*sin(joint[2]))-9.8*m_link[5]*(Dew*(cos(joint[3])*sin(joint[1])-cos(joint[1])*cos(joint[2])*sin(joint[3]))-mc_link[5][0]*(cos(joint[4])*(sin(joint[1])*sin(joint[3])+cos(joint[1])*cos(joint[2])*cos(joint[3]))+cos(joint[1])*sin(joint[2])*sin(joint[4]))-Dse*sin(joint[1])+mc_link[5][1]*(sin(joint[4])*(sin(joint[1])*sin(joint[3])+cos(joint[1])*cos(joint[2])*cos(joint[3]))-cos(joint[1])*cos(joint[4])*sin(joint[2]))+mc_link[5][2]*(cos(joint[3])*sin(joint[1])-cos(joint[1])*cos(joint[2])*sin(joint[3])))-9.8*m_link[6]*(Dew*(cos(joint[3])*sin(joint[1])-cos(joint[1])*cos(joint[2])*sin(joint[3]))+mc_link[6][0]*(sin(joint[5])*(cos(joint[3])*sin(joint[1])-cos(joint[1])*cos(joint[2])*sin(joint[3]))-cos(joint[5])*(cos(joint[4])*(sin(joint[1])*sin(joint[3])+cos(joint[1])*cos(joint[2])*cos(joint[3]))+cos(joint[1])*sin(joint[2])*sin(joint[4])))+mc_link[6][1]*(cos(joint[5])*(cos(joint[3])*sin(joint[1])-cos(joint[1])*cos(joint[2])*sin(joint[3]))+sin(joint[5])*(cos(joint[4])*(sin(joint[1])*sin(joint[3])+cos(joint[1])*cos(joint[2])*cos(joint[3]))+cos(joint[1])*sin(joint[2])*sin(joint[4])))-Dse*sin(joint[1])-mc_link[6][2]*(sin(joint[4])*(sin(joint[1])*sin(joint[3])+cos(joint[1])*cos(joint[2])*cos(joint[3]))-cos(joint[1])*cos(joint[4])*sin(joint[2])))+9.8*m_load*(mc_load[2]*(cos(joint[5])*(cos(joint[3])*sin(joint[1])-cos(joint[1])*cos(joint[2])*sin(joint[3]))+sin(joint[5])*(cos(joint[4])*(sin(joint[1])*sin(joint[3])+cos(joint[1])*cos(joint[2])*cos(joint[3]))+cos(joint[1])*sin(joint[2])*sin(joint[4])))-Dew*(cos(joint[3])*sin(joint[1])-cos(joint[1])*cos(joint[2])*sin(joint[3]))+Dwt*(cos(joint[5])*(cos(joint[3])*sin(joint[1])-cos(joint[1])*cos(joint[2])*sin(joint[3]))+sin(joint[5])*(cos(joint[4])*(sin(joint[1])*sin(joint[3])+cos(joint[1])*cos(joint[2])*cos(joint[3]))+cos(joint[1])*sin(joint[2])*sin(joint[4])))+mc_load[0]*(sin(joint[6])*(sin(joint[4])*(sin(joint[1])*sin(joint[3])+cos(joint[1])*cos(joint[2])*cos(joint[3]))-cos(joint[1])*cos(joint[4])*sin(joint[2]))-cos(joint[6])*(sin(joint[5])*(cos(joint[3])*sin(joint[1])-cos(joint[1])*cos(joint[2])*sin(joint[3]))-cos(joint[5])*(cos(joint[4])*(sin(joint[1])*sin(joint[3])+cos(joint[1])*cos(joint[2])*cos(joint[3]))+cos(joint[1])*sin(joint[2])*sin(joint[4]))))+mc_load[1]*(cos(joint[6])*(sin(joint[4])*(sin(joint[1])*sin(joint[3])+cos(joint[1])*cos(joint[2])*cos(joint[3]))-cos(joint[1])*cos(joint[4])*sin(joint[2]))+sin(joint[6])*(sin(joint[5])*(cos(joint[3])*sin(joint[1])-cos(joint[1])*cos(joint[2])*sin(joint[3]))-cos(joint[5])*(cos(joint[4])*(sin(joint[1])*sin(joint[3])+cos(joint[1])*cos(joint[2])*cos(joint[3]))+cos(joint[1])*sin(joint[2])*sin(joint[4]))))+Dse*sin(joint[1]))+9.8*m_link[4]*(mc_link[4][0]*(sin(joint[1])*sin(joint[3])+cos(joint[1])*cos(joint[2])*cos(joint[3]))+Dse*sin(joint[1])+mc_link[4][1]*(cos(joint[3])*sin(joint[1])-cos(joint[1])*cos(joint[2])*sin(joint[3]))+mc_link[4][2]*cos(joint[1])*sin(joint[2]))+9.8*m_link[2]*(mc_link[2][0]*cos(joint[1])-mc_link[2][1]*sin(joint[1]));

    gravity[2]=9.8*m_link[4]*(mc_link[4][2]*cos(joint[2])*sin(joint[1])-mc_link[4][0]*cos(joint[3])*sin(joint[1])*sin(joint[2])+mc_link[4][1]*sin(joint[1])*sin(joint[2])*sin(joint[3]))-9.8*m_link[3]*(mc_link[3][1]*cos(joint[2])*sin(joint[1])+mc_link[3][0]*sin(joint[1])*sin(joint[2]))-9.8*m_link[6]*(mc_link[6][2]*(cos(joint[2])*cos(joint[4])*sin(joint[1])+cos(joint[3])*sin(joint[1])*sin(joint[2])*sin(joint[4]))-mc_link[6][0]*(cos(joint[5])*(cos(joint[2])*sin(joint[1])*sin(joint[4])-cos(joint[3])*cos(joint[4])*sin(joint[1])*sin(joint[2]))-sin(joint[1])*sin(joint[2])*sin(joint[3])*sin(joint[5]))+mc_link[6][1]*(sin(joint[5])*(cos(joint[2])*sin(joint[1])*sin(joint[4])-cos(joint[3])*cos(joint[4])*sin(joint[1])*sin(joint[2]))+cos(joint[5])*sin(joint[1])*sin(joint[2])*sin(joint[3]))+Dew*sin(joint[1])*sin(joint[2])*sin(joint[3]))-9.8*m_load*(mc_load[0]*(sin(joint[6])*(cos(joint[2])*cos(joint[4])*sin(joint[1])+cos(joint[3])*sin(joint[1])*sin(joint[2])*sin(joint[4]))-cos(joint[6])*(cos(joint[5])*(cos(joint[2])*sin(joint[1])*sin(joint[4])-cos(joint[3])*cos(joint[4])*sin(joint[1])*sin(joint[2]))-sin(joint[1])*sin(joint[2])*sin(joint[3])*sin(joint[5])))-mc_load[2]*(sin(joint[5])*(cos(joint[2])*sin(joint[1])*sin(joint[4])-cos(joint[3])*cos(joint[4])*sin(joint[1])*sin(joint[2]))+cos(joint[5])*sin(joint[1])*sin(joint[2])*sin(joint[3]))-Dwt*(sin(joint[5])*(cos(joint[2])*sin(joint[1])*sin(joint[4])-cos(joint[3])*cos(joint[4])*sin(joint[1])*sin(joint[2]))+cos(joint[5])*sin(joint[1])*sin(joint[2])*sin(joint[3]))+mc_load[1]*(cos(joint[6])*(cos(joint[2])*cos(joint[4])*sin(joint[1])+cos(joint[3])*sin(joint[1])*sin(joint[2])*sin(joint[4]))+sin(joint[6])*(cos(joint[5])*(cos(joint[2])*sin(joint[1])*sin(joint[4])-cos(joint[3])*cos(joint[4])*sin(joint[1])*sin(joint[2]))-sin(joint[1])*sin(joint[2])*sin(joint[3])*sin(joint[5])))+Dew*sin(joint[1])*sin(joint[2])*sin(joint[3]))+9.8*m_link[5]*(mc_link[5][1]*(cos(joint[2])*cos(joint[4])*sin(joint[1])+cos(joint[3])*sin(joint[1])*sin(joint[2])*sin(joint[4]))+mc_link[5][0]*(cos(joint[2])*sin(joint[1])*sin(joint[4])-cos(joint[3])*cos(joint[4])*sin(joint[1])*sin(joint[2]))-mc_link[5][2]*sin(joint[1])*sin(joint[2])*sin(joint[3])-Dew*sin(joint[1])*sin(joint[2])*sin(joint[3]));

    gravity[3]=9.8*m_load*(mc_load[2]*(cos(joint[5])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))-cos(joint[4])*sin(joint[5])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3])))-Dew*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))+Dwt*(cos(joint[5])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))-cos(joint[4])*sin(joint[5])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3])))-mc_load[0]*(cos(joint[6])*(sin(joint[5])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))+cos(joint[4])*cos(joint[5])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3])))+sin(joint[4])*sin(joint[6])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3])))+mc_load[1]*(sin(joint[6])*(sin(joint[5])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))+cos(joint[4])*cos(joint[5])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3])))-cos(joint[6])*sin(joint[4])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3]))))-9.8*m_link[5]*(Dew*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))+mc_link[5][2]*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))+mc_link[5][0]*cos(joint[4])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3]))-mc_link[5][1]*sin(joint[4])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3])))-9.8*m_link[4]*(mc_link[4][0]*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3]))-mc_link[4][1]*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1])))-9.8*m_link[6]*(Dew*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))+mc_link[6][1]*(cos(joint[5])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))-cos(joint[4])*sin(joint[5])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3])))+mc_link[6][0]*(sin(joint[5])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))+cos(joint[4])*cos(joint[5])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3])))+mc_link[6][2]*sin(joint[4])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3])));

    gravity[4]=9.8*m_link[5]*(mc_link[5][1]*(cos(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))-sin(joint[1])*sin(joint[2])*sin(joint[4]))+mc_link[5][0]*(sin(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))+cos(joint[4])*sin(joint[1])*sin(joint[2])))-9.8*m_load*(mc_load[1]*(cos(joint[6])*(cos(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))-sin(joint[1])*sin(joint[2])*sin(joint[4]))+cos(joint[5])*sin(joint[6])*(sin(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))+cos(joint[4])*sin(joint[1])*sin(joint[2])))+mc_load[0]*(sin(joint[6])*(cos(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))-sin(joint[1])*sin(joint[2])*sin(joint[4]))-cos(joint[5])*cos(joint[6])*(sin(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))+cos(joint[4])*sin(joint[1])*sin(joint[2])))-mc_load[2]*sin(joint[5])*(sin(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))+cos(joint[4])*sin(joint[1])*sin(joint[2]))-Dwt*sin(joint[5])*(sin(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))+cos(joint[4])*sin(joint[1])*sin(joint[2])))-9.8*m_link[6]*(mc_link[6][2]*(cos(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))-sin(joint[1])*sin(joint[2])*sin(joint[4]))-mc_link[6][0]*cos(joint[5])*(sin(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))+cos(joint[4])*sin(joint[1])*sin(joint[2]))+mc_link[6][1]*sin(joint[5])*(sin(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))+cos(joint[4])*sin(joint[1])*sin(joint[2])));

    gravity[5]=9.8*m_load*(mc_load[2]*(sin(joint[5])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3]))-cos(joint[5])*(cos(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))-sin(joint[1])*sin(joint[2])*sin(joint[4])))+Dwt*(sin(joint[5])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3]))-cos(joint[5])*(cos(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))-sin(joint[1])*sin(joint[2])*sin(joint[4])))+mc_load[0]*cos(joint[6])*(cos(joint[5])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3]))+sin(joint[5])*(cos(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))-sin(joint[1])*sin(joint[2])*sin(joint[4])))-mc_load[1]*sin(joint[6])*(cos(joint[5])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3]))+sin(joint[5])*(cos(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))-sin(joint[1])*sin(joint[2])*sin(joint[4]))))-9.8*m_link[6]*(mc_link[6][1]*(sin(joint[5])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3]))-cos(joint[5])*(cos(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))-sin(joint[1])*sin(joint[2])*sin(joint[4])))-mc_link[6][0]*(cos(joint[5])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3]))+sin(joint[5])*(cos(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))-sin(joint[1])*sin(joint[2])*sin(joint[4]))));

    gravity[6]=9.8*m_load*(mc_load[1]*(sin(joint[6])*(sin(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))+cos(joint[4])*sin(joint[1])*sin(joint[2]))-cos(joint[6])*(sin(joint[5])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3]))-cos(joint[5])*(cos(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))-sin(joint[1])*sin(joint[2])*sin(joint[4]))))-mc_load[0]*(cos(joint[6])*(sin(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))+cos(joint[4])*sin(joint[1])*sin(joint[2]))+sin(joint[6])*(sin(joint[5])*(cos(joint[1])*cos(joint[3])+cos(joint[2])*sin(joint[1])*sin(joint[3]))-cos(joint[5])*(cos(joint[4])*(cos(joint[1])*sin(joint[3])-cos(joint[2])*cos(joint[3])*sin(joint[1]))-sin(joint[1])*sin(joint[2])*sin(joint[4])))));

}

mc_directeach::mc_directeach(elmo_ecat *_axis[]):
    axis(_axis),
    enable(false),
    busy(false),
    error(false),
    errorid(0),
    done(false),
    step(0),
    axis4_Sflag(0),
    axis4_Sk(-200),
    axis4_PosError(0),
    axis2_Sflag(0),
    axis2_PosError(0),
    axis2_Sk(-300)
{}

mc_directeach::~mc_directeach()
{}

void mc_directeach::run(bool bExec)
{
    for(int i=0;i<7;i++)
    {
        jointvelo[i]=(jointPosArc[i]-jointpos[i])*(180.0/PI)/(CYC_TIME/1000.0);
        jointpos[i]=jointPosArc[i];
        jointpostemp[i]=jointpos[i]-PI;
        axis_SmoothDirect[i]=2.0/(1+exp(-8.0*jointvelo[i]))-1;
    }

    switch(step){
    case 0:
        if(bExec)
        {
            busy=true;
            step++;
            printf("dt 0\n");
        }
        break;

    case 1:
        //printf("axis[1]->disop(): %d\n",axis[5]->disop());
        //for(int i=0;i<7;i++)
            axis[3]->modop(10);

        if (axis[0]->disop()==10) printf("axis[0]->disop(): %d\n",axis[0]->disop());
        if (axis[1]->disop()==10) printf("axis[1]->disop(): %d\n",axis[1]->disop());
        if (axis[2]->disop()==10) printf("axis[2]->disop(): %d\n",axis[2]->disop());
        if (axis[3]->disop()==10) printf("axis[3]->disop(): %d\n",axis[3]->disop());
        if (axis[4]->disop()==10) printf("axis[4]->disop(): %d\n",axis[4]->disop());
        if (axis[5]->disop()==10) printf("axis[5]->disop(): %d\n",axis[5]->disop());
        if (axis[6]->disop()==10) printf("axis[6]->disop(): %d\n",axis[6]->disop());

        if (//(axis[0]->disop()==10) &&
                //(axis[1]->disop()==10) &&
                //(axis[2]->disop()==10) &&
                (axis[3]->disop()==10) //&&
                //(axis[4]->disop()==10) &&
                //(axis[5]->disop()==10) &&
                //(axis[6]->disop()==10)
                )
         {
            step++;
            printf("tor 2");
        }
        break;

    case 2:
        gravitycomp(m_link,mc_link,m_load,mc_load,jointpostemp,axis_TorGravity);


        for(int i=0;i<7;i++)
        {
            if(fabs(jointpos[i]>PI/2 *(((i+1) % 2)*3 +1)))
            {
                axis_Dflag[i]=0;
                axis_Vflag[i]=0;
                if(i==2)
                {
                    axis2_Sflag=1;
                    if(jointpos[2]>2*PI)
                        axis2_PosError=jointpos[2]-2*PI;
                    else if (jointpos[2]<-2*PI)
                        axis2_PosError=jointpos[2]+2*PI;
                }

                if(i==4)
                {
                    axis4_Sflag=1;                     //轴4超过限位的标志位
                    if(jointpos[4]>2*PI)
                        axis4_PosError=jointpos[4]-2*PI;
                    else if(jointpos[4]<-2*PI)
                        axis4_PosError=jointpos[4]+2*PI;
                }
            }
            else if (fabs(jointvelo[i])>axis_VeloLimit[i])
                axis_Vflag[i]=0;
            else
            {
                axis_Dflag[i]=1;
                axis_Vflag[i]=1;
                if(i==2)
                    axis2_Sflag=0;                     	//轴2超过限位的标志位
                if (i==4)
                    axis4_Sflag=0;                    		//轴4超过限位的标志位
            }
        }

        //轴2的静摩擦定义
        if(jointvelo[1]>0.1)
            axis_Direct[1]=1;
        else if(jointvelo[1]<-0.1)
            axis_Direct[1]=-1;

        if(fabs(jointvelo[1])<0.1)
            axis2_StillOffset=70;
        else
            axis2_StillOffset=0;

        //轴6的静摩擦定义
        if(jointvelo[5]>0.1)
            axis_Direct[5]=1;
        else if(jointvelo[5]<-0.1)
            axis_Direct[5]=-1;


        if(fabs(jointvelo[5])<0.2)
            axis6_StillOffset=40;
        else
            axis6_StillOffset=0;

        if(bExec)
            axis_stopflag=1;
        else
            axis_stopflag=0;


        axis_TargetTorque[0]=int(axis_stopflag*(axis_Dflag[0]*axis_SmoothDirect[0]*axis_Dfriction[0]+axis_Vflag[0]*axis_Vfriction[0]*jointvelo[0]));
        axis_TargetTorque[1]=int(axis_Kt[1]*axis_TorGravity[1]+axis_stopflag*(axis_Dflag[1]*axis_SmoothDirect[1]*axis_Dfriction[1]+axis_Vflag[1]*axis_Vfriction[1]*jointvelo[1]+axis_Direct[1]*axis2_StillOffset));
        axis_TargetTorque[2]=int(axis_Kt[2]*axis_TorGravity[2]+axis_stopflag*(axis_Dflag[2]*axis_SmoothDirect[2]*axis_Dfriction[2]+axis_Vflag[2]*axis_Vfriction[2]*jointvelo[2]+axis2_Sflag*(axis2_Sk*axis2_PosError)));
        axis_TargetTorque[3]=int(axis_Kt[3]*axis_TorGravity[3]+axis_stopflag*(axis_Dflag[3]*axis_SmoothDirect[3]*axis_Dfriction[3]+axis_Vflag[3]*axis_Vfriction[3]*jointvelo[3]));
        axis_TargetTorque[4]=int(axis_Kt[4]*axis_TorGravity[4]+axis_stopflag*(axis_Dflag[4]*axis_SmoothDirect[4]*axis_Dfriction[4]+axis_Vflag[4]*axis_Vfriction[4]*jointvelo[4]+axis4_Sflag*(axis4_Sk*axis4_PosError)));
        axis_TargetTorque[5]=int(axis_Kt[5]*axis_TorGravity[5]+axis_stopflag*(axis_Dflag[5]*axis_SmoothDirect[5]*axis_Dfriction[5]+axis_Vflag[5]*axis_Vfriction[5]*jointvelo[5]+axis_Direct[5]*axis6_StillOffset));
        axis_TargetTorque[6]=int(axis_Kt[6]*axis_TorGravity[6]+axis_stopflag*(axis_Dflag[6]*axis_SmoothDirect[6]*axis_Dfriction[6]+axis_Vflag[6]*axis_Vfriction[6]*jointvelo[6]));

        if(!(cycle_counter%100))
        {
           int i=3;
            //for(int i=0;i<7;i++)
            {
                printf("%d: jointpostemp: %f     TorGravity: %f     TargetTorque: %d       ActTorque: %d\n",i,jointpostemp[i],axis_TorGravity[i],axis_TargetTorque[i],axis[i]->acttor());

                //printf("axis_ActTorque[%d]): %f\n",i,axis[i]->acttor());
                //printf("TargetTorque[%d): %d      ",i,axis_TargetTorque[i]);
                //printf("ActTorque[%d]): %f\n",i,axis[i]->acttor());
            }
            printf("\n");
        }

//        for(int i=0;i<7;i++)
            axis[3]->tartor(axis_TargetTorque[3]);
        if(!bExec &&
           fabs(jointvelo[0])<0.5 &&
           fabs(jointvelo[1])<0.5 &&
           fabs(jointvelo[2])<0.5 &&
           fabs(jointvelo[3])<0.5 &&
           fabs(jointvelo[4])<0.5 &&
           fabs(jointvelo[5])<0.5 &&
           fabs(jointvelo[6])<0.5)
            step++;
        break;

    case 3:
        for(int i=0;i<7;i++)
            axis[i]->modop(8);
        if (axis[0]->disop()==8 &&
                axis[1]->disop()==8 &&
                axis[2]->disop()==8 &&
                axis[3]->disop()==8 &&
                axis[4]->disop()==8 &&
                axis[5]->disop()==8 &&
                axis[6]->disop()==8)
            step=1;
        break;

    }
}

