#include <math.h>
#include <stdio.h>

#include "robot_jog.h"
#include "kine.h"

robot_jog::robot_jog(elmo_ecat *_axis[]):
    axis(_axis),
    enable(false),
    busy(false),
    error(false),
    errorid(0),
    done(false),
    step(0),
    joint_mcjog({mc_jog(_axis[0]),
    mc_jog(_axis[1]),
    mc_jog(_axis[2]),
    mc_jog(_axis[3]),
    mc_jog(_axis[4]),
    mc_jog(_axis[5]),
    mc_jog(_axis[6])}),
    endJogcp({jog_cp(),jog_cp(),jog_cp(),jog_cp(),jog_cp(),jog_cp()})
{}

robot_jog::~robot_jog()
{}

void robot_jog::run(bool _bExec,int _jogMode,int _jogVel, int* _jogJoint,int* _jogEnd)
{
    if(error) return;

    switch (step){
    case 0:
        if(_bExec)
        {
            busy=true;
            error=false;
            done=false;
            step++;
        }
        break;

    case 1:
        if(!_bExec)step=9999;

        if(_jogMode==1)     //1:jog joint 2:jog end
            step=20;
        else if(_jogMode==2)
            step=30;
        else
            break;

        for(int i=0;i<7;i++)
            joint_mcjog[i].run(false,false);
        for(int i=0;i<6;i++)
            endJogcp[i].run(false,false);

        break;

    case 20:
        for(int i=0;i<7;i++)
        {
            if(_jogJoint[i]==1)
            {
                joint_fw[i]=true;
                joint_bw[i]=false;
            }
            else if(_jogJoint[i]==-1)
            {
                joint_fw[i]=false;
                joint_bw[i]=true;
            }
            else
            {
                joint_fw[i]=false;
                joint_bw[i]=false;
            }
        }
        for(int i=0;i<7;i++)
            joint_mcjog[i].run(joint_fw[i],joint_bw[i],10,50,100);

        if(_jogMode!=1)
            if(!(joint_mcjog[0].busy||
                 joint_mcjog[1].busy||
                 joint_mcjog[2].busy||
                 joint_mcjog[3].busy||
                 joint_mcjog[4].busy||
                 joint_mcjog[5].busy||
                 joint_mcjog[6].busy))
                step=1;
        break;

    case 30:
        for(int i=0;i<6;i++)
            endStartpos[i]=endPos[i];
        step++;
        break;
    case 31:
        for(int i=0;i<6;i++)
        {
            if(_jogEnd[i]==1)
            {
                end_fw[i]=true;
                end_bw[i]=false;
            }
            else if(_jogEnd[i]==-1)
            {
                end_fw[i]=false;
                end_bw[i]=true;
            }
            else
            {
                end_fw[i]=false;
                end_bw[i]=false;
            }
            endTarpos[i]=endStartpos[i]+endJogcp[i].run(end_fw[i],end_bw[i],20,50,100);
        }

        if(endJogcp[0].bquit||
                endJogcp[1].bquit||
                endJogcp[2].bquit||
                endJogcp[3].bquit||
                endJogcp[4].bquit||
                endJogcp[5].bquit)
            step++;

        for(int i=0;i<3;i++)
            endTarPosM[i]=endTarpos[i]/1000.0;
        for(int i=3;i<6;i++)
            endTarPosM[i]=endTarpos[i]*PI/180.0;

        for(int i=0;i<7;i++)
            jointIkposArc[i]=jointPosArc[i];

        Ikine_rpy(endTarPosM,jointIkposArc,&psi_last);

        for(int i=0;i<6;i++)
        {
            jointIkpos[i]=jointIkposArc[i]*180.0/PI;
            axis[i]->tarpos_bias_modulo(jointIkpos[i]);
        }
        break;

    case 32:
        for(int i=0;i<6;i++)
            endTarpos[i]=endStartpos[i]+endJogcp[i].run(false,false,20,50,100);

        for(int i=0;i<3;i++)
            endTarPosM[i]=endTarpos[i]/1000.0;
        for(int i=3;i<6;i++)
            endTarPosM[i]=endTarpos[i]*PI/180.0;

        for(int i=0;i<7;i++)
            jointIkposArc[i]=jointPosArc[i];

        Ikine_rpy(endTarPosM,jointIkposArc,&psi_last);

        for(int i=0;i<6;i++)
        {
            jointIkpos[i]=jointIkposArc[i]*180.0/PI;
            axis[i]->tarpos_bias_modulo(jointIkpos[i]);
        }


        if(!(endJogcp[0].cping||
             endJogcp[1].cping||
             endJogcp[2].cping||
             endJogcp[3].cping||
             endJogcp[4].cping||
             endJogcp[5].cping))
            step=1;
        break;

    case 9999:
        busy=false;
        done=true;
        step=0;
        break;
    }
}

