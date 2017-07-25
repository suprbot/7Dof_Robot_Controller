/****************************provide by suprbot2015@gmail.com*******************************/
#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/mman.h>
#include <rtdm/rtdm.h>
#include <native/task.h>
#include <native/sem.h>
#include <native/mutex.h>
#include <native/timer.h>
#include <native/task.h>
#include <native/pipe.h>
#include <native/intr.h>
#include <rtdk.h>
#include <pthread.h>
#include <math.h>
#include <stdbool.h>

#include "rcconfig.h"
#include "global_variables.h"

#include "ecrt.h"
#include "elmo_ecat.h"
#include "mc_power.h"
#include "mc_moverelative.h"
#include "mc_moveabsolute.h"
#include "mc_jog.h"
#include "robot_jog.h"
#include "socket_server.h"
#include "kine.h"
#include "mc_directeach.h"

#define CYC_TIME 10
#define Muti_AXISES 1

RT_TASK ecat_task;
RT_TASK pos_task;
static pthread_t t_reg;

static int run = 1;

/*****************************************************************************/

static ec_master_t *master = ecrt_request_master(0);
static ec_master_state_t master_state = {};

elmo_ecat* joints[7]={new elmo_ecat(master,0,0,CYC_TIME),
                      new elmo_ecat(master,0,1,CYC_TIME),
                      new elmo_ecat(master,0,2,CYC_TIME),
                      new elmo_ecat(master,0,3,CYC_TIME),
                      new elmo_ecat(master,0,4,CYC_TIME),
                      new elmo_ecat(master,0,5,CYC_TIME),
                      new elmo_ecat(master,0,6,CYC_TIME)};

mc_power axis_enable[7]={mc_power(joints[0]),
                         mc_power(joints[1]),
                         mc_power(joints[2]),
                         mc_power(joints[3]),
                         mc_power(joints[4]),
                         mc_power(joints[5]),
                         mc_power(joints[6])};

robot_jog robot_Jog(joints);

mc_directeach mc_Directeach(joints);

/*****************************************************************************/
/****************************************************************************/

void ecat_task_proc(void *arg)
{
    rt_task_set_periodic(NULL, TM_NOW, 1000000*CYC_TIME); // ns  CYC_TIME ms

    while(run){
        rt_task_wait_period(NULL);
        cycle_counter++;

        for(int i=0;i<7;i++)
            joints[i]->sync_pdos();
    }
}

void pos_task_proc(void *arg)
{
    rt_task_set_periodic(NULL, TM_NOW, 1000000*CYC_TIME); // ns  CYC_TIME ms

    double endPosM[6];

    int step=0;

    while(run){
        rt_task_wait_period(NULL);

        switch(step){
        case 0:
            for(int i=0;i<7;i++)
                axis_enable[i].run(true) ;

            if(axis_enable[0].done &&
                    axis_enable[1].done &&
                    axis_enable[2].done &&
                    axis_enable[3].done &&
                    axis_enable[4].done &&
                    axis_enable[5].done &&
                    axis_enable[6].done)
            {step=2;printf("power done\n");}
            break;

        case 1:
            for(int i=0;i<7;i++)
            {
                jointPos[i]=joints[i]->actpos_bias_modulo();
                rc2tp.jointPos[i]=jointPos[i];
                jointPosArc[i]=jointPos[i]*PI/180.0;
            }

            Fkine_rpy(jointPosArc,endPosM);

            for(int i=0;i<3;i++)
            {
                endPos[i]=endPosM[i]*1000.0;
                rc2tp.endPos[i]=endPos[i];
            }
            for(int i=3;i<6;i++)
            {
                endPos[i]=endPosM[i]*180.0/PI;
                rc2tp.endPos[i]=endPos[i];
            }

            robot_Jog.run(tp2rc.exec,tp2rc.jogMode,0,tp2rc.jogjoint,tp2rc.jogend);
            break;

        case 2:
            for(int i=0;i<7;i++)
            {
                jointPos[i]=joints[i]->actpos_bias_modulo();
                rc2tp.jointPos[i]=jointPos[i];
                jointPosArc[i]=jointPos[i]*PI/180.0;
            }

            mc_Directeach.run(true);
            break;
        }
    }
}

/****************************************************************************/
/****************************************************************************/

void signal_handler(int sig)
{
    run = 0;
}

/****************************************************************************/
int main(int argc, char **argv)
{
    int ret;

    /* Perform auto-init of rt_print buffers if the task doesn't do so */
    rt_print_auto_init(1);

    signal(SIGTERM, signal_handler);
    signal(SIGINT, signal_handler);

    mlockall(MCL_CURRENT | MCL_FUTURE);

    for(int i=0;i<7;i++)
        joints[i]->config_pdos();

    printf("Activating master...\n");
    if (ecrt_master_activate(master))
        fprintf(stderr, "Activating master failed!\n");

    for(int i=0;i<7;i++)
        joints[i]->link_pdos();

    joints[0]->setname("joint 0");
    joints[1]->setname("joint 1");
    joints[2]->setname("joint 2");
    joints[3]->setname("joint 3");
    joints[4]->setname("joint 4");
    joints[5]->setname("joint 5");
    joints[6]->setname("joint 6");

    ret = pthread_create(&t_reg, NULL, tcp_server_thread, NULL);

    ret = rt_task_create(&ecat_task, "ecat_task", 0, 80, T_FPU);
    if (ret < 0) {
        fprintf(stderr, "Failed to create task: %s\n", strerror(-ret));
        return -1;
    }

    ret = rt_task_create(&pos_task, "pos_task", 0, 70, T_FPU);
    if (ret < 0) {
        fprintf(stderr, "Failed to create task: %s\n", strerror(-ret));
        return -1;
    }

    printf("Starting task...\n");
    ret = rt_task_start(&ecat_task, &ecat_task_proc, NULL);
    if (ret < 0) {
        fprintf(stderr, "Failed to start task: %s\n", strerror(-ret));
        return -1;
    }

    ret = rt_task_start(&pos_task, &pos_task_proc, NULL);
    if (ret < 0) {
        fprintf(stderr, "Failed to start task: %s\n", strerror(-ret));
        return -1;
    }

    while (run) {
        sched_yield();
    }

    printf("Deleting realtime task...\n");
    rt_task_delete(&ecat_task);
    rt_task_delete(&pos_task);

    printf("End of Program\n");
    ecrt_release_master(master);

    return 0;
}

/****************************************************************************/
