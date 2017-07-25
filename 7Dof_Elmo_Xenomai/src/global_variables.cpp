#include <stdio.h>

#include "global_variables.h"

/******demo******/
unsigned char CommandStackAppendPoint=0;

unsigned char SerialRxDataBuffer[4]={0,1,2,3};

struct Command CommandStack[4];

extern void InitCommandStack();//default: exturn;  otherwise can be defined as static
/****************/
int cycle_counter=0;

mes_rc2tp rc2tp={false,{0,0,0,0,0,0,0},{0,0,0,0,0,0}};
mes_tp2rc tp2rc={false,0,0,0,{0,0,0,0,0,0,0},{0,0,0,0,0,0}};

double jointPos[7]={0,0,0,0,0,0,0};
double jointPosArc[7]={0,0,0,0,0,0,0};
double endPos[6]={0,0,0,0,0,0};

double psi_last=10;
