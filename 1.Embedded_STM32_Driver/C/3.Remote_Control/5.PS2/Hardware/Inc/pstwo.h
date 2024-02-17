/*
  ******************************************************************************
  * @author
  *机电创新学会
	*Vinci机器人队
	*董佳辉
  ******************************************************************************
  */
	
#ifndef __PSTWO_H_
#define __PSTWO_H_
#include "main.h"

void PS2Data_Receive(void);
void PS2Data_Get(void);
void PS2Rocker_correct(void);
void PS2Data_Acquire(void);
void PS2Data_Clear(void);

typedef struct
{
	uint8_t SELECT,L3,R3,START,UP,RIGHT,DOWN,LEFT;
	uint8_t L2,R2,L1,R1,TRIANGLE,CIRCULAR,X,SQUARE;
	uint8_t PSS_RX,PSS_RY,PSS_LX,PSS_LY;
}PS2_DataTypedef;


typedef struct
{
	int SELECT,L3,R3,START,UP,RIGHT,DOWN,LEFT;
	int L2,R2,L1,R1,TRIANGLE,CIRCULAR,X,SQUARE;
	int PSS_RX,PSS_RY,PSS_LX,PSS_LY;
}PS2_UserDataTypedef;


#endif

