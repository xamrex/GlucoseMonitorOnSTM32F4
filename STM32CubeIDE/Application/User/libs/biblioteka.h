/*
 * biblioteka.h
 *
 *  Created on: 17 lip 2023
 *      Author: Xamrex
 */

#ifndef APPLICATION_USER_LIBS_BIBLIOTEKA_H_
#define APPLICATION_USER_LIBS_BIBLIOTEKA_H_
/*************** BELLOW YOU CAN ADJUST VALUES******************/
#define HowManyMins 60*4   //max is 240 -> Maximum number of minutes that you want to ee
#define MinTargetRange 70 // (in mg/dl)
#define MaxTargetRange 170 //(in mg/dl)

/*************** eND OF ADJUSTMENTS******************/
#define OK 1
#define error 0

typedef struct
{
	volatile float delta5m, delta10m, delta15m,delta30m,delta60m;
	volatile float t1,t2,t3; //diff in delta		t1=diff betwen acutal value and last value
	volatile int timer1s;
	volatile int ActualStep; //Indicate in main.c what step is actually running
	volatile int DrawPoints; //if 1 - draw points on plot
	volatile int values[HowManyMins];
	volatile int LastValue; //Last received BG Value (newest)
	volatile int LastValueTime; //how mins ago returned last value
	volatile int NewData;  //flag that told that data is in array values [0;123;0;0;0;0;0;231;0;0;0;0;0;93,.....,0;0;0;]

}BGStruct;


int CalculateDelta(void);
int calculateDiffinDelta(void);
int CalculateDelta2(int time); //smaller funcion to help find deltas.
float CalculateAbs (float value);

#endif /* APPLICATION_USER_LIBS_BIBLIOTEKA_H_ */
