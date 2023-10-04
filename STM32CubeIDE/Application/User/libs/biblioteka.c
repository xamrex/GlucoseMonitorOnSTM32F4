/*
 * biblioteka.c
 *
 *  Created on: 17 lip 2023
 *      Author: Xamrex
 */
#include "main.h"
#include "biblioteka.h"
#include <math.h>
typedef struct
{
	int Bvalue;
	int Btime;
}LastValuesStruct;
LastValuesStruct LV;
#define deltatreshold 4
extern BGStruct BG;


int CalculateDelta2(int time){
	for (int i=time; i< HowManyMins -time; i++){
		if (BG.values[i]!=0 ){ //if value different than 0 is found
			LV.Btime=i;
			LV.Bvalue=BG.values[i];
			return 1;
		}//all is fine and values are found, finish loop
	}
	return 0;
}



int CalculateDelta(){
	BG.delta5m=0; //clear  DELTA diff between ACTUAL and selected time.
	BG.delta10m=0; //clear
	BG.delta15m=0; //clear
	BG.delta30m=0; //clear
	BG.delta60m=0; //clear

	if( CalculateDelta2(0)) { //if we found delta0
		BG.LastValue=LV.Bvalue;
		BG.LastValueTime=LV.Btime;

		//calculate 1st delta - 5min
		if (CalculateDelta2(BG.LastValueTime+5)){ //if we found delta 5
			BG.delta5m=((float)(BG.LastValue - LV.Bvalue) / (LV.Btime - BG.LastValueTime  ) *5);

			//calculate 2nd delta - 10min
			if (CalculateDelta2(BG.LastValueTime+10)){ //if we found delta 10
				BG.delta10m=((float)(BG.LastValue - LV.Bvalue) / (LV.Btime - BG.LastValueTime  ) *10);

				if (CalculateDelta2(BG.LastValueTime+15)){ //if we found delta 15
					BG.delta15m=((float)(BG.LastValue - LV.Bvalue) / (LV.Btime - BG.LastValueTime  ) *15);

					if (CalculateDelta2(BG.LastValueTime+30)){ //if we found delta 30
						BG.delta30m=((float)(BG.LastValue - LV.Bvalue) / (LV.Btime - BG.LastValueTime  ) *30);

						if (CalculateDelta2(BG.LastValueTime+60)){ //if we found delta 60
							BG.delta60m=((float)(BG.LastValue - LV.Bvalue) / (LV.Btime - BG.LastValueTime  ) *60);
							//last calculation return OK
							return OK;
						}
					}
				}
			}
		}
	}

	return error;
}

int calculateDiffinDelta(){
	BG.t1=99;
	BG.t2=99;
	BG.t3=99;

	BG.t1= fabsf(BG.delta5m);
	BG.t2= fabsf(BG.delta10m-BG.delta5m);
	BG.t3= fabsf(BG.delta15m-BG.delta10m);

if (BG.t1!=99 && BG.t2!=99 && BG.t3!=99)  return OK; //if all is calculated return 1
return error;

}


 float CalculateAbs(float value){
 return   (value < 0) ? -value : value;
 }
