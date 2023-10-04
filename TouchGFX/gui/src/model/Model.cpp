#include <gui/model/Model.hpp>
#include <gui/model/ModelListener.hpp>
#ifndef SIMULATOR
#include "biblioteka.h"
extern BGStruct BG;
int Fade; // 1-start fade
int ticktimer16ms;
int FlagStartTimer;
#endif

Model::Model() : modelListener(0)
{

}

void Model::tick()
{
#ifndef SIMULATOR

//to fade text START
//Fade in, wait 16ms* 312 then fade out
//if(Fade){
//	modelListener->FadeBGin();
//	FlagStartTimer=1;
//	Fade=0;
//}
if (FlagStartTimer){
	ticktimer16ms++;

}
if (ticktimer16ms >=312){ //
	ticktimer16ms=0; //clear
	FlagStartTimer=0; //clear
	modelListener->FadeBGout();
}
//to fade text END

//show current step (for debug only)
modelListener->ShowStep(); //show current step
modelListener->UpdateCircle();	//update circle.



if (BG.DrawPoints) { //if we get request form main.c to draw points and calculate delta
	if (BG.values[0] !=0){ //fade only when BG received.
	//start fade
	modelListener->FadeBGin();
	FlagStartTimer=1;
	}
	modelListener->DrawPoint();
	modelListener->UpdateDelta(); //update delta
}
#endif

#ifdef SIMULATOR
	modelListener->DrawPoint();
#endif
}
