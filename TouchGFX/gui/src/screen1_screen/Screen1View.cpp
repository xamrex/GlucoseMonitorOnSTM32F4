#include <gui/screen1_screen/Screen1View.hpp>
#include <touchgfx/Color.hpp>
#ifndef SIMULATOR
#include "biblioteka.h"
extern BGStruct BG;
#endif
Screen1View::Screen1View()
{

}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();
    //add horizontal line with  MIN target on plot
    graph1Min.addDataPoint(-240, MinTargetRange);
	graph1Min.addDataPoint(0, MinTargetRange);

	//add horizontal line with MAX target on plot
	graph1Max.addDataPoint(-240, MaxTargetRange);
	graph1Max.addDataPoint(0, MaxTargetRange);
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}
void Screen1View::UpdateGraphXAxis(){
	    graph1.setGraphRangeX(-60*howManyHrs, 0); //range -60* (1 to 4)
	    graph1MajorXAxisGrid.setInterval(10*howManyHrs);
	    graph1MajorXAxisLabel.setInterval(10*howManyHrs);
	    graph1.invalidate();
}
void Screen1View::functionZoomIn(){
	howManyHrs--;
	if (howManyHrs<1) howManyHrs=1;
	UpdateGraphXAxis();
}
void Screen1View::functionZoomOut(){
	howManyHrs++;
#ifdef SIMULATOR
#define HowManyMins 240
#endif
	if (howManyHrs>HowManyMins/60) howManyHrs=4;
	UpdateGraphXAxis();

	}
void Screen1View::DrawPoint2(){
#ifndef SIMULATOR
	graph1.clear();
	for (int i=0;i<HowManyMins;i++){
		if (BG.values[i]){ //plot only when BG is not 0 [dont plot zeros]
			graph1.addDataPoint(-i, BG.values[i]);  /// change to graph1.addDataPoint(-i, BG.values[i]);
		}
	}
	Unicode::snprintf(textLastBGValueBuffer1,TEXTLASTBGVALUEBUFFER1_SIZE,"%d",BG.LastValue); //
	Unicode::snprintf(textLastBGValueBuffer2,TEXTLASTBGVALUEBUFFER2_SIZE,"%d",BG.LastValueTime); //
	//color fade bg
	if(BG.values[0]<MinTargetRange) 	textBGFade.setColor(touchgfx::Color::getColorFromRGB(255, 0, 0));
	else if(BG.values[0]<MaxTargetRange) 	textBGFade.setColor(touchgfx::Color::getColorFromRGB(0, 255, 0));
	else if(BG.values[0]>MaxTargetRange) 	textBGFade.setColor(touchgfx::Color::getColorFromRGB(255, 0, 0));

	Unicode::snprintf(textBGFadeBuffer,TEXTBGFADE_SIZE,"%d",BG.LastValue); //
	//textLastBGValueBuffer[0]='1';
	//textLastBGValueBuffer[1]='2';
	//textLastBGValueBuffer[2]='3';
	textLastBGValue.invalidate();

	BG.DrawPoints=0;

//
//	graph1.addDataPoint(-20.0,22);
//	graph1.addDataPoint(-25.0,33);
//	graph1.addDataPoint(-30.0,44);
#endif
#ifdef SIMULATOR
	graph1.addDataPoint(-20,100);
	graph1.addDataPoint(-30,125);
	graph1.addDataPoint(-40,80);
#endif
}

void Screen1View::ShowStep2(){
#ifndef SIMULATOR
	Unicode::snprintf(textStepBuffer,TEXTSTEP_SIZE,"%d",BG.ActualStep);
	textStep.invalidate();
#endif
}

void Screen1View::UpdateCircle2(){
#ifndef SIMULATOR
	circleProgress1.setValue(BG.timer1s);
	graph1Min.addDataPoint(-240,70);
	graph1Min.addDataPoint(0,70);
#endif
}

void Screen1View::FadeBGin2(){
#ifndef SIMULATOR
	 textBGFade.clearFadeAnimationEndedAction();
	        textBGFade.setFadeAnimationDelay(1);
	        textBGFade.startFadeAnimation(255, 74, touchgfx::EasingEquations::linearEaseInOut);
#endif
}
void Screen1View::FadeBGout2(){
#ifndef SIMULATOR
	 textBGFade.clearFadeAnimationEndedAction();
	        textBGFade.setFadeAnimationDelay(1);
	        textBGFade.startFadeAnimation(0, 74, touchgfx::EasingEquations::linearEaseInOut);
#endif
}

void Screen1View::UpdateDelta2(){
#ifndef SIMULATOR

	Unicode::snprintfFloat(textDelta5and10Buffer1 ,TEXTDELTA5AND10BUFFER1_SIZE,"%.2f" ,BG.delta5m);
	Unicode::snprintfFloat(textDelta5and10Buffer2 ,TEXTDELTA5AND10BUFFER2_SIZE,"%.2f" ,BG.delta10m);
	textDelta5and10.invalidate();

	Unicode::snprintfFloat(textDelta30and60Buffer1 ,TEXTDELTA30AND60BUFFER1_SIZE,"%.2f" ,BG.delta30m);
	Unicode::snprintfFloat(textDelta30and60Buffer2 ,TEXTDELTA30AND60BUFFER2_SIZE,"%.2f" ,BG.delta60m);
	textDelta30and60.invalidate();

	float params[3] = { BG.t1, BG.t2 , BG.t3}; //needed
	Unicode::snprintfFloats(textt123Buffer1 ,TEXTT123BUFFER1_SIZE,"t1=%.2f t2=%.2f t3=%.2f", params);

	// if ( t0<t1<t2) arrow left  (deccelerate)
	// elseif (t0>t1>t2) arrow rigt (accelerate)
	// else cant draw trend.

	if( BG.t1<BG.t2 && BG.t2<BG.t3 ){
		Unicode::snprintf(textt123Buffer2 ,TEXTT123BUFFER2_SIZE," <--");
	}
	else if ( BG.t1>BG.t2 && BG.t2>BG.t3){
		Unicode::snprintf(textt123Buffer2 ,TEXTT123BUFFER2_SIZE," -->");
	}
	else{
		Unicode::snprintf(textt123Buffer2 ,TEXTT123BUFFER2_SIZE," ---");
	}

	textt123.invalidate();

#endif
}
