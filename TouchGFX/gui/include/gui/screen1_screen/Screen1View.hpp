#ifndef SCREEN1VIEW_HPP
#define SCREEN1VIEW_HPP

#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

class Screen1View : public Screen1ViewBase
{
public:
    Screen1View();
    virtual ~Screen1View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();
    virtual void DrawPoint2();
    virtual void ShowStep2();
    virtual void UpdateCircle2();
    virtual void UpdateDelta2();
    virtual void FadeBGin2();
    virtual void FadeBGout2();
	virtual void functionZoomIn();
	virtual void functionZoomOut();
	virtual void UpdateGraphXAxis();


protected:
	int howManyHrs=3; //indicates how many hrs are visible on plot.
};

#endif // SCREEN1VIEW_HPP
