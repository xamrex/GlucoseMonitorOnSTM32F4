#include <gui/screen1_screen/Screen1View.hpp>
#include <gui/screen1_screen/Screen1Presenter.hpp>

Screen1Presenter::Screen1Presenter(Screen1View& v)
    : view(v)
{

}

void Screen1Presenter::activate()
{

}

void Screen1Presenter::deactivate()
{

}
void Screen1Presenter::DrawPoint()
{
	view.DrawPoint2();
}
void Screen1Presenter::ShowStep()
{
	view.ShowStep2();
}
void Screen1Presenter::UpdateCircle()
{
	view.UpdateCircle2();
}
void Screen1Presenter::UpdateDelta()
{
	view.UpdateDelta2();
}
void Screen1Presenter::FadeBGin()
{
	view.FadeBGin2();
}
void Screen1Presenter::FadeBGout()
{
	view.FadeBGout2();
}
