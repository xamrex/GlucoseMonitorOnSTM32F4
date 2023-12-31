/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <touchgfx/canvas_widget_renderer/CanvasWidgetRenderer.hpp>
#include <touchgfx/Color.hpp>
#include <images/BitmapDatabase.hpp>
#include <texts/TextKeysAndLanguages.hpp>

Screen1ViewBase::Screen1ViewBase() :
    buttonCallback(this, &Screen1ViewBase::buttonCallbackHandler)
{
    touchgfx::CanvasWidgetRenderer::setupBuffer(canvasBuffer, CANVAS_BUFFER_SIZE);
    
    __background.setPosition(0, 0, 320, 240);
    __background.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    add(__background);

    image1.setXY(-3, 0);
    image1.setBitmap(touchgfx::Bitmap(BITMAP_BG_ID));
    add(image1);

    textDelta30and60.setPosition(1, 213, 128, 12);
    textDelta30and60.setColor(touchgfx::Color::getColorFromRGB(41, 130, 255));
    textDelta30and60.setLinespacing(0);
    touchgfx::Unicode::snprintf(textDelta30and60Buffer1, TEXTDELTA30AND60BUFFER1_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_BGBQ).getText());
    textDelta30and60.setWildcard1(textDelta30and60Buffer1);
    touchgfx::Unicode::snprintf(textDelta30and60Buffer2, TEXTDELTA30AND60BUFFER2_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_XNY0).getText());
    textDelta30and60.setWildcard2(textDelta30and60Buffer2);
    textDelta30and60.setTypedText(touchgfx::TypedText(T___SINGLEUSE_I6EP));
    add(textDelta30and60);

    textDelta5and10.setPosition(2, 201, 128, 12);
    textDelta5and10.setColor(touchgfx::Color::getColorFromRGB(41, 130, 255));
    textDelta5and10.setLinespacing(0);
    touchgfx::Unicode::snprintf(textDelta5and10Buffer1, TEXTDELTA5AND10BUFFER1_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_6098).getText());
    textDelta5and10.setWildcard1(textDelta5and10Buffer1);
    touchgfx::Unicode::snprintf(textDelta5and10Buffer2, TEXTDELTA5AND10BUFFER2_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_P37R).getText());
    textDelta5and10.setWildcard2(textDelta5and10Buffer2);
    textDelta5and10.setTypedText(touchgfx::TypedText(T___SINGLEUSE_Z096));
    add(textDelta5and10);

    textLastBGValue.setPosition(4, 186, 166, 15);
    textLastBGValue.setColor(touchgfx::Color::getColorFromRGB(245, 20, 20));
    textLastBGValue.setLinespacing(0);
    touchgfx::Unicode::snprintf(textLastBGValueBuffer1, TEXTLASTBGVALUEBUFFER1_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_L9L1).getText());
    textLastBGValue.setWildcard1(textLastBGValueBuffer1);
    touchgfx::Unicode::snprintf(textLastBGValueBuffer2, TEXTLASTBGVALUEBUFFER2_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_4H00).getText());
    textLastBGValue.setWildcard2(textLastBGValueBuffer2);
    textLastBGValue.setTypedText(touchgfx::TypedText(T___SINGLEUSE_5077));
    add(textLastBGValue);

    boxWithBorder1.setPosition(0, 0, 320, 185);
    boxWithBorder1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    boxWithBorder1.setBorderColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    boxWithBorder1.setBorderSize(5);
    add(boxWithBorder1);

    graph1.setPosition(7, 9, 306, 183);
    graph1.setScaleX(1);
    graph1.setScaleY(1);
    graph1.setGraphAreaMargin(0, 20, 0, 23);
    graph1.setGraphAreaPadding(5, 5, 10, 5);
    graph1.setGraphRangeX(-180, 0);
    graph1.setGraphRangeY(50, 250);
    graph1MinorXAxisGrid.setColor(touchgfx::Color::getColorFromRGB(20, 151, 197));
    graph1MinorXAxisGrid.setInterval(10);
    graph1MinorXAxisGrid.setLineWidth(1);
    graph1MinorXAxisGrid.setScale(1);
    graph1MinorXAxisGrid.setMajorGrid(graph1MajorXAxisGrid);
    graph1.addGraphElement(graph1MinorXAxisGrid);

    graph1MinorYAxisGrid.setColor(touchgfx::Color::getColorFromRGB(20, 151, 197));
    graph1MinorYAxisGrid.setInterval(10);
    graph1MinorYAxisGrid.setLineWidth(1);
    graph1MinorYAxisGrid.setScale(1);
    graph1MinorYAxisGrid.setMajorGrid(graph1MajorYAxisGrid);
    graph1.addGraphElement(graph1MinorYAxisGrid);

    graph1MajorXAxisGrid.setColor(touchgfx::Color::getColorFromRGB(20, 151, 197));
    graph1MajorXAxisGrid.setInterval(30);
    graph1MajorXAxisGrid.setLineWidth(2);
    graph1MajorXAxisGrid.setScale(1);
    graph1.addGraphElement(graph1MajorXAxisGrid);

    graph1MajorYAxisGrid.setColor(touchgfx::Color::getColorFromRGB(20, 151, 197));
    graph1MajorYAxisGrid.setInterval(50);
    graph1MajorYAxisGrid.setLineWidth(2);
    graph1MajorYAxisGrid.setScale(1);
    graph1.addGraphElement(graph1MajorYAxisGrid);

    graph1MajorXAxisLabel.setInterval(30);
    graph1MajorXAxisLabel.setLabelTypedText(touchgfx::TypedText(T___SINGLEUSE_YM6G));
    graph1MajorXAxisLabel.setColor(touchgfx::Color::getColorFromRGB(20, 151, 197));
    graph1MajorXAxisLabel.setScale(1);
    graph1.addBottomElement(graph1MajorXAxisLabel);

    graph1MajorYAxisLabel.setInterval(50);
    graph1MajorYAxisLabel.setLabelTypedText(touchgfx::TypedText(T___SINGLEUSE_SG0A));
    graph1MajorYAxisLabel.setColor(touchgfx::Color::getColorFromRGB(20, 151, 197));
    graph1MajorYAxisLabel.setScale(1);
    graph1.addLeftElement(graph1MajorYAxisLabel);

    graph1Line1Painter.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    graph1Line1.setPainter(graph1Line1Painter);
    graph1Line1.setLineWidth(2);
    graph1.addGraphElement(graph1Line1);

    graph1Dots1Painter.setColor(touchgfx::Color::getColorFromRGB(0, 76, 255));
    graph1Dots1.setPainter(graph1Dots1Painter);
    graph1Dots1.setDotWidth(5);
    graph1.addGraphElement(graph1Dots1);


    add(graph1);

    graph1Min.setPosition(7, 9, 306, 183);
    graph1Min.setScaleX(1);
    graph1Min.setScaleY(1);
    graph1Min.setGraphAreaMargin(0, 20, 0, 23);
    graph1Min.setGraphAreaPadding(5, 5, 10, 5);
    graph1Min.setGraphRangeX(-240, 0);
    graph1Min.setGraphRangeY(50, 250);
    graph1MinLine1Painter.setColor(touchgfx::Color::getColorFromRGB(255, 0, 0));
    graph1MinLine1.setPainter(graph1MinLine1Painter);
    graph1MinLine1.setLineWidth(2);
    graph1Min.addGraphElement(graph1MinLine1);

    add(graph1Min);

    graph1Max.setPosition(7, 9, 306, 183);
    graph1Max.setScaleX(1);
    graph1Max.setScaleY(1);
    graph1Max.setGraphAreaMargin(0, 20, 0, 23);
    graph1Max.setGraphAreaPadding(5, 5, 10, 5);
    graph1Max.setGraphRangeX(-240, 0);
    graph1Max.setGraphRangeY(50, 250);
    graph1MaxLine1Painter.setColor(touchgfx::Color::getColorFromRGB(255, 0, 0));
    graph1MaxLine1.setPainter(graph1MaxLine1Painter);
    graph1MaxLine1.setLineWidth(2);
    graph1Max.addGraphElement(graph1MaxLine1);

    add(graph1Max);

    textStep.setPosition(250, 221, 46, 15);
    textStep.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    textStep.setLinespacing(0);
    Unicode::snprintf(textStepBuffer, TEXTSTEP_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_3HBK).getText());
    textStep.setWildcard(textStepBuffer);
    textStep.setTypedText(touchgfx::TypedText(T___SINGLEUSE_C6LL));
    add(textStep);

    buttonZoomIn.setXY(244, 184);
    buttonZoomIn.setBitmaps(touchgfx::Bitmap(BITMAP_GTK_ZOOM_IN2_ID), touchgfx::Bitmap(BITMAP_GTK_ZOOM_IN2_ID));
    buttonZoomIn.setAction(buttonCallback);
    add(buttonZoomIn);

    buttonZoomOut.setXY(284, 184);
    buttonZoomOut.setBitmaps(touchgfx::Bitmap(BITMAP_GTK_ZOOM_OUT2_ID), touchgfx::Bitmap(BITMAP_GTK_ZOOM_OUT2_ID));
    buttonZoomOut.setAction(buttonCallback);
    add(buttonZoomOut);

    circleProgress1.setXY(298, 218);
    circleProgress1.setProgressIndicatorPosition(0, 0, 20, 20);
    circleProgress1.setRange(0, 59);
    circleProgress1.setCenter(10, 10);
    circleProgress1.setRadius(9);
    circleProgress1.setLineWidth(0.6f);
    circleProgress1.setStartEndAngle(0, 360);
    circleProgress1.setCapPrecision(180);
    circleProgress1.setBackground(touchgfx::Bitmap(BITMAP_SMALL1_ID));
    circleProgress1Painter.setColor(touchgfx::Color::getColorFromRGB(0, 240, 255));
    circleProgress1.setPainter(circleProgress1Painter);
    circleProgress1.setValue(0);
    add(circleProgress1);

    textBGFade.setPosition(0, 0, 313, 105);
    textBGFade.setColor(touchgfx::Color::getColorFromRGB(152, 36, 224));
    textBGFade.setLinespacing(0);
    Unicode::snprintf(textBGFadeBuffer, TEXTBGFADE_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_R0HF).getText());
    textBGFade.setWildcard(textBGFadeBuffer);
    textBGFade.setTypedText(touchgfx::TypedText(T___SINGLEUSE_MC1C));
    textBGFade.setAlpha(0);
    add(textBGFade);

    textt123.setPosition(2, 226, 167, 12);
    textt123.setColor(touchgfx::Color::getColorFromRGB(0, 74, 178));
    textt123.setLinespacing(0);
    touchgfx::Unicode::snprintf(textt123Buffer1, TEXTT123BUFFER1_SIZE, "%s", touchgfx::TypedText(T___SINGLEUSE_DOJO).getText());
    textt123.setWildcard1(textt123Buffer1);
    textt123Buffer1[0] = 0;
    textt123.setWildcard2(textt123Buffer2);
    textt123.setTypedText(touchgfx::TypedText(T___SINGLEUSE_5CI4));
    add(textt123);
}

Screen1ViewBase::~Screen1ViewBase()
{
    touchgfx::CanvasWidgetRenderer::resetBuffer();
}

void Screen1ViewBase::setupScreen()
{

}

void Screen1ViewBase::buttonCallbackHandler(const touchgfx::AbstractButton& src)
{
    if (&src == &buttonZoomIn)
    {
        //InterractionZoomIn
        //When buttonZoomIn clicked call virtual function
        //Call functionZoomIn
        functionZoomIn();
    }
    if (&src == &buttonZoomOut)
    {
        //InterractionZoomOut
        //When buttonZoomOut clicked call virtual function
        //Call functionZoomOut
        functionZoomOut();
    }
}

void Screen1ViewBase::handleKeyEvent(uint8_t key)
{
    if(1 == key)
    {
        //FADENeededInteractionForLibrary
        //When hardware button 1 clicked fade textBGFade
        //Fade textBGFade to alpha:0 with LinearInOut easing in 1234 ms (74 Ticks)
        textBGFade.clearFadeAnimationEndedAction();
        textBGFade.setFadeAnimationDelay(1);
        textBGFade.startFadeAnimation(0, 74, touchgfx::EasingEquations::linearEaseInOut);
    
    }
}
