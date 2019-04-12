#ifndef CGUI_H
#define CGUI_H

#include <math.h>
#include <SDL/SDL.h>
#include <SDL/SDL_ttf.h>
#include "CRawImage.h"
#include "SStructDefs.h"


class CGui
{
    public:

        CGui(int wi,int he,int scale, const char* font_path);

        ~CGui();

        void drawImage(CRawImage* image);

        void displayHelp(bool extended);

        void drawStats(int x,int y,STrackedObject o, bool D2);

        void drawTimeStats(int evalTime,int numBots);

        void guideCalibration(int calibNum,float a,float b);

        void saveScreen(int a = -1);

        void clearStats();

        void update();

        void drawEllipse(SSegment s,STrackedObject t);
        void drawLine(float sx1,float sx2,float sy1,float sy2);

    private:
        SDL_Surface *screen;
        TTF_Font *smallFont;
        int width,height, scale;
        int averageTime, maxTime, numStats, num;	
};

#endif

/* end of CGui.h */
