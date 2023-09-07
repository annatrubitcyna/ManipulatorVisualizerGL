#ifndef __FONT_H
#define __FONT_H
#include <stdio.h>
#include <vector>
#include <functional>  
#include <wchar.h>
//#include <ftgl/ftgl.h>
#include <FTGL/FTFont.h>
#include <FTGL/FTGLBitmapFont.h>
#include <FTGL/FTGLOutlineFont.h>
#include <ftgl/FTGLPixmapFont.h>
#include <ftgl/FTGLPolygonFont.h>
#include <ftgl/FTGLTextureFont.h>
class CFont{
	public:
		CFont(std::string ttf = "C:/Windows/Fonts/arial.ttf", int FSize=24, int FDepth=32);
		//CFont();
		//~CFont();
		FTFont* Font;
		//float Advance(const wchar_t* string);
		void Print(float x, float y, const wchar_t* text);
};
#endif

void drawLine(float x1, float y1, float x2, float y2);
//typedef void (*Callback)();

class Table {
	public:
		CFont* font_;
		CFont* mathFont_;
		float xStart_;
		float yStart_;
		float xShift_;
		float yShift_;
		float yTextShift_;
		float xTitleShift_; //for bigger/less column with row names
		float yTitleShift_;
		float yMainTitleShift_;

		int kSymb_; //with  sign and point and integer part (for float less than 1000: kSymb-4 numbers after point)
		int kRows_; //without title
		int kColumns_;

		float wholeWidth_;
		float wholeHeight_;

		std::wstring mainTitle_;
		std::vector<std::wstring> columnTitles_; //corner name there - size = kColumns+1
		std::vector<std::wstring> rowTitles_; //size = kRows

		std::vector<std::vector<std::wstring>> data_; //size = (kRows, kColumns)
		std::vector<std::vector<std::function<void()>>> callbacks_; //callbacks if the button have been pressed


		Table(CFont* font, int kRows=1, int kColumns=1, int kSymb = 6, float xShift = 12, float yShift = 7.0f, float yTextShift = 2.0f);
		Table();

		void setPosition(float xStart, float yStart);
		void addMainTitle(std::wstring mainTitle, float yMainTitleShift = 7.0f);
		void addColumnTitles(std::vector<std::wstring> columnTitles, float yTitleShift = 7.0f);
		void addRowTitles(std::vector<std::wstring> rowTitles, float xTitleShift);
		void setData(std::vector<std::vector<std::wstring>> data);
		void setCallbacks(std::vector<std::vector<std::function<void()>>> callbacks);

		void printTable();
		void mousePress(float x, float y);

		std::vector<std::vector<std::function<void()>>> initNullCallbacks();
		bool isInside(float x, float y);

};