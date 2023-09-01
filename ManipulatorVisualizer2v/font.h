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
		CFont(const char* ttf, int FSize, int FDepth);
		CFont();
		~CFont();
		FTFont* Font;
		//float Advance(const wchar_t* string);
		void Print(float x, float y, const wchar_t* text);
};
#endif

void drawLine(float x1, float y1, float x2, float y2);

class Table {
	public:
		CFont* font_;

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

		//std::vector<std::vector<float>> data; //size = (kRows, kColumns)
		std::vector<std::vector<std::wstring>> data_;
		std::vector<std::vector<std::function<void(int)>>> callbacks_; //callbacks if the button have been pressed
		//functions.emplace_back(std::bind(a, _1, _1)); _ if you want to use it
		//std::function<void(int)> a = NULL;


		Table(CFont* font, int kRows=1, int kColumns=1, int kSymb = 6, float xShift = 12, float yShift = 7.0f, float yTextShift = 2.0f);

		void setPosition(float xStart, float yStart);
		void addMainTitle(std::wstring mainTitle, float yMainTitleShift = 7.0f);
		void addColumnTitles(std::vector<std::wstring> columnTitles, float yTitleShift = 7.0f);
		void addRowTitles(std::vector<std::wstring> rowTitles, float xTitleShift);
		void setData(std::vector<std::vector<std::wstring>> data);
		void setCallbacks(std::vector<std::vector<std::function<void(int)>>> callbacks);

		void printTable();
		void mousePress(float x, float y);
		/*int i = (x - xStart - xTitleShift) / xShift;
		int j = (y - yStart - yMainTitleShift - yTitleShift) / yShift;
		callbacks[i][j];*/
};