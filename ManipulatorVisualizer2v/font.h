#ifndef __FONT_H
#define __FONT_H
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
		~CFont();
		FTFont* Font;
		void Print(float x, float y, const wchar_t* text);
};
#endif
