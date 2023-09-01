#include <GL/glut.h>
#include <iostream>

#include "font.h"

CFont::CFont(const char* ttf, int FSize, int FDepth)
{
	this->Font = new FTGLPixmapFont(ttf);
	if (!Font->FaceSize(FSize)) {
		//MessageBox(NULL, "Cant set font FaceSize", "Error", MB_OK);
		std::cout << "ERROR::Cant set font FaceSize" << std::endl;
		exit(1);
	}
	//Font->Depth(FDepth);
	Font->CharMap(ft_encoding_unicode);
	//printf("%i\n", Font->charSize.Width());
}
void CFont::Print(float x, float y, const wchar_t* text)
{
	glPushMatrix(); //to print more than one text
	//glLoadIdentity();
	glDisable(GL_TEXTURE_2D);
	// move to coordinates
	glTranslatef(x, y, -1);
	glRasterPos2f(0, 0);

	//was checking
	/*glBegin(GL_LINES);
	glVertex2f(0.333, 20);
	glVertex2f(0.666, 50);
	glEnd(); 

	glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, 'T');*/

	Font->Render(text);
	glEnable(GL_TEXTURE_2D);
	glPopMatrix();
}

//==========================================================================================================================|
//																															|
//														_TABLE																|
//																															|
//==========================================================================================================================|

void drawLine(float x1, float y1, float x2, float y2)
{
	glBegin(GL_LINES);
	glVertex2f(x1, y1);
	glVertex2f(x2, y2);
	glEnd();
}

Table::Table(CFont* font, int kRows, int kColumns,int kSymb, float xShift, float yShift, float yTextShift)
{
	font_ = font;
	kRows_ = kRows;
	kColumns_ = kColumns;
	yShift_ = yShift;
	yTextShift_ = yTextShift;
	kSymb_ = kSymb;
	if (xShift == 0)
		xShift_ = ((font_->Font->Advance(L"0")) * (kSymb - 1) + font_->Font->Advance(L".")) *200/ glutGet(GLUT_WINDOW_WIDTH) * 1.2; //to make little shift
	else
		xShift_ = xShift;

	xStart_ = 0;
	yStart_ = 0;
	yMainTitleShift_ = 0;
	mainTitle_ = L"";
	yTitleShift_ = 0;
	xTitleShift_ = 0;
	wholeWidth_ = kColumns_ * xShift_;
	wholeHeight_ = kRows_ * yShift_;
}

void Table::setPosition(float xStart, float yStart) 
{
	xStart_ = xStart;
	yStart_ = yStart;
}
void Table::addMainTitle(std::wstring mainTitle, float yMainTitleShift)
{
	yMainTitleShift_ = yMainTitleShift;
	mainTitle_ = mainTitle;
	wholeHeight_ += yMainTitleShift_;
}
void Table::addColumnTitles(std::vector<std::wstring> columnTitles, float yTitleShift)
{
	yTitleShift_ = yTitleShift;
	columnTitles_ = columnTitles;
	wholeHeight_ += yTitleShift_;
}
void Table::addRowTitles(std::vector<std::wstring> rowTitles, float xTitleShift)
{
	xTitleShift_ = xTitleShift;
	rowTitles_ = rowTitles;
	wholeWidth_ += xTitleShift_;
}
void Table::setData(std::vector<std::vector<std::wstring>> data) 
{
	data_ = data;
}
void Table::setCallbacks(std::vector<std::vector<std::function<void(int)>>> callbacks)
{
	callbacks_ = callbacks;
}

void Table::printTable() {
	float x; //helping variables
	float y;
	int k;
	std::wstring text;

	//horizontal lines
	float xEnd = xStart_ + wholeWidth_;
	float yEnd = yStart_ + wholeHeight_;
	//first
	drawLine(xStart_, yStart_,
		     xEnd, yStart_);
	//second
	if(yMainTitleShift_!=0) 
		drawLine(xStart_, yStart_ + yMainTitleShift_,
				 xEnd, yStart_ + yMainTitleShift_);
	y = yStart_ + yMainTitleShift_ + yTitleShift_;
	for (int i = 0; i < kRows_+1; i++) {
		drawLine(xStart_, y + i * yShift_,
				 xEnd, y + i * yShift_);
	}

	//vertical lines
	//first
	drawLine(xStart_, yStart_,
			 xStart_, yEnd);
	y = yStart_ + yMainTitleShift_;
	x = xStart_ + xTitleShift_;
	for (int i = 0; i < kColumns_ ; i++) {
		drawLine(x + i * xShift_, y,
				 x + i * xShift_, yEnd);
	}
	//last
	drawLine(xEnd, yStart_,
			 xEnd, yEnd);

	//print titles
	//main
	if (yMainTitleShift_ != 0) {
		float titleLen = font_->Font->Advance(mainTitle_.c_str()) *200 / glutGet(GLUT_WINDOW_WIDTH);
		x = xStart_ + ((xEnd - xStart_) - titleLen) / 2;
		y = yStart_ + yMainTitleShift_ - yTextShift_;
		font_->Print(x,y, mainTitle_.c_str());
	}
	//column
	if (yTitleShift_ != 0) {
		for (int i = 0; i < columnTitles_.size(); i++) {
			float titleLen = font_->Font->Advance(columnTitles_[i].c_str()) * 200 / glutGet(GLUT_WINDOW_WIDTH);
			if(i==0) x = xStart_ +  (xTitleShift_ - titleLen) / 2;
			else x = xStart_ + xTitleShift_ + (i-1)*xShift_ + (xShift_ - titleLen) / 2;
			y = yStart_ + yMainTitleShift_ + yTitleShift_ - yTextShift_;
			font_->Print(x, y, columnTitles_[i].c_str());
		}
	}
	//rows
	if (xTitleShift_ != 0) {
		for (int i = 0; i < rowTitles_.size(); i++) {
			float titleLen = font_->Font->Advance(rowTitles_[i].c_str()) * 200 / glutGet(GLUT_WINDOW_WIDTH);
			x = xStart_ +(xTitleShift_ - titleLen) / 2;
			y = yStart_ + yMainTitleShift_ + yTitleShift_ + yShift_*(i+1) - yTextShift_;
			font_->Print(x, y,rowTitles_[i].c_str());
		}
	}

	//data
	if (data_.size() == kRows_) {
		if (data_[0].size() == kColumns_) {

			for (int i = 0; i < kRows_; i++) {
				for (int j = 0; j < kColumns_; j++) {
					text = data_[i][j];
					float titleLen = font_->Font->Advance(text.c_str()) * 200 / glutGet(GLUT_WINDOW_WIDTH);
					x = xStart_ + xTitleShift_ + j * xShift_ + (xShift_ - titleLen) / 2;
					y = yStart_ + yMainTitleShift_ + yTitleShift_ + yShift_ * (i + 1) - yTextShift_;
					font_->Print(x, y, text.c_str());
				}
			}
		}
	}
}
void Table::mousePress(float x, float y) {
	int i = floor((x - xStart_ - xTitleShift_) / xShift_);
	int j = floor((y - yStart_ - yMainTitleShift_ - yTitleShift_) / yShift_);
	callbacks_[i][j];
}