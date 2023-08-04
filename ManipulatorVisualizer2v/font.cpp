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
}
void CFont::Print(float x, float y, const wchar_t* text)
{
	glPushMatrix(); //to print more than one text
	//glLoadIdentity();
	glDisable(GL_TEXTURE_2D);
	// move to coordinates
	glTranslatef(x, y, -1);
	glRasterPos2f(-1, 0.5);

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