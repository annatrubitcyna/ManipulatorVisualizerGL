// This application shows balls bouncing on a checkerboard, with no respect
// for the laws of Newtonian Mechanics.  There's a little spotlight to make
// the animation interesting, and arrow keys move the camera for even more
// fun.


#include <cmath>
#include <stdio.h>



#ifdef __APPLE_CC__
#include <GLUT/glut.h>
#else
#include <GL/freeglut.h>
#endif

//#include <freeglut_ext.h>

#include "Manipulator.h"

// This is the number of frames per second to render.
static const int FPS = 60;

// Colors
GLfloat WHITE[] = { 1, 1, 1 };
GLfloat RED[] = { 1, 0, 0 };
GLfloat GREEN[] = { 0, 1, 0 };
GLfloat MAGENTA[] = { 1, 0, 1 };

int getWindowWidth() 
{
	return 1600;
}

int getWindowHeight()
{
	return 900;
}

int getScreneToCentreDistance()
{
	return 400;
}


/////////////////////////////////////////////

//Manipulator

////////////////////////////////////////////
Manipulator manipulator;


//////////////////////////////////////////////

//Camera

/////////////////////////////////////////////
class Camera
{
private:
	double xAngle_; //angle that camera rotate about x axis
	double yAngle_;
	double zAngle_;
	double x_;
	double y_;
	double z_;
public:
	Camera() : xAngle_(0), yAngle_(0), zAngle_(0), x_(getScreneToCentreDistance()), y_(0), z_(0) {}
	double getXAngle() { return xAngle_; }
	double getYAngle() { return yAngle_; }
	double getZAngle() { return zAngle_; }
	double getX() { return x_; }
	double getY() { return y_; }
	double getZ() { return z_; }
	void setPosition(double x, double y, double z, double xAngle, double yAngle, double zAngle) {
		x_ = x;
		y_ = y;
		z_ = z;
		xAngle_ = xAngle;
		yAngle_ = yAngle;
		zAngle_ = zAngle;
	}
	void translate(double dx, double dy, double dz)
	{
		x_ += dx;
		y_ += dy;
		z_ += dz;
	}
	void rotate(double dx, double dy, double dz)
	{
		xAngle_ += dx;
		yAngle_ += dy;
		zAngle_ += dz;
	}
};

Camera camera;

////////////////////////////////////////////////
// 
// Keys
// 
/////////////////////////////////////////////////


// Moves the camera according to the key pressed, then ask to refresh the
// display.
void special(int key, int, int)
{
	switch (key) {
		//case GLUT_KEY_LEFT: camera.moveLeft(); break;
		//case GLUT_KEY_RIGHT: camera.moveRight(); break;
		//case GLUT_KEY_UP: camera.moveUp(); break;
		//case GLUT_KEY_DOWN: camera.moveDown(); break;
	}
	glutPostRedisplay();
};

void keyboard(unsigned char key, int x, int y) {
	double cameraX0 = getScreneToCentreDistance();
	printf("%d", camera.getZ());
	switch (key) {
		case 'o':
			camera.setPosition(cameraX0, 0, 0, 0, 0, 0);
			break;
		case 'p':
			if (camera.getZAngle() != 0.0) {
				camera.setPosition(cameraX0, 0, 0, 0, 0, 0);
			}
			else {
				camera.setPosition(cameraX0, 0, 0, 0, 0, 180);
			}
			break;
		case 'k': 
			if (camera.getZAngle() != -90.0) {
				camera.setPosition(cameraX0, 0, 0, 0, 0, -90);
			}
			else {
				camera.setPosition(cameraX0, 0, 0, 0, 0, 90);
			}
			break;
		case 'l':
			if (camera.getYAngle()!= -90.0) {
				camera.setPosition(cameraX0, 0, 0, 0, -90, 0);
			}
			else {
				camera.setPosition(cameraX0, 0, 0, 0, 90, 0);
			}
			break;
	}

	if (key == 'o') {
	}
	else if (key == 'o') {
	}
};


///////////////////////////////////////////////////
//
//Mouse
//
//////////////////////////////////////////////////
bool isCameraRotate = false;

void mouseButton(int button, int state, int x, int y)
{
	if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN) {
		isCameraRotate = true;
	}
	if (button == GLUT_MIDDLE_BUTTON && state == GLUT_UP) {
		isCameraRotate = false;
	}
}

int lastMx = 0; //last mouse point
int lastMy = 0;
float getYRotSpeed() //Speed of rotation of model around Y axis
{
	return 3;
}
float getZRotSpeed()
{
	return 6;
}

void mouseMove(int x, int y)
{
	if (isCameraRotate) {
		lastMx = (int)x - lastMx;
		lastMy = (int)y - lastMy;
		if (lastMx != -1) {
			camera.rotate(0, lastMy / getYRotSpeed(), (lastMx) / getZRotSpeed());
		}
	}
	lastMx = (float)x;
	lastMy = (float)y;
}

void mousePassiveMove(int x, int y)
{
	lastMx = (float)x;
	lastMy = (float)y;
}

float getTrSpeed()
{
	return 0.5;
}
void mouseWheel(int button, int dir, int x, int y)
{
	if (dir > 0){
		camera.translate(getTrSpeed(), 0, 0);
	}
	else{
		camera.translate(-getTrSpeed(), 0, 0);
	}
}

///////////////////////////////////////////////////
//
//GLUT functions
//
///////////////////////////////////////////////////


// Application-specific initialization: Set up global lighting parameters
// and create display lists.
void init()
{
	//glEnable(GL_DEPTH_TEST);
	//glLightfv(GL_LIGHT0, GL_DIFFUSE, WHITE);
	//glLightfv(GL_LIGHT0, GL_SPECULAR, WHITE);
	//glMaterialfv(GL_FRONT, GL_SPECULAR, WHITE);
	//glMaterialf(GL_FRONT, GL_SHININESS, 30);
	//glEnable(GL_LIGHTING);
	//glEnable(GL_LIGHT0);
	glDisable(GL_LIGHTING);
}


void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.29, 0.29, 0.49, 1);
	glLoadIdentity(); //clean matrix

	//coordinates transformation about Camera
	gluLookAt(camera.getX(), camera.getY(), camera.getZ(), //camera point
		0.0, 0.0, 0.0,       //scene centre
		0.0, 0.0, 1.0);      //z scene vector
	
	//smoothRX(targetCamRX); //сглаживание
	//smoothRY(targetCamRY);
	//smoothRZ(targetCamRZ);

	glRotatef(camera.getXAngle(), 1.0, 0.0, 0.0);
	glRotatef(camera.getYAngle(), 0.0, 1.0, 0.0);
	glRotatef(camera.getZAngle(), 0.0, 0.0, 1.0);

	//gluPerspective(40.0, GLfloat(getWindowWidth()) / GLfloat(getWindowHeight()), 1.0, 150.0);

	manipulator.drawManipulator();

	glFlush(); //clean buffers
	glutSwapBuffers(); //&
}

//????????????????????????????
// On reshape, constructs a camera that perfectly fits the window.
void reshape(GLint w, GLint h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, GLfloat(w) / GLfloat(h), 1.0, 1000.0);
	glMatrixMode(GL_MODELVIEW);
}

// Requests to draw the next frame.
void timer(int v)
{
	glutPostRedisplay();
	glutTimerFunc(1000 / FPS, timer, v); //frames per secon
}

///////////////////////////////////////////////////
//
//Main
//
//////////////////////////////////////////////////


// Initializes GLUT and enters the main loop.
// main arguments from console, argc= numbers, argv = array of arguments
int main(int argc, char** argv) {
	glutInit(&argc, argv);  //initialize the GLUT library
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH); //parameters rgb- colors (0-1), double neeed to animation, depth-&
	glutInitWindowSize(getWindowWidth(), getWindowHeight());
	glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH) - getWindowWidth()) / 2,     //centre
		(glutGet(GLUT_SCREEN_HEIGHT) - getWindowHeight()) / 2); //left top corner offset
	glutCreateWindow("Manipulator Visualizer");
	glutDisplayFunc(display); //setup
	glutReshapeFunc(reshape); //changing window size
	glutSpecialFunc(special); //special keyboard callback
	glutKeyboardFunc(keyboard); //letters keyboard callback

	//may be need to keys
	/*glutIgnoreKeyRepeat(1);

	glutSpecialUpFunc(releaseKey);*/

	//glClearDepth(200.0);
	//glDisable(GL_DEPTH_TEST);
	//glDepthFunc(GL_LEQUAL);*/

	//glDepthFunc(GL_ALWAYS);

	glutMouseFunc(mouseButton);
	glutMotionFunc(mouseMove);
	glutPassiveMotionFunc(mousePassiveMove);
	glutMouseWheelFunc(mouseWheel);
	glutTimerFunc(100, timer, 0); //timer function every 100 ms- loop
	init(); //light
	glutMainLoop(); //enters the GLUT event processing loop
}