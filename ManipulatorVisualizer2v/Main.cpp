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
	return 800;
}

//==========================================================================================================================|
//																															|
//													MANIPULATOR	DECLARATION													|
//																															|
//==========================================================================================================================|

double l[] = {0, 100, 100, 100, 100}; //array of link lengths
std::vector<double> a = { 0, l[2], 0, 0, 0, 0};
std::vector<double> alpha = { PI / 2, 0, PI / 2, -PI / 2, PI / 2, 0};
std::vector<double> d = { l[1], 0, 0, l[3], 0, l[4]};
std::vector<double> dTh = { 0, 0, PI/2, 0, 0, 0};
Point baseCoords = Point(0.0, 0.0, 0.0);
std::vector<double> angles = { 0, PI/3, 0, 0.0, 0.5, 0.3 };
//std::vector<double> angles = { 0, 0, 0 , 0.0, 0, 0 };
Manipulator manipulator2 = Manipulator(6, a, alpha,d, dTh, baseCoords, angles);

Eigen::Vector<double, 6> uT(0, 0, 1, 0, 0, 0);
std::vector<Eigen::Vector<double, 6>> unitTwists = { uT, uT, uT, uT, uT, uT };
Eigen::Matrix4d H10T0 { {1,0,0, 0},
						{0,0,-1,0},
						{0,1,0,l[1]},
						{0,0,0,1} };
Eigen::Matrix4d H21T0 { {1,0,0, l[2]},
						{0,1,0,0},
						{0,0,1,0},
						{0,0,0,1} };
Eigen::Matrix4d H32T0 { {0,0,1, 0},
						{1,0,0,0},
						{0,1,0,0},
						{0,0,0,1} };
Eigen::Matrix4d H43T0 { {1,0,0, 0},
						{0,0,1,0},
						{0,-1,0,l[3]},
						{0,0,0,1} };
Eigen::Matrix4d H54T0 { {1,0,0, 0},
						{0,0,-1,0},
						{0,1,0,0},
						{0,0,0,1} };

Eigen::Matrix4d H65T0 { {1,0,0, 0},
						{0,1,0,0},
						{0,0,1,l[4]},
						{0,0,0,1} };
std::vector<Eigen::Matrix4d> Hiim1T0 = { H10T0, H21T0, H32T0, H43T0, H54T0, H65T0 };
Manipulator manipulator = Manipulator(6, unitTwists, Hiim1T0, baseCoords, angles);


//==========================================================================================================================|
//																															|
//														CAMERA																|
//																															|
//==========================================================================================================================|

class Camera
{
private:
	float xAngle_; //angle that camera rotate about x axis
	float yAngle_;
	float zAngle_;
	float x_;
	float y_;
	float z_;
public:
	Camera() : xAngle_(0), yAngle_(0), zAngle_(0), x_(getScreneToCentreDistance()), y_(0), z_(0) {}
	float getXAngle() { return xAngle_; }
	float getYAngle() { return yAngle_; }
	float getZAngle() { return zAngle_; }
	float getX() { return x_; }
	float getY() { return y_; }
	float getZ() { return z_; }
	void setPosition(float x, float y, float z, float xAngle, float yAngle, float zAngle) {
		x_ = x;
		y_ = y;
		z_ = z;
		xAngle_ = xAngle;
		yAngle_ = yAngle;
		zAngle_ = zAngle;
	}
	void translate(float dx, float dy, float dz)
	{
		x_ += dx;
		y_ += dy;
		z_ += dz;
	}
	void rotate(float dx, float dy, float dz)
	{
		xAngle_ += dx;
		yAngle_ += dy;
		zAngle_ += dz;
	}
};

Camera camera;

//==========================================================================================================================|
//																															|
//														KEYS																|
//																															|
//==========================================================================================================================|


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
double getAngularSpeed() 
{
	return toRadians(5);
}
void keyboard(unsigned char key, int x, int y) 
{
	float cameraX0 = getScreneToCentreDistance();
	switch (key) {
		//standard Camera positions
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
		//angles changing
		case 'q':
			manipulator.changeAngle(1, getAngularSpeed());
			break;
		case 'w':
			manipulator.changeAngle(1, -getAngularSpeed());
			break;
		case 'e':
			manipulator.changeAngle(2, getAngularSpeed());
			break;
		case 'r':
			manipulator.changeAngle(2, -getAngularSpeed());
			break;
		case 't':
			manipulator.changeAngle(3, getAngularSpeed());
			break;
		case 'y':
			manipulator.changeAngle(3, -getAngularSpeed());
			break;
		case 'u':
			manipulator.changeAngle(4, getAngularSpeed());
			break;
		case 'i':
			manipulator.changeAngle(4, -getAngularSpeed());
			break;
		case 'a':
			manipulator.changeAngle(5, getAngularSpeed());
			break;
		case 's':
			manipulator.changeAngle(5, -getAngularSpeed());
			break;
		case 'd':
			manipulator.changeAngle(6, getAngularSpeed());
			break;
		case 'f':
			manipulator.changeAngle(6, -getAngularSpeed());
			break;
		case 'g':
			manipulator.changeAngle(7, getAngularSpeed());
			break;
		case 'h':
			manipulator.changeAngle(7, -getAngularSpeed());
			break;
	}

	if (key == 'o') {
	}
	else if (key == 'o') {
	}
};


//==========================================================================================================================|
//																															|
//														MOUSE																|
//																															|
//==========================================================================================================================|

class Mouse
{
private:
	float x_;
	float y_;
	float z_;
	float zPr_;
public:
	Mouse() : x_(0), y_(0), z_(0), zPr_(0){}
	float getX() { return x_; }
	float getY() { return y_; }
	float getZ() { return z_; }
	float getZPr() { return zPr_; }
	void setPosition(float x, float y, float z) {
		x_ = x;
		y_ = y;
		z_ = z;
	}
	void setZpr(float zPr) {
		zPr_ = zPr;
	}
};

Mouse mouse;

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
		lastMx = x - lastMx;
		lastMy = y - lastMy;
		if (lastMx != -1) {
			camera.rotate(0, lastMy / getYRotSpeed(), (lastMx) / getZRotSpeed());
		}
	}
	lastMx = x;
	lastMy = y;
}

void mousePassiveMove(int x, int y)
{
	lastMx = x;
	lastMy = y;
}

float getTrSpeed()
{
	return 10;
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

void mouseWheelToMousePoint(int button, int dir, int x, int y) {
	if (dir > 0) {
		mouse.setPosition(x, y, mouse.getZ() + getTrSpeed());
	}
	else {
		mouse.setPosition(x, y, mouse.getZ() - getTrSpeed());
	}
}

//==========================================================================================================================|
//																															|
//														GLUT function														|
//																															|
//==========================================================================================================================|

// Application-specific initialization: Set up global lighting parameters
// and create display lists.
void init()
{
	glDisable(GL_LIGHTING);
}


void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glClearColor(0.29, 0.29, 0.49, 1);
	glLoadIdentity(); //clean matrix

	//mouse wheel
	gluLookAt(camera.getX(), camera.getY(), camera.getZ(),      //camera point
		0.0, 0.0, 0.0,											//scene center
		0.0, 0.0, 1.0);											//z scene vector


	glRotatef(camera.getXAngle(), 1.0, 0.0, 0.0);
	glRotatef(camera.getYAngle(), 0.0, 1.0, 0.0);
	glRotatef(camera.getZAngle(), 0.0, 0.0, 1.0);

	/*glTranslatef(0, mouse.getX()-getWindowWidth()/2, mouse.getY()-getWindowHeight()/2);
	glTranslatef(mouse.getZ(), 0, 0);
	glTranslatef(0, -(mouse.getX() - getWindowWidth() / 2), -(mouse.getY() - getWindowHeight() / 2));*/

	manipulator2.drawBaseCoordSystem();
	manipulator2.drawManipulator();
	manipulator2.drawCoordSystems();
	manipulator2.drawAngles();


	manipulator.drawBaseCoordSystem();
	manipulator.drawManipulator();
	manipulator.drawCoordSystems();
	manipulator.drawAngles();
	

	glFlush(); //clean buffers
	glutSwapBuffers(); //&
}

// On reshape, constructs a camera that perfectly fits the window.
void reshape(GLint w, GLint h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, GLfloat(h) / GLfloat(h), 1.0, getScreneToCentreDistance()*3);
	glMatrixMode(GL_MODELVIEW);
}

// Requests to draw the next frame.
void timer(int v)
{
	glutPostRedisplay();
	glutTimerFunc(1000 / FPS, timer, v); //frames per second
}

//==========================================================================================================================|
//																															|
//														MAIN																|
//																															|
//==========================================================================================================================|


// Initializes GLUT and enters the main loop.
// main arguments from console, argc= numbers, argv = array of arguments
int main(int argc, char** argv) {
	glutInit(&argc, argv);  //initialize the GLUT library
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH); //parameters rgb- colors (0-1), double need to animation, depth-&
	glutInitWindowSize(getWindowWidth(), getWindowHeight());
	glutInitWindowPosition((glutGet(GLUT_SCREEN_WIDTH) - getWindowWidth()) / 2,     //center
		(glutGet(GLUT_SCREEN_HEIGHT) - getWindowHeight()) / 2); //left top corner offset
	glutCreateWindow("Manipulator Visualizer");
	glutDisplayFunc(display); //setup
	glutReshapeFunc(reshape); //changing window size

	glutSpecialFunc(special); //special keyboard callback
	glutKeyboardFunc(keyboard); //letters keyboard callback

	glutMouseFunc(mouseButton);
	glutMotionFunc(mouseMove);
	glutPassiveMotionFunc(mousePassiveMove);
	glutMouseWheelFunc(mouseWheel);

	glutTimerFunc(100, timer, 0); //timer function every 100 ms- loop
	init(); //light
	glutMainLoop(); //enters the GLUT event processing loop
}