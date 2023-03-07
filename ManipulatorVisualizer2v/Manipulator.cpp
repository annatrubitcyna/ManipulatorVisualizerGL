#include <math.h>
#include <GL/glut.h>
#include "Manipulator.h"

/////////////////////////////////////////////////////
/// Points
/////////////////////////////////////////////////////

double distance(Point p1, Point p2) {
	return pow((pow((p1.x - p2.x), 2.0) + pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2)), 0.5);
}

/////////////////////////////////////////////////////
/// Angle
/////////////////////////////////////////////////////

Angle::Angle(double value)
{
	double a = fmod(value, 2 * PI);
	if (a > PI) {
		a -= 2 * PI;
	}
	if (a <= -PI) {
		a += 2 * PI;
	}
	value_ = a;
}
double toAngleFormat(double a) //function to keep all angles in one range (-PI,PI]
{
	a = fmod(a, 2 * PI);
	if (a > PI) {
		a -= 2 * PI;
	}
	if (a <= -PI) {
		a += 2 * PI;
	}
	return a;
}
//&&&
double Angle::toAngleFormat(Angle angle) //function to keep all angles in one range (-PI,PI]
{
	double a = angle.get();
	a = fmod(a, 2 * PI);
	if (a > PI) {
		a -= 2 * PI;
	}
	if (a <= -PI) {
		a += 2 * PI;
	}
	return a;
}
double Angle::get() { return value_; }
double Angle::getInDeg() { return value_ * 180.0 / PI; }
void Angle::set(double angle) { value_ = toAngleFormat(angle); } //angles in radians because cos, sin and other math works with radians
void Angle::setInDeg(double angle) { set(angle * PI / 180.0); }

Angle operator+(Angle a1, Angle a2)
{
	return Angle(a1.get() + a2.get());
};

Angle operator-(Angle a1, Angle a2)
{
	return Angle(a1.get() - a2.get());
};

/////////////////////////////////////////////////////
/// Manipulator
/////////////////////////////////////////////////////

Manipulator::Manipulator(int kAxis, std::vector<double> a, std::vector<double> alpha, std::vector<double> d)
{
	forwardKinematicsMethod = DH;
	kAxis_ = kAxis;
	a_ = a;
	alpha_ = alpha;
	d_ = d;
}

Manipulator::Manipulator(int kAxis, std::vector<Eigen::Vector<double, Eigen::Dynamic>> unitTwists, std::vector<Eigen::Matrix4d> H0)
{
	forwardKinematicsMethod = EXP;
	kAxis_ = kAxis;
	unitTwists_ = unitTwists;
	H0_ = H0;
}

void Manipulator::forwardKinematicDH() {
	 
	for (int i = 0; i < kAxis_; i++) {
		double th = angles_[i].get();
		//transformation matrix from i-1 coordinate system to i coordinate system
		Eigen::Matrix4d Hi  { {cos(th),  -sin(th) * cos(alpha_[i]),  sin(th) * sin(alpha_[i]),   a_[i] * cos(th)},
							  {sin(th),  cos(th) * cos(alpha_[i]),   -cos(th) * sin(alpha_[i]),  a_[i] * sin(th)},
							  {0,        sin(alpha_[i]),             cos(alpha_[i]),             d_[i]          },
							  {0,        0,                          0,                          1              } };
		if (i != 0) {
			H_[i] = Hi * H_[i - 1];  //transformation matrix from 0 coordinate system to i coordinate system
		}
	}



}

float getLinkWidth() {
	return 4.0;
}

enum COLOR {
	RED,
	BLUE,
	GREEN,
	YELLOW,
	GB,
	PINK,
	ORANGE,
	VIOLET,
	WHITE,
	GYE
};

GLfloat* getColor(COLOR i)
{
	//Red, blue, green, yellow, gb, pink, orange,violet, white, g-ye
	GLfloat colors[10][3] = { {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0},{1.0, 1.0, 0.0}, {0.0, 1.0, 1.0},
								    {1.0, 0.0, 1.0}, {1.0, 0.7, 0.0}, {1.0, 1.0, 1.0}, {0.7, 1.0, 0.0}, {0.7, 0.0, 1.0} };
	return colors[i];
}

void Manipulator::drawManipulator()   
{
	glLineWidth(getLinkWidth());
	for (int i = 0; i < kAxis_; i++) 
	{
		if (distance(joints_[i], joints_[i + 1]) > 0.0001) 
		{
			glColor3f(getColor((COLOR)i)[0], getColor((COLOR)i)[1], getColor((COLOR)i)[2]);
			glBegin(GL_LINES);
			glVertex3f(joints_[i].x, joints_[i].y, joints_[i].z);
			glVertex3f(joints_[i+1].x, joints_[i+1].y, joints_[i+1].z);
			glEnd();
		}
	}
}

Eigen::Matrix3d getR(Eigen::Matrix4d H) {
	return H.block<3,3>(0,0); 
}

float getAxisWidth() {
	return 4.0;
}

double getAxisLen() {
	return 5;
}

void Manipulator::drawCoordSystem(int i)
{
	Eigen::Matrix3d R = getR(H_[i]);
	glLineWidth(getAxisWidth());

	for (int j = 0; j < 3; j++) { //cycle to draw 3 axis (X, Y,Z);
		Eigen::Vector3d x0(0, 0, 0);
		x0[i] = getAxisLen();
		Eigen::Vector3d csAxis = R * x0;
		glColor3f(getColor((COLOR)j)[0], getColor((COLOR)j)[1], getColor((COLOR)j)[2]);
		glBegin(GL_LINES);
		glVertex3f(joints_[i].x, joints_[i].y, joints_[i].z);
		glVertex3f(joints_[i].x + csAxis[0], joints_[i].y + csAxis[1], joints_[i].z + csAxis[2]);
		glEnd();
	}
}

void Manipulator::drawCoordSystems()
{
	for (int i = 0; i < kAxis_; i++) {
		drawCoordSystem(i);
	}
}

void Manipulator::drawOrientation()
{
	drawCoordSystem(kAxis_-1);
}


double getAnglesLineWidth() {
	return 2;
}

void Manipulator::drawAngles()  //need to explain which this angles are 
{
	glColor3f(getColor(WHITE)[0], getColor(WHITE)[1], getColor(WHITE)[2]);
	glLineWidth(getAnglesLineWidth());

	//pushMatrix();
	//for (int i = 0; i < 6; i++) {
	//	if (angles[i] > 0)  arc(0, 0, 100, 100, 0, angles[i]);
	//	else arc(0, 0, 100, 100, angles[i], 0);
	//	//переходим в следующую систему координат, чтобы там нарисовать угол
	//	if (i == 2) rotateZ(angles[i] + PI / 2);
	//	else rotateZ(angles[i]);
	//	translate(0, 0, d[i + 1]);
	//	translate(a[i + 1], 0, 0);
	//	rotateX(alpha[i + 1]);

	//}
	//popMatrix();
}

