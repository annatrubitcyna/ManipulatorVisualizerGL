#include <math.h>
#include <GL/glut.h>
#include "Manipulator.h"

Eigen::Matrix3d getR(Eigen::Matrix4d H)
{
	return H.block<3, 3>(0, 0);
}

GLfloat* getColor(COLOR i)
{
	//Red, blue, green, yellow, gb, pink, orange,violet, white, g-ye
	GLfloat colors[10][3] = { {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0},{1.0, 1.0, 0.0}, {0.0, 1.0, 1.0},
									{1.0, 0.0, 1.0}, {1.0, 0.7, 0.0}, {1.0, 1.0, 1.0}, {0.7, 1.0, 0.0}, {0.7, 0.0, 1.0} };
	return colors[i];
}

/////////////////////////////////////////////////////
/// Points
/////////////////////////////////////////////////////
Point::Point(double xp = 0.0, double yp = 0.0, double zp = 0.0) 
{
	x = xp;
	y = yp;
	z = zp;
}
Point::Point(Eigen::Vector4d point) 
{
	this->x = point[0];
	this->y = point[1];
	this->z = point[2];

}
Point::Point(Eigen::Vector3d point) 
{
	this->x = point[0];
	this->y = point[1];
	this->z = point[2];
}

double distance(Point p1, Point p2) {
	return pow((pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2)), 0.5);
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

Manipulator::Manipulator(int kAxis, std::vector<double> a, std::vector<double> alpha, std::vector<double> d, Point baseCoords, std::vector<double> angles)
{
	//printf("%i", a.size());
	//try {
	//	if (a.size() < kAxis_ or a.size() > kAxis_) {
	//		printf("!!!");
	//		throw differentArraySizeException();
	//	}
	//}
	//catch (differentArraySizeException e) {
	//	printf("Angles array size should be equal with kAxis");
	//}
	forwardKinematicsMethod_ = DH;
	kAxis_ = kAxis;
	a_ = a;
	alpha_ = alpha;
	d_ = d;
	joints_.push_back(baseCoords);

	coords_=Point(0,0,0);
	R_=Eigen::Matrix3d::Zero();
	H_.push_back(Eigen::Matrix4d::Identity());
	for (int i = 0; i < kAxis_; i++) {
		joints_.push_back(Point(0.0, 0.0, 0.0));
		angles_.push_back(Angle(0.0));
		H_.push_back(Eigen::Matrix4d::Zero());
	}
	setAngles(angles);
}

Manipulator::Manipulator(int kAxis, std::vector<double> a, std::vector<double> alpha, std::vector<double> d, Point coords, Eigen::Matrix3d R)
{
	forwardKinematicsMethod_ = DH;
	kAxis_ = kAxis;
	a_ = a;
	alpha_ = alpha;
	d_ = d;
	setPosition(coords, R);
}

Manipulator::Manipulator(int kAxis, std::vector<Eigen::Vector<double, Eigen::Dynamic>> unitTwists, std::vector<Eigen::Matrix4d> H0, std::vector<double> angles)
{
	forwardKinematicsMethod_ = EXP;
	kAxis_ = kAxis;
	unitTwists_ = unitTwists;
	H0_ = H0;
	setAngles(angles);
}

Manipulator::Manipulator(int kAxis, std::vector<Eigen::Vector<double, Eigen::Dynamic>> unitTwists, std::vector<Eigen::Matrix4d> H0, Point coords, Eigen::Matrix3d R)
{
	forwardKinematicsMethod_ = EXP;
	kAxis_ = kAxis;
	unitTwists_ = unitTwists;
	H0_ = H0;
	setPosition(coords, R);
}

void Manipulator::setAngles(std::vector<double> angles) 
{
	//if (angles.size() < kAxis_ or angles.size() > kAxis_) {
	//	throw("Angles array size should be equal with kAxis");
	//}

	for (int i = 0; i < angles.size(); i++) {
		angles_[i]=Angle(angles[i]);
	}
	if (forwardKinematicsMethod_ == DH){
		forwardKinematicDH();
	}
	if (forwardKinematicsMethod_ == EXP) {
		forwardKinematicEXP();
	}
}

void Manipulator::setPosition(Point coords, Eigen::Matrix3d R)
{
	coords_ = coords;
	R_ = R;
	inverseKinematic();
}

void Manipulator::forwardKinematicDH() 
{
	Eigen::Vector4d Q { {0}, {0}, {0}, {1} };

	for (int i = 0; i < kAxis_; i++) {
		double th = angles_[i].get();
		//transformation matrix from i-1 coordinate system to i coordinate system
		Eigen::Matrix4d Hi  { {cos(th),  -sin(th) * cos(alpha_[i]),  sin(th) * sin(alpha_[i]),   a_[i] * cos(th)},
							  {sin(th),  cos(th) * cos(alpha_[i]),   -cos(th) * sin(alpha_[i]),  a_[i] * sin(th)},
							  {0,        sin(alpha_[i]),             cos(alpha_[i]),             d_[i]          },
							  {0,        0,                          0,                          1              } };
		H_[i+1] = H_[i] * Hi;  //transformation matrix from 0 coordinate system to i coordinate system 

		Eigen::Vector4d joint=H_[i+1]*Q;
		joints_[i+1] = Point(joint);
	}
	R_ = getR(H_[kAxis_-1]);
	coords_ = joints_[kAxis_-1];
}

void Manipulator::forwardKinematicEXP()
{
}

void Manipulator::inverseKinematic()
{
}

float getLinkWidth() 
{
	return 4.0;
}

void Manipulator::drawManipulator()   
{
	glLineWidth(getLinkWidth());
	for (int i = 0; i < kAxis_; i++) 
	{
		if (distance(joints_[i], joints_[i+1]) > 0.0001)
		{
			glColor3f(getColor((COLOR)i)[0], getColor((COLOR)i)[1], getColor((COLOR)i)[2]);
			glBegin(GL_LINES);
			glVertex3f(joints_[i].x, joints_[i].y, joints_[i].z);
			glVertex3f(joints_[i + 1].x, joints_[i + 1].y, joints_[i + 1].z);
			glEnd();
		}
	}
}

float getAxisWidth() 
{
	return 4.0;
}

double getAxisLen() 
{
	return 15;
}

void Manipulator::drawCoordSystem(int i, double axisLen)
{
	Eigen::Matrix3d R = getR(H_[i]);
	glLineWidth(getAxisWidth());

	for (int j = 0; j < 3; j++) { //cycle to draw 3 axis (X, Y,Z);
		Eigen::Vector3d x0(0, 0, 0);
		x0[j] = axisLen;
		Eigen::Vector3d coordSystemAxis = R * x0;
		glColor3f(getColor((COLOR)j)[0], getColor((COLOR)j)[1], getColor((COLOR)j)[2]);
		glBegin(GL_LINES);
		glVertex3f(joints_[i].x, joints_[i].y, joints_[i].z);
		glVertex3f(joints_[i].x + coordSystemAxis[0], joints_[i].y + coordSystemAxis[1], joints_[i].z + coordSystemAxis[2]);
		glEnd();
	}
}

void Manipulator::drawCoordSystems()
{
	for (int i = 0; i < kAxis_+1; i++) {
		drawCoordSystem(i, getAxisLen());
	}
}

void Manipulator::drawOrientation()
{
	drawCoordSystem(kAxis_, getAxisLen());
}


double getAnglesLineWidth() 
{
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

void Manipulator::drawBaseCoordSystem() {
	drawCoordSystem(0, 40.0);
}

