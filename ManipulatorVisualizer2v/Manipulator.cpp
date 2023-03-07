#include <math.h>
#include <GL/glut.h>
#include "Manipulator.h"

double toRadians(double angle)
{
	return angle * PI / 180;
}

double toDegrees(double angle)
{
	return angle * 180 / PI;
}

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

//==========================================================================================================================|
//																															|
//														POINTS																|
//																															|
//==========================================================================================================================|

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

//==========================================================================================================================|
//																															|
//														ANGLE																|
//																															|
//==========================================================================================================================|

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

//==========================================================================================================================|
//																															|
//														MNIPULATOR																|
//																															|
//==========================================================================================================================|

void Manipulator::initializeVectorsAsNull() {
	coords_ = Point(0, 0, 0);
	R_ = Eigen::Matrix3d::Zero();
	H_.push_back(Eigen::Matrix4d::Identity());
	for (int i = 0; i < kAxis_; i++) {
		joints_.push_back(Point(0.0, 0.0, 0.0));
		angles_.push_back(Angle(0.0));
		H_.push_back(Eigen::Matrix4d::Zero());
	}
}
Manipulator::Manipulator(int kAxis, std::vector<double> a, std::vector<double> alpha, std::vector<double> d, Point baseCoords, std::vector<double> angles)
{
	////printf("%i", a.size());
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

	initializeVectorsAsNull();
	setAngles(angles);
}

Manipulator::Manipulator(int kAxis, std::vector<double> a, std::vector<double> alpha, std::vector<double> d, Point baseCoords, Point coords, Eigen::Matrix3d R)
{
	forwardKinematicsMethod_ = DH;
	kAxis_ = kAxis;
	a_ = a;
	alpha_ = alpha;
	d_ = d;
	joints_.push_back(baseCoords);
	initializeVectorsAsNull();
	setPosition(coords, R);
}

Manipulator::Manipulator(int kAxis, std::vector<Eigen::Vector<double, 6>> unitTwists, std::vector<Eigen::Matrix4d> Hiim1T0, Point baseCoords, std::vector<double> angles)
{
	forwardKinematicsMethod_ = EXP;
	kAxis_ = kAxis;
	unitTwists_ = unitTwists;
	Hiim1T0_ = Hiim1T0;
	joints_.push_back(baseCoords);
	initializeVectorsAsNull();
	setAngles(angles);
}

Manipulator::Manipulator(int kAxis, std::vector<Eigen::Vector<double, 6>> unitTwists, std::vector<Eigen::Matrix4d> Hiim1T0, Point baseCoords, Point coords, Eigen::Matrix3d R)
{
	forwardKinematicsMethod_ = EXP;
	kAxis_ = kAxis;
	unitTwists_ = unitTwists;
	Hiim1T0_ = Hiim1T0;
	joints_.push_back(baseCoords);
	initializeVectorsAsNull();
	setPosition(coords, R);
}

//==========================================================================================================================|
//																															|
//													TRANSFER FUNCTIONS														|
//																															|
//==========================================================================================================================|

void Manipulator::changeAngle(int i, double dAngle) 
{
	if (i <= kAxis_) {
		angles_[i-1] = Angle(angles_[i-1] + dAngle);
		if (forwardKinematicsMethod_ == DH) {
			forwardKinematicDH();
		}
		else if (forwardKinematicsMethod_ == EXP) {
			forwardKinematicEXP();
		}
	}
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
	else if (forwardKinematicsMethod_ == EXP) {
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

//==========================================================================================================================|
//													Forward Kinematic EXP														|
//==========================================================================================================================|

Eigen::Matrix3d skewSymmetricMatrixFromVector(Eigen::Vector3d x)
{
	Eigen::Matrix3d skewSymmetricMatrix	{{  0,   -x[2],  x[1] },
										 { x[2],   0,   -x[0] },
										 {-x[1],  x[0],   0   }};
	return skewSymmetricMatrix;
}

//int the result don't need
//Eigen::Matrix4d twistMatrixFromVector(Eigen::Vector<double, 6 > twist)
//{
//	Eigen::Vector3d w = twist.segment(0, 3);
//	Eigen::Vector3d v = twist.segment(3, 3);
//	Eigen::Matrix4d twistMatrix = Eigen::Matrix4d::Zero();
//	twistMatrix.block(0, 0, 3, 3) = skewSymmetricMatrixFromVector(w);
//	twistMatrix.block(0, 3, 3, 1) = v;
//	return twistMatrix;
//}

Eigen::Matrix3d getApproxExpWt(Eigen::Vector3d w)
{
	Eigen::Matrix3d wt = skewSymmetricMatrixFromVector(w);
	double wNorm = w.norm();
	if (round(wNorm*10000) != 0) {
		wt = wt/ wNorm;
	}
	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d ExpWt = I + wt * sin(wNorm) + wt * wt * (1 - cos(wNorm));
	return ExpWt;
}

Eigen::Matrix4d getApproxExpTwist(Eigen::Vector<double, 6> twist)
{
	Eigen::Vector3d w = twist.segment(0, 3);
	Eigen::Vector3d v = twist.segment(3, 3);
	Eigen::Matrix3d ExpWt = getApproxExpWt(w);
	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
	Eigen::Vector3d ExpTwist21 = (I - ExpWt) * (w.cross(v)) + w.transpose() * v * w;
	if ((round(w.norm()*10000)) != 0)
		ExpTwist21 = ExpTwist21 / pow(w.norm(), 2);
	Eigen::Matrix4d ExpTwist = Eigen::Matrix4d::Zero();
	ExpTwist.block(0, 0, 3, 3) = ExpWt;
	ExpTwist.block(0, 3, 3, 1) = ExpTwist21;
	ExpTwist(3, 3) = 1;
	return ExpTwist;
}
//
void Manipulator::forwardKinematicEXP()
{
	Eigen::Vector4d Q{ {0}, {0}, {0}, {1} };

	for (int i = 0; i < kAxis_; i++) {
		Eigen::Matrix4d ExpTwist = getApproxExpTwist(angles_[i].get() * unitTwists_[i]);
		H_[i + 1] = H_[i] * ExpTwist * Hiim1T0_[i];

		Eigen::Vector4d joint = H_[i + 1] * Q;
		joints_[i + 1] = Point(joint);
	}
	R_ = getR(H_[kAxis_ - 1]);
	coords_ = joints_[kAxis_ - 1];
}

void Manipulator::inverseKinematic()
{
}

//==========================================================================================================================|
//																															|
//														DRAWING																|
//																															|
//==========================================================================================================================|

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

