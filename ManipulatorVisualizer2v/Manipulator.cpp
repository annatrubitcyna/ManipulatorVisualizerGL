#pragma comment(lib, "ftgl_dynamic_MT.lib")

#include <iostream>
#include <math.h>

#include <ftgl/FTGLTextureFont.h>
//#include <GL/glut.h>
#include "Manipulator.h"

#include "font.h"
#include <wchar.h>

#include <fstream>

const char* ttf = "C:/Windows/Fonts/arial.ttf";
const char* ttfM = "C:/Windows/Fonts/cambria.ttc";
const char* ttf2 = "C:/Users/Ann/Downloads/FTGL/arial.ttf";
CFont* Font = new CFont(ttf, 24, 32);
CFont* FontM = new CFont(ttfM, 24, 32);

std::string sep = "\n----------------------------------------\n";
Eigen::IOFormat OctaveFmt(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
//std::cout << expWt << "\n";
double sq(double a)
{
	return pow(a, 2);
}
double toRadians(double angle)
{
	return angle * PI / 180;
}

double toDegrees(double angle)
{
	return angle * 180 / PI;
}

//void drawLine(float x1, float y1, float x2, float y2) 
//{
//	glBegin(GL_LINES);
//	glVertex2f(x1, y1);
//	glVertex2f(x2, y2);
//	glEnd();
//}

Eigen::Matrix3d getR(Eigen::Matrix4d H)
{
	return H.block<3, 3>(0, 0);
}

GLfloat* getColor(COLOR i)
{
	//Red, green, blue, yellow, green-blue, pink, orange,violet, white, green-yellow
	GLfloat colors[10][3] = { {1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}, {1.0, 1.0, 0.0}, {0.0, 1.0, 1.0},
									{1.0, 0.0, 1.0}, {1.0, 0.7, 0.0}, {0.7, 1.0, 0.0}, {1.0, 1.0, 1.0}, {0.7, 0.0, 1.0} };
	return colors[i];
}

//std::wstring roundN(double a, int n) 
//{
//	std::wstring b = std::to_wstring(round(a * pow(10, n)) / pow(10, n)).substr(0, n+);
//	return b;
//}

//==========================================================================================================================|
//																															|
//														_POINTS																|
//																															|
//==========================================================================================================================|

Point::Point(double xp , double yp , double zp ) 
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
	return pow((sq(p1.x - p2.x) + sq(p1.y - p2.y) + sq(p1.z - p2.z)), 0.5);
}

Point operator+(Point p1, Point p2)
{
	return Point(p1.x+p2.x, p1.y+p2.y, p1.z+p2.z);
};

Point operator-(Point p1, Point p2)
{
	return Point(p1.x - p2.x, p1.y - p2.y, p1.z - p2.z);
};

Eigen::Vector3d Point::toVector()
{
	return Eigen::Vector3d{ x, y, z };
}

//==========================================================================================================================|
//																															|
//														_ANGLE																|
//																															|
//==========================================================================================================================|

Angle::Angle(double value) //(-PI, PI]
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
//													_MANIPULATOR															|
//																															|
//==========================================================================================================================|

void Manipulator::initializeVectorsAsNull() {
	coords_ = Point(0, 0, 0);
	R_ = Eigen::Matrix3d::Zero();
	EulerAngles_ = findEulerAngles(R_);
	H_.push_back(Eigen::Matrix4d::Identity());
	joints_.push_back(Point(0.0, 0.0, 0.0));
	for (int i = 0; i < kAxis_; i++) {
		joints_.push_back(Point(0.0, 0.0, 0.0));
		angles_.push_back(Angle(0.0));
		H_.push_back(Eigen::Matrix4d::Zero());
		if (i < 3)	isChangedByMouse_.push_back(0);
	}
	isCubePressed = false;
}

Manipulator::Manipulator(int kAxis, std::vector<double> a, std::vector<double> alpha, std::vector<double> d, std::vector<double> dTh)
{
	forwardKinematicsMethod_ = DH;
	kAxis_ = kAxis;
	a_ = a;
	alpha_ = alpha;
	d_ = d;
	dTh_ = dTh;
	initializeVectorsAsNull();
	J_.resize(kAxis_, kAxis_);
	setAngles(angles_);
	kJoints_ = 0;
	for (int i = 0; i < kAxis_; i++) {
		if (distance(joints_[i], joints_[i + 1]) > 0.0001) kJoints_ += 1;
	}
	error_ = OK;
}

Manipulator::Manipulator(int kAxis, std::vector<Eigen::Vector<double, 6>> unitTwists, std::vector<Eigen::Matrix4d> Hiim1T0)
{
	forwardKinematicsMethod_ = EXP;
	kAxis_ = kAxis;
	unitTwists_ = unitTwists;
	Hiim1T0_ = Hiim1T0;
	initializeVectorsAsNull();
	geomJ_.resize(kAxis_, kAxis_);
	setAngles(angles_);
	kJoints_ = 0;
	for (int i = 0; i < kAxis_; i++) {
		if (distance(joints_[i], joints_[i + 1]) > 0.0001) kJoints_ += 1;
	}
	error_ = OK;
}

//==========================================================================================================================|
//																															|
//													_TRANSFER FUNCTIONS														|
//																															|
//==========================================================================================================================|

void Manipulator::changeAngle(int i, double dAngle) 
{
	if (i <= kAxis_ && i>0) {
		angles_[i-1] = Angle(angles_[i-1] + dAngle);
		forwardKinematics();
	}
}

void Manipulator::setAngles(std::vector<Angle> angles)
{
	if (forwardKinematicsMethod_ == DH) {
		for (int i = 0; i < kAxis_; i++) {
			angles_[i] = Angle(angles[i].get() + dTh_[i]);
		}
	}
	else {
		for (int i = 0; i < kAxis_; i++) {
			angles_[i] = angles[i];
		}
	}
	forwardKinematics();
}

void Manipulator::setAngles(std::vector<double> angles) 
{
	if (forwardKinematicsMethod_ == DH) {
		for (int i = 0; i < kAxis_; i++) {
			angles_[i] = Angle(angles[i]+dTh_[i]);
		}
	}
	else {
		for (int i = 0; i < angles.size(); i++) {
			angles_[i] = Angle(angles[i]);
		}
	}
	forwardKinematics();
}

void Manipulator::changePosition(Point dCoords)
{
	coords_ = coords_ + dCoords;
	inverseKinematics();
	forwardKinematics();
}

void Manipulator::setPosition(Point coords, Eigen::Matrix3d R)
{
	coords_ = coords;
	R_ = R;
	inverseKinematics();
	forwardKinematics();
}

void Manipulator::setPosition(Point coords)
{
	setPosition(coords, Eigen::Matrix3d::Identity());
}

Eigen::Matrix3d getStandardRotMatrixX(Angle angle) {
	float an = angle.get();
	Eigen::Matrix3d R { {1, 0, 0},
						{0, cos(an), -sin(an)},
						{0, sin(an), cos(an)} };
	return R;
}
Eigen::Matrix3d getStandardRotMatrixY(Angle angle) {
	float an = angle.get();
	Eigen::Matrix3d R { {cos(an), 0, sin(an)},
						{0, 1, 0},
						{-sin(an), 0, cos(an)} };
	return R;
}
Eigen::Matrix3d getStandardRotMatrixZ(Angle angle) {
	float an = angle.get();
	Eigen::Matrix3d R { {cos(an),-sin(an),0},
						{sin(an), cos(an), 0},
						{0, 0, 1} };
	return R;
}

void Manipulator::changeOrientation(int axis, float dAngle) {
	Eigen::Matrix3d R=R_;
	if (axis == 1) { //x-axis
		R = R_ * getStandardRotMatrixX(Angle(dAngle));
	}
	else if (axis == 2) { //y-axis
		R= R_ * getStandardRotMatrixY(Angle(dAngle));
	}
	else if (axis == 3) { //z-axis
		R = R_ * getStandardRotMatrixZ(Angle(dAngle));
	}
	setOrientation(R);
}


void Manipulator::setOrientation(Eigen::Matrix3d R) {
	setPosition(coords_, R);
}

//==========================================================================================================================|
//													_Forward Kinematic														|
//==========================================================================================================================|

void Manipulator::forwardKinematics()
{
	if (forwardKinematicsMethod_ == DH) {
		int start_time = clock(); // начальное время
		forwardKinematicsDH();
		int end_time = clock(); // конечное время
		int search_time_DH = (end_time - start_time); // искомое время
		/*printf("DH_time: ");
		printf("%i\n", search_time_DH);*/
	}
	else {
		int start_time = clock(); // начальное время
		forwardKinematicsEXP();
		int end_time = clock(); // конечное время
		int search_time_DH = (end_time - start_time); // искомое время
		/*printf("EXP_time: ");
		printf("%i\n", search_time_DH);*/
	}
	EulerAngles_ = findEulerAngles(R_);
}

void Manipulator::forwardKinematicsDH() 
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
	R_ = getR(H_[kAxis_]);
	coords_ = joints_[kAxis_];
}

void Manipulator::countJacobian() 
{
	for (int i = 1; i < kAxis_+1; i++) {
		Eigen::Matrix3d Rim1 = getR(H_[i - 1]);
		Eigen::Vector3d z (0, 0, 1);
		Eigen::Vector3d zi0 = Rim1*z;
		Eigen::Vector3d pki = (coords_ - joints_[i]).toVector();
		Eigen::Vector3d Jvi = zi0.cross(pki);
		J_.block(0, i, 3, 1) = Jvi;
		J_.block(3, i, 3, 1) = zi0;
	}
}

//==========================================================================================================================|
//													_Forward Kinematic EXP													|
//==========================================================================================================================|

Eigen::Matrix3d skewSymmetricMatrixFromVector(Eigen::Vector3d x)
{
	Eigen::Matrix3d skewSymmetricMatrix	{{  0,   -x[2],  x[1] },
										 { x[2],   0,   -x[0] },
										 {-x[1],  x[0],   0   }};
	return skewSymmetricMatrix;
}

Eigen::Vector3d vectorFromSkewSymmetricMatrix(Eigen::Matrix3d m)
{
	Eigen::Vector3d vector(-m(1,2), m(0,2), -m(0,1) );
	return vector;
}

Eigen::Matrix4d twistMatrixFromVector(Eigen::Vector<double, 6 > twist)
{
	Eigen::Vector3d w = twist.segment(0, 3);
	Eigen::Vector3d v = twist.segment(3, 3);
	Eigen::Matrix4d twistMatrix = Eigen::Matrix4d::Zero();
	twistMatrix.block(0, 0, 3, 3) = skewSymmetricMatrixFromVector(w);
	twistMatrix.block(0, 3, 3, 1) = v;
	return twistMatrix;
}

Eigen::Vector<double, 6> twistVectorFromMatrix(Eigen::Matrix4d twist)
{
	Eigen::Matrix3d wt = twist.block(0, 0, 3, 3);
	Eigen::Vector3d w = vectorFromSkewSymmetricMatrix(wt);
	Eigen::Vector3d v = twist.block(0, 3, 3, 1);
	Eigen::Vector<double, 6> twistVector = Eigen::Vector<double, 6>::Zero();
	twistVector.segment(0, 3) = w;
	twistVector.segment(3, 3) = v;
	return twistVector;
}

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
	Eigen::Vector3d w = twist.head(3);
	Eigen::Vector3d v = twist.tail(3);
	Eigen::Matrix3d expWt = getApproxExpWt(w);
	Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
	Eigen::Vector3d expTwist21 = (I - expWt) * (w.cross(v)) + w.transpose() * v * w;
	if ((round(w.norm()*10000)) != 0)
		expTwist21 = expTwist21 / pow(w.norm(), 2);
	Eigen::Matrix4d expTwist = Eigen::Matrix4d::Zero();
	expTwist.block(0, 0, 3, 3) = expWt;
	expTwist.block(0, 3, 3, 1) = expTwist21;
	expTwist(3, 3) = 1;
	return expTwist;
}
//
void Manipulator::forwardKinematicsEXP()
{
	Eigen::Vector4d Q{ {0}, {0}, {0}, {1} };

	for (int i = 0; i < kAxis_; i++) {
		Eigen::Vector<double, 6> Twist = angles_[i].get() * unitTwists_[i];
		Eigen::Matrix4d ExpTwist = getApproxExpTwist(Twist);
		H_[i + 1] = H_[i] * ExpTwist * Hiim1T0_[i];

		Eigen::Vector4d joint = H_[i + 1] * Q;
		joints_[i + 1] = Point(joint);
	}
	R_ = getR(H_[kAxis_]);
	coords_ = joints_[kAxis_];
}

void Manipulator::countGeomJacobian()
{
	for (int i = 1; i < kAxis_+1; i++) {
		Eigen::Matrix4d twist = H_[i - 1]*twistMatrixFromVector(unitTwists_[i])*H_[i - 1];
		Eigen::Vector<double, 6> twistV = twistVectorFromMatrix(twist);
		geomJ_.block(0, i, kAxis_, 1) = twistV;
	}
}

void Manipulator::inverseKinematics()
{
}

//==========================================================================================================================|
//																															|
//													_DRAWING																|
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
	
	//change while mouse is pressed
	if (isChangedByMouse_[0] == 1)
		changeByMouse(isChangedByMouse_[1], isChangedByMouse_[2]);
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
double getAngleRadius() 
{
	return 30;
}


void arc(double x, double y, double rad, double startAngle, double endAngle)
{
	double arcStep = PI / 1000;
	for (double i = startAngle; i <= endAngle; i += arcStep)
	{
		glBegin(GL_LINES);
		glVertex3d(x + rad * cos(i), y + rad * sin(i), 0);
		glVertex3d(x + rad * cos(i + arcStep), y + rad * sin(i + arcStep), 0);
		glEnd();
	}
}

void Manipulator::drawAngles()  //need to explain which this angles are 
{
	glColor3f(getColor(WHITE)[0], getColor(WHITE)[1], getColor(WHITE)[2]);
	glLineWidth(getAnglesLineWidth());

	glMatrixMode(GL_MODELVIEW);
	for (int i = 0; i < kAxis_; i++) {
		glPushMatrix();
		glMultMatrixd(H_[i].data());
		if (angles_[i].get() > 0) {
			arc(0, 0, getAngleRadius(), 0, angles_[i].get());
		}
		else {
			arc(0, 0, getAngleRadius(), angles_[i].get(), 0);
		}
		glPopMatrix();
	}
}

void Manipulator::drawBaseCoordSystem() {
	drawCoordSystem(0, 40.0);
}

//==========================================================================================================================|
//																															|
//														_PRINTING															|
//																															|
//==========================================================================================================================|

float getXShift() 
{
	return 12;
}
float getYShift() 
{
	return 7;
}
float getXStartT(int kAxis)
{
	return (200 - getXShift() * (kAxis + 1)) / 2;
}
float getYStartT() 
{
	return 2;
}
float getXStartL()
{
	return 1;
}
float getYStartL(int kAxis)
{
	return (200 - getYShift() * (kAxis + 1)) / 2 +20;
}
float getXStartR()
{
	return 200 - 37;
}
float getYStartR()
{
	return (200 - getYShift() * 7) / 2;
}
float getXShiftR1() { //2 column
	return 30;
}
float getXShiftR2() { //1 column
	return 5; 
}
//==========================================================================================================================|
//													_Angles Printing														|
//==========================================================================================================================|
void Manipulator::printAngles()
{
	//printf("%f\n", Font->Font->Advance(L"abc"));
	Table angleTable=Table(Font, 4, kAxis_, 6, 0);
	std::vector<std::wstring> columnTitles = {L"№"};
	for (int i = 0; i < kAxis_; i++) columnTitles.push_back(std::to_wstring(i + 1));
	angleTable.addColumnTitles(columnTitles);
	std::vector<std::wstring> rowTitles = { L"deg", L"rad", L"more", L"less"};
	angleTable.addRowTitles(rowTitles, 10);
	float xStart = (200 - angleTable.wholeWidth_) / 2;
	angleTable.setPosition(xStart, 2);
	std::vector<std::vector<std::wstring>> data(angleTable.kRows_);
	for (int i = 0; i < kAxis_; i++) {
		data[0].push_back(std::to_wstring(toDegrees(angles_[i].get())).substr(0, angleTable.kSymb_));
		data[1].push_back(std::to_wstring(angles_[i].get()).substr(0, angleTable.kSymb_)); //angle in radians
		data[2].push_back(L"\u2191");
		data[3].push_back(L"\u2193");
	}
	angleTable.setData(data);
	angleTable.printTable();
	//void setCallbacks(std::vector<std::vector<std::function<void(int)>>> callbacks);*/
}

void Manipulator::printAngles2() 
{	
	//glRasterPos2f(0.3, 0.3);
	glDisable(GL_DEPTH_TEST);
	glColor3f(1.0f, 1.0f, 1.0f);   // set color to white

	int kSimb = 6; // number of simbols after comma for degrees=kSimb-4, for radians=kSimb-2
	float xShift = getXShift();
	float xStart = getXStartT(kAxis_);
	float yShift = getYShift();
	float yStart = getYStartT();
	int kLines = 5;

	//horizontal lines
	glColor3f(0.493, 0.493, 0.833);
	for (int i = 0; i < kLines+1; i++) {
		drawLine(xStart, yStart+i*yShift,
			     xStart + xShift * (kAxis_+1) -2, yStart + i * yShift);
	}
	//vertical lines
	drawLine(xStart, yStart, xStart,
		yStart + yShift * kLines);
	for (int i = 1; i < kAxis_+1; i++) {
		float x = xStart + xShift * i + 0.5;
		//vertical lines
		drawLine(x - 2, yStart,
			x - 2, yStart + yShift * kLines);
	}
	// last vertical line
	float x = xStart + xShift * (kAxis_ + 1) - 2;
	drawLine(x, yStart,
		     x, yStart + yShift * kLines);

	glColor3f(1, 1, 1);


	Font->Print(xStart+ xShift/4, yShift, L"№");
	Font->Print(xStart + xShift / 4, yShift*2, L"deg");
	Font->Print(xStart + xShift / 4, yShift * 3, L"rad");
	Font->Print(xStart + xShift / 4, yShift * 4, L"more");
	Font->Print(xStart + xShift / 4, yShift * 5, L"less");

	
	for (int i = 0; i < kAxis_; i++) {
		float x = xStart + xShift * (i+1)+0.5;


		//angle number
		Font->Print(x+4, yShift, std::to_wstring(i+1).c_str());
		//angle in degrees
		std::wstring text = std::to_wstring(toDegrees(angles_[i].get())).substr(0, kSimb);
		Font->Print(x, yShift*2, text.c_str());
		//angle in radians
		text = std::to_wstring(angles_[i].get()).substr(0, kSimb);
		Font->Print(x, yShift*3, text.c_str());

		Font->Print(x+4, yShift * 4, L"\u2191");
		Font->Print(x+4, yShift * 5, L"\u2193");
	}

	glEnable(GL_DEPTH_TEST);
}

//==========================================================================================================================|
//													_Coords Printing														|
//==========================================================================================================================|
void Manipulator::printCoords() 
{
	glRasterPos2f(0.3, 0.3);
	glDisable(GL_DEPTH_TEST);
	glColor3f(1.0f, 1.0f, 1.0f);   // set color to white

	int kSimb = 6; // number of simbols after comma for coordinates=kSimb-4
	float xShift = getXShift();
	float xStart = getXStartL();
	float yShift = getYShift();
	float yStart = getYStartL(kAxis_);
	int kColumns = 5;
	int kRows = kJoints_;

	glColor3f(0.493, 0.493, 0.833);
	//vertical lines
	drawLine(xStart, yStart, 
			 xStart, yStart + yShift * (kRows + 3));
	for (int i = 1; i < kColumns; i++) {
		drawLine(xStart + i * xShift - xShift/4, yStart, 
				 xStart + i * xShift - xShift / 4, yStart + yShift * (kRows + 3) );
	}
	//horizontal lines
	for (int i = 0; i < kRows + 4; i++) {
		drawLine(xStart, yStart + i * yShift,
			xStart + xShift * (kColumns - 1) - xShift / 4, yStart + i * yShift);
	}
	glColor3f(1, 1, 1);

	float y = yStart + yShift - 2;
	Font->Print(xStart + xShift / 4+1, y, L"№");
	Font->Print(xStart + xShift + xShift / 4+0.5, y, L"x");
	Font->Print(xStart + 2 * xShift + xShift / 4+0.5, y, L"y");
	Font->Print(xStart + 3 * xShift + xShift / 4+0.5, y, L"z");
	Font->Print(xStart + 2, yStart + yShift * (kRows + 2) - 2, L"more");
	Font->Print(xStart + 2, yStart + yShift * (kRows + 3) - 2, L"less");

	
	int j = 0;
	for (int i = 1; i < kAxis_+1; i++) {
		if (distance(joints_[i-1], joints_[i]) > 0.0001) {
			float y = yStart + yShift * (j + 2) - 2;
			//coords number
			Font->Print(xStart + xShift / 4+1, y, std::to_wstring(j + 1).c_str());
			//x, y, z coordinates
			std::wstring text = std::to_wstring(joints_[i].x).substr(0, kSimb);
			Font->Print(xStart + xShift-0.5, y, text.c_str());
			text = std::to_wstring(joints_[i].y).substr(0, kSimb);
			Font->Print(xStart + xShift*2-0.5, y, text.c_str());
			text = std::to_wstring(joints_[i].z).substr(0, kSimb);
			Font->Print(xStart + xShift*3-0.5, y, text.c_str());
			j += 1;
		}
	}
	//more and less
	y= yStart + yShift * (kRows + 2)-2;
	Font->Print(xStart + xShift - 0.5+4, y, L"\u2191");
	Font->Print(xStart + xShift * 2 - 0.5+4, y, L"\u2191");
	Font->Print(xStart + xShift * 3 - 0.5+4, y, L"\u2191");
	y = yStart + yShift * (kRows + 3) - 2;
	Font->Print(xStart + xShift - 0.5+4, y, L"\u2193");
	Font->Print(xStart + xShift * 2 - 0.5+4, y, L"\u2193");
	Font->Print(xStart + xShift * 3 - 0.5+4, y, L"\u2193");

	//frame around grip coordinates
	glColor3f(1.0f, 0.0f, 0.0f); //change color to red
	drawLine(xStart-0.5, yStart+yShift*kRows,
		     xStart-0.5, yStart + yShift * (kRows + 1));
	drawLine(xStart + (kColumns-1) * xShift - xShift / 4+0.5, yStart + yShift * kRows,
		     xStart + (kColumns - 1) * xShift - xShift / 4+0.5, yStart + yShift * (kRows + 1));
	drawLine(xStart-0.5, yStart+yShift * kRows - 0.5,
	         xStart + (kColumns - 1) * xShift - xShift / 4 + 0.5, yStart + yShift * kRows-0.5);
	drawLine(xStart - 0.5, yStart + yShift * (kRows+1)+0.5, 
			 xStart + (kColumns - 1) * xShift - xShift / 4 + 0.5, yStart +yShift * (kRows+1)+0.5);

	glEnable(GL_DEPTH_TEST);
}
//==========================================================================================================================|
//													_Orientation Printing													|
//==========================================================================================================================|
void Manipulator::printOrientation() {
	glRasterPos2f(0.3, 0.3);
	glDisable(GL_DEPTH_TEST);
	glColor3f(1.0f, 1.0f, 1.0f);   // set color to white

	int kSimb = 6; // number of simbols after comma for coordinates=kSimb-4
	float xShift = getXShift();
	float xStart = getXStartL();
	float yShift = getYShift();
	float yStart = getYStartT();
	int kColumns = 3;
	int kARows = 3; //3-euler angles, 1-empty row, 5-R matrix 
	int kERows = 1;
	int kRRows = 7;
	int kRows = kARows + kERows + kRRows;

	glColor3f(0.493, 0.493, 0.833);
	//vertical lines
	for (int i = 0; i < kColumns+1; i++) {
		if (i == 0 | i == kColumns) {
			drawLine(xStart + i * xShift, yStart,
				xStart + i * xShift, yStart + yShift * kARows);
			drawLine(xStart + i * xShift, yStart + yShift * (kARows + kERows),
				xStart + i * xShift, yStart + yShift * (kRows));
		}
		else {
			drawLine(xStart + i * xShift, yStart+yShift,
				xStart + i * xShift, yStart + yShift * kARows);
			drawLine(xStart + i * xShift, yStart + yShift * (kARows + kERows+1),
				xStart + i * xShift, yStart + yShift * (kRows));
		}
	}
	//horizontal lines
	for (int i = 0; i < kRows + 1; i++) {
		drawLine(xStart, yStart + i * yShift,
			xStart + xShift * (kColumns), yStart + i * yShift);
	}

	glColor4f(1.0f, 1.0f, 1.0f, 1.0f);

	float y = yStart + yShift - 2;
	Font->Print(xStart + xShift  -2, y, L"Euler angles");
	y = yStart + 2 * yShift - 2;
	Font->Print(xStart  + xShift / 3 + 1.5, y, L"z1");
	Font->Print(xStart + xShift + xShift / 3 + 1.5, y, L"y2");
	Font->Print(xStart + 2 * xShift + xShift / 3 + 1.5, y, L"z3");
	y= yStart + (kARows + kERows + 1) * yShift - 2;
	Font->Print(xStart + +xShift / 2 + 3, y, L"Rotation matrix");

	y = yStart + (kARows + kERows + 2) * yShift - 2;
	glColor3f(getColor((COLOR)0)[0], getColor((COLOR)0)[1], getColor((COLOR)0)[2]);
	Font->Print(xStart + xShift / 3 + 2, y, L"x");
	glColor3f(getColor((COLOR)1)[0], getColor((COLOR)1)[1], getColor((COLOR)1)[2]);
	Font->Print(xStart +  xShift + xShift / 3 + 2, y, L"y");
	glColor3f(getColor((COLOR)2)[0], getColor((COLOR)2)[1], getColor((COLOR)2)[2]);
	Font->Print(xStart + 2 * xShift + xShift / 3 + 2, y, L"z");
	glColor3f(1.0f, 1.0f, 1.0f);

	
	y = yStart + 3 * yShift - 2;
	for (int i = 0; i < 3; i++) {
		std::wstring text = std::to_wstring(toDegrees(EulerAngles_[i].get())).substr(0, kSimb);
		Font->Print(xStart + xShift*i + xShift/4 - 0.5, y, text.c_str());
	}
	for (int j = 0; j < 3; j++) {
		float x = xStart + xShift * j + xShift / 4 - 0.5;
		for (int i = 0; i < 3; i++) {
			y = yStart + yShift * (kARows + kERows + i + 3) - 2;
			
			std::wstring text = std::to_wstring(R_(i, j)).substr(0, kSimb);
			Font->Print(x, y, text.c_str());
		}
		FontM->Print(x+3, yStart + yShift * (kARows + kERows + 3 + 3)-2, L"\u21BB");
		FontM->Print(x+3, yStart + yShift * (kARows + kERows + 4 + 3)-2, L"\u21BA");
	}
}
//==========================================================================================================================|
//													_Function Printing													|
//==========================================================================================================================|
std::wstring Error_to_string(Error error)
{
	switch (error)
	{
	case OK:   return L"OK";
	case OUT_OF_WORKSPACE:   return L"OUT_OF_SPACE";
	}
}
void Manipulator::printFunctions()
{
	glRasterPos2f(0.3, 0.3);
	glDisable(GL_DEPTH_TEST);
	glColor3f(1.0f, 1.0f, 1.0f);   // set color to white

	float xShift1 = getXShiftR1();
	float xShift2 = getXShiftR2();
	float xStart = getXStartR();
	float yShift = getYShift();
	float yStart = getYStartR();
	int kRows = 7;

	//vertical lines
	glColor3f(0.493, 0.493, 0.833);
	float yEnd = yStart + yShift * kRows;
	drawLine(xStart, yStart,
			 xStart, yEnd);
	drawLine(xStart+xShift1, yStart,
			 xStart+xShift1, yEnd);
	drawLine(xStart + xShift1+xShift2, yStart,
		xStart + xShift1+xShift2, yEnd);
	//horizontal 
	for (int i = 0; i < kRows + 1; i++) {
		drawLine(xStart, yStart + i * yShift,
			xStart + xShift1 + xShift2, yStart + i * yShift);
	}
	glColor3f(1, 1, 1);

	std::wstring text = Error_to_string(error_);
	Font->Print(xStart + 2, yStart + yShift-2, text.c_str());
	Font->Print(xStart + 2, yStart + 2*yShift - 2, L"starting position");
	Font->Print(xStart + 2, yStart + 3 * yShift - 2, L"mouse control");
	Font->Print(xStart + 2, yStart + 4 * yShift - 2, L"go with speed");
	Font->Print(xStart + 2, yStart + 5 * yShift - 2, L"go with grip speed");
	Font->Print(xStart + 2, yStart + 6 * yShift - 2, L"go with grip speed T");
	Font->Print(xStart + 2, yStart + 7 * yShift - 2, L"go by GCODE's");
	//view: pictures from Solidworks 
	// f (f, b), (r, l), (t, b) 
	// * (*, x), (->, <-), (|, |)

}

//==========================================================================================================================|
//															_Cubes															|
//==========================================================================================================================|
enum CubeOr {
	NON,
	RIGHT,
	FRONT, 
	LEFT,
	BACK,
	TOP,
	BOTTOM
};
void drawCube(float ltCornerX, float ltCornerY, float a, float shift, int i) {
	//glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
	//glVertex3f(1.0f, 1.0f, -1.0f);
	//glVertex3f(1.0f, 1.0f, 1.0f);
	//glVertex3f(1.0f, -1.0f, 1.0f);
	//glVertex3f(1.0f, -1.0f, -1.0f);
	//glEnd();  // End of drawing color-cube

	float ax = a * glutGet(GLUT_WINDOW_HEIGHT) / glutGet(GLUT_WINDOW_WIDTH);

	//back
	drawLine(ltCornerX, ltCornerY, 
			 ltCornerX + ax, ltCornerY); //-
	drawLine(ltCornerX, ltCornerY, 
			 ltCornerX, ltCornerY + a); //|
	drawLine(ltCornerX + ax, ltCornerY, 
			 ltCornerX + ax, ltCornerY + a); // |
	drawLine(ltCornerX, ltCornerY + a, 
			 ltCornerX + ax, ltCornerY + a); //_

	//front
	float lbCornerX = ltCornerX - shift;
	float lbCornerY = ltCornerY + shift;
	drawLine(lbCornerX, lbCornerY, 
			lbCornerX + ax, lbCornerY); //-
	drawLine(lbCornerX, lbCornerY, 
			 lbCornerX, lbCornerY + a); //|
	drawLine(lbCornerX + ax, lbCornerY, 
			 lbCornerX + ax, lbCornerY + a); // |
	drawLine(lbCornerX, lbCornerY + a, 
			 lbCornerX + ax, lbCornerY + a); //_

	//left
	drawLine(ltCornerX, ltCornerY, 
			 ltCornerX - shift, ltCornerY + shift); //-
	drawLine(ltCornerX, ltCornerY + a, 
			 ltCornerX - shift, ltCornerY + a + shift); //_

	//right
	drawLine(ltCornerX+ax, ltCornerY, 
			 ltCornerX + ax - shift, ltCornerY + shift); //-
	drawLine(ltCornerX+ax, ltCornerY + a,
			 ltCornerX + ax - shift, ltCornerY + a + shift); //_

	glColor3f(0.2, 0.2, 0.2);
	if (i == 1) { //left
		glBegin(GL_QUAD_STRIP);
		glVertex2f(ltCornerX, ltCornerY);
		glVertex2f(ltCornerX - shift,ltCornerY + shift); //-
		glVertex2f(ltCornerX, ltCornerY + a);
		glVertex2f(ltCornerX - shift, ltCornerY + a + shift);
		glEnd();
	}
	if (i == 2) { //front
		glBegin(GL_QUAD_STRIP);
		glVertex2f(lbCornerX, lbCornerY);
		glVertex2f(lbCornerX, lbCornerY + a); //|
		glVertex2f(lbCornerX + ax, lbCornerY);
		glVertex2f(lbCornerX + ax, lbCornerY + a); // |
		glEnd();
	}
	if (i == 3) { //right
		glBegin(GL_QUAD_STRIP);
		glVertex2f(ltCornerX + ax, ltCornerY);
		glVertex2f(ltCornerX + ax - shift, ltCornerY + shift); //-
		glVertex2f(ltCornerX + ax, ltCornerY + a);
		glVertex2f(ltCornerX + ax - shift, ltCornerY + a + shift);
		glEnd();
	}
	if (i == 4) { //back
		glBegin(GL_QUAD_STRIP);
		glVertex2f(ltCornerX, ltCornerY);
		glVertex2f(ltCornerX, ltCornerY + a); //|
		glVertex2f(ltCornerX + ax, ltCornerY);
		glVertex2f(ltCornerX + ax, ltCornerY + a); // |
		glEnd();
	}
	if (i == 5) { //top
		glBegin(GL_QUAD_STRIP);
		glVertex2f(ltCornerX, ltCornerY);
		glVertex2f(ltCornerX - shift, ltCornerY + shift);
		glVertex2f(ltCornerX+ax, ltCornerY);
		glVertex2f(ltCornerX+ax - shift, ltCornerY + shift);
		glEnd();
	}
	if (i == 6) { //bottom
		glBegin(GL_QUAD_STRIP);
		glVertex2f(ltCornerX, ltCornerY + a);
		glVertex2f(ltCornerX - shift, ltCornerY + a + shift);
		glVertex2f(ltCornerX + ax, ltCornerY + a);
		glVertex2f(ltCornerX + ax - shift, ltCornerY + a + shift);
		glEnd();
	}

}
void drawRect(float x, float y, float w, float h) {
	glBegin(GL_QUADS);
	glVertex3f(x, y, 0);
	glVertex3f(x + w, y, 0);
	glVertex3f(x + w, y + h, 0);
	glVertex3f(x, y + h, 0);
	glEnd();
}
void drawCubesPanel() {
	glColor3f(0.8, 0.8, 1);
	//Panel coordinates
	float xStart = 13;
	float yStart = 150;
	float width = 35;
	float height = 40;
	drawRect(xStart, yStart, width, height);
	glColor3f(0.4, 0.4, 0.4);
	float a = 7;
	float shift = 3;
	float between = 2;
	float ax = a * glutGet(GLUT_WINDOW_HEIGHT) / glutGet(GLUT_WINDOW_WIDTH);
	for (int i = 0; i < 4; i++) {
		drawCube(xStart + (ax + shift + between)*i + 5, yStart + height / 2 - (a + shift) / 2, a, shift, i+1);
	}
	drawCube(xStart + (ax + shift + between)  + 5, yStart + height / 2 - (a + shift) / 2 - (a + shift + between), a, shift, TOP);
	drawCube(xStart + (ax + shift + between) + 5, yStart + height / 2 - (a + shift) / 2 + (a + shift + between), a, shift, BOTTOM);

	

}
void Manipulator::drawOrientationCubes() {
	glPushMatrix(); //to print more than one text
	//glLoadIdentity();
	glDisable(GL_TEXTURE_2D);
	// move to coordinates
	glTranslatef(0, 0, -1);
	glRasterPos2f(-1, 0.5);
	int xStart = 2;
	int yStart = 182;
	int a = 10;
	int shift = 3;
	drawCube(xStart+shift,yStart,a,shift, 0);
	glEnable(GL_TEXTURE_2D);
	glPopMatrix();
	if (isCubePressed) {
		drawCubesPanel();
	}
}
void Manipulator::printInfo() {
	printAngles();
	printCoords();
	printOrientation();
	printFunctions();
	drawOrientationCubes();
}
//==========================================================================================================================|
//																															|
//														 _MOUSE 															|
//																															|
//==========================================================================================================================|
int Manipulator::changeByMouse(float x, float y) {
	//angles
	float angularSpeed = toRadians(1);
	float xShift = getXShift();
	float xStart = getXStartT(kAxis_);
	float yShift = getYShift();
	float yStart = getYStartT();
	//x = xStart + xShift * (i+1)+0.5-2;
	int i = floor((x - xStart - 0.5 + 2) / xShift);
	if (0 < i && i < kAxis_ + 1) { //angles
		if (yStart + yShift * 3 < y && y < yStart + yShift * 4) {
			isChangedByMouse_[0] = 1; isChangedByMouse_[1] = x; isChangedByMouse_[2] = y;
			changeAngle(i, angularSpeed);
		}
		else if (yStart + yShift * 4 < y && y < yStart + yShift * 5) {
			isChangedByMouse_[0] = 1; isChangedByMouse_[1] = x; isChangedByMouse_[2] = y;
			changeAngle(i, -angularSpeed);
		}
	}
	//coords
	float linearSpeed = 3;
	xShift = getXShift();
	xStart = getXStartL();
	yShift = getYShift();
	yStart = getYStartL(kAxis_);
	//xStart + i * xShift - xShift / 4
	i = floor((x - xStart + xShift / 4) / xShift);
	int sign = 0;
	if (yStart + yShift * (kJoints_ + 1) < y && y < yStart + yShift * (kJoints_ + 2)) sign = 1;
	if (yStart + yShift * (kJoints_ + 2) < y && y < yStart + yShift * (kJoints_ + 3)) sign = -1;
	Point dCoords = Point(0, 0, 0);
	if (i == 1) dCoords.x = sign * linearSpeed;
	if (i == 2) dCoords.y = sign * linearSpeed;
	if (i == 3) dCoords.z = sign * linearSpeed;
	if (distance(dCoords, Point(0, 0, 0)) > 0.0001) {
		isChangedByMouse_[0] = 1; isChangedByMouse_[1] = x; isChangedByMouse_[2] = y;
		changePosition(dCoords);
	}

	//orientation
	xShift = getXShift();
	xStart = getXStartL();
	yShift = getYShift();
	yStart = getYStartT();
	int kRows = 11;
	float angularOrientSpeed = toRadians(1);
	i = floor((x - xStart) / xShift) + 1;
	if (0 < i && i < 4) {
		if (yStart + (kRows - 2) * yShift < y && y < yStart + (kRows - 1) * yShift) {
			changeOrientation(i, angularOrientSpeed);
			isChangedByMouse_[0] = 1; isChangedByMouse_[1] = x; isChangedByMouse_[2] = y;
		}
		if (yStart + (kRows - 1) * yShift < y && y < yStart + (kRows)*yShift) {
			changeOrientation(i, -angularOrientSpeed);
			isChangedByMouse_[0] = 1; isChangedByMouse_[1] = x; isChangedByMouse_[2] = y;
		}
	}

	//functions
	if (getXStartR() < x && x < getXStartR() + getXShiftR1()) {
		if (getYStartR() + getYShift() < y && y < getYStartR() + 2 * getYShift()) {
			for (int i = 0; i < kAxis_; i++) { //start position
				angles_[i] = Angle(0);
			}
			setAngles(angles_);
		}
		if (getYStartR() + 6 * getYShift() < y && y < getYStartR() + 7 * getYShift()) { // go by GCODE
			goByGCODE("AbsoluteCube1.gcode");
		}
	}
	//cubePanel
	xStart = 13;
	yStart = 150;
	float width = 35;
	float height = 40;
	float a = 7;
	float shift = 3;
	float between = 2;
	float ax = a * glutGet(GLUT_WINDOW_HEIGHT) / glutGet(GLUT_WINDOW_WIDTH);
	if (isCubePressed) {
		i = 0;
		if (yStart + height / 2 - (a + shift) / 2 < y && y < yStart + height / 2 - (a + shift) / 2 + a + shift) {
			i = floor((x - (xStart - shift+5)) / (ax + shift + between)) + 1;
		}
		if (xStart + (a + shift + between) - shift < x && x < xStart + (a + shift + between) + ax + 5 && yStart + height / 2 - (a + shift) / 2 - (a + shift - between) < y && y < yStart + height / 2 - (a + shift) / 2 - (a + shift - between) + shift + a) {
			i = 5;
		}
		if (xStart + (a + shift + between) - shift < x && x < xStart + (a + shift + between) + ax + 5 && yStart + height / 2 - (a + shift) / 2 + (a + shift - between) < y && y < yStart + height / 2 - (a + shift) / 2 + (a + shift - between) + shift + a) {
			i = 6;
		}
		if (i != 0) {
			return i;
		}
	}

	//cube
	xStart = 2;
	yStart = 182;
	a = 10;
	shift = 3;
	if (xStart < x && x < xStart + a + shift && yStart < y && y < yStart + yShift) {
		isCubePressed = !isCubePressed;
	}
	return 0;
}
void Manipulator::stopChangeByMouse() {
	isChangedByMouse_[0] = 0;
}

//==========================================================================================================================|
//																															|
//														_VIEW 														    	|
//																															|
//==========================================================================================================================|

//==========================================================================================================================|
//																															|
//														_GCODES 															|
//																															|
//==========================================================================================================================|
std::vector<float> parse(std::string line) {
		line += ' ';
		int k_gcode = 5;
		float table_x = 50;
		float table_y = 50;
		float table_z = 50;
		float x_g=table_x;
		float y_g=table_y;
		float z_g=table_z;
		float e_g=0;
		float f_g=0;
		
		int i;
		i = line.rfind("Z") + 1;
		if (i != 0) {    // because if Z hasn't been found that line.find=-1, i=0
			std::string z_gs = "";
			while (((i == (line.length() - 1)) | (line[i] == ' ') | (line[i] == ';')) == false) {
				z_gs += line[i];
				i += 1;
			}
			z_g = table_z + std::stof(z_gs) / k_gcode;
		}

		i = line.rfind("X") + 1;
		if (i != 0) {
			std::string x_gs = "";
			while (((i == line.length() - 1) | (line[i] == ' ') | (line[i] == ';')) == false) {
				x_gs += line[i];
				i += 1;
			}
			x_g = table_x + std::stof(x_gs) / k_gcode;
		}

		i = line.rfind("Y") + 1;
		if (i != 0) {
			std::string y_gs = "";
			while (((i == line.length() - 1) | (line[i] == ' ') | (line[i] == ';')) == false) {
				y_gs += line[i];
				i += 1;
			}
			y_g = table_y + std::stof(y_gs) / k_gcode;
		}

		i = line.rfind("E") + 1;
		if (i != 0) {
			std::string e_gs = "";
			while (((i == line.length() - 1) | (line[i] == ' ') | (line[i] == ';')) == false) {
				e_gs += line[i];
				i += 1;
			}
			e_g = std::stof(e_gs);
		}

		i = line.rfind("F") + 1;
		if (i != 0) {
			std::string f_gs = "";
			while (((i == line.length() - 1) | (line[i - 1] == ' ') | (line[i - 1] == ';')) == false) {
				f_gs += line[i];
				i += 1;
				//print(i);
			}
			f_g = std::stof(f_gs);
		}
		std::vector<float> new_point_r = { x_g, y_g, z_g, e_g, f_g };
		return new_point_r;
}

void Manipulator::goByGCODE(std::string fileName) {
	std::string line;
	std::ifstream file(fileName);

	while (std::getline(file, line)) {
		if (line.rfind("G1 ", 0) == 0) {
			std::vector<float> targetPoint = parse(line);
		}
	}
	file.close();
}



//==========================================================================================================================|
//																															|
//												 _THREE AXIS RRR MANIPULATOR												|
//																															|
//==========================================================================================================================|
ThreeAxisRrrManipulator::ThreeAxisRrrManipulator()
{
}
ThreeAxisRrrManipulator::ThreeAxisRrrManipulator(std::vector<double> l)
{
	forwardKinematicsMethod_ = DH;
	kAxis_ = 3;
	l_ = l;
	a_ = {0, l[1], l[2]};
	alpha_ = {PI/2, 0, 0};
	d_ = {l[0], 0, 0};
	dTh_ = {0, 0, 0};

	/*a_ = { 0, l[1], 0};
	alpha_ = { PI / 2, 0, PI / 2};
	d_ = { l[0], 0, 0, l[2], 0, l[3] };
	dTh_ = { 0, 0, PI / 2, 0, 0, 0 };*/

	initializeVectorsAsNull();
	J_.resize(kAxis_, kAxis_);
	setAngles(angles_);
	kJoints_ = 0;
	for (int i = 0; i < kAxis_; i++) {
		if (distance(joints_[i], joints_[i + 1]) > 0.0001) kJoints_ += 1;
	}
	error_ = OK;
}

bool firstAngleCloserToThird(Angle first, Angle second, Angle third)
{
	return abs((third - first).get()) < abs((third - second).get());
}

void ThreeAxisRrrManipulator::inverseKinematics()
{
	double r1 = sqrt(sq(coords_.x) + sq(coords_.y));
	double r2 = coords_.z - d_[0];
	double r3 = sqrt(sq(r1) + sq(r2));

	if (abs((sq(a_[1]) + sq(r3) - sq(a_[2])) / (2 * a_[1] * r3)) <= 1) {
		Angle psi1 = Angle(acos((sq(a_[1]) + sq(r3) - sq(a_[2])) / (2 * a_[1] * r3)));
		Angle psi2 = Angle(atan2(r2, r1));
		if (abs((sq(a_[1]) + sq(a_[2]) - sq(r3)) / (2 * a_[1] * a_[2])) <= 1) {
			Angle psi3 = Angle(acos((sq(a_[1]) + sq(a_[2]) - sq(r3)) / (2 * a_[1] * a_[2])));
			error_ = OK;

			Angle t11 = Angle(atan2(coords_.y, coords_.x));
			Angle t12 = t11 + Angle(PI);

			Angle t211 = psi2 + psi1;
			Angle t212 = psi2 - psi1;

			Angle t221 = Angle(PI) - (psi2 + psi1);
			Angle t222 = Angle(PI) - (psi2 - psi1);



			Angle t31 = psi3 - Angle(PI);
			Angle t32 = Angle(PI) - psi3;

			if (firstAngleCloserToThird(t11, t12, angles_[0])) {
				angles_[0] = t11;
				if (firstAngleCloserToThird(t211, t212, angles_[1])) {
					angles_[1] = t211;
					angles_[2] = t31;
				}
				else {
					angles_[1] = t212;
					angles_[2] = t32;
				}
			}
			else {
				angles_[0] = t12;
				if (firstAngleCloserToThird(t221, t222, angles_[1])) {
					angles_[1] = t221;
					angles_[2] = t32;
				}
				else {
					angles_[1] = t222;
					angles_[2] = t31;
				}
			}

		}
		else {
			error_ = OUT_OF_WORKSPACE;
		}
	}
	else {
		error_ = OUT_OF_WORKSPACE;
	}
	
}

std::vector<Angle> ThreeAxisRrrManipulator::getAngles() {
	return angles_;
}

Eigen::Matrix4d ThreeAxisRrrManipulator::getH() {
	return H_[kAxis_];
}

Error ThreeAxisRrrManipulator::getError() {
	return error_;
}

//==========================================================================================================================|
//																															|
//										    	_SIX AXIS STANDARD MANIPULATOR												|
//																															|
//==========================================================================================================================|

SixAxisStandardManipulator::SixAxisStandardManipulator(ForwardKinematicsMethod method,  std::vector<double> l)
{
	forwardKinematicsMethod_ = method;
	kAxis_ = 6;
	l_ = l;
	if (method == DH) {
		a_ = { 0, l[1], 0, 0, 0, 0 };
		alpha_ = { PI / 2, 0, PI / 2, -PI / 2, PI / 2, 0 };
		d_ = { l[0], 0, 0, l[2], 0, l[3] };
		dTh_ = { 0, 0, PI / 2, 0, 0, 0 };
	}
	else if (method == EXP) {
		Eigen::Vector<double, 6> uT(0, 0, 1, 0, 0, 0);
		std::vector<Eigen::Vector<double, 6>> unitTwists = { uT, uT, uT, uT, uT, uT };
		unitTwists_ = unitTwists;
		Eigen::Matrix4d H10T0{ {1,0,0, 0},
								{0,0,-1,0},
								{0,1,0,l[0]},
								{0,0,0,1} };
		Eigen::Matrix4d H21T0{ {1,0,0, l[1]},
								{0,1,0,0},
								{0,0,1,0},
								{0,0,0,1} };
		Eigen::Matrix4d H32T0{ {0,0,1, 0},
								{1,0,0,0},
								{0,1,0,0},
								{0,0,0,1} };
		Eigen::Matrix4d H43T0{ {1,0,0, 0},
								{0,0,1,0},
								{0,-1,0,l[2]},
								{0,0,0,1} };
		Eigen::Matrix4d H54T0{ {1,0,0, 0},
								{0,0,-1,0},
								{0,1,0,0},
								{0,0,0,1} };

		Eigen::Matrix4d H65T0{ {1,0,0, 0},
								{0,1,0,0},
								{0,0,1,l[3]},
								{0,0,0,1} };
		std::vector<Eigen::Matrix4d> Hiim1T0 = { H10T0, H21T0, H32T0, H43T0, H54T0, H65T0 };
		Hiim1T0_ = Hiim1T0;
	}
	initializeVectorsAsNull();
	J_.resize(kAxis_, kAxis_);
	setAngles(angles_);
	kJoints_ = 0;
	for (int i = 0; i < kAxis_; i++) {
		if (distance(joints_[i], joints_[i + 1]) > 0.0001) kJoints_ += 1;
	}
	error_ = OK;
	firstThreeAxis= ThreeAxisRrrManipulator(l_);
}

std::array<Angle, 3>  findEulerAngles(Eigen::Matrix3d R) {
	//Angles around z, y', z''
	Angle z3; Angle y2; Angle z1;
	if (R(2,2)==1) {
		y2 = Angle(0);
		z3 = Angle(0); //choose any t6
		z1 = Angle(atan2(R(1,0), R(0,0)) - z3.get());
	}
	else if (R(2,2)==-1) {
		y2 = Angle(PI);
		z3 = Angle(0); //выбираем t6 любым
		z1 = Angle(atan2(-R(0,1), -R(0,0) + z3.get()));
	}
	else {
		int sign = 1; //1 or -1
		y2 = Angle(atan2 (sign * sqrt(1 - sq(R(2,2))),   R(2,2)));
		z1 = Angle(atan2 (sign * R(1,2), sign * R(0,2)));
		z3 = Angle(atan2 (sign * R(2,1), -sign * R(2,0))); 
	}
	std::array<Angle, 3> angles = { z1, y2, z3 };
	return angles;
}
Eigen::Matrix3d SixAxisStandardManipulator::getR3() {
	for (int i = 0; i < 3; i++) {
		double th = angles_[i].get();
		//transformation matrix from i-1 coordinate system to i coordinate system
		Eigen::Matrix4d Hi{ {cos(th),  -sin(th) * cos(alpha_[i]),  sin(th) * sin(alpha_[i]),   a_[i] * cos(th)},
							  {sin(th),  cos(th) * cos(alpha_[i]),   -cos(th) * sin(alpha_[i]),  a_[i] * sin(th)},
							  {0,        sin(alpha_[i]),             cos(alpha_[i]),             d_[i]          },
							  {0,        0,                          0,                          1              } };
		H_[i + 1] = H_[i] * Hi;  //transformation matrix from 0 coordinate system to i coordinate system 
	}
	return getR(H_[3]);
}



void SixAxisStandardManipulator::inverseKinematics() 
{
	Eigen::Vector3d zv (0, 0, 1);
	Eigen::Vector3d p46 = d_[5] * R_ * zv;
	Point p04 = coords_ - Point(p46);

	firstThreeAxis.setPosition(p04);
	std::vector<Angle> angles3 = firstThreeAxis.getAngles();
	error_ = firstThreeAxis.getError();
	if (error_ != OUT_OF_WORKSPACE) {
		angles_[0] = angles3[0]; angles_[1] = angles3[1]; angles_[2] = angles3[2] + PI / 2;
		Eigen::Matrix3d R3 = getR3();
		Eigen::Matrix3d R36 = R3.transpose() * R_;
		std::array<Angle, 3> angles36 = findEulerAngles(R36);
		angles_[3] = angles36[0]; angles_[4] = angles36[1]; angles_[5] = angles36[2];
		//angles_[3] = 0; angles_[4] = 0; angles_[5]=0;
		angles_[2] = angles3[2];
		setAngles(angles_);
	}
}