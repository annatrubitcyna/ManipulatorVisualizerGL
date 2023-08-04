#pragma comment(lib, "ftgl_dynamic_MT.lib")

#include <iostream>
#include <math.h>

#include <ftgl/FTGLTextureFont.h>
//#include <GL/glut.h>
#include "Manipulator.h"

#include "font.h"
#include <wchar.h>


const char* ttf = "C:/Windows/Fonts/arial.ttf";
const char* ttf2 = "C:/Users/Ann/Downloads/FTGL/arial.ttf";
CFont* Font = new CFont(ttf, 24, 32);

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

void drawLine(float x1, float y1, float x2, float y2) 
{
	glBegin(GL_LINES);
	glVertex2f(x1, y1);
	glVertex2f(x2, y2);
	glEnd();
}

Eigen::Matrix3d getR(Eigen::Matrix4d H)
{
	return H.block<3, 3>(0, 0);
}

GLfloat* getColor(COLOR i)
{
	//Red, blue, green, yellow, green-blue, pink, orange,violet, white, green-yellow
	GLfloat colors[10][3] = { {1.0, 0.0, 0.0}, {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0},{1.0, 1.0, 0.0}, {0.0, 1.0, 1.0},
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
//														POINTS																|
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
//														ANGLE																|
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
//														MANIPULATOR															|
//																															|
//==========================================================================================================================|

void Manipulator::initializeVectorsAsNull() {
	coords_ = Point(0, 0, 0);
	R_ = Eigen::Matrix3d::Zero();
	H_.push_back(Eigen::Matrix4d::Identity());
	joints_.push_back(Point(0.0, 0.0, 0.0));
	for (int i = 0; i < kAxis_; i++) {
		joints_.push_back(Point(0.0, 0.0, 0.0));
		angles_.push_back(Angle(0.0));
		H_.push_back(Eigen::Matrix4d::Zero());
		if (i < 3)	isChangedByMouse_.push_back(0);
	}
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
		forwardKinematic();
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
	forwardKinematic();
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
	forwardKinematic();
}

void Manipulator::changePosition(Point dCoords)
{
	coords_ = coords_ + dCoords;
	inverseKinematic();
	forwardKinematic();
}

void Manipulator::setPosition(Point coords, Eigen::Matrix3d R)
{
	coords_ = coords;
	R_ = R;
	inverseKinematic();
	forwardKinematic();
}

void Manipulator::setPosition(Point coords)
{
	setPosition(coords, Eigen::Matrix3d::Identity());
}

//==========================================================================================================================|
//													Forward Kinematic														|
//==========================================================================================================================|

void Manipulator::forwardKinematic()
{
	if (forwardKinematicsMethod_ == DH) {
		int start_time = clock(); // начальное время
		forwardKinematicDH();
		int end_time = clock(); // конечное время
		int search_time_DH = (end_time - start_time); // искомое время
		/*printf("DH_time: ");
		printf("%i\n", search_time_DH);*/
	}
	else {
		int start_time = clock(); // начальное время
		forwardKinematicEXP();
		int end_time = clock(); // конечное время
		int search_time_DH = (end_time - start_time); // искомое время
		/*printf("EXP_time: ");
		printf("%i\n", search_time_DH);*/
	}
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
//													Forward Kinematic EXP													|
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
void Manipulator::forwardKinematicEXP()
{
	Eigen::Vector4d Q{ {0}, {0}, {0}, {1} };

	for (int i = 0; i < kAxis_; i++) {
		Eigen::Vector<double, 6> Twist = angles_[i].get() * unitTwists_[i];
		Eigen::Matrix4d ExpTwist = getApproxExpTwist(Twist);
		H_[i + 1] = H_[i] * ExpTwist * Hiim1T0_[i];

		Eigen::Vector4d joint = H_[i + 1] * Q;
		joints_[i + 1] = Point(joint);
	}
	R_ = getR(H_[kAxis_ - 1]);
	coords_ = joints_[kAxis_ - 1];
}

void Manipulator::countGeomJacobian()
{
	for (int i = 1; i < kAxis_+1; i++) {
		Eigen::Matrix4d twist = H_[i - 1]*twistMatrixFromVector(unitTwists_[i])*H_[i - 1];
		Eigen::Vector<double, 6> twistV = twistVectorFromMatrix(twist);
		geomJ_.block(0, i, kAxis_, 1) = twistV;
	}
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
//														PRINTING															|
//																															|
//==========================================================================================================================|
//void printTable(float xStart, float yStart, float xShift, float yShift, std::vector<const wchar_t*> columnNames,
//	std::vector<const wchar_t*> rowNames, std::vector<std::vector<const wchar_t*>> data)
//{
//	int kRows = rowNames.size();
//	int kColumns = columnNames.size();
//
//	glRasterPos2f(0.3, 0.3);
//	glDisable(GL_DEPTH_TEST);
//	glColor3f(1.0f, 1.0f, 1.0f);   // set color to white
//
//	//horizontal lines
//	for (int i = 0; i < kRows + 1; i++) {
//		glBegin(GL_LINES);
//		glVertex2f(xStart, yStart + i * yShift);
//		glVertex2f(xStart + xShift * kRows - 2, yStart + i * yShift);
//		glEnd();
//	}
//	//vertical lines 
//	for (int i = 0; i < kColumns + 1; i++) {
//		glBegin(GL_LINES);
//		glVertex2f(xStart + xShift * i - 2, yStart);
//		glVertex2f(xStart + xShift * i - 2, yStart + yShift * kRows);
//		glEnd();
//	}
//}

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
	return 5;
}
float getYStartL(int kAxis)
{
	return (200 - getYShift() * (kAxis + 1)) / 2;
}
float getXStartR()
{
	return 200 - 50;
}
float getYStartR()
{
	return (200 - getYShift() * 7) / 2;
}

void Manipulator::printAngles() 
{	
	glRasterPos2f(0.3, 0.3);
	glDisable(GL_DEPTH_TEST);
	glColor3f(1.0f, 1.0f, 1.0f);   // set color to white

	int kSimb = 6; // number of simbols after comma for degrees=kSimb-4, for radians=kSimb-2
	float xShift = getXShift();
	float xStart = getXStartT(kAxis_);
	float yShift = getYShift();
	float yStart = getYStartT();
	int kLines = 5;
	//horizontal lines
	for (int i = 0; i < kLines+1; i++) {
		glBegin(GL_LINES);
		glVertex2f(xStart, yStart+i*yShift );
		glVertex2f(xStart + xShift * (kAxis_+1) -2, yStart + i * yShift);
		glEnd();
	}

	Font->Print(xStart+ xShift/4, yShift, L"№");
	Font->Print(xStart + xShift / 4, yShift*2, L"deg");
	Font->Print(xStart + xShift / 4, yShift * 3, L"rad");
	Font->Print(xStart + xShift / 4, yShift * 4, L"more");
	Font->Print(xStart + xShift / 4, yShift * 5, L"less");

	//first vertical line
	glBegin(GL_LINES);
	glVertex2f(xStart, yStart);
	glVertex2f(xStart, yStart+yShift * kLines);
	glEnd();
	for (int i = 0; i < kAxis_; i++) {
		float x = xStart + xShift * (i+1)+0.5;
		//vertical lines
		glBegin(GL_LINES);
		glVertex2f(x-2, yStart);
		glVertex2f(x-2, yStart + yShift * kLines);
		glEnd();

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
	// last vertical line
	float x = xStart + xShift * (kAxis_+1) -2;
	glBegin(GL_LINES);
	glVertex2f(x, yStart);
	glVertex2f(x, yStart + yShift * kLines);
	glEnd();

	glEnable(GL_DEPTH_TEST);
}

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
	int kRows = 0;
	for (int i = 1; i < kAxis_; i++){
		if (distance(joints_[i], joints_[i + 1]) > 0.0001){
			kRows += 1;
		}
	}

	//vertical lines
	glBegin(GL_LINES);
	glVertex2f(xStart, yStart);
	glVertex2f(xStart, yStart + yShift * (kRows + 3));
	glEnd();
	for (int i = 1; i < kColumns; i++) {
		glBegin(GL_LINES);
		glVertex2f(xStart + i * xShift - xShift/4, yStart);
		glVertex2f(xStart + i * xShift - xShift / 4, yStart + yShift * (kRows + 3) );
		glEnd();
	}

	float y = yStart + yShift - 2;
	Font->Print(xStart + xShift / 4+1, y, L"№");
	Font->Print(xStart + xShift + xShift / 4, y, L"x");
	Font->Print(xStart + 2 * xShift + xShift / 4, y, L"y");
	Font->Print(xStart + 3 * xShift + xShift / 4, y, L"z");
	Font->Print(xStart + 2, yStart + yShift * (kRows + 2) - 2, L"more");
	Font->Print(xStart + 2, yStart + yShift * (kRows + 3) - 2, L"less");

	//horizontal 
	for (int i = 0; i < kRows + 4; i++) {
		glBegin(GL_LINES);
		glVertex2f(xStart, yStart+i*yShift);
		glVertex2f(xStart + xShift * (kColumns-1) - xShift / 4, yStart+i*yShift);
		glEnd();
	}
	int j = 0;
	for (int i = 1; i < kAxis_; i++) {
		if (distance(joints_[i], joints_[i + 1]) > 0.0001) {
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

			//Font->Print(x + 4, yShift * 4, L"\u2191");
			//Font->Print(x + 4, yShift * 5, L"\u2193");
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
	glBegin(GL_LINES);
	glVertex2f(xStart-0.5, yStart+yShift*kRows);
	glVertex2f(xStart-0.5, yStart + yShift * (kRows + 1));
	glEnd();
	glBegin(GL_LINES);
	glVertex2f(xStart + (kColumns-1) * xShift - xShift / 4+0.5, yStart + yShift * kRows);
	glVertex2f(xStart + (kColumns - 1) * xShift - xShift / 4+0.5, yStart + yShift * (kRows + 1));
	glEnd();
	glBegin(GL_LINES);
	glVertex2f(xStart-0.5, yStart+yShift * kRows - 0.5);
	glVertex2f(xStart + (kColumns - 1) * xShift - xShift / 4 + 0.5, yStart + yShift * kRows-0.5);
	glEnd();
	glBegin(GL_LINES);
	glVertex2f(xStart - 0.5, yStart + yShift * (kRows+1)+0.5);
	glVertex2f(xStart + (kColumns - 1) * xShift - xShift / 4 + 0.5, yStart +yShift * (kRows+1)+0.5);
	glEnd();

	glEnable(GL_DEPTH_TEST);
}
void Manipulator::printFunctions()
{
	glRasterPos2f(0.3, 0.3);
	glDisable(GL_DEPTH_TEST);
	glColor3f(1.0f, 1.0f, 1.0f);   // set color to white

	float xShift1 = 30;
	float xShift2 = 5;
	float xStart = getXStartR();
	float yShift = getYShift();
	float yStart = getYStartR();
	int kRows = 7;

	//vertical lines
	float yEnd = yStart + yShift * kRows;
	drawLine(xStart, yStart,
			 xStart, yEnd);
	drawLine(xStart+xShift1, yStart,
			 xStart+xShift1, yEnd);
	drawLine(xStart + xShift1+xShift2, yStart,
		xStart + xShift1+xShift2, yEnd);

	Font->Print(xStart + 2, yStart+yShift-2, L"error");
	Font->Print(xStart + 2, yStart +2*yShift - 2, L"starting position");
	Font->Print(xStart + 2, yStart + 3 * yShift - 2, L"mouse control");
	Font->Print(xStart + 2, yStart + 4 * yShift - 2, L"go with speed");
	Font->Print(xStart + 2, yStart + 5 * yShift - 2, L"go with grip speed");
	Font->Print(xStart + 2, yStart + 6 * yShift - 2, L"go with grip speed T");
	Font->Print(xStart + 2, yStart + 7 * yShift - 2, L"go by GCODE's");
	// f (f, b), (r, l), (t, b) 
	// * (*, x), (->, <-), (|, |)

	//horizontal 
	for (int i = 0; i < kRows + 1; i++) {
		drawLine(xStart, yStart + i * yShift,
				 xStart + xShift1+xShift2, yStart + i * yShift);
	}

}
void Manipulator::printInfo() {
	printAngles();
	printCoords();
	printFunctions();
}
void Manipulator::changeByMouse(float x, float y) {
	float angularSpeed= toRadians(1);
	float xShift = getXShift();
	float xStart = getXStartT(kAxis_);
	float yShift = getYShift();
	float yStart = getYStartT();
	int i = round((x - xStart - 0.5) / xShift );
	if (0 < i < kAxis_) {
		if (yStart + yShift * 3 < y && y < yStart + yShift * 4) {
			isChangedByMouse_[0] = 1; isChangedByMouse_[1] = x; isChangedByMouse_[2] = y;
			changeAngle(i, angularSpeed);
		}
		if (yStart + yShift * 4 < y && y < yStart + yShift * 5) {
			isChangedByMouse_[0] = 1; isChangedByMouse_[1] = x; isChangedByMouse_[2] = y;
			changeAngle(i, -angularSpeed);
		}
	}
}
void Manipulator::stopChangeByMouse() {
	isChangedByMouse_[0] = 0;
}



//==========================================================================================================================|
//																															|
//													TREE AXIS RRR MANIPULATOR												|
//																															|
//==========================================================================================================================|

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
}

bool firstAngleCloserToThird(Angle first, Angle second, Angle third)
{
	return abs((third - first).get()) < abs((third - second).get());
}

void ThreeAxisRrrManipulator::inverseKinematic()
{
	double r1 = sqrt(sq(coords_.x) + sq(coords_.y));
	double r2 = coords_.z - d_[0];
	double r3 = sqrt(sq(r1) + sq(r2));


	if (abs((sq(a_[1]) + sq(r3) - sq(a_[2])) / (2 * a_[1] * r3)) <= 1) {
		Angle psi1 = Angle(acos((sq(a_[1]) + sq(r3) - sq(a_[2])) / (2 * a_[1] * r3)));
		Angle psi2 = Angle(atan2(r2, r1));
		if (abs((sq(a_[1]) + sq(a_[2]) - sq(r3)) / (2 * a_[1] * a_[2])) <= 1) {
			Angle psi3 = Angle(acos((sq(a_[1]) + sq(a_[2]) - sq(r3)) / (2 * a_[1] * a_[2])));
			//exeption = "OK";

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
			//exeption = "threeAxisBackwardTransferEx";
		}
	}
	else {
		//exeption = "threeAxisBackwardTransferEx";
	}
	
}

std::vector<Angle> ThreeAxisRrrManipulator::getAngles() {
	return angles_;
}

Eigen::Matrix4d ThreeAxisRrrManipulator::getH() {
	return H_[kAxis_];
}

//==========================================================================================================================|
//																															|
//												SIX AXIS STANDARD MANIPULATOR												|
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
}

std::array<Angle, 3>  findEulerAngles(Eigen::Matrix3d R) {
	//Angles around z, y', z''
	Angle z3; Angle y2; Angle z1;
	if (R(2,2) == 1) {
		y2 = Angle(0);
		z3 = Angle(0); //choose any t6
		z1 = Angle(atan2(R(1,0), R(0,0)) - z3.get());
	}
	else if (R(2,2) == -1) {
		y2 = Angle(PI);
		z3 = Angle(0); //выбираем t6 любым
		z1 = Angle(atan2(-R(0,1), -R(0,0) + z3.get()));
	}
	else {
		int sign = 1; //1 or -1
		y2 = atan2 (sign * sqrt(1 - sq(R(2,2))),   R(2,2));
		z1 = atan2 (sign * R(1,2),                 sign * R(0,2));
		z3 = atan2 (sign * R(2,1),                -sign * R(2,0)); 
	}
	std::array<Angle, 3> angles = { z1, y2, z3 };
	return angles;

}

void SixAxisStandardManipulator::inverseKinematic() 
{
	Eigen::Vector3d zv (0, 0, 1);
	Eigen::Vector3d p46 = d_[5] * getR(H_[kAxis_]) * zv;
	Point p04 = coords_ - Point(p46);

	ThreeAxisRrrManipulator firstThreeAxis = ThreeAxisRrrManipulator(l_);
	firstThreeAxis.setPosition(p04);
	std::vector<Angle> angles3 = firstThreeAxis.getAngles();
	angles_[0] = angles3[0]; angles_[1] = angles3[1]; angles_[2] = angles3[2];
	Eigen::Matrix3d R3 = getR(firstThreeAxis.getH());
	Eigen::Matrix3d R36 = R3.transpose()*getR(H_[kAxis_]);
	std::array<Angle, 3> angles36 = findEulerAngles(R36);
	angles_[3] = angles36[0]; angles_[4] = angles36[1]; angles_[5] = angles36[2];
	setAngles(angles_);
}