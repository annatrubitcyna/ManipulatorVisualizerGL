#include <stdio.h>
#include <vector>
#include <GL/glut.h>
//#include <glm.hpp>
#include <Eigen/Dense>
#include "font.h"




const double PI = 3.141592653589793238463;

double toRadians(double angle);

double toDegrees(double angle);


enum COLOR {
	RED,
	GREEN,
	BLUE,
	YELLOW,
	GREEN_BLUE,
	PINK,
	ORANGE,
	VIOLET,
	WHITE,
	GREEN_YELLOW
};

GLfloat* getColor(COLOR i);

Eigen::Matrix3d getR(Eigen::Matrix4d H);

class Point {
public:
	double x;
	double y;
	double z;
	Point() {};
	Point(double x, double y, double z);
	Point(Eigen::Vector4d point);
	Point(Eigen::Vector3d point);
	Eigen::Vector3d toVector();
	//void set(double x, double y, double z);
};

class Angle { 
private:
	double value_;
public:
	Angle() {};
	Angle(double value);
	//double toAngleFormat(Angle angle);
	double get();
	double getInDeg();
	void set(double angle);
	void setInDeg(double angle);
};

std::array<Angle, 3>  findEulerAngles(Eigen::Matrix3d R);

class differentArraySizeException {};

enum ForwardKinematicsMethod {
	DH, //the Denavit-Hartenberg method
	EXP //matrix exponent product method
};

enum Error {
	OK,               //no error 
	OUT_OF_WORKSPACE,  //can't solve inverse kinematics problem 
	JACOBIAN_DEGENERATION
};

//class Tables {
//	Table angularTable;
//	Table coordsTable;
//	Table eulerAnglesTable;
//	Table orientationTable;
//	Table functionTable;
//	Tables();
//};


class Manipulator {
protected:
	//main parameters
	int kAxis_;
	int kJoints_;
	ForwardKinematicsMethod forwardKinematicsMethod_;

	// Denavit-Hartenberg parameters
	std::vector<double> a_; //kAxis, distance along xi from zi-1 to zi 
	std::vector<double> alpha_; //kAxis, angle around xi from zi-1 to zi
	std::vector<double> d_; //kAxis, distance along zi-1 from xi-1 to xi
	std::vector<double> dTh_; //kAxis, if angle around zi-1 from xi-1 to xi is not angle that we set

	// matrix exponent product method parameters
	std::vector<Eigen::Vector<double, 6>> unitTwists_; //kAAxis, 1; [vx, vy, vz, r^w+lw], |v|=1, |w|=1
	std::vector<Eigen::Matrix4d> Hiim1T0_; //kAxis; homogeneous transformation matrices between coordinate systems i and i-1 in start position

	//changing parameters
	Point coords_;
	std::vector<Point> joints_;
	std::vector<Angle> angles_; //angle around zi-1 from xi-1 to xi
	std::vector<Eigen::Matrix4d> H_; //homogeneous transformation matrices (kAxis)
	Eigen::Matrix3d R_; //rotation matrix for grip
	std::array<Angle, 3> EulerAngles_;
	Error error_;
	
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> J_; // DH kAxis, kAxis
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> geomJ_; //EXP kAxis, kAxis

	Table angleTable_;
	Table coordTable_;
	Table eulerAngleTable_;
	Table orientationTable_;
	Table functionTable_;
	bool areTablesInit;

	bool isCubePressed;
	std::vector<int> isChangedByMouse_; //1 or 0- yes or not, 2-only once ; x y mouse coords 
	int isGoWithSpeed_; //1-goWithSpeed, 2-goWithAngularSpeed; 
	Point targetCoords_;
	float speed_;
	double prTime_;
	int isGoByGcodes_;
	Point previousCoords_;
	std::vector<float> parsePoint_;
	std::vector<Point> gcodeTrajectory_;

	void initializeVectorsAsNull();
	void setAngles(std::vector<Angle> angles);

	void forwardKinematics();
	void forwardKinematicsDH();
	void forwardKinematicsEXP();
	virtual void inverseKinematics(); //different to different types of manipulator

	void countJacobian();
	void countGeomJacobian();

	void initAngleTable();
	void initCoordTable();
	void initOrientationTables();
	void initFunctionTable();
	void initTables();

	void printAngles();
	void printCoords();
	void printOrientation();
	void printFunctions();
	void checkStartingPosition(std::vector<Angle> angles);
	void drawOrientationCubes();

	std::vector<float> parse(std::string line);
	void changeGoByGcode();
	void goByGCODE();
	void changeGoWithSpeed(Point targetCoords, float speed);
	void goWithSpeed(Point targetCoords, float speed);
	void changeGoWithAngularSpeed(Point targetCoords, float speed);
	void goWithAngularSpeed(Point targetCoords, float speed);
public:
	Manipulator() {};
	Manipulator(int kAxis, std::vector<double> a, std::vector<double> alpha, std::vector<double> d, std::vector<double> dTh);
	Manipulator(int kAxis, std::vector<Eigen::Vector<double, 6>> unitTwists, std::vector<Eigen::Matrix4d> Hiim1T0);

	void changeBaseCoords(Point baseCoords);
	void changeAngle(int i, double aAngle);
	void changePosition(Point dCoords);
	void changeOrientation(int axis, float dAngle);
	void setPosition(Point coords, Eigen::Matrix3d R); //only for manipulators with inverse 
	void setPosition(Point coords);
	void setAngles(std::vector<double> angles);
	void setOrientation(Eigen::Matrix3d R);
	void goToStartingPosition();

	void drawManipulator();
	void drawCoordSystem(int i, double axisLen);
	void drawCoordSystems();
	void drawOrientation();
	void drawAngles();

	void drawBaseCoordSystem();

	//void drawAnglesMoreUnderstandable(); only for 6-axis manipulator

	void printInfo();
	void printSimpleInfo();

	int changeByMouse(float x, float y);
	void stopChangeByMouse();
};


class ThreeAxisRrrManipulator : public Manipulator
{
protected:
	std::vector<double> l_;
	void inverseKinematics();
public:
	ThreeAxisRrrManipulator();
	ThreeAxisRrrManipulator(std::vector<double> l);
	ThreeAxisRrrManipulator(std::vector<double> a, std::vector<double> alpha, std::vector<double> d, std::vector<double> dTh) :
		Manipulator(3, a, alpha, d, dTh) {};
	ThreeAxisRrrManipulator(int kAxis, std::vector<Eigen::Vector<double, 6>> unitTwists, std::vector<Eigen::Matrix4d> Hiim1T0) :
		Manipulator(3, unitTwists, Hiim1T0) {};

	std::vector<Angle> getAngles();
	Eigen::Matrix4d getH();
	Error getError();
};


class SixAxisStandardManipulator : public Manipulator
{
protected:
	std::vector<double> l_; //linkLength
	void inverseKinematics();
	ThreeAxisRrrManipulator firstThreeAxis;
public:
	SixAxisStandardManipulator(ForwardKinematicsMethod method, std::vector<double> linkLength);
};