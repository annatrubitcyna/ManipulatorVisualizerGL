#include <stdio.h>
#include <vector>
#include <glm.hpp>
#include <Eigen/Dense>



const double PI = 3.141592653589793238463;

double toRadians(double angle);

double toDegrees(double angle);

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

GLfloat* getColor(COLOR i);

Eigen::Matrix3d getR(Eigen::Matrix4d H);

class Point {
public:
	double x;
	double y;
	double z;
	Point(double x, double y, double z);
	Point(Eigen::Vector4d point);
	Point(Eigen::Vector3d point);

	//void set(double x, double y, double z);
};

class Angle { 
private:
	double value_;
public:
	Angle(double value);
	//double toAngleFormat(Angle angle);
	double get();
	double getInDeg();
	void set(double angle);
	void setInDeg(double angle);
};

class differentArraySizeException {};

enum ForwardKinematicsMethod {
	DH, //the Denavit-Hartenberg method
	EXP //matrix exponent product method
};

class Manipulator {
protected:
	//main parameters
	int kAxis_;
	ForwardKinematicsMethod forwardKinematicsMethod_;

	// Denavit-Hartenberg parameters
	std::vector<double> a_; //kAxis, distance along xi from zi-1 to zi 
	std::vector<double> alpha_; //kAxis, angle around xi from zi-1 to zi
	std::vector<double> d_; //kAxis, distance along zi-1 from xi-1 to xi

	// matrix exponent product method parameters
	std::vector<Eigen::Vector<double, 6>> unitTwists_; //kAAxis, 1; [vx, vy, vz, r^w+lw], |v|=1, |w|=1
	std::vector<Eigen::Matrix4d> Hiim1T0_; //kAxis; homogeneous transformation matrices between coordinate systems i and i-1 in start position

	//changing parameters
	Point coords_;
	std::vector<Point> joints_;
	std::vector<Angle> angles_; //angle around zi-1 from xi-1 to xi
	std::vector<Eigen::Matrix4d> H_; //homogeneous transformation matrices (kAxis)
	Eigen::Matrix3d R_; //rotation matrix for grip
	
	//Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> J; // DH kAxis, kAxis
	//Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> geomJ; //EXP kAxis, kAxis

	////flags
	//bool isAnglesChanged_;

	//physics

	void initializeVectorsAsNull();

	void forwardKinematicDH();
	void forwardKinematicEXP();
	void inverseKinematic(); //different to different types of manipulator

public:
	Manipulator(int kAxis, std::vector<double> a, std::vector<double> alpha, std::vector<double> d, Point baseCoords, std::vector<double> angles);
	Manipulator(int kAxis, std::vector<double> a, std::vector<double> alpha, std::vector<double> d, Point baseCoords, Point coords, Eigen::Matrix3d R); //only for manipulators with inverse 
	Manipulator(int kAxis, std::vector<Eigen::Vector<double, 6>> unitTwists, std::vector<Eigen::Matrix4d> Hiim1T0, Point baseCoords, std::vector<double> angles);
	Manipulator(int kAxis, std::vector<Eigen::Vector<double, 6>> unitTwists, std::vector<Eigen::Matrix4d> Hiim1T0, Point baseCoords, Point coords, Eigen::Matrix3d R);

	void changeAngle(int i, double aAngle);
	void setPosition(Point coords, Eigen::Matrix3d R);
	void setPosition(Point coords);
	void setAngles(std::vector<double> angles);

	void drawManipulator();
	void drawCoordSystem(int i, double axisLen);
	void drawCoordSystems();
	void drawOrientation();
	void drawAngles();

	void drawBaseCoordSystem();

	//void drawAngles(); only for 6-axis manipulator
};

//abstract class Manipulator manipulators with inverse kinematic