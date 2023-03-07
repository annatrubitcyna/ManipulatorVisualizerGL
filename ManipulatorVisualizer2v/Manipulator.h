#include <stdio.h>
#include <vector>
#include <glm.hpp>
#include <Eigen/Dense>



const double PI = 3.141592653589793238463;

class Point {
public:
	double x;
	double y;
	double z;
	Point() : x(0), y(0), z(0) {}
	//void set(double x, double y, double z);
};

class Angle { 
private:
	double value_;
public:
	Angle(double value);
	double toAngleFormat(Angle angle);
	double get();
	double getInDeg();
	void set(double angle);
	void setInDeg(double angle);
};

enum ForwardKinematicsMethod {
	DH, //the Denavit-Hartenberg method
	EXP //matrix exponent product method
};

class Manipulator {
protected:
	//main parameters
	int kAxis_;
	ForwardKinematicsMethod forwardKinematicsMethod;

	// Denavit-Hartenberg parameters
	std::vector<double> a_; //kAxis, distance along xi from zi-1 to zi 
	std::vector<double> alpha_; //kAxis, angle around xi from zi-1 to zi
	std::vector<double> d_; //kAxis, distance along zi-1 from xi-1 to xi

	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> J; //kAxis, kAxis

	// matrix exponent product method parameters
	std::vector<Eigen::Vector<double, Eigen::Dynamic>> unitTwists_; //kAAxis, kAxis, 1; [vx, vy, vz, r^w+lw], |v|=1, |w|=1
	std::vector<Eigen::Matrix4d> H0_; //kAxis; homogeneous transformation matrices between coordinate systems in start position

	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> geomJ; //kAxis, kAxis

	//changing parameters
	Point coords_;
	std::vector<Point> joints_;
	std::vector<Angle> angles_; //angle around zi-1 from xi-1 to xi
	std::vector<Eigen::Matrix4d> H_; //homogeneous transformation matrices (kAxis)

	//flags
	bool isAnglesChanged_;

	//physics


	void forwardKinematicDH();
	void forwardKinematicEXP();
	void inverseKinematic(); //different to different types of manipulator

	void drawCoordSystem(int i);

public:
	Manipulator(int kAxis, std::vector<double> a, std::vector<double> alpha, std::vector<double> d);
	Manipulator(int kAxis, std::vector<Eigen::Vector<double, Eigen::Dynamic>> unitTwists, std::vector<Eigen::Matrix4d> H0);

	void setPosition(Point coords, glm::dmat3x3 R);
	void setPosition(Point coords);
	void setAngles(std::vector<double> angles);

	void drawManipulator();
	void drawCoordSystems();
	void drawOrientation();
	void drawAngles();

	//void drawAngles(); only for 6-axis manipulator
};

