class Manipulator {
private:
	double parameters;
	int kAxis;
	// DEnavit-Hartenberg parameters
	/*double[2] a;
	double[] alpha;
	double[] d;*/
public:
	Manipulator() : parameters(0.0) {};
	void drawManipulator();
};