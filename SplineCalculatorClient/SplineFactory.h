#pragma once

#include <vector>

#define PI 3.14159265358979323846

using namespace std;

struct Point {
	double x;
	double y;
};

struct MotionData {
	double acc;
	double vel;
	double startV;
	double endV;
	bool forwards;
};

struct MotionPose {
	double angle;
	double velocity;
	Point loc;
};

class SplineFactory
{
public:
	Point p0, p1, p2, p3;
	const double TOLERANCE = 1e-5;
	const double T_STEP = 1e-3;
	const double PERIOD = 5e-4;

	vector<Point> calculatedPoints;
	
	SplineFactory(Point, Point, Point, Point);
	~SplineFactory();

	void calculate(MotionData, vector<MotionPose> *);
	double binaryFind(double, double, int);
	Point cubicCalc(double);
	double pointDistance(Point, Point);
};


