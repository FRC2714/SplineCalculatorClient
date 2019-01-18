#include "SplineFactory.h"

SplineFactory::SplineFactory(Point p0, Point p1, Point p2, Point p3)
{
	//Initialize variables
	this->p0 = p0;
	this->p1 = p1;
	this->p2 = p2;
	this->p3 = p3;
}

void SplineFactory::calculate(MotionData data, vector<MotionPose> * controlPath) {
	data.acc *= PERIOD;
	data.vel *= PERIOD;
	double frontT = 1, backT = 0;
	double frontV = data.startV * PERIOD;
	double backV = data.endV * PERIOD;

	for (int i = 0; frontT < backT; i++) {
		if(frontV < data.vel) frontV += data.acc * PERIOD;
		if(backV < data.vel) backV += data.acc * PERIOD;

		frontT = binaryFind(frontT, frontV, i);
		backT = binaryFind(backT, -backV, i + 1);
	}

	for (int i = 0; i < calculatedPoints.size() - 1; i++) {
		double angle;
		
		Point change = {
			calculatedPoints.at(i + 1).x - calculatedPoints.at(i).x,
			calculatedPoints.at(i + 1).y - calculatedPoints.at(i).y
		};

		if (change.y == 0) {
			if (change.x > 0) angle = 0;
			else angle = 180;
		} else if (change.x == 0) {
			if (change.y > 0) angle = 90;
			else angle = 270;
		}

		if (change.x < 0)
			angle = atan2(change.y, change.x) / PI * 180 + 180;
		else if(change.y > 0)
			angle = atan2(change.y, change.x) / PI * 180;
		else
			angle = atan2(change.y, change.x) / PI * 180 + 3600;

		double velocity = pointDistance(calculatedPoints.at(i + 1), calculatedPoints.at(i)) / PERIOD;

		if (!data.forwards) {
			velocity *= -1;
			angle += 180;
			if (angle > 360) angle -= 360;
		}

		MotionPose mp = { angle, velocity, calculatedPoints.at(i) };
		controlPath->push_back(mp);
	}
}

SplineFactory::~SplineFactory()
{

}

double SplineFactory::binaryFind(double startT, double dist, int loc)
{
	double tStepMod = T_STEP;
	double inv = 1;
	double dir = 1, prevDir = 1;
	if (dist < 0)
		inv = dir = prevDir = -1;

	Point start = cubicCalc(startT), newPoint;

	double cDist = 0;
	double tDist = abs(dist);
	double dDist = tDist - cDist; //TODO remove cDist if safe

	do {
		if (dDist * inv < 0) {
			dir = -1;
			if (dir != prevDir) tStepMod /= 2;
		} else {
			dir = 1;
			if (dir != prevDir) tStepMod /= 2; //TODO remove nested conditionals
		}
		
		startT += tStepMod * dir;

		newPoint = cubicCalc(startT);

		cDist = pointDistance(start, newPoint);

		prevDir = dir;
		dDist = tDist - cDist;
	} while (abs(dDist) > TOLERANCE);

	calculatedPoints.push_back(newPoint);

	return startT;
}

Point SplineFactory::cubicCalc(double t) {
	Point p = {
		pow(1 - t, 3) * p0.x +
		3 * pow(1 - t, 2) * t * p1.x +
		3 * (1 - t) * t * t * p2.x +
		3 * t * t * t * p3.x,
		pow(1 - t, 3) * p0.y +
		3 * pow(1 - t, 2) * t * p1.y +
		3 * (1 - t) * t * t * p2.y +
		3 * t * t * t * p3.y
	};
	return p;
}

double SplineFactory::pointDistance(Point p1, Point p2) {
	return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}