#include <iostream>
#include <ostream>
#ifndef POINT_H
#define POINT_H

class Point
{
private:
	double x;
	double y;
	double z;

public:
	Point();
	Point(double x, double y, double z);
	Point(const Point& other);
	~Point() {};

	Point& operator= (const Point& other);
	friend std::ostream& operator<<(std::ostream& os, const Point& p);

	double getX();
	double getY();
	double getZ();

	Point& setX(double x);
	Point& setY(double y);
	Point& setZ(double z);
	Point& setPoint(double x, double y, double z);

	double getDistanceFrom(const Point& p1);
};

#endif