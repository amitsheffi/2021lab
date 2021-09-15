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
	int index;

public:
	Point();
	Point(double x, double y, double z);
	Point(double x, double y, double z, int index);
	Point(const Point& other);
	~Point() {};

	Point& operator= (const Point& other);
	friend std::ostream& operator<<(std::ostream& os, const Point& p);

	double getX();
	double getY();
	double getZ();
	int getIndex();

	Point& setX(double x);
	Point& setY(double y);
	Point& setZ(double z);
	Point& setIndex(int index);
	Point& setPoint(double x, double y, double z, int index);

	double getDistanceFrom(const Point& p1);
};

#endif