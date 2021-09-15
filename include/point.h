// a class for the points in the map

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
	// declaring constructors and distructors
	Point();
	Point(double x, double y, double z);
	Point(double x, double y, double z, int index);
	Point(const Point& other);
	~Point() {};

	// overiding operator = so we can work with the Point and operator << so we can print the Point
	Point& operator= (const Point& other);
	friend std::ostream& operator<<(std::ostream& os, const Point& p);

	// declaring getters and setters
	double getX();
	double getY();
	double getZ();
	int getIndex();

	Point& setX(double x);
	Point& setY(double y);
	Point& setZ(double z);
	Point& setIndex(int index);
	Point& setPoint(double x, double y, double z, int index);

	// a distance function for the calculations
	double getDistanceFrom(const Point& p1);
};

#endif
