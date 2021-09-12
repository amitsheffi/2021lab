#include "point.h"
// Point constructor
Point::Point()
{
    this->x = 0;
    this->y = 0;
    this->z = 0;
}
Point::Point(double x, double y, double z)
{
    this->x = x;
    this->y = y;
    this->z = z;
}
Point::Point(const Point& other)
{
    this->x = other.x;
    this->y = other.y;
    this->z = other.z;
}
Point& Point::operator=(const Point& other)
{
    if (this == &other)
        return *this;
    this->x = other.x;
    this->y = other.y;
    this->z = other.z;
    return *this;
}
std::ostream& operator<<(std::ostream& os, const Point& p)
{
    std::cout << "X:" << p.x << " Y:" << p.y << " Z:" << p.z;
    return os;
}
// implement getters
double Point::getX()
{
    return this->x;
}
double Point::getY()
{
    return this->y;
}
double Point::getZ()
{
    return this->z;
}
// implement setters
Point& Point::setX(double x)
{
    this->x = x;
    return *this;
}
Point& Point::setY(double y)
{
    this->y = y;
    return *this;
}
Point& Point::setZ(double z)
{
    this->z = z;
    return *this;
}
Point& Point::setPoint(double x, double y, double z)
{
    this->x = x;
    this->y = y;
    this->z = z;
    return *this;
}
// implement distance function
double Point::getDistanceFrom(const Point& p1)
{
    return ((p1.x - this->x) * (p1.x - this->x) + (p1.z - this->z) * (p1.z- this->z));
}