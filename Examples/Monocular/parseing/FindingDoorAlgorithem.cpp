// FindingDoorAlgorithem.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "point.h"
#include <algorithm>


using namespace std;
vector<Point> parsePoints(string filePath);
vector<Point> getMaxPoints(vector<Point>& points, Point targetP, int num);
Point cleanPoints(vector<Point>& maxPoints);
int getNeighbors(vector<Point>& points, Point& p);

int main()
{
	int doorPointIndex = 0;
	int number = 10;
	double angle;
	Point p(0, 0, 0);
	vector<Point> points = parsePoints("data\\pointDataVered.csv");
	vector<Point> maxPoints = getMaxPoints(points, p, number);
	int maxNeighbors = getNeighbors(points, maxPoints[0]);
	for (int i = 1; i < number; i++)
	{
		if (getNeighbors(points, maxPoints[i]) > maxNeighbors)
			doorPointIndex = i;
	}
	//Point t = cleanPoints(maxPoints);
	//angle = atan2(0, 0);
	cout << maxPoints[doorPointIndex] << endl;
}

vector<Point> parsePoints(string filePath)
{

	// File pointer
	fstream fin;

	// Open an existing file
	fin.open(filePath, ios::in);

	vector<Point> points;
	string line, token;
	string delimiter = ",";
	size_t pos;
	int counter;
	double value;

	points.clear();

	while (getline(fin, line)) // main loop
	{
		pos = 0;
		counter = 0;
		Point point(0, 0, 0);
		while ((pos = line.find(delimiter)) != string::npos) {
			token = line.substr(0, pos);
			value = stod(token);
			if (counter == 0)
			{
				point.setX(value);
			}
			else if (counter == 1)
			{
				point.setY(value);
			}
			line.erase(0, pos + delimiter.length());
			counter++;
		}
		value = stod(line);
		point.setZ(value);
		points.push_back(point);
	}
	return points;
}

vector<Point> getMaxPoints(vector<Point>& points, Point targetP, int num)
{
	vector<Point> maxPoints(num);
	for (int i = 0; i < points.size(); i++)
	{
		for (int j = 0; j < num; j++)
		{
			if (points[i].getDistanceFrom(targetP) > maxPoints[j].getDistanceFrom(targetP))
			{
				maxPoints[j] = points[i];
				break;
			}
		}
	}
	return maxPoints;
}


Point cleanPoints(vector<Point>& maxPoints)
{
	double x = 0, z = 0;
	for (int i = 0; i < maxPoints.size(); i++)
	{
		x += maxPoints[i].getX();
		z += maxPoints[i].getZ();
	}
	return Point(x / maxPoints.size(), 0, z / maxPoints.size());
}

int getNeighbors(vector<Point>& points, Point& p)
{
	int counter = 0;
	for (int i = 0; i < points.size(); i++)
	{
		if (points[i].getDistanceFrom(p) < 0.1)
		{
			counter++;
		}
	}
	return counter - 1;
}
