// FindingDoorAlgorithem.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "point.h"
#include <algorithm>
#include <math.h>

using namespace std;
vector<Point> parsePoints(string filePath);
vector<Point> getMaxPoints(vector<Point>& points, Point targetP, int num);
Point cleanPoints(vector<Point>& maxPoints);
int getNeighbors(vector<Point>& points, Point& p);
Point findAvaregePoint(vector<Point>& points);
vector<Point> getMaximumInterval(vector<Point>& points, Point targetP, int intervalSize);
Point getDoorUsingSlices(vector<Point>& points, Point targetP, int degreeOfSlice);
bool sortFunctionDistance(Point p1, Point p2);
bool sortFunctionIndex(Point p1, Point p2);
void sortOutOutliers(vector<Point>& points, Point targetP, int minNumberOfNeighbors, double neighborDistance);
void savePointToCsv(vector<Point>& points);
Point getMaxPoint(vector<Point>& points, Point& targetP);

int main()
{
	Point p(0, 0, 0, 0);
	vector<Point> points = parsePoints("data\\pointDataVered.csv");
	savePointToCsv(points);
}

Point getMaxPoint(vector<Point>& points, Point& targetP)
{
	Point max(0, 0, 0);
	for (Point p : points)
	{
		if (p.getDistanceFrom(targetP) > max.getDistanceFrom(targetP))
			max = p;
	}
	return max;
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
	int counter, indexCounter = 0;
	double value;

	points.clear();

	while (getline(fin, line)) // main loop
	{
		pos = 0;
		counter = 0;
		Point point(0, 0, 0, 0);
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
		point.setIndex(indexCounter);
		points.push_back(point);
		indexCounter++;
	}
	return points;
}

void savePointToCsv(vector<Point>& points)
{
	ofstream file1, file2;
	Point z(0, 0, 0);
	file1.open("E:\\university\\summer semester 2nd year\\LAB\\FindDoorAlgorithem\\csvData\\fileAllPoints.csv");
	file2.open("E:\\university\\summer semester 2nd year\\LAB\\FindDoorAlgorithem\\csvData\\fileNoOutliers.csv");
	for (Point& p : points)
	{
		file1 << p.getX() << ',' << p.getY() << ',' << p.getZ() << '\n';
	}
	sortOutOutliers(points, z, 10, 0.1);
	for (Point& p : points)
	{
		file2 << p.getX() << ',' << p.getY() << ',' << p.getZ() << '\n';
	}
}

vector<Point> getMaxPoints(vector<Point>& points, Point targetP, int num)
{
	vector<Point> maxPoints(num);
	Point tmp1, tmp2;
	for (int i = 0; i < points.size(); i++)
	{
		tmp1 = points[i];
		for (int j = 0; j < num; j++)
		{
			if (tmp1.getDistanceFrom(targetP) > maxPoints[j].getDistanceFrom(targetP))
			{
				tmp2 = maxPoints[j];
				maxPoints[j] = tmp1;
				tmp1 = tmp2;
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

Point findAvaregePoint(vector<Point>& points)
{
	Point p;
	double x = 0, y = 0, z = 0;
	for (int i = 0; i < points.size(); i++)
	{
		x += points[i].getX();
		y += points[i].getY();
		z += points[i].getZ();
	}
	x = x / points.size();
	y = y / points.size();
	z = z / points.size();
	p.setX(x);
	p.setY(y);
	p.setZ(z);
	return p;
}

vector<Point> getMaximumInterval(vector<Point>& points, Point targetP, int intervalSize)
{
	int startingIntervalIndex = 0;
	double sumMaxInterval = 0, tmpInterval;
	vector<Point> interval;
	for (int i = 0; i < points.size() - intervalSize + 1; i++)
	{
		tmpInterval = 0;
		for (int j = i; (j - i) < intervalSize; j++)
		{
			tmpInterval += points[j].getDistanceFrom(targetP);
		}
		if (tmpInterval > sumMaxInterval)
		{
			sumMaxInterval = tmpInterval;
			startingIntervalIndex = i;
		}
	}
	for (int i = startingIntervalIndex; i - startingIntervalIndex < intervalSize; i++)
	{
		interval.push_back(points[i]);
	}
	return interval;
}

Point getDoorUsingSlices(vector<Point>& points, Point targetP, int degreeOfSlice)
{
	vector<vector<Point>> slices(360 / degreeOfSlice);
	vector<Point> door;
	double angle, maxSliceDistance = 0, sliceDist;
	int degree;
	int maxSliceIndex = 0, i = 0;
	for (Point& p : points)
	{
		angle = atan2(p.getX() - targetP.getX(), p.getZ() - targetP.getZ());
		angle = angle * 180 / M_PI;
		angle += 180;
		degree = floor(angle / (360 / degreeOfSlice));
		slices[degree].push_back(p);
	}
	for (auto& slice : slices)
	{
		sliceDist = 0;
		for (Point& p : slice)
		{
			sliceDist += p.getDistanceFrom(targetP);
		}
		sliceDist = sliceDist / slice.size();
		if (sliceDist > maxSliceDistance)
		{
			maxSliceIndex = i;
			maxSliceDistance = sliceDist;
		}
		i++;
	}
	door = getMaxPoints(slices[maxSliceIndex], targetP, 1);
	return door[0];
}

void sortOutOutliers(vector<Point>& points, Point targetP, int minNumberOfNeighbors, double neighborDistance)
{
	int numOfNeighbors;
	double currentDistance;
	Point p;
	vector<Point> goodPoints;
	sort(points.begin(), points.end(), sortFunctionDistance);
	for (int i = 0; i < points.size(); i++)
	{
		numOfNeighbors = 0;
		currentDistance = points[i].getDistanceFrom(p);
		for (int j = i + 1; j < points.size(); j++)
		{
			if (std::abs(currentDistance - points[j].getDistanceFrom(p)) > neighborDistance)
				break;
			if (points[i].getDistanceFrom(points[j]) < neighborDistance)
				numOfNeighbors++;
		}
		for (int j = i - 1; j >= 0; j--)
		{
			if (std::abs(currentDistance - points[j].getDistanceFrom(p)) > neighborDistance)
				break;
			if (points[i].getDistanceFrom(points[j]) < neighborDistance)
				numOfNeighbors++;
		}

		if (numOfNeighbors >= minNumberOfNeighbors)
			goodPoints.push_back(points[i]);
		
	}
	sort(goodPoints.begin(), goodPoints.end(), sortFunctionIndex);
	points = goodPoints;
	//cout << goodPoints.size() << endl;
}

bool sortFunctionDistance(Point p1, Point p2)
{
	Point z;
	return (p1.getDistanceFrom(z) < p2.getDistanceFrom(z));
}

bool sortFunctionIndex(Point p1, Point p2)
{
	return p1.getIndex() < p2.getIndex();
}