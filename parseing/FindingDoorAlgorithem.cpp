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
vector<Point> parsePoints(string filePath); // gets the file path for the CSV file and returns a Vector of the Points of the room
vector<Point> getMaxPoints(vector<Point>& points, Point targetP, int num); // gets a vector of points, a point to calculate distance to and a number of points to return and returns the X number of points that are furthest away for targetP
Point findAvaregePoint(vector<Point>& points); // gets a vector of points and returns the avarege point
vector<Point> getMaximumInterval(vector<Point>& points, Point targetP, int intervalSize); // gets a vector of points, a target point and an int and returns a vector of points that are continuous in the original vector that have the maximum sum of distance from targetP
Point getDoorUsingSlices(vector<Point>& points, Point targetP, int degreeOfSlice); // gets a vector of point, a target point and the degree of each slice and returns the door Point, using slices of a given degree to calculate
bool sortFunctionDistance(Point p1, Point p2); // a bool function that gets 2 points and returns true if P2 is further from (0,0,0) than P1, and false otherwise. used to sort the points
bool sortFunctionIndex(Point p1, Point p2); // a bool function that gets 2 points and returns true if P2.index > P1.index and false otherwise. used to reorder the points
void sortOutOutliers(vector<Point>& points, int minNumberOfNeighbors, double neighborDistance); // gets a vector of points, number of neighbors and distance from neighbors required to be considered a good point. the function is void but it changes the point vector to be without the outliers
void savePointToCsv(vector<Point>& points); // gets a vector of points and saves it to 2 different CSV files, 1 of the original vector and 1 of the vector without outliers
Point getMaxPoint(vector<Point>& points, Point& targetP); // gets a vector and a target point and returns the point that is furthest away from the target point

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
		if (p.getDistanceFrom(targetP) > max.getDistanceFrom(targetP)) // check if we have a new max point and if yes switch them
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

	while (getline(fin, line)) // main loop, puts in line the new line and exits the while loop when the file ends
	{
		pos = 0;
		counter = 0;
		Point point(0, 0, 0, 0);
		while ((pos = line.find(delimiter)) != string::npos) { // changing the position based to the next delimiter
			token = line.substr(0, pos);
			value = stod(token); // token is the number from the csv file so change it to double type
			if (counter == 0) // checks if the number is the 1st 2nd or 3rd number in the line
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
		point.setIndex(indexCounter); // set the index of the point to be the index that this point has in the vector
		points.push_back(point); // insert the point to the vector
		indexCounter++;
	}
	return points;
}

void savePointToCsv(vector<Point>& points)
{
	ofstream file1, file2;
	// open 2 files. one for the original map and one for the map without outliers
	file1.open("E:\\university\\summer semester 2nd year\\LAB\\FindDoorAlgorithem\\csvData\\fileAllPoints.csv");
	file2.open("E:\\university\\summer semester 2nd year\\LAB\\FindDoorAlgorithem\\csvData\\fileNoOutliers.csv");
	for (Point& p : points)
	{
		file1 << p.getX() << ',' << p.getY() << ',' << p.getZ() << '\n'; // write all the points to the first csv file
	}
	sortOutOutliers(points, 10, 0.1); // remove the outliers
	for (Point& p : points)
	{
		file2 << p.getX() << ',' << p.getY() << ',' << p.getZ() << '\n'; // write all the points without the outliers to the second csv file
	}
}

vector<Point> getMaxPoints(vector<Point>& points, Point targetP, int num)
{
	vector<Point> maxPoints(num);
	Point tmp1, tmp2;
	for (int i = 0; i < points.size(); i++)
	{
		tmp1 = points[i];
		for (int j = 0; j < num; j++) // compares the point to all the points in maxPoints
		{
			if (tmp1.getDistanceFrom(targetP) > maxPoints[j].getDistanceFrom(targetP))  // switch between the points if we need to
			{
				tmp2 = maxPoints[j];
				maxPoints[j] = tmp1;
				tmp1 = tmp2;
			}
		}
	}
	return maxPoints;
}

Point findAvaregePoint(vector<Point>& points)
{
	Point p;
	double x = 0, y = 0, z = 0;
	for (int i = 0; i < points.size(); i++) // add all the values of the Points
	{
		x += points[i].getX();
		y += points[i].getY();
		z += points[i].getZ();
	}
	// makes them avarege values
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
	for (int i = 0; i < points.size() - intervalSize + 1; i++) // go over all the points in the map
	{
		tmpInterval = 0;
		for (int j = i; (j - i) < intervalSize; j++) // calculate the current interval total distance
		{
			tmpInterval += points[j].getDistanceFrom(targetP);
		}
		if (tmpInterval > sumMaxInterval) // if the current intervalis bigger than the max interval switch them
		{
			sumMaxInterval = tmpInterval;
			startingIntervalIndex = i;
		}
	}
	for (int i = startingIntervalIndex; i - startingIntervalIndex < intervalSize; i++) // push back the points from the points vector to a new interval vector
	{
		interval.push_back(points[i]);
	}
	return interval;
}

Point getDoorUsingSlices(vector<Point>& points, Point targetP, int degreeOfSlice)
{
	vector<vector<Point>> slices(360 / degreeOfSlice); // create a new vector of vectors of points, 1 vector of points for each degree of slice we want
	vector<Point> door;
	double angle, maxSliceDistance = 0, sliceDist;
	int degree;
	int maxSliceIndex = 0, i = 0;
	for (Point& p : points) // go over all the points in the vector and insert each point to the right vector
	{
		angle = atan2(p.getX() - targetP.getX(), p.getZ() - targetP.getZ());
		angle = angle * 180 / M_PI;
		angle += 180;
		degree = floor(angle / (360 / degreeOfSlice));
		slices[degree].push_back(p);
	}
	for (auto& slice : slices) // calculate the slice that has the maximum avarege distance from targetP
	{
		sliceDist = 0;
		for (Point& p : slice)
		{
			sliceDist += p.getDistanceFrom(targetP);
		}
		sliceDist = sliceDist / slice.size();
		if (sliceDist > maxSliceDistance) // if we have a new max slice switch them
		{
			maxSliceIndex = i;
			maxSliceDistance = sliceDist;
		}
		i++;
	}
	door = getMaxPoint(slices[maxSliceIndex], targetP); // return the maximum point in the maximum slice
	return door;
}

void sortOutOutliers(vector<Point>& points, int minNumberOfNeighbors, double neighborDistance)
{
	int numOfNeighbors;
	double currentDistance;
	Point p;
	vector<Point> goodPoints;
	sort(points.begin(), points.end(), sortFunctionDistance); // sort the points vector by distance from (0, 0, 0)
	for (int i = 0; i < points.size(); i++) // go over all the points and count how many neighbors each point has
	{
		numOfNeighbors = 0;
		currentDistance = points[i].getDistanceFrom(p);
		for (int j = i + 1; j < points.size(); j++) // check all the points with indexes that are bigger than i, until we break. this way we don't need to check all the points so the time complexity goes down from n^2 to O(n) in a very good probability
		{
			if (std::abs(currentDistance - points[j].getDistanceFrom(p)) > neighborDistance) // if the absolute difference between the current point distance to (0, 0, 0) and the candidate distance from (0, 0, 0) is bigger than the treshhold we have for maximum neighbor distance than the candidate can't be a neighbor and we need to break because it is guaranteed that every point with a bigger index can't be a neighbor either
				break;
			if (points[i].getDistanceFrom(points[j]) < neighborDistance) // count the candidate as a neighbor if the distance between both points is less than the threshhold
				numOfNeighbors++;
		}
		for (int j = i - 1; j >= 0; j--) // do exactly the same as the above for loop but for all the smaller indexes
		{
			if (std::abs(currentDistance - points[j].getDistanceFrom(p)) > neighborDistance)
				break;
			if (points[i].getDistanceFrom(points[j]) < neighborDistance)
				numOfNeighbors++;
		}

		if (numOfNeighbors >= minNumberOfNeighbors) // if the point dosen't have enough neighbors than we remove the point
			goodPoints.push_back(points[i]);
	}
	sort(goodPoints.begin(), goodPoints.end(), sortFunctionIndex); // because we sorted the original vector we want to return the points to the original order so we sort out the vector again using the indexes
	points = goodPoints; // copy the good points vector to the original one, we do this because erease is an expensive action so this is better for time complexity
}

bool sortFunctionDistance(Point p1, Point p2)
{
	Point z;
	return (p1.getDistanceFrom(z) < p2.getDistanceFrom(z)); // return true if p2 is further from (0, 0, 0) than p1
}

bool sortFunctionIndex(Point p1, Point p2)
{
	return p1.getIndex() < p2.getIndex(); // return true if p2 index is bigger than p1
}
