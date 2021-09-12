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
vector<Point> getDoorPoint(vector<Point>& points, Point targetP);
Point cleanPoints(vector<Point>& maxPoints);

int main()
{
    Point p(0, 0, 0);
    vector<Point> points = parsePoints("data\\PointData.csv");
    vector<Point> maxPoints = getDoorPoint(points, p);
    Point door = cleanPoints(maxPoints);
    cout << door << endl;
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

vector<Point> getDoorPoint(vector<Point>& points, Point targetP)
{
    vector<Point> maxPoints(50);
    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < 50; j++)
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
    return Point(x/50, 0, z/50);
}


