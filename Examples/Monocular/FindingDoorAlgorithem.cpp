// FindingDoorAlgorithem.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include "point.h"

using namespace std;
vector<Point> parsePoints(string filePath);

int main()
{
    vector<Point> points = parsePoints("data\\PointData.csv");

}

vector<Point> parsePoints(string filePath)
{

    // File pointer
    fstream fin;

    // Open an existing file
    fin.open(filePath, ios::in);

    // Read the Data from the file
    // as String Vector
    vector<Point> points;
    string line, token;
    string delimiter = ",";
    size_t pos;
    int counter, n;
    double value;

    points.clear();

    while (getline(fin, line))
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

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
