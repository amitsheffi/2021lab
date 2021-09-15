/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include <unistd.h>
#include<opencv2/core/core.hpp>
#include <thread>
#include<System.h>
#include <Converter.h>
#include "ctello.h"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include <point.h>
#include <math.h>

using namespace std;
using ctello::Tello;
using cv::CAP_FFMPEG;
using cv::imshow;
using cv::line;
using cv::Mat;
using cv::Point2i;
using cv::resize;
using cv::Size;
using cv::Vec3b;
using cv::VideoCapture;
using cv::waitKey;

const char* const TELLO_STREAM_URL{"udp://0.0.0.0:11111"};
volatile bool inThreadMove, inThreadFrame;
bool video;
Tello tello{};
Mat im;
bool localized = false;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

void saveMap(ORB_SLAM2::System& SLAM);

// all of the algorithm functions
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

Point findingDoorPoint(); // returns the point of the door

// a thread that controls the Drone movement
void MoveDrone()
{
	while(!video) // wait for the capture before you take off
		sleep(0.01);
	tello.SendCommand("takeoff");
	while(!(tello.ReceiveResponse()));
	sleep(0.1);
	tello.SendCommand("down 15");
	while(!(tello.ReceiveResponse()));
	sleep(0.1);
	sleep(5); // it takes a while before a picture is shown so wait 5 sec
	while(!localized) // while we wait for the frames move up and down so the drone can get localized
	{
		tello.SendCommand("up 15");
		while(!(tello.ReceiveResponse()));
		sleep(0.1);
		tello.SendCommand("down 15");
		while(!(tello.ReceiveResponse()));
		sleep(0.1);
	}
	for(int i = 0; i < 360; i = i + 24) // turn 360 degrees 24 degrees a time while moving up and down to help the drone capture frames
	{
		tello.SendCommand("cw 24");
		while(!(tello.ReceiveResponse()));
		sleep(0.1);
		tello.SendCommand("up 30");
		while(!(tello.ReceiveResponse()));
		sleep(0.1);
		tello.SendCommand("down 30");
		while(!(tello.ReceiveResponse()));
		sleep(0.3);
	}
	inThreadMove = false;
}
// a thread that get the frames from the drone
void GetFrame()
{
	tello.SendCommand("streamon");
	while (!(tello.ReceiveResponse()));
	VideoCapture capture{TELLO_STREAM_URL, CAP_FFMPEG};
	video = true;
	// main loop, get the frames from capture and put it in our global matrics
	while(inThreadMove)
	{
		capture >> im;
		sleep(0.02);
	}
	inThreadFrame = false;
	while(true) // continue to capture frames for getting out of the door algorithm
	{
		capture >> im;
		sleep(0.02);
	}
}


int main(int argc, char **argv)
{
  if(argc != 3)
  {
      cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence" << endl;
      return 1;
  }
	if(!tello.Bind()) // check if tello is alive
	{
		return 0;
	}
	Point target, current;
	double yaw, targetAngle, movingAngle;
	Mat pose;
	video = false;
	inThreadFrame = true;
	inThreadMove = true;
	thread th1(GetFrame); // call thread GetFrame
  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
	thread th2(MoveDrone); // call thread MoveDrone
	while(!video)
	{
		sleep(0.01); // we need to wait until the video is online before we try to display images
	}
	while(inThreadFrame)
	{
    if(!im.empty())
    {
      pose = SLAM.TrackMonocular(im,0.02); // if the image is not empty pass the images to SLAM
      localized = !pose.empty();
    }
  }
  cout << "ended getting frames" << endl;
	saveMap(SLAM); // save the map
	target = findingDoorPoint(); // call the finding door point algorithm
  pose = SLAM.TrackMonocular(im,0.02); // get the current point from the MONOCULAR
  Mat Rwc = pose.rowRange(0,3).colRange(0,3).t();
  Mat twc = -Rwc*pose.rowRange(0,3).col(3); // get the current point matrics using the internet code
  pose = pose.rowRange(0,3).colRange(0,3);
  yaw = atan2(pose.at<float>(2,1),pose.at<float>(1,1)); // get the yaw using the internet code
  current.setX(twc.at<float>(0));
  current.setY(twc.at<float>(2));
  current.setZ(twc.at<float>(1));
  cout << "current point: " << endl << current << endl;
  cout << "target point: " << endl << target << endl;
  targetAngle = atan2(current.getX() - target.getX(), current.getZ() - target.getZ()); // calculate the angle using atan2, the door point and our current location
  yaw = yaw * (180 / M_PI); // going from radians to degree
  targetAngle = targetAngle * (180 / M_PI); // going from radians to degree
  cout << "target angle is: " << targetAngle << endl;
  cout << "yaw is: " << yaw << endl;
  yaw += 90; // fixing the yaw to be from X axes
  targetAngle += 180; // fixing target angle to be from 0 to 360 and not from -180 to 180
  cout << "yaw after calc is: " << yaw << endl;
  cout << "targetAngle after calc is: " << targetAngle << endl;
  movingAngle = yaw - targetAngle; // calculate moving angle
  cout << "moving angle is: " << movingAngle << endl;
  tello.SendCommand("cw " + to_string(movingAngle)); // rotate the drone to face the door
  while(!(tello.ReceiveResponse()));
  sleep(1);
	for(int i = 0; i < 10; i ++) // move forward
	{
    tello.SendCommand("forward 40");
    while(!(tello.ReceiveResponse()));
		sleep(0.1);
	}
	tello.SendCommand("land");
	while(!(tello.ReceiveResponse()));
	sleep(0.1);
	cout << "landed" << endl;
  SLAM.Shutdown();
  return 0;
}

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream f;
    f.open(strFile.c_str());

    // skip first three lines
    string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        string s;
        getline(f,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}

void saveMap(ORB_SLAM2::System& SLAM){
    std::vector<ORB_SLAM2::MapPoint*> mapPoints = SLAM.GetMap()->GetAllMapPoints();
    std::ofstream pointData;
    pointData.open("/tmp/pointData.csv");
    for(auto p : mapPoints) {
        if (p != NULL && !p->isBad())
        {
            auto point = p->GetWorldPos();
            Eigen::Matrix<double, 3, 1> v = ORB_SLAM2::Converter::toVector3d(point);
            pointData << v.x() << "," << v.y() << "," << v.z()<<  std::endl;
        }
    }
    pointData.close();
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

Point findingDoorPoint()
{
	Point p(0, 0, 0), a(0, 0 ,0);
	vector<Point> points = parsePoints("/tmp/pointData.csv"); // create a vector of the map from the csv file we just saved
	sortOutOutliers(points, 10, 0.1); // sort out the outliers
	vector<Point> interval = getMaximumInterval(points, p, 10); // return the maximum interval
	a = findAvaregePoint(interval); // get the avarege of the interval
	return a;
}
