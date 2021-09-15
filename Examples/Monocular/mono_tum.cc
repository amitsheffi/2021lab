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

vector<Point> parsePoints(string filePath); // parsing points function

vector<Point> getMaximumInterval(vector<Point>& points, Point targetP, int intervalSize); // returns the max interval

bool sortFunction(Point p1, Point p2); // a sorting function for the cleaning noises

void sortOutOutliers(vector<Point>& points, Point targetP, int minNumberOfNeighbors, double neighborDistance); // cleans the noises

Point getMaxPoint(vector<Point>& points, Point& targetP);

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
	// main loop, make the drone turn 30 degrees clockwise
	while(!localized)
	{
		tello.SendCommand("up 15");
		while(!(tello.ReceiveResponse()));
		sleep(0.1);
		tello.SendCommand("down 15");
		while(!(tello.ReceiveResponse()));
		sleep(0.1);
	}
	for(int i = 0; i < 360; i = i + 24)
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
	while(true)
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
    // Stop all threads
	saveMap(SLAM);
	target = findingDoorPoint();
		pose = SLAM.TrackMonocular(im,0.02);
		Mat Rwc = pose.rowRange(0,3).colRange(0,3).t();
    	Mat twc = -Rwc*pose.rowRange(0,3).col(3);
    	pose = pose.rowRange(0,3).colRange(0,3);
		yaw = atan2(pose.at<float>(2,1),pose.at<float>(1,1));
		current.setX(twc.at<float>(0));
		current.setY(twc.at<float>(2));
		current.setZ(twc.at<float>(1));
		cout << "current point: " << endl << current << endl;
		cout << "target point: " << endl << target << endl;
		targetAngle = atan2(current.getX() - target.getX(), current.getZ() - target.getZ());
		yaw = yaw * (180 / M_PI); // going from radians to degree
		targetAngle = targetAngle * (180 / M_PI); // going from radians to degree
		cout << "target angle is: " << targetAngle << endl;
		cout << "yaw is: " << yaw << endl;
		yaw += 90; // fixing the yaw to be from X axes
		targetAngle += 180;
		cout << "yaw after calc is: " << yaw << endl;
		cout << "targetAngle after calc is: " << targetAngle << endl;
		movingAngle = yaw - targetAngle;
		cout << "moving angle is: " << movingAngle << endl;
		tello.SendCommand("cw " + to_string(movingAngle)); // rotate the drone to face the door
		while(!(tello.ReceiveResponse()));
		sleep(1);
	for(int i = 0; i < 10; i ++)
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
	for(int i = startingIntervalIndex; i - startingIntervalIndex < intervalSize; i++)
	{
			interval.push_back(points[i]);
	}
	return interval;
}

bool sortFunction(Point p1, Point p2)
{
	Point z(0, 0, 0);
	return (p1.getDistanceFrom(z) < p2.getDistanceFrom(z));
}

void sortOutOutliers(vector<Point>& points, Point targetP, int minNumberOfNeighbors, double neighborDistance)
{
	int numOfNeighbors;
	double currentDistance;
	Point p(0, 0, 0);
	vector<Point> goodPoints;
	sort(points.begin(), points.end(), sortFunction);
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
		/*
		if (numOfNeighbors < minNumberOfNeighbors)
		{
			points.erase(points.begin() + i);
			i--;
		}
		*/
	}
	points = goodPoints;
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

Point findingDoorPoint()
{
	Point p(0, 0, 0), r(0, 0 ,0);
	vector<Point> points = parsePoints("/tmp/pointData.csv");
	//sortOutOutliers(points, p, 10, 0.1);
	vector<Point> interval = getMaximumInterval(points, p, 10);
	r = getMaxPoint(interval, p);
	return r;
}

