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
bool inThread1, inThread2;
bool video;
Tello tello{};
Mat im;

void LoadImages(const string &strFile, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);
// a thread that controls the Drone movement
void MoveDrone()
{
	while(!video) // wait for the capture before you take off
		sleep(0.01);
	tello.SendCommand("takeoff");
	while(!(tello.ReceiveResponse()));
	sleep(5); // it takes a while before a picture is shown so wait 5 sec
	// main loop, make the drone turn 30 degrees clockwise
	while(inThread1)
	{
		tello.SendCommand("cw 30");
		while(!(tello.ReceiveResponse()));
		sleep(0.5);
	}
	tello.SendCommand("land"); 
	while(!(tello.ReceiveResponse()));
	inThread2 = false;
}
// a thread that get the frames from the drone
void GetFrame()
{
	tello.SendCommand("streamon"); 
	while (!(tello.ReceiveResponse()));
	VideoCapture capture{TELLO_STREAM_URL, CAP_FFMPEG};
	video = true;
	// main loop, get the frames from capture and put it in our global matrics
	for(int n = 0; n < 50000; n++) 
	{
		capture >> im;
		sleep(0.02);
	}
	inThread1 = false;
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
	video = false;
	inThread1 = true;
	inThread2 = true;
	thread th1(GetFrame); // call thread 1
	thread th2(MoveDrone); // call thread 2
	// Create SLAM system. It initializes all system threads and gets ready to process frames.
	ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
	while(!video)
	{
		sleep(0.01); // we need to wait until the video is online before we try to display images
	}
	while(inThread1)
	{
        if(!im.empty())
        {
			SLAM.TrackMonocular(im,0.02); // if the image is not empty pass the images to SLAM
        }
    }

    // Stop all threads
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
