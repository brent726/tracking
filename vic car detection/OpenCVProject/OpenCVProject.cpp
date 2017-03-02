



#include "stdafx.h"
#include "highgui.h"
#include "cv.h"
#include "conio.h"
#include <vector>   
#include <iostream>
#include "Vehicle.h"
#include "C:\opencv\build\include\opencv2\video\background_segm.hpp"

using namespace std;
using namespace cv;



int main()
{
	//their grayscale images (needed for absdiff() function)
	Mat grayImage1,grayImage2;
	//sobel parameters
	Mat grad;
	int ddepth = CV_16S;
	int scale = 1;
	int delta = 0;
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;
    int count;
  	double area, ar;
	vector<Vec4i> hierarchy;
    Mat frame, img, prevImg, temp, gray, vehicle_ROI, img_temp;
    VideoCapture cap("C:\\Users\\PCBLAB_01\\Desktop\\60m.mp4");
	//VideoCapture cap("C:\\Users\\PCBLAB_01\\Desktop\\1stVideoFeb8(edited).mp4");
	//VideoCapture cap("\\\\Mac\\Home\\Desktop\\DroneVideos\\60m.mp4");
    //vector<vector<Point> > contours;
    //vector<Rect> cars;
    //namedWindow("Frame");
    
	double fps = cap.get(CV_CAP_PROP_FPS);
   bool needToInit=false;

    while(true)
    {

		Mat img1 = imread("car1.png", 0);
		Mat img2 = imread("car2.png", 0);	
		bool pause= false;


		img1.convertTo(img1, CV_64FC1, 1.0/255, 0);
		img2.convertTo(img2, CV_64FC1, 1.0/255, 0);

		Mat u = Mat::zeros(img1.rows, img1.cols, CV_64FC1);
		Mat v = Mat::zeros(img1.rows, img1.cols, CV_64FC1);

		getLucasKanadeOpticalFlow(img1, img2, u, v);


        switch(waitKey(100/fps)){
				case 27: //'esc' key has been pressed, exit program.
				return 0;

			
				case 112: //'p' has been pressed. this will pause/resume the code.
				pause = !pause;
				if(pause == true){ cout<<"Code paused, press 'p' again to resume"<<endl;
				while (pause == true){
					//stay in this loop until 
					switch (waitKey()){
						//a switch statement inside a switch statement? Mind blown.
					case 112: 
						//change pause back to false
						pause = false;
						cout<<"Code Resumed"<<endl;
						break;
					case 115:
						//screen capture
						//imwrite( "E:/capturedimage1.jpg", frame1 );
						cout<<"Capture Image"<<endl;
						break;
					}
				}
			}
		}
	}
       
       
       
        
      
    
}
