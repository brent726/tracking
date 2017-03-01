



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
        count=0;
        cap >> frame;
        
		if( frame.empty() )
            break;

		cv::cvtColor(frame,grayImage1,COLOR_BGR2GRAY);
		
		//lucas kanade optical flow
		/*stringstream ss;
        ss << count;
        string s = ss.str();
        int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
        double fontScale = 2;
        int thickness = 3;
        cv::Point textOrg(10, 130);
        cv::putText(frame, s, textOrg, fontFace, fontScale, Scalar(0,255,0), thickness,5);*/
        //imshow("gray image",grayImage1);
		
        int win_size = 10;
        int maxCorners = 200;
        double qualityLevel = 0.01;
        double minDistance = 1;
        int blockSize = 3;
        double k = 0.04;
        vector<Point2f> img_corners;
        img_corners.reserve(maxCorners);
        vector<Point2f> prevImg_corners;
        prevImg_corners.reserve(maxCorners);

		

		vector<uchar> features_found;
		features_found.reserve(maxCorners);
		vector<float> feature_errors;
		feature_errors.reserve(maxCorners);

		if(needToInit)
		{

			goodFeaturesToTrack(grayImage1, img_corners, maxCorners,qualityLevel,minDistance,Mat(),blockSize,true);
			printf("AAAA");
			cornerSubPix( grayImage1, img_corners, Size( win_size, win_size ), Size( -1, -1 ),
                     TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );
		}   
		else if(!img_corners.empty())
		{
			calcOpticalFlowPyrLK( grayImage1, prevImg, img_corners, prevImg_corners, features_found, feature_errors ,
                             Size( win_size, win_size ), 3,
                             cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0, k);

			for( int i=0; i < features_found.size(); i++ ){

            Point2f p0( ceil( img_corners[i].x ), ceil( img_corners[i].y ) );
            Point2f p1( ceil( prevImg_corners[i].x ), ceil( prevImg_corners[i].y ) );
            line( frame, p0, p1, CV_RGB(255,0,0), 5 );
			}
		}
       
        
       

		imshow("Frame",frame);

		//sobel
		/*Sobel( grayImage1, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
		convertScaleAbs( grad_x, abs_grad_x );
		Sobel( grayImage1, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
		convertScaleAbs( grad_y, abs_grad_y );
		addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
		cv::imshow("Sobel Image", grad);*/ 
		/*Mat thresholdImage;
		cv::threshold(grad,thresholdImage,25,255,THRESH_BINARY);
		cv::imshow("Threshold Image", thresholdImage);*/
		//morphologyEx(thresholdImage,thresholdImage,MORPH_OPEN,Mat::ones(3,3,CV_8SC1),Point(1,1),2);
		//cv::imshow("Morphed Image", thresholdImage);
		
		 if(false)
		{
			//searchForVehicle(thresholdImage,frame);
		}
        // imshow("Frame",frame);
      
		needToInit=false;
		bool pause= false;
        switch(waitKey(100/fps)){
				case 27: //'esc' key has been pressed, exit program.
				return 0;

				case 'r':
					needToInit = true;
					break;
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
		prevImg = grayImage1;
	}
        //prevImg = img;
       
       
        
      
    
}
