



#include "stdafx.h"
#include "highgui.h"
#include "cv.h"
#include "conio.h"
#include <vector>   
#include <windows.h>
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
	
    vector<vector<Point> > contours;
    vector<Rect> cars;
    namedWindow("Frame");
    
	double fps = cap.get(CV_CAP_PROP_FPS);
   
    while(true)
    {
        count=0;
        cap >> frame;
        
		if( frame.empty() )
            break;
		
		cv::cvtColor(frame,grayImage1,COLOR_BGR2GRAY);
		imshow("BGR to Gray", grayImage1);
		//GaussianBlur(grayImage1, grayImage1, Size(15,15), 0, 0, BORDER_DEFAULT );
		//sobel 
	
		Sobel( grayImage1, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
		convertScaleAbs( grad_x, abs_grad_x );
		Sobel( grayImage1, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
		convertScaleAbs( grad_y, abs_grad_y );
		addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
		cv::imshow("Sobel Image", grad); 

		//getting pedestrian out
		findContours(grad,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours
		// remove very small contours
		vector<vector<Point> > contours_poly( contours.size() );
		vector<Rect> boundRect( contours.size() );

		if (hierarchy.size() > 0) {
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				cout << "\n Countour area: " << contourArea(contours[index]);
		}
       
      // calcOpticalFlowPyrLK( fgimg, prevImg, img_corners, prevImg_corners, features_found, feature_errors ,
                             //Size( win_size, win_size ), 3,
                             //cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0, k);
         imshow("Frame",frame);
      
  
		bool pause= false;
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
        //prevImg = img;
       // prevImg = frame;
       
        
      
    
}
