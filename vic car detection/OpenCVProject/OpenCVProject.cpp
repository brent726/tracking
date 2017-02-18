



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
	
    int count;
  	double area, ar;
	vector<Vec4i> hierarchy;
    Mat frame, fore, img, prevImg, temp, gray, vehicle_ROI, img_temp;
	boolean update_bg_model=true;
    VideoCapture cap("C:\\Users\\PCBLAB_01\\Desktop\\60m.mp4");
    BackgroundSubtractorMOG2 bg;//(50, 25, false);
	BackgroundSubtractorMOG2 bg_model;
    vector<vector<Point> > contours;
    vector<Rect> cars;
    namedWindow("Frame");
    
	double fps = cap.get(CV_CAP_PROP_FPS);

    cap >> img_temp;
    cvtColor(img_temp, gray, CV_BGR2GRAY);
    gray.convertTo(temp, CV_8U);
    bilateralFilter(temp, prevImg, 5, 20, 20);
   
    while(true)
    {
        count=0;
        cap >> frame;
        
		if( frame.empty() )
            break;

		Mat fgimg, fgmask;

		if( fgimg.empty() )
          fgimg.create(frame.size(), frame.type());

		 //update the model
        bg_model(frame, fgmask, update_bg_model ? -1 : 0);

		fgimg = Scalar::all(0);
        frame.copyTo(fgimg, fgmask);
		Mat bgimg;
        bg_model.getBackgroundImage(bgimg);
		//imshow("image", frame);
        //imshow("foreground mask", fgmask);
        imshow("foreground image", fgimg);

		if(!bgimg.empty())
          imshow("mean background image", bgimg );

		char w = (char)waitKey(30);
        if( w == 27 ) break;
        if( w == ' ' )
        {
            update_bg_model = !update_bg_model;
            if(update_bg_model)
                printf("Background update is on\n");
            else
                printf("Background update is off\n");
        }
        


		 //update the model
       //bg_model(img, fgmask, update_bg_model ? -1 : 0);
        cvtColor(fgimg, gray, CV_BGR2GRAY);
		imshow("gray",gray);
        gray.convertTo(temp, CV_8U);
		
        bilateralFilter(temp, prevImg, 5, 20, 20);
		imshow("temp prev Img",gray);

        bg.operator()(fgimg,fore);
		imshow("fore",fore);
		
		
        //erode(fore,fore,Mat());
        //dilate(fore,fore,Mat());
        findContours(fore,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
		for( int i = 0; i< contours.size(); i++ )
		{
         cv::drawContours(fore, contours, i, Scalar(255, 255, 255), 1);
		}
		//imshow("fore dilated",fore);
        vector<vector<Point> > contours_poly(contours.size());
		vector<Rect> boundRect(contours.size());
        
        for(size_t i = 0; i < contours.size(); i++ )
		{
			approxPolyDP( Mat(contours[i]), contours_poly[i], 10, true );
			boundRect[i] = boundingRect( Mat(contours_poly[i]) );
			//rectangle( fgimg, boundRect[i].tl(), boundRect[i].br(), Scalar(255,0,0), 2, 8, 0 );
            
			vehicle_ROI = fgimg(boundRect[i]);
            area = contourArea(contours[i], false);
            ar = vehicle_ROI.cols/vehicle_ROI.rows;
            if(area > 450.0 && ar > 0.8)
            {
				rectangle( fgimg, boundRect[i].tl(), boundRect[i].br(), Scalar(255,0,0), 2, 8, 0 );
                count=count+1;
            }
        }
		
		for( int i = 0; i< contours.size(); i++ )
		{
         cv::drawContours(fgimg, contours, i, Scalar(255, 255, 255), 1);
		}
		imshow("frame Rect",fgimg);
		printf("Count: %d\n", count);
        stringstream ss;
        ss << count;
        string s = ss.str();
        int fontFace = FONT_HERSHEY_SCRIPT_SIMPLEX;
        double fontScale = 2;
        int thickness = 3;
        cv::Point textOrg(10, 130);
        cv::putText(frame, s, textOrg, fontFace, fontScale, Scalar(0,255,0), thickness,5);
       
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

       //goodFeaturesToTrack(fgimg, img_corners, maxCorners,qualityLevel,minDistance,Mat(),blockSize,true);

        /*cornerSubPix( fgimg, img_corners, Size( win_size, win_size ), Size( -1, -1 ),
                     TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );
        
        vector<uchar> features_found;
        features_found.reserve(maxCorners);
        vector<float> feature_errors;
        feature_errors.reserve(maxCorners);
        
       calcOpticalFlowPyrLK( fgimg, prevImg, img_corners, prevImg_corners, features_found, feature_errors ,
                             Size( win_size, win_size ), 3,
                             cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0, k);
        
        for( int i=0; i < features_found.size(); i++ ){

            Point2f p0( ceil( img_corners[i].x ), ceil( img_corners[i].y ) );
            Point2f p1( ceil( prevImg_corners[i].x ), ceil( prevImg_corners[i].y ) );
            line( frame, p0, p1, CV_RGB(255,0,0), 5 );
        }*/
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
		
        //prevImg = img;
        prevImg = frame;
        imshow("Frame",frame);
        
        if(waitKey(5) >= 0)
            break;
    }
    
}
