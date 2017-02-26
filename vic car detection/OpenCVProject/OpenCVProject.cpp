



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

const int MIN_OBJECT_AREA = 5*5;


//string
int capnum = 0;
char path[70];
char path_gray[70];

//int to string helper function
string intToString(int number){

	//this function has a number input and string output
	std::stringstream ss;
	ss << number;
	return ss.str();
}

void drawObject(vector<Vehicle> VehicleCars,Mat &frame){

	for(int i =0; i<VehicleCars.size(); i++){
	//cv::circle(frame,cv::Point(VehicleCars.at(i).getXPos(),VehicleCars.at(i).getYPos()),20,cv::Scalar(0,255,0));
		rectangle( frame, VehicleCars.at(i).getTl(), VehicleCars.at(i).getBr(), cv::Scalar(0,255,0), 2, 8, 0 );
	//location of the object
	//cv::putText(frame,intToString(VehicleCars.at(i).getXPos())+ " , " + intToString(VehicleCars.at(i).getYPos()),cv::Point(VehicleCars.at(i).getXPos(),VehicleCars.at(i).getYPos()+20),1,1,Scalar(0,255,0));
	//cv::putText(frame,intToString(VehicleCars.at(i).getID()),cv::Point(VehicleCars.at(i).getXPos(),VehicleCars.at(i).getYPos()+20),1,1,Scalar(0,255,0));
	}
	cv::putText(frame,"Number of Vehicles: "+intToString(VehicleCars.size()),cv::Point(20,20),1,1,Scalar(0,255,0));
	//printf("Number of Vehicles : %d",VehicleCars.size());
}

void searchForVehicle(Mat thresholdImage, Mat &cameraFeed){
	
	vector<Vehicle> vehicles;

	//eg. we draw to the cameraFeed to be displayed in the main() function.
	bool objectDetected = false;
	Mat temp;
	thresholdImage.copyTo(temp);
	//these two vectors needed for output of findContours
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	//findContours(temp,contours,hierarchy,CV_RETR_CCOMP,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours
	//findContours(temp,contours,hierarchy,RETR_TREE,CV_CHAIN_APPROX_SIMPLE );// retrieves all contours tree
	findContours(temp,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE );// retrieves external contours
	// remove very small contours
	vector<vector<Point> > contours_poly( contours.size() );
	vector<Rect> boundRect( contours.size() );

	//if contours vector is not empty, we have found some objects
	double refArea = 0;
	if (hierarchy.size() > 0) {
		
			for (int index = 0; index >= 0; index = hierarchy[index][0]) {

				cout << "\n Countour area: " << contourArea(contours[index]);
				
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area>MIN_OBJECT_AREA){
					printf("  ID : %d Area: %lf",index,area);

					approxPolyDP( Mat(contours[index]), contours_poly[index], 3, true );
					boundRect[index] = boundingRect( Mat(contours_poly[index]) );

					int width =  boundRect[index].width;
					int height = boundRect[index].height;

					cout << " Width: " << width;
					cout << " Height: " << height;
					printf("\n");

					if(width>50 && width <140 && height>30 && height<50)
					{
					Vehicle car;
					
					car.setXPos(moment.m10/area);
					car.setYPos(moment.m01/area);
					car.setID(index);
					car.setBr(boundRect[index].br());
					car.setTl(boundRect[index].tl());

					vehicles.push_back(car);
					}
				}
			}
			//draw object location on screen
			drawObject(vehicles,cameraFeed);
	}

}

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
   // VideoCapture cap("C:\\Users\\PCBLAB_01\\Desktop\\60m.mp4");
	//VideoCapture cap("C:\\Users\\PCBLAB_01\\Desktop\\1stVideoFeb8(edited).mp4");
	VideoCapture cap("\\\\Mac\\Home\\Desktop\\DroneVideos\\60m.mp4");
    vector<vector<Point> > contours;
    vector<Rect> cars;
    //namedWindow("Frame");
    
	double fps = cap.get(CV_CAP_PROP_FPS);
   
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

        goodFeaturesToTrack(grayImage1, img_corners, maxCorners,qualityLevel,minDistance,Mat(),blockSize,true);
		printf("AAAA");
        cornerSubPix( grayImage1, img_corners, Size( win_size, win_size ), Size( -1, -1 ),
                     TermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03 ) );
        printf("BBB");
        vector<uchar> features_found;
        features_found.reserve(maxCorners);
        vector<float> feature_errors;
        feature_errors.reserve(maxCorners);
        
        calcOpticalFlowPyrLK( grayImage1, prevImg, img_corners, prevImg_corners, features_found, feature_errors ,
                             Size( win_size, win_size ), 3,
                             cvTermCriteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.3 ), 0, k);
        printf("CCC");
       for( int i=0; i < features_found.size(); i++ ){

            Point2f p0( ceil( img_corners[i].x ), ceil( img_corners[i].y ) );
            Point2f p1( ceil( prevImg_corners[i].x ), ceil( prevImg_corners[i].y ) );
            line( frame, p0, p1, CV_RGB(255,0,0), 5 );
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
		prevImg = grayImage1;
	}
        //prevImg = img;
       
       
        
      
    
}
