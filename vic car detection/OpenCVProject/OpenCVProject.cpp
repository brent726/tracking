
#include "stdafx.h"
#include "highgui.h"
#include "cv.h"
#include "conio.h"
#include <vector>   
#include <windows.h>
#include <iostream>
#include "Vehicle.h"

using namespace std;
using namespace cv;


//default capture width and height
const int FRAME_WIDTH = 1280;
const int FRAME_HEIGHT = 720;
//our sensitivity value to be used in the absdiff() function
const static int SENSITIVITY_VALUE = 40;
//size of blur used to smooth the intensity image output from absdiff() function
const static int BLUR_SIZE = 3;
//we'll have just one object to search for
const int MAX_NUM_OBJECTS=50;
const int MIN_OBJECT_AREA = 5*5;

const int MAX_OBJECT_AREA = 100*50;
//bounding rectangle of the object, we will use the center of this as its position.
Rect objectBoundingRectangle = Rect(0,0,0,0);

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
Mat sobel(Mat gray){
	Mat edges;

	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	Mat edges_x, edges_y;
	Mat abs_edges_x, abs_edges_y;
	Sobel(gray, edges_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs( edges_x, abs_edges_x );
	Sobel(gray, edges_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(edges_y, abs_edges_y);
	addWeighted(abs_edges_x, 0.5, abs_edges_y, 0.5, 0, edges);

	return edges;
}

Mat canny(Mat src)
{
	Mat detected_edges;

	int edgeThresh = 1;
	int lowThreshold = 250;
	int highThreshold = 750;
	int kernel_size = 5;
	Canny(src, detected_edges, lowThreshold, highThreshold, kernel_size);

	return detected_edges;
 }
int main(){

	//some boolean variables for added functionality
	bool objectDetected = false;
	//these two can be toggled by pressing 'd' or 't'
	bool debugMode = false;
	bool trackingEnabled = false;
	//pause and resume code
	bool pause = false;
	//set up the matrices that we will need
	//the two frames we will be comparing
	Mat frame1,frame2;
	Mat frame1_full,frame2_full;
	//their grayscale images (needed for absdiff() function)
	Mat grayImage1,grayImage2;
	//resulting difference image
	Mat differenceImage;
	//thresholded difference image (for use in findContours() function)
	Mat thresholdImage;
	Mat newthresholdImage;
	//sobel parameters
	Mat grad;
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	//road size
	Rect ROAD(1,219,1279,327);
	//video capture object
	VideoCapture capture;
	capture.open("\\\\Mac\\Home\\Desktop\\DroneVideos\\60m.mp4");
	double fps = capture.get(CV_CAP_PROP_FPS);
	if(!capture.isOpened())
	{
		cout<<"ERROR ACQUIRING VIDEO FEED\n";
		waitKey();
		return -1;
	}
	cout<<"Video FPS: "<<fps<<endl;
	//video frame details
	double dWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
	cout << "Frame Size = " << dWidth << "x" << dHeight << endl;
	

	//video start
	//while(capture.get(CV_CAP_PROP_POS_FRAMES)<capture.get(CV_CAP_PROP_FRAME_COUNT)-1)
	//{
	capture.read(frame1);
	//frame1 = imread("E:/Project Alpha/Dataset/New/1.png", CV_LOAD_IMAGE_COLOR);
	frame1=imread("2.PNG");
	
	Mat hsvImg;
	cvtColor(frame1, hsvImg, CV_BGR2HSV);
	Mat channel[3];
	split(hsvImg, channel);
	//channel[0] = Mat(hsvImg.rows, hsvImg.cols, CV_8UC1, 100);//Set H
	//channel[1] = Mat(hsvImg.rows, hsvImg.cols, CV_8UC1, 80);//Set S
	channel[2] = Mat(hsvImg.rows, hsvImg.cols, CV_8UC1, 200);//Set V
	//Merge channels
	merge(channel, 3, hsvImg);
	//imshow("Frame", frame1);
	Mat rgbImg;
	cvtColor(hsvImg, rgbImg, CV_HSV2BGR);
	imshow("1. \"Remove Shadows\"", rgbImg);

	cv::cvtColor(rgbImg,grayImage1,COLOR_BGR2GRAY);
	normalize(grayImage1, grayImage1, 0, 255, NORM_MINMAX, CV_8UC1);
	imshow("2. Grayscale", grayImage1);
	//GaussianBlur(grayImage1, grayImage1, Size(15,15), 0, 0, BORDER_DEFAULT );
	//sobel 
	/*Sobel( grayImage1, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
	convertScaleAbs( grad_x, abs_grad_x );
	Sobel( grayImage1, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
    convertScaleAbs( grad_y, abs_grad_y );
	 addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
	cv::imshow("Sobel Image", grad);*/
	FILE * f;
	f = fopen("hsv.txt", "w");
	for(int i=0;i<hsvImg.size().width;i++)
	{
		for(int j=0;j<hsvImg.size().height;j++)
		{
			
			fprintf(f,"h=%d s:%d v:%d\n", hsvImg.at<Vec3b>(Point(i, j)).val[0], hsvImg.at<Vec3b>(Point(i, j)).val[1], hsvImg.at<Vec3b>(Point(i, j)).val[2]);
		}
	}

	//3. Edge detector
	GaussianBlur(grayImage1, grayImage1, Size(3,3), 0, 0, BORDER_DEFAULT);
	Mat edges;
	bool useCanny = false;
	if(useCanny){
		edges = canny(grayImage1);
	} else {
		//Use Sobel filter and thresholding.
		edges = sobel(grayImage1);
		//Automatic thresholding
		//threshold(edges, edges, 0, 255, cv::THRESH_OTSU);
		//Manual thresholding
		threshold(edges, edges, 25, 255, cv::THRESH_BINARY);
	}

	imshow("3. Edge Detector", edges);
	//4. Dilate
	Mat dilateGrad = edges;
	int dilateType = MORPH_RECT;
	int dilateSize = 1;
	Mat elementDilate = getStructuringElement(dilateType,
		Size(2*dilateSize + 1, 2*dilateSize+1),
		Point(dilateSize, dilateSize));
	dilate(edges, dilateGrad, elementDilate);
	imshow("4. Dilate", dilateGrad);
	
	//5. Floodfill
	Mat floodFilled = cv::Mat::zeros(dilateGrad.rows+2, dilateGrad.cols+2, CV_8U);
	floodFill(dilateGrad, floodFilled, cv::Point(0, 0), 0, 0, cv::Scalar(), cv::Scalar(), 4 + (255 << 8) + cv::FLOODFILL_MASK_ONLY);
	floodFilled = cv::Scalar::all(255) - floodFilled;
	Mat temp;
	floodFilled(Rect(1, 1, dilateGrad.cols-2, dilateGrad.rows-2)).copyTo(temp);
	floodFilled = temp;
	imshow("5. Floodfill", floodFilled);

	if(true)
	{

		searchForVehicle(floodFilled,frame1);
	}
	imshow("Frame1",frame1);
	//waitKey(0);
	//waitkey for video
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
						imwrite( "E:/capturedimage1.jpg", frame1 );
						cout<<"Capture Image"<<endl;
						break;
					}
				}
				}
			//}
	}//video end
	return 0;

}