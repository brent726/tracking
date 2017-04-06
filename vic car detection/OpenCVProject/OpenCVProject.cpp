// Pyramid L-K optical flow example
//

#include "stdafx.h"
#include "highgui.h"
#include "cv.h"
#include "conio.h"
#include <vector>   
#include <iostream>
#include "Vehicle.h"

using namespace std;
using namespace cv;
using namespace std;

static const int MAX_CORNERS = 1000;

//default capture width and height
const int FRAME_WIDTH = 1280;
const int FRAME_HEIGHT = 720;
//our sensitivity value to be used in the absdiff() function
const static int SENSITIVITY_VALUE = 40;
//size of blur used to smooth the intensity image output from absdiff() function
const static int BLUR_SIZE = 3;
//we'll have just one object to search for
const int MAX_NUM_OBJECTS=50;

const int MAX_OBJECT_AREA = 100*50;
//bounding rectangle of the object, we will use the center of this as its position.
Rect objectBoundingRectangle = Rect(0,0,0,0);

//string
int capnum = 0;
char path[70];
char path_gray[70];

#define ATD at<double>
#define elif else if

#ifndef bool
	#define bool int
	#define false ((bool)0)
	#define true ((bool)1)
#endif

//global variables
Point2f point;
bool addRemovePt = false;
bool addRefPt = false;

const int MIN_OBJECT_AREA = 5 * 5;
int HroadTopX = 0;
int HroadTopY = 0;
int HroadBotX = 0;
int HroadBotY = 0;
int VroadTopX = 0;
int VroadTopY = 0;
int VroadBotX = 0;
int VroadBotY = 0;


//functions
static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/);
void addPoints(int x, int y);
string intToString(int number);
void drawObject(vector<Vehicle> VehicleCars, Mat &frame);
void searchForVehicle(Mat thresholdImage, vector<Vehicle> &vehicles);
Rect roadDetectionHorizontal(Mat labRoadImage);
Rect roadDetectionVertical(Mat roadImage);
Scalar speedSpectrum(double speed);
Mat sobelDetection(Mat curFrame);
void setLabel(cv::Mat& im, const std::string label, const cv::Point & pointor);
Point2i vehicleCountTopAndBot(vector<Vehicle> &vehicles, int halfY);
Mat roadDetectionPreprocessing (Mat roadImage, int orientation);
Mat addCenterLinesHorizontal(Mat binRoadImage);
Mat addCenterLinesVeritcal(Mat binRoadImage);
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
          cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {
          cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
          cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
    
}



void drawObject(vector<Vehicle> VehicleCars,Mat &frame){

	for(int i =0; i<VehicleCars.size(); i++){
	//cv::circle(frame,cv::Point(VehicleCars.at(i).getXPos(),VehicleCars.at(i).getYPos()),20,cv::Scalar(0,255,0));
		rectangle( frame, VehicleCars.at(i).getTl(), VehicleCars.at(i).getBr(), cv::Scalar(0,255,0), 1, 8, 0 );
		//rectangle( frame, VehicleCars.at(i).getTl(), VehicleCars.at(i).getBr(), cv::Scalar(255), CV_FILLED);
	//location of the object
	//cv::putText(frame,intToString(VehicleCars.at(i).getXPos())+ " , " + intToString(VehicleCars.at(i).getYPos()),cv::Point(VehicleCars.at(i).getXPos(),VehicleCars.at(i).getYPos()+20),1,1,Scalar(0,255,0));
	//cv::putText(frame,intToString(VehicleCars.at(i).getID()),cv::Point(VehicleCars.at(i).getXPos(),VehicleCars.at(i).getYPos()+20),1,1,Scalar(0,255,0));
	}
	cv::putText(frame,"Number of Vehicles: "+intToString(VehicleCars.size()),cv::Point(20,20),1,1,Scalar(0,255,0));
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

				//cout << "\n Countour area: " << contourArea(contours[index]);
				
				Moments moment = moments((cv::Mat)contours[index]);
				double area = moment.m00;

				//if the area is less than 20 px by 20px then it is probably just noise
				//if the area is the same as the 3/2 of the image size, probably just a bad filter
				//we only want the object with the largest area so we safe a reference area each
				//iteration and compare it to the area in the next iteration.
				if(area>MIN_OBJECT_AREA){
					//printf("  ID : %d Area: %lf",index,area);

					approxPolyDP( Mat(contours[index]), contours_poly[index], 3, true );
					boundRect[index] = boundingRect( Mat(contours_poly[index]) );

					int width =  boundRect[index].width;
					int height = boundRect[index].height;

					//cout << " Width: " << width;
					//cout << " Height: " << height;
					//printf("\n");

					//if(width>50 && width <140 && height>30 && height<50)
					if((width>25 && width<120) && (height>20 && height<45))
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
			// imshow("vechicles result",cameraFeed);
	}

}

int main(int argc, char** argv) {
  // Initialize, load two images from the file system, and
  // allocate the images and other structures we will need for
  // results.
  bool pause=false;
 //road Rectangle
	Rect HroadBorder(0, 0, 0, 0);
	Rect HROAD(0, 0, 0, 0);
	Rect VroadBorder(0, 0, 0, 0);
	Rect VROAD(0, 0, 0, 0);

  Mat curFrame, prevFrame,prevResultFrame, curResultFrame;
  
 // VideoCapture cap("C:\\Users\\PCBLAB_01\\Desktop\\sampleVideo.avi");

 //VideoCapture cap("\\\\Mac\\Home\\Desktop\\DroneVideos\\DJI\\DJI_0014.MP4");
  //DJI_0009Take3.MP4 DJI_0010Take4.MP4 DJI_0005Take2.MP4 DJI_0008Take1.mp4
 VideoCapture cap("C:\\Users\\PCBLAB_01\\Desktop\\dji\\DJI_0010.MP4");

 //VideoCapture cap("\\\\Mac\\Home\\Desktop\\DroneVideos\\DJI_0005Take2.MP4");
// VideoCapture cap("C:\\Users\\PCBLAB_01\\Desktop\\dji\\DJI_0005Take2.MP4");
  //DJI_0009Take3.MP4 DJI_0010Take4.MP4 DJI_0005Take2.MP4 DJI_0011Take5.MP4 DJI_0008Take1.MP4
 

  double fps = cap.get(CV_CAP_PROP_FPS);
  int framepos=0;
  Size img_sz;
  Mat origFrame;
  double i,j;
  Mat gray;
  Mat labRoadImage;
  Mat sobelImage, LKImage,comboResultImage;
  int intersection=1;
  printf("Intersection= %d",intersection);
  Mat image, labRoadImageVertical;
  int dWidth=1280;
  int dHeight =720;
   Size frameSize(static_cast<int>(dWidth), static_cast<int>(dHeight));
  //VideoWriter oVideoWriter("roadDJI0014.mov", CV_FOURCC('m', 'p', '4', 'v'), fps, frameSize, true); //initialize the VideoWriter object
 	//if (!oVideoWriter.isOpened()) //if not initialize the VideoWriter successfully, exit the program
	//{
		//cout << "ERROR: Failed to write the video" << endl;
		//return -1;
	//}
  
  int  totalframes = (int)cap.get(CV_CAP_PROP_FRAME_COUNT) - 1;

  while(true)
  {
	      
		  cap >> image;
		  image.copyTo(origFrame);// for final output
		 framepos=(int)cap.get(CV_CAP_PROP_POS_FRAMES);
		  //road detection
		if (framepos == 1 || framepos % 300 == 0)
		{
			labRoadImage=roadDetectionPreprocessing (image,0);
			HroadBorder = roadDetectionHorizontal(labRoadImage);
			HroadTopX = HroadBorder.tl().x;
			HroadTopY = HroadBorder.tl().y;
			HroadBotX = HroadBorder.br().x;
			HroadBotY = HroadBorder.br().y;
			HROAD = Rect(HroadTopX, HroadTopY, (HroadBotX - HroadTopX), (HroadBotY - HroadTopY));//horizontal road dimensions

			if(intersection == 1)
			{ 
				labRoadImageVertical=roadDetectionPreprocessing (image,1);
				VroadBorder = roadDetectionVertical(labRoadImageVertical);
				VroadTopX = VroadBorder.tl().x;
				VroadTopY = VroadBorder.tl().y;
				VroadBotX = VroadBorder.br().x;
				VroadBotY = VroadBorder.br().y;
				VROAD = Rect(VroadTopX, VroadTopY, (VroadBotX - VroadTopX), (VroadBotY - VroadTopY));//vertical road dimensions
				
			}


		}

		//horizontal
		int halfY = (HroadBorder.br().y + HroadBorder.tl().y) / 2;
		Point HhalfFirstpoint = Point(HroadBorder.tl().x, halfY);
		Point HhalfSecondPoint = Point(HroadBorder.br().x, halfY);
		
		//draw rectangular shape of result of road detection
		rectangle(image, HroadBorder.tl(), HroadBorder.br(), Scalar(255, 0, 0), 1, 8, 0);
		rectangle(image, VroadBorder.tl(), VroadBorder.br(), Scalar(255, 0, 0), 1, 8, 0);

		
		imshow("road Image", image);
		//oVideoWriter.write(image);
		cout << "Frame: " << framepos << "/" << totalframes << endl;
		//line(image, HhalfFirstpoint, HhalfSecondPoint, Scalar(255, 0, 0), 1, 8, 0);
		//image.copyTo(curFrame);
		//cvtColor( curFrame, curFrame, COLOR_BGR2GRAY);

		/*********for horizontal to blacken the sides*********/
		//printf("tl=%d, br=%d", HroadBorder.tl().y, HroadBorder.br().y);
		/*for (i = 0;i<curFrame.size().width;i++)
		{
			for (j = 0;j<curFrame.size().height;j++)
			{
				if ((j < HroadBorder.tl().y)||(j > HroadBorder.br().y))  // these threshold values must be tuned or determined automatically!
				{	
					curFrame.at<uchar>(j, i)= 0;
				}
			}
		}*/
		
		//imshow("curFrame black", curFrame);
		/*********for horizontal to blacken the sides*********/
		if(framepos!=1)
		 {
				 //Mat u = Mat::zeros(prevFrame.rows, prevFrame.cols, CV_64FC1);
				// Mat v = Mat::zeros(prevFrame.rows, prevFrame.cols, CV_64FC1);
				
				 // LKImage = LKDetection(prevFrame, curFrame, u, v);
					
			      //sobelImage=sobelDetection(curFrame);
				// imshow("sobel", sobelImage);
				 //comboResultImage= Mat::zeros(origFrame.rows, origFrame.cols, CV_8U);
				  //comboResultImage=LKImage+sobelImage;
				// Mat tempResultBW;
				 //comboResultImage.copyTo(tempResultBW);
				
				//searchForVehicle(comboResultImage, tempResultBW);
				//searchForVehicle(sobelImage, origFrame);
				//imshow("detect result", origFrame);

			
				
		 }
		
			//prevFrame=curFrame;
			//prevFrame.convertTo(prevFrame, CV_64F, 1.0/255);
			//imshow("prev frame", prevFrame);
			 // system("PAUSE");
			  pause=false;
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
			  //cv::waitKey(0);
  }
  return 0;
}
//int to string helper function
string intToString(int number){

	//this function has a number input and string output
	std::stringstream ss;
	ss << number;
	return ss.str();
}

Mat labThresholdingStraight(Mat labRoadImage)
{
	int x, y;
	Mat im(labRoadImage.size().height, labRoadImage.size().width, CV_8UC1, Scalar(0));
	for (x = 0; x<labRoadImage.size().width; x++)
	{
		for (y = 0; y<labRoadImage.size().height; y++)
		{
				if ((labRoadImage.at<Vec3b>(y, x)[1]>125) && (labRoadImage.at<Vec3b>(y, x)[1]<131))  // these threshold values must be tuned or determined automatically!
				{
					if ((labRoadImage.at<Vec3b>(y, x)[2]>120) && (labRoadImage.at<Vec3b>(y, x)[2]<128)) //these threshold values must be tuned or determined automatically!
						//changing the pixel intensity to white
						im.at<uchar>(y, x) = 255;
					if ((labRoadImage.at<Vec3b>(y, x)[2]>133) && (labRoadImage.at<Vec3b>(y, x)[2]<140)) //these threshold values must be tuned or determined automatically!
						//changing the pixel intensity to white
						im.at<uchar>(y, x) = 255;
				}
		}


	}
	return im;

}

Mat roadDetectionPreprocessing (Mat roadImage, int orientation)
{
	Mat labRoadImage;
	Mat roadImageBlur;
	GaussianBlur(roadImage, roadImageBlur, Size(21, 21), 0, 0, BORDER_DEFAULT);
	cvtColor(roadImageBlur, labRoadImage, COLOR_BGR2Lab);
	Mat Lab[3];


	int imageCenter;
	
	//horizontal
	if(orientation==0)
	{
		imageCenter=roadImage.size().height/2;
		for (int x = 0; x<labRoadImage.size().width; x++)
		{
			for (int y = 0;y<labRoadImage.size().height; y++)
			{
				if(y==imageCenter)
				{
					labRoadImage.at<Vec3b>(y, x)[1]=127;
					labRoadImage.at<Vec3b>(y, x)[2]=127;
				}

			}
		}
	}
	//vertical
	if(orientation==1)
	{
		imageCenter=roadImage.size().width/2;
		for (int y = 0; y<labRoadImage.size().height; y++)
		{
			for (int x = 0; x<labRoadImage.size().width; x++)
			{
				if(x==imageCenter)
				{
					labRoadImage.at<Vec3b>(y, x)[1]=127;
					labRoadImage.at<Vec3b>(y, x)[2]=127;
				}

			}
		}
	}
	return labRoadImage;

}
Mat addCenterLinesHorizontal(Mat binRoadImage)
{
	int imageCenter=binRoadImage.size().height/2;
	int yCenter=0;
	int centerIncrement=imageCenter+1;
	int centerDecrement=imageCenter-1;
	Mat tempResultLowerPortion=Mat::zeros(binRoadImage.rows, binRoadImage.cols, CV_8U);
	Mat tempResultUpperPortion=Mat::zeros(binRoadImage.rows, binRoadImage.cols, CV_8U);
	int flagCopy=0;
		for (int y = 0;y<binRoadImage.size().height; y++)
		{
			if(y==imageCenter)
			{
				while((binRoadImage.at<uchar>(centerIncrement,yCenter)==0))
				{
						for (yCenter = 0; yCenter<binRoadImage.size().width; yCenter++)
					    {
								tempResultLowerPortion.at<uchar>(centerIncrement, yCenter) = 225;  // these threshold values must be tuned or determined automatically!
						}
						yCenter=0;
					    centerIncrement++;
						if(centerIncrement==(imageCenter+80))
						{
							flagCopy=1;
							break;
						}
				}
				if(flagCopy==0)
				{
					binRoadImage=binRoadImage+tempResultLowerPortion;
				}
				yCenter=0;
				flagCopy=0;
				while( (binRoadImage.at<uchar>(centerDecrement,yCenter)==0))
				{
					for (yCenter = 0; yCenter<binRoadImage.size().width; yCenter++)
					{
							tempResultUpperPortion.at<uchar>(centerDecrement, yCenter) = 225;  // these threshold values must be tuned or determined automatically!
					}
					yCenter=0;
					centerDecrement--;		
				    if(centerDecrement==(imageCenter-80))
					{
						flagCopy=1;
						break;
					}
				}
				if(flagCopy==0)
				{
					binRoadImage=binRoadImage+tempResultUpperPortion;
				}
				flagCopy=0;
			}
		}
		//imshow("tempResultUpperPortion", tempResultUpperPortion);
		//imshow("tempResulLowerPortion", tempResultLowerPortion);
		return binRoadImage;
		
}
Mat addCenterLinesVeritcal(Mat binRoadImage)
{
	int imageCenter=binRoadImage.size().width/2;
	int xCenter=0;
	int centerIncrement=imageCenter+1;
	int centerDecrement=imageCenter-1;
	Mat tempResultLowerPortion=Mat::zeros(binRoadImage.rows, binRoadImage.cols, CV_8U);
	Mat tempResultUpperPortion=Mat::zeros(binRoadImage.rows, binRoadImage.cols, CV_8U);
	int flagCopy=0;
		for (int y = 0;y<binRoadImage.size().height; y++)
		{
			if(y==imageCenter)
			{
				while((binRoadImage.at<uchar>(centerIncrement,xCenter)==0))
				{
						for (xCenter = 0; xCenter<binRoadImage.size().width; xCenter++)
					    {
								tempResultLowerPortion.at<uchar>(centerIncrement, xCenter) = 225;  // these threshold values must be tuned or determined automatically!
						}
						xCenter=0;
					    centerIncrement++;
						if(centerIncrement==(imageCenter+80))
						{
							flagCopy=1;
							break;
						}
				}
				if(flagCopy==0)
				{
					binRoadImage=binRoadImage+tempResultLowerPortion;
				}
				xCenter=0;
				flagCopy=0;
				while( (binRoadImage.at<uchar>(centerDecrement,xCenter)==0))
				{
					for (xCenter = 0; xCenter<binRoadImage.size().width; xCenter++)
					{
							tempResultUpperPortion.at<uchar>(centerDecrement, xCenter) = 225;  // these threshold values must be tuned or determined automatically!
					}
					xCenter=0;
					centerDecrement--;		
				    if(centerDecrement==(imageCenter-80))
					{
						flagCopy=1;
						break;
					}
				}
				if(flagCopy==0)
				{
					binRoadImage=binRoadImage+tempResultUpperPortion;
				}
				flagCopy=0;
			}
		}
		//imshow("tempResultUpperPortion", tempResultUpperPortion);
		//imshow("tempResulLowerPortion", tempResultLowerPortion);
		return binRoadImage;
		
}
Rect roadDetectionHorizontal(Mat labRoadImage)
{
	int x, y;
	double area;
	Rect bounding_rect;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	Mat origFrame = labRoadImage.clone();
	Mat dst;
	Mat im;

		im = labThresholdingStraight(labRoadImage);


	imshow("threshold", im);
	// Create a structuring element (SE)
	
	int counter = 0;
	int arrayPercentH[1280];//only 720 height is needed
							//int arrayPercentV[720];
	float percentage;
	x = 0;
	//FILE *f;
	//f = fopen("frequency.txt", "w");
	/*****for horizontal*****/

	for (int y = 0; y<im.size().height; y++)
	{
		for (x = 0, counter = 0; x<im.size().width; x++)
		{
			if (im.at<uchar>(y, x) == 255)  // these threshold values must be tuned or determined automatically!
			{
				counter++;
			}
		}
		percentage = (float)counter / im.size().width;
		//printf("y:%d, percentage: %f\n",y,percentage);
		//fprintf(f, " %d, %f\n", y, percentage);

		if (percentage >= 0.50)
		{
			arrayPercentH[y] = 255;
		}
		else
		{
			arrayPercentH[y] = 0;
		}


		//imshow("im",im);

	}
	/*****for horizontal*****/


	/***for horizontal**/
	for (int y = 0; y<im.size().height; y++)
	{
		for (x = 0; x<im.size().width; x++)
		{
			im.at<uchar>(y, x) = arrayPercentH[y];  // these threshold values must be tuned or determined automatically!
		}
	}
	/***for horizontal**/

	
	//imshow("bin before adding white",im);
	
	
	//}
	addCenterLinesHorizontal(im);
	//imshow("bin after adding white",im);
	
	int imageCenter=im.size().height/2;
	int dilate_size = 2;
	Mat dilateElement = getStructuringElement( MORPH_RECT, Size( 2*dilate_size + 1, 2*dilate_size+1 ), Point( dilate_size, dilate_size ) );
		
				for (int i = 0; i<4; i++)
				{
					//Apply erosion or dilation on the image
					dilate(im, im, dilateElement);
				}
			
			

		
	//imshow("Result Road Dilate",im);

	double largest_area = 0;
	double a;
	int largest_contour_index;


     findContours(im, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	 for (int i = 0; i< contours.size(); i++)
	 {
		a = contourArea(contours[i], false);  //  Find the area of contour
		if (a>largest_area)
		{
			largest_area = a;
			largest_contour_index = i;                //Store the index of largest contour
			bounding_rect = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
		}
	}
	//drawContours(origFrame, contours, largest_contour_index, Scalar(255), 1, 8, hierarchy);
	//imshow("largest area horizontal", origFrame);
	return bounding_rect;
}

Rect roadDetectionVertical(Mat labRoadImage)
{
	int x, y;
	double area;
	Rect bounding_rect;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	//Mat origFrame = roadImage.clone();
	//imshow("origFrame", origFrame);
	Mat dst;

	Mat im(labRoadImage.size().height, labRoadImage.size().width, CV_8UC1, Scalar(0));

	im = labThresholdingStraight(labRoadImage);
	imshow("threshold",im);

	int counter = 0;
	int arrayPercentV[1280];
	float percentage;

	/*****for vertical*****/

	for (int x = 0; x<im.size().width; x++)
	{
		for (y = 0, counter = 0; y<im.size().height; y++)
		{
			if (im.at<uchar>(y, x) == 255)  // these threshold values must be tuned or determined automatically!
			{
				counter++;
			}
		}
		percentage = (float)counter / im.size().height;

		if (percentage >= 0.50)
		{
			arrayPercentV[x] = 255;
		}
		else
		{
			arrayPercentV[x] = 0;
		}

	}
	/*****for vertical*****/
	for (int x = 0; x<im.size().width; x++)
	{
		for (y = 0; y<im.size().height; y++)
		{
			im.at<uchar>(y, x) = arrayPercentV[x];  // these threshold values must be tuned or determined automatically!
		}
	}
	/***for vertical**/


	imshow("bin reaches percentage",im);
	addCenterLinesVeritcal(im);
	//imshow("bin after adding white",im);
	int dilate_size = 2;
	Mat dilateElement = getStructuringElement( MORPH_RECT, Size( 2*dilate_size + 1, 2*dilate_size+1 ), Point( dilate_size, dilate_size ) );
	for (int i = 0; i<4; i++)
    {
			//Apply erosion or dilation on the image
			dilate(im, im, dilateElement);
	}
	double largest_area = 0;
	double a;
	int largest_contour_index;


     findContours(im, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
	 for (int i = 0; i< contours.size(); i++)
	 {
		a = contourArea(contours[i], false);  //  Find the area of contour
		if (a>largest_area)
		{
			largest_area = a;
			largest_contour_index = i;                //Store the index of largest contour
			bounding_rect = boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
		}
		 drawContours( im, contours,largest_contour_index, Scalar(255, 255, 255), CV_FILLED);	
	}
	
	 imshow("largest area found vertical",im);
	return bounding_rect;
}


Scalar speedSpectrum(double speed)
{
	double R = 0.05 * (40 - speed);
	double G = 0.05 * speed;
	R = (R > 1) ? 1 : (R < 0.06) ? 0 : R;
	G = (G > 1) ? 1 : (G < 0.06) ? 0 : G;

	return Scalar(0.0, G * 255, R * 255);
}

static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/)
{
	if (event == EVENT_LBUTTONDOWN)
	{
		point = Point2f((float)x, (float)y);
		std::cout << "x: " << x << " y: " << y << endl;
		//addRemovePt = true;
	}
}

void setLabel(cv::Mat& im, const std::string label, const cv::Point & pointor )
{
	int fontface = cv::FONT_HERSHEY_TRIPLEX;
	double scale = 0.6;
	int thickness = 1;
	int baseline = 0;

	cv::Size text = cv::getTextSize(label, fontface, scale, thickness, &baseline);
	cv::rectangle(im, pointor +cv::Point(0, baseline), pointor +cv::Point(text.width, -text.height), CV_RGB(255, 255, 255), CV_FILLED);
	cv::putText(im, label, pointor, fontface, scale, CV_RGB(0, 0, 0), thickness, 8);
}

Point2i vehicleCountTopAndBot(vector<Vehicle> &vehicles, int halfY)
{
	int Top = 0;
	int Bot = 0;
	for (int i = 0; i < vehicles.size(); i++) {
		if (vehicles.at(i).getYPos() < halfY)
		{
			Top++;
		}
		else
		{
			Bot++;
		}
	}

	return Point2i(Top, Bot);
}

Mat sobelDetection(Mat curFrame)
{
	//sobel parameters
	Mat gray;
	int i, j;
	Mat grad;
	Mat grad_x, grad_y;
	Mat abs_grad_x, abs_grad_y;
	int scale = 1;
	int delta = 0;
	int ddepth = CV_16S;
	Mat curFrameCpy = curFrame.clone();
	Mat sobelResult = curFrame.clone();
	Mat thresholdImage;
	vector< vector<Point> > contours;
	vector<Vec4i> hierarchy;
	//find contours of filtered image using openCV findContours function
	vector<vector<Point> > contours_poly(contours.size());
	vector<Rect> boundRect(contours.size());
	double minVal, maxVal;
	minMaxLoc(curFrameCpy, &minVal, &maxVal);  //find  minimum  and  maximum  intensities
	curFrameCpy.convertTo(gray, CV_8U, 255.0 / (maxVal - minVal), -minVal);
	GaussianBlur(gray, gray, Size(15, 15), 0, 0, 1);
	//imshow("gray",gray);
	Sobel(gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT);
    convertScaleAbs(grad_x, abs_grad_x);
	Sobel(gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT);
	convertScaleAbs(grad_y, abs_grad_y);
	addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);
	//cv::imshow("Sobel Image", grad);
	//cv::threshold(abs_grad_y, thresholdImage, 30, 255, THRESH_BINARY);
	cv::threshold(grad, thresholdImage, 25, 255, THRESH_BINARY);
	//cv::imshow("Threshold Image", thresholdImage);
	morphologyEx(thresholdImage, thresholdImage, MORPH_OPEN, Mat::ones(2, 2, CV_8SC1), Point(1, 1), 2);
	//cv::imshow("Sobel Morphed Image", thresholdImage);
	
	return thresholdImage;

}
