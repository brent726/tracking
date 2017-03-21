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
Rect roadDetectionHorizontal(Mat roadImage, int intersection);
Rect roadDetectionVertical(Mat roadImage);
Scalar speedSpectrum(double speed);
void setLabel(cv::Mat& im, const std::string label, const cv::Point & pointor);
Point2i vehicleCountTopAndBot(vector<Vehicle> &vehicles, int halfY);

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

Mat get_fx(Mat &src1, Mat &src2)
{
	Mat fx;
	Mat kernel= Mat::ones(2,2,CV_64FC1);
	//Mat kernel= Mat::ones(2,2,CV_8U);
	kernel.ATD(0,0)=-1.0;
	kernel.ATD(1, 0) = -1.0;


	Mat dst1, dst2;
	filter2D(src1,dst1,-1,kernel);
	filter2D(src2, dst2,-1, kernel );

	fx=dst1+dst2;
	//imshow("fx dst1",dst1);
	//imshow("fx",fx);
	return fx;

}

Mat get_fy(Mat &src1, Mat &src2){
    Mat fy;
    Mat kernel = Mat::ones(2, 2, CV_64FC1);
	//Mat kernel = Mat::ones(2, 2, CV_8U);
    kernel.ATD(0, 0) = -1.0;
    kernel.ATD(0, 1) = -1.0;

    Mat dst1, dst2;
    filter2D(src1, dst1, -1, kernel);
    filter2D(src2, dst2, -1, kernel);
	 
    fy = dst1 + dst2;

	//imshow("fy dst1",dst1);
	//imshow("fy",fy);
    return fy;
}

Mat get_ft(Mat &src1, Mat &src2){
    Mat ft;
    Mat kernel = Mat::ones(2, 2, CV_64FC1);
	//Mat kernel = Mat::ones(2, 2, CV_8U);
    kernel = kernel.mul(-1);

    Mat dst1, dst2;
    filter2D(src1, dst1, -1, kernel);
    kernel = kernel.mul(-1);
    filter2D(src2, dst2, -1, kernel);

    ft = dst1 + dst2;
	//imshow("ft dst1",dst1);
	//imshow("ft dst2",dst2);
	//imshow("ft",ft);
    return ft;
}
bool isInsideImage(int y, int x, Mat &m){
    int width = m.cols;
    int height = m.rows;
    if(x >= 0 && x < width && y >= 0 && y < height) return true;
    else return false;
}

double get_Sum9(Mat &m, int y, int x)
{
	if(x < 0 || x >= m.cols) return 0;
    if(y < 0 || y >= m.rows) return 0;

	double val=0.0;
	int tmp=0;
	if(isInsideImage(y-1,x-1,m))
	{
		++tmp;
		val+=m.ATD(y-1,x-1);
	}
	if(isInsideImage(y - 1, x, m)){
        ++ tmp;
        val += m.ATD(y - 1, x);
    }
	 if(isInsideImage(y - 1, x + 1, m)){
        ++ tmp;
        val += m.ATD(y - 1, x + 1);
    }
    if(isInsideImage(y, x - 1, m)){
        ++ tmp;
        val += m.ATD(y, x - 1);
    }
    if(isInsideImage(y, x, m)){
        ++ tmp;
        val += m.ATD(y, x);
    }
    if(isInsideImage(y, x + 1, m)){
        ++ tmp;
        val += m.ATD(y, x + 1);
    }
    if(isInsideImage(y + 1, x - 1, m)){
        ++ tmp;
        val += m.ATD(y + 1, x - 1);
    }
    if(isInsideImage(y + 1, x, m)){
        ++ tmp;
        val += m.ATD(y + 1, x);
    }
    if(isInsideImage(y + 1, x + 1, m)){
        ++ tmp;
        val += m.ATD(y + 1, x + 1);
    }
    if(tmp == 9) return val;
    else return m.ATD(y, x) * 9;
}
Mat get_Sum9_Mat(Mat &m){
   Mat res = Mat::zeros(m.rows, m.cols, CV_64FC1);
   //Mat res = Mat::zeros(m.rows, m.cols, CV_8U);
    for(int i = 1; i < m.rows - 1; i++){
        for(int j = 1; j < m.cols - 1; j++){
            res.ATD(i, j) = get_Sum9(m, i, j);
        }
    }
    return res;
}
Mat getLucasKanadeOpticalFlow(Mat &img1, Mat &img2, Mat u, Mat v){
	
    Mat fx = get_fx(img1, img2);
    Mat ft = get_ft(img1, img2);
	Mat fy = get_fy(img1, img2);
	 //imshow("ft",ft);
	//imshow("fx",fx);
	//imshow("fx",fx);
	//imshow("fy",fy);
	// Mat ft8u;
	 //ft.convertTo(ft8u,  CV_8U,  255.0/(maxVal  -  minVal),  -minVal);
	 
     Mat fx2 = fx.mul(fx);
	 //imshow("fx2",fx2);
	 //imshow("dilate lines fx",fx);
     Mat fy2 = fy.mul(fy);
	 //imshow("fy2",fy2);
     Mat fxfy = fx.mul(fy);
	//imshow("fxfy",fxfy);
    Mat fxft = fx.mul(ft);
	//imshow("fxft",fxft);

	 
    Mat fyft = fy.mul(ft);
	 //imshow("fyft",fyft);
	 int dilate_size = 2;  
    Mat dilateElement = getStructuringElement(cv::MORPH_RECT,Size(2 * dilate_size + 1, 2* dilate_size + 1),Point(dilate_size, dilate_size) );
	dilate(fyft,fyft,dilateElement); 
   //imshow("dilate lines fyft",fyft);
	 Mat sumfx2 = get_Sum9_Mat(fx2);
	imshow("sumfx2",sumfx2);
	Mat sumfy2 = get_Sum9_Mat(fy2);
	 //imshow("sumfy2",sumfy2);
	Mat sumfxft = get_Sum9_Mat(fxft);
	//imshow("sumfxft",sumfxft);
    Mat sumfxfy = get_Sum9_Mat(fxfy);
	 //imshow("sumfxfy",sumfxfy);
     Mat sumfyft = get_Sum9_Mat(fyft);

	 //imshow("resultn sum fx and fy", resultfx2fy2);
	Mat tmp = sumfx2.mul(sumfy2) - sumfxfy.mul(sumfxfy);
	// imshow("tmp",tmp);
    u = sumfxfy.mul(sumfyft) - sumfy2.mul(sumfxft);
	v = sumfxft.mul(sumfxfy) - sumfx2.mul(sumfyft);
	//imshow("v",v);
    divide(u, tmp, u);
    divide(v, tmp, v);
	//imshow("u",u);
	
	 return sumfx2;
}
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
		rectangle( frame, VehicleCars.at(i).getTl(), VehicleCars.at(i).getBr(), cv::Scalar(0,255,0), 1, 8, 0 );
		//rectangle( frame, VehicleCars.at(i).getTl(), VehicleCars.at(i).getBr(), cv::Scalar(255), CV_FILLED);
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
Mat sobelDetection(Mat curFrame)
{
	//sobel parameters
				Mat gray;
				int i,j;
				Mat grad;
				Mat grad_x, grad_y;
				Mat abs_grad_x, abs_grad_y;
				int scale = 1;
				int delta = 0;
				int ddepth = CV_16S;
				Mat curFrameCpy=curFrame.clone();
				Mat sobelResult=curFrame.clone();
				Mat thresholdImage;
				vector< vector<Point> > contours;
				vector<Vec4i> hierarchy;
				//find contours of filtered image using openCV findContours function
			    vector<vector<Point> > contours_poly( contours.size() );
			    vector<Rect> boundRect( contours.size() );
				double minVal, maxVal;
				//sobel 
				minMaxLoc(curFrameCpy,  &minVal,  &maxVal);  //find  minimum  and  maximum  intensities
				curFrameCpy.convertTo(gray,  CV_8U,  255.0/(maxVal  -  minVal),  -minVal);

				GaussianBlur(gray, gray, Size(15,15), 0, 0, 1); //13, 13
				//imshow("gray",gray);
				//Sobel( gray, grad_x, ddepth, 1, 0, 3, scale, delta, );
				//Sobel( gray, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
				//convertScaleAbs( grad_x, abs_grad_x );
				//Sobel( gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
				Sobel( gray, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
				convertScaleAbs( grad_y, abs_grad_y );
				//addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
				//cv::imshow("Sobel X", abs_grad_x);
				//cv::imshow("Sobel Y", abs_grad_y);
				//cv::imshow("Sobel Image", grad);
				cv::threshold(abs_grad_y,thresholdImage,30,255,THRESH_BINARY);
				//cv::threshold(abs_grad_y,thresholdImage,20,255,THRESH_BINARY);
				//cv::imshow("Threshold Image", thresholdImage);
			  
				morphologyEx(thresholdImage,thresholdImage,MORPH_OPEN,Mat::ones(2,2,CV_8SC1),Point(1,1),2);

				//cv::imshow("Sobel Morphed Image", thresholdImage);

				return thresholdImage;

}
Mat LKDetection(Mat prevFrame, Mat curFrame, Mat u, Mat v)
{
				int i;
				 Mat thresholdImage;
				 Mat frameCurOrig;
				Size img_sz = curFrame.size();
				frameCurOrig=curFrame.clone(); 
				//imshow("curFrame",curFrame);
				//Mat u = Mat::zeros(prevFrame.rows, prevFrame.cols, CV_64FC1);
				//Mat v = Mat::zeros(prevFrame.rows, prevFrame.cols, CV_64FC1);
				//Mat u = Mat::zeros(prevFrame.rows, prevFrame.cols, CV_8U);
				//Mat v = Mat::zeros(prevFrame.rows, prevFrame.cols, CV_8U);

				/***************LK**********************/
				Mat grayLK;
				double  minVal,  maxVal;
				curFrame.convertTo(curFrame, CV_64F, 1.0/255);
				prevFrame.convertTo(prevFrame, CV_64F, 1.0/255);
				GaussianBlur(prevFrame, prevFrame, Size(3, 3), 0, 0, BORDER_DEFAULT);
				GaussianBlur(curFrame, curFrame, Size(3, 3), 0, 0, BORDER_DEFAULT);
				//imshow("curFrame", curFrame);
				Mat LKResultImage=getLucasKanadeOpticalFlow(prevFrame, curFrame, u, v);
				//minMaxLoc(LKResultImage,  &minVal,  &maxVal);  //find  minimum  and  maximum  intensities
				//LKResultImage.convertTo(grayLK,  CV_8U,  255.0/(maxVal  -  minVal),  -minVal);
			
				// imshow("gray LK",grayLK);
				// cv::threshold(grayLK,thresholdImage,18,255,THRESH_BINARY);
				//cv::threshold(grayLK,thresholdImage,10,255,THRESH_BINARY);
				//cv::imshow("LK Threshold Image", thresholdImage);
				
				return LKResultImage;
			

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
 //VideoCapture cap("\\\\Mac\\Home\\Desktop\\DroneVideos\\DJI_0005Take2.MP4");
// VideoCapture cap("C:\\Users\\PCBLAB_01\\Desktop\\dji\\DJI_0005Take2.MP4");
  //DJI_0009Take3.MP4 DJI_0010Take4.MP4 DJI_0005Take2.MP4 DJI_0011Take5.MP4 DJI_0008Take1.MP4
 VideoCapture cap("C:\\Users\\PCBLAB_01\\Desktop\\dji\\DJI_0010Take4.MP4");

  double fps = cap.get(CV_CAP_PROP_FPS);
  int framepos=0;
  Size img_sz;
  Mat origFrame;
  double i,j;
  Mat gray;
  Mat sobelImage, LKImage,comboResultImage;
  int intersection=0;
  Mat image;
  while(true)
  {
	      
		  cap >> image;
		  image.copyTo(origFrame);// for final output
		 framepos=(int)cap.get(CV_CAP_PROP_POS_FRAMES);
		  //road detection
		/*if (framepos == 1 || framepos % 10 == 0)
		{*/
			HroadBorder = roadDetectionHorizontal(image, intersection);
			HroadTopX = HroadBorder.tl().x;
			HroadTopY = HroadBorder.tl().y;
			HroadBotX = HroadBorder.br().x;
			HroadBotY = HroadBorder.br().y;
			HROAD = Rect(HroadTopX, HroadTopY, (HroadBotX - HroadTopX), (HroadBotY - HroadTopY));//horizontal road dimensions

			if(intersection == 1)
			{ 
				VroadBorder = roadDetectionVertical(image);
				VroadTopX = VroadBorder.tl().x;
				VroadTopY = VroadBorder.tl().y;
				VroadBotX = VroadBorder.br().x;
				VroadBotY = VroadBorder.br().y;
				VROAD = Rect(VroadTopX, VroadTopY, (VroadBotX - VroadTopX), (VroadBotY - VroadTopY));//vertical road dimensions
			}


		//}


		//horizontal
		int halfY = (HroadBorder.br().y + HroadBorder.tl().y) / 2;
		Point HhalfFirstpoint = Point(HroadBorder.tl().x, halfY);
		Point HhalfSecondPoint = Point(HroadBorder.br().x, halfY);
		
		//draw rectangular shape of result of road detection
		rectangle(image, HroadBorder.tl(), HroadBorder.br(), Scalar(255, 0, 0), 1, 8, 0);
		//line(image, HhalfFirstpoint, HhalfSecondPoint, Scalar(255, 0, 0), 1, 8, 0);
		image.copyTo(curFrame);
		cvtColor( curFrame, curFrame, COLOR_BGR2GRAY);

		/*********for horizontal to blacken the sides*********/
		//printf("tl=%d, br=%d", HroadBorder.tl().y, HroadBorder.br().y);
		for (i = 0;i<curFrame.size().width;i++)
		{
			for (j = 0;j<curFrame.size().height;j++)
			{
				if ((j < HroadBorder.tl().y)||(j > HroadBorder.br().y))  // these threshold values must be tuned or determined automatically!
				{	
					curFrame.at<uchar>(j, i)= 0;
				}
			}
		}
		//imshow("curFrame black", curFrame);
		/*********for horizontal to blacken the sides*********/
		if(framepos!=1)
		 {
				 Mat u = Mat::zeros(prevFrame.rows, prevFrame.cols, CV_64FC1);
				 Mat v = Mat::zeros(prevFrame.rows, prevFrame.cols, CV_64FC1);
				
				  LKImage = LKDetection(prevFrame, curFrame, u, v);
					
			      sobelImage=sobelDetection(curFrame);
				// imshow("sobel", sobelImage);
				 //comboResultImage= Mat::zeros(origFrame.rows, origFrame.cols, CV_8U);
				  comboResultImage=LKImage+sobelImage;
				// Mat tempResultBW;
				 //comboResultImage.copyTo(tempResultBW);
				morphologyEx(comboResultImage,comboResultImage,MORPH_OPEN,Mat::ones(2,2,CV_8SC1),Point(1,1),BORDER_DEFAULT);
			   imshow("combo result",comboResultImage);
				
				//searchForVehicle(comboResultImage, tempResultBW);
				//searchForVehicle(comboResultImage, origFrame);
				//imshow("detect result", origFrame);

			
				
		 }
		
			prevFrame=curFrame;
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
Mat labThresholdingIntersection(Mat labRoadImage)
{
	Mat im(labRoadImage.size().height, labRoadImage.size().width, CV_8UC1, Scalar(0));
	 for (int x = 0;x<labRoadImage.size().width;x++)
	{
		for (int y = 0;y<labRoadImage.size().height;y++)
		{
			if ((labRoadImage.at<Vec3b>(y, x)[1]>120) && (labRoadImage.at<Vec3b>(y, x)[1]<135))
			{
				if ((labRoadImage.at<Vec3b>(y, x)[2]>120) && (labRoadImage.at<Vec3b>(y, x)[2]<135))
				{
					//changing the pixel intensity to white
					im.at<uchar>(y, x) = 255;
				}
			}
		}
	}
	 return im;
}
Mat labThresholdingStraight(Mat labRoadImage)
{
	int x, y;
	Mat im(labRoadImage.size().height, labRoadImage.size().width, CV_8UC1, Scalar(0));
	 for (x = 0;x<labRoadImage.size().width;x++)
	{
		for (y = 0;y<labRoadImage.size().height;y++)
		{
			if ((labRoadImage.at<Vec3b>(y, x)[1]>122) && (labRoadImage.at<Vec3b>(y, x)[1]<131))  // straight these threshold values must be tuned or determined automatically!
			{
				if ((labRoadImage.at<Vec3b>(y, x)[2]>118) && (labRoadImage.at<Vec3b>(y, x)[2]<129)) //straight these threshold values must be tuned or determined automatically!
				{
					//changing the pixel intensity to white
					im.at<uchar>(y, x) = 255;
				}
			}
		}
	}
	 return im;

}
Rect roadDetectionHorizontal(Mat roadImage, int intersection)
{
	int x, y;
	double area;
	Rect bounding_rect;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;

	Mat dst;
	Mat labRoadImage;
	Mat im(roadImage.size().height, roadImage.size().width, CV_8UC1, Scalar(0));
	Mat dstImg(roadImage.size().height, roadImage.size().width, CV_8UC1, Scalar(0));

	Mat origFrame=roadImage.clone();
	Mat gradImage;
	Mat roadImageBlur;
	GaussianBlur(roadImage, roadImageBlur, Size(17, 17), 0, 0, BORDER_DEFAULT);
	/// Total Gradient (approximate)
	//addWeighted(roadImage, 1, roadBlurImage, 1, 0, gradImage);

	//imshow( "Road Image Sobel", roadImage );
	cvtColor(roadImageBlur, labRoadImage, COLOR_BGR2Lab);
	//imshow("lab uimage", labRoadImage);
	
   if(intersection==0)
   {
	   im=labThresholdingStraight(labRoadImage);
   }
   else if(intersection==1)
   {
		im= labThresholdingIntersection(labRoadImage);
   }

	//imshow("threshold", im);
	// Create a structuring element (SE)
	int morph_size = 2;
	Mat element = getStructuringElement(MORPH_RECT, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));

	for (int i = 0;i<2;i++)
	{
		morphologyEx(im, im, 2, element, Point(-1, -1), i);
	}
	
	//Canny(im,im,50,150);
	//imshow("Canny Image", im);
	//imshow("Image", im);
	int counter = 0;
	int arrayPercentH[1280];//only 720 height is needed
	//int arrayPercentV[720];
	float percentage;
	x = 0;
	//FILE *f;
	//f = fopen("frequency.txt", "w");
	/*****for horizontal*****/
	
	for (int y = 0;y<im.size().height;y++)
	{
		for (x = 0, counter = 0;x<im.size().width;x++)
		{
			if (im.at<uchar>(y, x) == 255)  // these threshold values must be tuned or determined automatically!
			{
				counter++;
			}
		}
		percentage = (float)counter / im.size().width;
		//printf("y:%d, percentage: %f\n",y,percentage);
		//fprintf(f, " %d, %f\n", y, percentage);
		
		if (percentage >= 0.60)
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
	for (int y = 0;y<im.size().height;y++)
	{
		for (x = 0;x<im.size().width;x++)
		{
			im.at<uchar>(y, x) = arrayPercentH[y];  // these threshold values must be tuned or determined automatically!
		}
	}
	/***for horizontal**/
	

	//imshow("bin",im);

	int dilate_size = 2;
	Mat dilateElement = getStructuringElement(cv::MORPH_RECT, Size(2 * dilate_size + 1, 2 * dilate_size + 1), Point(dilate_size, dilate_size));

	for (int i = 0;i<6;i++)
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

Rect roadDetectionVertical(Mat roadImage)
{
	int x, y;
	double area;
	Rect bounding_rect;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	Mat origFrame=roadImage.clone();
	//imshow("origFrame", origFrame);
	Mat dst;
	Mat labRoadImage;
	Mat im(roadImage.size().height, roadImage.size().width, CV_8UC1, Scalar(0));
	Mat dstImg(roadImage.size().height, roadImage.size().width, CV_8UC1, Scalar(0));

	Mat roadImageBlur;
	GaussianBlur(roadImage, roadImageBlur, Size(3, 3), 0, 0, BORDER_DEFAULT);
	
	cvtColor(roadImageBlur, labRoadImage, COLOR_BGR2Lab);
	im= labThresholdingIntersection(labRoadImage);
	// Create a structuring element (SE)
	int morph_size = 2;
	Mat element = getStructuringElement(MORPH_RECT, Size(2 * morph_size + 1, 2 * morph_size + 1), Point(morph_size, morph_size));

	for (int i = 0; i<2; i++)
	{
		morphologyEx(im, im, 2, element, Point(-1, -1), i);
	}

	//Canny(im,im,50,150);
	//imshow("Canny Image", im);
	//imshow("Image", im);
	int counter = 0;
	//int arrayPercentH[1280];
	int arrayPercentV[1280];
	float percentage;
	x = 0; y = 0;
	//FILE *f;
	//f = fopen("frequency.txt", "w");
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

		if (percentage >= 0.60)
		{
			arrayPercentV[x] = 255;
		}
		else
		{
			arrayPercentV[x] = 0;
		}


		//imshow("im",im);

	}
	/*****for vertical*****/


	/***for vertical**/
	for (int x = 0; x<im.size().width; x++)
	{
		for (y = 0; y<im.size().height; y++)
		{
			im.at<uchar>(y, x) = arrayPercentV[x];  // these threshold values must be tuned or determined automatically!
		}
	}
	/***for verticaltal**/


	//imshow("bin",im);

	int dilate_size = 2;
	Mat dilateElement = getStructuringElement(cv::MORPH_RECT, Size(2 * dilate_size + 1, 2 * dilate_size + 1), Point(dilate_size, dilate_size));

	for (int i = 0; i<6; i++)
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
	drawContours(origFrame, contours, largest_contour_index, Scalar(255), 1, 8, hierarchy);
	imshow("largest area vertical", origFrame);
	//imshow("largest area", dstImg);
	return bounding_rect;
}
