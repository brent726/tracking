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
const int MIN_OBJECT_AREA = 5*5;

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
Mat getLucasKanadeOpticalFlow(Mat &img1, Mat &img2, Mat &u, Mat &v){
	
    Mat fx = get_fx(img1, img2);
     Mat fy = get_fy(img1, img2);
     Mat ft = get_ft(img1, img2);
	 //imshow("fx",fx);
	Mat fx2 = fx.mul(fx);
	 //imshow("fx2",fx2);
	 int dilate_size = 3;  
     Mat dilateElement = getStructuringElement(cv::MORPH_RECT,Size(2 * dilate_size + 1, 2* dilate_size + 1),Point(dilate_size, dilate_size) );
	 dilate(fx,fx,dilateElement); 
	 imshow("dilate lines fx",fx);
     //Mat fy2 = fy.mul(fy);
	 //imshow("fy2",fy2);
     //Mat fxfy = fx.mul(fy);
	 //imshow("fxfy",fxfy);
     //Mat fxft = fx.mul(ft);
	 //imshow("fxft",fxft);

     //dilateElement = getStructuringElement(cv::MORPH_RECT,Size(2 * dilate_size + 1, 2* dilate_size + 1),Point(dilate_size, dilate_size) );
	 //dilate(fxft,fxft,dilateElement); 
	 //imshow("dilate lines fxft",fxft);
    /* Mat fyft = fy.mul(ft);
	 imshow("fyft",fyft);*/

	 /*Mat sumfx2 = get_Sum9_Mat(fx2);
	 imshow("sumfx2",sumfx2);
	 Mat sumfy2 = get_Sum9_Mat(fy2);
	 imshow("sumfy2",sumfy2);
	 Mat sumfxft = get_Sum9_Mat(fxft);
	 imshow("sumfxft",sumfxft);
     Mat sumfxfy = get_Sum9_Mat(fxfy);
	 imshow("sumfxfy",sumfxfy);
     Mat sumfyft = get_Sum9_Mat(fyft);

	 Mat tmp = sumfx2.mul(sumfy2) - sumfxfy.mul(sumfxfy);
	 imshow("tmp",tmp);
    u = sumfxfy.mul(sumfyft) - sumfy2.mul(sumfxft);
    v = sumfxft.mul(sumfxfy) - sumfx2.mul(sumfyft);
	imshow("u",u);
	imshow("v",v);
    divide(u, tmp, u);
    divide(v, tmp, v);*/
	 return fx;
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

					//if(width>50 && width <140 && height>30 && height<50)
					if(width>50 &&width<150 && height>30&&height<50)
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
int main(int argc, char** argv) {

  //if( argc != 3 ) { help( argv ); exit( -1 ); }

  // Initialize, load two images from the file system, and
  // allocate the images and other structures we will need for
  // results.
  //
  //Mat imgA = imread( "newStock2.png");
  //Mat imgAOrig=imgA.clone();
  
  //Mat imgB = imread( "newStock1.png");
  bool pause=false;
 
  Mat curFrame, prevFrame,prevResultFrame, curResultFrame;
  VideoCapture cap("C:\\Users\\PCBLAB_01\\Desktop\\sampleVideo.avi");
 // VideoCapture cap("\\\\Mac\\Home\\Desktop\\DroneVideos\\Thesis\\sampleVideo.avi");
  double fps = cap.get(CV_CAP_PROP_FPS);
  int framepos;
  Size img_sz;
  Mat frameCurOrig;
  int i,j;
  Mat gray;
  while(true)
  {
	      
		  cap >> curFrame;
		  frameCurOrig=curFrame.clone();
		  framepos=(int)cap.get(CV_CAP_PROP_POS_FRAMES);
		  curResultFrame = curFrame.clone();
		  
		  //cvtColor( curFrame, curFrame, COLOR_BGR2GRAY); 
	
		  curFrame.convertTo(curFrame, CV_64FC1, 1.0/255);

		  if(!curFrame.empty())
		  {
			 //curFrame.convertTo(curFrame, CV_64FC1,1.0/255);
		
			//prevFrame.convertTo(prevFrame, CV_64FC1, 1.0/255.0, 0);
			 //curFrame.convertTo(curFrame, CV_8U, 1.0/255, 0);
			imshow("gray", curFrame);
			
					  /*for(i=0;i<curFrame.rows;i++)
					  {
						for(j=0;j<curFrame.cols;j++)
						{
							printf("curframe: %d",curFrame.ATD(i,j));
						
						}
					 }*/
		  }
		if(framepos!=1)
		 {
				 
				 img_sz = curFrame.size();
				  
				//imshow("curFrame",curFrame);
				//Mat u = Mat::zeros(prevFrame.rows, prevFrame.cols, CV_64FC1);
				//Mat v = Mat::zeros(prevFrame.rows, prevFrame.cols, CV_64FC1);
				Mat u = Mat::zeros(prevFrame.rows, prevFrame.cols, CV_8U);
				Mat v = Mat::zeros(prevFrame.rows, prevFrame.cols, CV_8U);

				Mat LKResultImage=getLucasKanadeOpticalFlow(prevFrame, curFrame, u, v);
				//Mat bwLKResult=Mat::zeros(prevFrame.rows, prevFrame.cols, CV_64FC1);
				//Mat bwLKResult=Mat::zeros(prevFrame.rows, prevFrame.cols, CV_64FC1);
				/*Mat bwLKResult=Mat::zeros(prevFrame.rows, prevFrame.cols, CV_8U);
				printf("rows:%d cols: %d",curFrame.rows,curFrame.cols);
				for(i=0;i<curFrame.rows;i++)
				{
					for(j=0;j<curFrame.cols;j++)
					{
					    //printf("curframe: %d",curFrame.ATD(i,j));
						//system("PAUSE");
						if(curFrame.ATD(i,j)>5)
						{
							bwLKResult.ATD(i,j)=255;
						}
					}
				}*/
				
				//imshow("bw",bwLKResult);
		  }
		 
			  
			 // prevFrame=curFrame;
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

