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

void help( char** argv ) {
  cout << "Call: " <<argv[0] <<" [image1] [image2]" << endl;
  cout << "Demonstrates Pyramid Lucas-Kanade optical flow." << endl;
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
  double fps = cap.get(CV_CAP_PROP_FPS);
  int framepos;
  Size img_sz;
  Mat frameCurOrig;
  while(true)
  {
		  cap >> curFrame;
		  frameCurOrig=curFrame.clone();
		  framepos=(int)cap.get(CV_CAP_PROP_POS_FRAMES);
		  curResultFrame = curFrame.clone();
		  cvtColor(curFrame, curFrame, COLOR_BGR2GRAY); 
		 // imshow("current frame",curFrame);
		  if(framepos!=1)
		  {
				 //imshow("prev frame",prevFrame);
				  img_sz = curFrame.size();
				  int win_size = 10;
				  prevResultFrame = prevFrame.clone();
				  //curResultFrame = curFrame.clone();
				  //cvtColor(prevFrame, prevFrame, COLOR_BGR2GRAY); 
				  //cvtColor(curFrame, curFrame, COLOR_BGR2GRAY); 
				  //imshow("cur frame",curFrame);
				  //imshow("prev frame",prevFrame);
				  
				  // The first thing we need to do is get the features
				  // we want to track.
				  //
				  vector<Point2f > cornersA, cornersB;
				  const int MAX_CORNERS = 500;
				  cv::goodFeaturesToTrack(
					curFrame,                         // Image to track
					cornersA,                     // Vector of detected corners (output)
					MAX_CORNERS,                  // Keep up to this many corners
					0.01,                         // Quality level (percent of maximum)
					5,                            // Min distance between corners
					noArray(),                // Mask
					3,                            // Block size
					false,                        // true: Harris, false: Shi-Tomasi
					0.04                          // method specific parameter
				  );

				  cornerSubPix(
					curFrame,                         // Input image
					cornersA,                     // Vector of corners (input and output)
					Size(win_size, win_size), // Half side length of search window
					Size(-1,-1),              // Half side length of dead zone (-1=none)
					TermCriteria(
					  TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
					  20,                         // Maximum number of iterations
					  0.03                        // Minimum change per iteration
					)
				  );

				  // Call the Lucas Kanade algorithm
				  //
				  vector<uchar> features_found;
				  cv::calcOpticalFlowPyrLK(
					prevFrame,                         // Previous image
					curFrame,                         // Next image
					cornersA,                     // Previous set of corners (from imgA)
					cornersB,                     // Next set of corners (from imgB)
					features_found,               // Output vector, each is 1 for tracked
					cv::noArray(),                // Output vector, lists errors (optional)
					cv::Size( win_size*2+1, win_size*2+1 ), // Search window size
					5,                            // Maximum pyramid level to construct
					cv::TermCriteria(
					  cv::TermCriteria::MAX_ITER | cv::TermCriteria::EPS,
					  20,                         // Maximum number of iterations
					  0.3                         // Minimum change per iteration
					)
				  );

				  Mat temp=frameCurOrig.clone();
				  Mat binCurFrame(curFrame.rows, curFrame.cols, CV_8UC1, Scalar(0));
				  // Now make some image of what we are looking at:
				  // Note that if you want to track cornersB further, i.e.
				  // pass them as input to the next calcOpticalFlowPyrLK,
				  // you would need to "compress" the vector, i.e., exclude points for which
				  // features_found[i] == false.
				  for( int i = 0; i < (int)cornersA.size(); i++ ) {
					if( !features_found[i] )
					  continue;
					line(
					  //imgResultA,                        // Draw onto this image
					  binCurFrame,
					  cornersA[i],                 // Starting here
					  cornersB[i],                 // Ending here
					  cv::Scalar(255),        // This color
					  2,                           // This many pixels wide
					  CV_AA                  // Draw line in this style
					  );
					line(
					  //prevFrame,                        // Draw onto this image
					  temp,
					  cornersA[i],                 // Starting here
					  cornersB[i],                 // Ending here
					  cv::Scalar(0,255,0),        // This color
					  2,                           // This many pixels wide
					  CV_AA                  // Draw line in this style
					  );
				  }
				  cv::imshow( "LK Optical Flow result A", temp);
					//sobel parameters
					Mat grad;
					Mat grad_x, grad_y;
					Mat abs_grad_x, abs_grad_y;
					int scale = 1;
					int delta = 0;
					int ddepth = CV_16S;
					Mat thresholdImage;

					GaussianBlur(curFrame, curFrame, Size(15,15), 0, 0, BORDER_DEFAULT );
					//sobel 
					Sobel( curFrame, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
					convertScaleAbs( grad_x, abs_grad_x );
					Sobel( curFrame, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
					convertScaleAbs( grad_y, abs_grad_y );
					addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
					//cv::imshow("Sobel Image", grad);
					cv::threshold(grad,thresholdImage,25,255,THRESH_BINARY);
					//cv::imshow("Threshold Image", thresholdImage);
					//morphologyEx(thresholdImage,thresholdImage,MORPH_OPEN,Mat::ones(3,3,CV_8SC1),Point(1,1),2);
					//cv::imshow("Morphed Image", thresholdImage);
					curResultFrame=thresholdImage+binCurFrame;

					  //cv::imshow( "ImageA", curFrame );
					  //cv::imshow( "ImageB", prevFrame );
					  //cv::imshow( "Combined LK Optical Flow + Sobel result A", curResultFrame );
					 // cv::imshow( "LK Optical Flow result B", imgResult);

					  Mat frame1=frameCurOrig.clone();
					  Mat threshCpy=thresholdImage.clone();
					  if(true)
					   {

						searchForVehicle(threshCpy,frame1);
					   }
					  imshow("sobel",frame1);
					  frame1=frameCurOrig.clone();
					  if(true)
					  {
						searchForVehicle(curResultFrame,frame1);
					  }
					  imshow("output lk+sobel",frame1);
		      }
		 
			  
			  prevFrame=curFrame;
			  //waitKey();
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

