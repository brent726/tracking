



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

	 Mat fx2 = fx.mul(fx);
	 imshow("fx2",fx2);
	 int dilate_size = 2;  
     Mat dilateElement = getStructuringElement(cv::MORPH_RECT,Size(2 * dilate_size + 1, 2* dilate_size + 1),Point(dilate_size, dilate_size) );
	 dilate(fx,fx,dilateElement); 
	 imshow("dilate lines fx",fx);
     Mat fy2 = fy.mul(fy);
	 imshow("fy2",fy2);
     Mat fxfy = fx.mul(fy);
	 imshow("fxfy",fxfy);
     Mat fxft = fx.mul(ft);
	 imshow("fxft",fxft);

     dilateElement = getStructuringElement(cv::MORPH_RECT,Size(2 * dilate_size + 1, 2* dilate_size + 1),Point(dilate_size, dilate_size) );
	 dilate(fxft,fxft,dilateElement); 
	 imshow("dilate lines fxft",fxft);
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

	
//    saveMat(u, "U"); 
//    saveMat(v, "V");   
	 return fxft;
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
	//VideoCapture cap("\\\\Mac\\Home\\Desktop\\DroneVideos\\60m.mp4");
    //vector<vector<Point> > contours;
    //vector<Rect> cars;
    //namedWindow("Frame");
    
	//double fps = cap.get(CV_CAP_PROP_FPS);

    //while(true)
    //{

		Mat img1 = imread("stock1.png");
		Mat img2 = imread("stock2.png");	
		bool pause= false;

		imshow("img1",img1);
		imshow("img2",img2);

		//convert to 1 channel 64 bit
		img1.convertTo(img1, CV_64FC1, 1.0/255, 0);
		img2.convertTo(img2, CV_64FC1, 1.0/255, 0);

		imshow("CV_64FC1 img1",img1);
		imshow("CV_64FC1 img2",img2);
		
		//make empty images
		Mat u = Mat::zeros(img1.rows, img1.cols, CV_64FC1);
		Mat v = Mat::zeros(img1.rows, img1.cols, CV_64FC1);

		 getLucasKanadeOpticalFlow(img1, img2, u, v);

		waitKey();
       /* switch(waitKey(100/fps)){
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
		}*/
	//}
       
       
       
        
      
    
}
