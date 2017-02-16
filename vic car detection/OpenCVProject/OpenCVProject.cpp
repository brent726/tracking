
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


static void help()
{
 printf("\nDo background segmentation, especially demonstrating the use of cvUpdateBGStatModel().\n"
"Learns the background at the start and then segments.\n"
"Learning is togged by the space key. Will read from file or camera\n"
"Usage: \n"
"			./bgfg_segm [--camera]=<use camera, if this key is present>, [--file_name]=<path to movie file> \n\n");
}

const char* keys =
{
    "{c |camera   |true    | use camera or not}"
    "{fn|file_name|tree.avi | movie file             }"
};


int main(){

	//some boolean variables for added functionality
	bool pause=false;
	//these two can be toggled by pressing 'd' or 't'
	
	//road size
	Rect ROAD(1,219,1279,327);
	//video capture object
	VideoCapture capture;
	
	capture.open("C:\\Users\\PCBLAB_01\\Desktop\\60m.mp4");
	double fps = capture.get(CV_CAP_PROP_FPS);
	if(!capture.isOpened())
	{
		cout<<"ERROR ACQUIRING VIDEO FEED\n";
		getch();
		waitKey();
		return -1;
	}
	cout<<"Video FPS: "<<fps<<endl;
	//video frame details
	double dWidth = capture.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
	double dHeight = capture.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video
	cout << "Frame Size = " << dWidth << "x" << dHeight << endl;
	

	help();

//    bool useCamera = parser.get<bool>("camera");
    //string file = parser.get<string>("file_name");
  
    bool update_bg_model = true;
/*
    if( useCamera )
        cap.open(0);
    else
        cap.open(file.c_str());
    parser.printParams();

    if( !cap.isOpened() )
    {
        printf("can not open camera or video file\n");
        return -1;
    }
*/
    namedWindow("image", WINDOW_NORMAL);
    namedWindow("foreground mask", WINDOW_NORMAL);
    namedWindow("foreground image", WINDOW_NORMAL);
    namedWindow("mean background image", WINDOW_NORMAL);

    BackgroundSubtractorMOG2 bg_model;//(100, 3, 0.3, 5);

    Mat img, fgmask, fgimg;

    for(;;)
    {
        capture >> img;

        if( img.empty() )
            break;

        //cvtColor(_img, img, COLOR_BGR2GRAY);

        if( fgimg.empty() )
          fgimg.create(img.size(), img.type());

        //update the model
        bg_model(img, fgmask, update_bg_model ? -1 : 0);

        fgimg = Scalar::all(0);
        img.copyTo(fgimg, fgmask);

        Mat bgimg;
        bg_model.getBackgroundImage(bgimg);

        imshow("image", img);
        imshow("foreground mask", fgmask);
        imshow("foreground image", fgimg);
        if(!bgimg.empty())
          imshow("mean background image", bgimg );

        char k = (char)waitKey(30);
        if( k == 27 ) break;
        if( k == ' ' )
        {
            update_bg_model = !update_bg_model;
            if(update_bg_model)
                printf("Background update is on\n");
            else
                printf("Background update is off\n");
        }
    }
	waitKey();
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
						//imwrite( "E:/capturedimage1.jpg", frame1 );
						cout<<"Capture Image"<<endl;
						break;
					}
				}
				}
		}
	//}//video end
	return 0;

}