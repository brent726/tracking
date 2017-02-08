#pragma once
#include <opencv\cv.h>
#include <opencv\highgui.h>
class Vehicle
{
public:
	Vehicle(void);
	~Vehicle(void);

	int getXPos();
	void setXPos(int x);

	int getYPos();
	void setYPos(int y);

	int getID();
	void setID(int IDnum);

	cv::Point getTl();
	void setTl(cv::Point tlpoint);

	cv::Point getBr();
	void setBr(cv::Point brpoint);

private:

	int xPos, yPos, ID;
	cv::Point tl;
	cv::Point br;
};

