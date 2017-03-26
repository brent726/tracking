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

	float getC2BX();
	void setC2BX(float x);

	float getC2BY();
	void setC2BY(float y);

	std::vector<cv::Point2f> const &getPoints(int index) const;

	void appendPoints(cv::Point2f point, int index);

	double getSpeed();
	void setSpeed(double spd);

private:

	int xPos, yPos, ID;
	cv::Point tl;
	cv::Point br;
	float center2BorderX, center2BorderY;
	std::vector<cv::Point2f> points[2];
	double speed;
};

