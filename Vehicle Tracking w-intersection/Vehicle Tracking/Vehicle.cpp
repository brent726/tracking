#include "Vehicle.h"
#include <opencv\cv.h>
#include <opencv\highgui.h>

Vehicle::Vehicle(void)
{
}


Vehicle::~Vehicle(void)
{
}

int Vehicle::getXPos(){

	return Vehicle::xPos;

}

void Vehicle::setXPos(int x){

	Vehicle::xPos = x;

}

int Vehicle::getYPos(){

	return Vehicle::yPos;

}

void Vehicle::setYPos(int y){

	Vehicle::yPos = y;

}

int Vehicle::getID(){

	return Vehicle::ID;

}

void Vehicle::setID(int IDnum){

	Vehicle::ID = IDnum;
}

cv::Point Vehicle::getTl(){

	return Vehicle::tl;
}

void Vehicle::setTl(cv::Point tlpoint)
{
	Vehicle::tl = tlpoint;
}

cv::Point Vehicle::getBr()
{
	return Vehicle::br;
}

void Vehicle::setBr(cv::Point brpoint)
{
	Vehicle::br = brpoint;
}

float Vehicle::getC2BX()
{
	return Vehicle::center2BorderX;
}

void Vehicle::setC2BX(float x)
{
	Vehicle::center2BorderX = x;
}

float Vehicle::getC2BY()
{
	return Vehicle::center2BorderY;
}

void Vehicle::setC2BY(float y)
{
	Vehicle::center2BorderY = y;
}

std::vector<cv::Point2f> const & Vehicle::getPoints(int index) const
{
	return Vehicle::points[index];
}

void Vehicle::appendPoints(cv::Point2f point, int index)
{
	Vehicle::points[index].push_back(point);
}

double Vehicle::getSpeed()
{
	return Vehicle::speed;
}

void Vehicle::setSpeed(double spd)
{
	Vehicle::speed = spd;
}
