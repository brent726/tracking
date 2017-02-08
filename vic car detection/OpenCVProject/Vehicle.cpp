#include "StdAfx.h"
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