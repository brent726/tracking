#pragma once

#include <iostream>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "SerialCom.h"

using namespace std;

class Roboto
{
private:

	SerialCom comm;
	HANDLE hComm;
	bool activated;

public:
	Roboto(void);
	~Roboto(void);
	bool connectPort(void);
	bool disconnectPort(void);
	bool initialMove();
	bool moveToBoard(int slot);
	bool moveToComponents(int caps);
	bool PincerControl(int choice);
};


class Control : public Roboto
{

	int ResNum;
	int CapNum;
	int LEDnum;

public:

	bool SuperControl(char data[]);

};

