
#include "stdafx.h"
#include "Roboto.h"


bool Control::SuperControl(char data[])
{
	int enable = 1;
	int i = 0;
	
	ResNum = 0;
	CapNum = 0;
	LEDnum = 0;

	if (connectPort()) {

	initialMove();

	while (enable) {
		printf("\nRes : %i	Cap : %i	Led : %i\n", ResNum, CapNum, LEDnum);
		switch (data[i])
		{
		case 'R':
			printf("\nGETTING RESISTOR\n");
			PincerControl(1);				Sleep(1000);

			switch (ResNum)
			{
			case 0:
				moveToComponents(1);		Sleep(1000);
				break;
			case 1:
				moveToComponents(2);		Sleep(1000);
				break;
			case 2:
				moveToComponents(3);		Sleep(1200);
				break;
			default:
				cout << "\nOut of resistors!" << endl;
				break;

			}ResNum++;

			PincerControl(2);				Sleep(1000);
			initialMove();					Sleep(1000);
			moveToBoard(i + 1);				Sleep(1200);
			PincerControl(1);				Sleep(1000);
			initialMove();					Sleep(1000);

			break;

		case 'C':

			printf("\nGETTING CAPACITOR\n");
			PincerControl(1);				Sleep(1000);

			switch (CapNum)
			{
			case 0:
				moveToComponents(4);		Sleep(1000);
				break;
			case 1:
				moveToComponents(5);		Sleep(1000);
				break;
			case 2:
				moveToComponents(6);		Sleep(1200);
				break;
			default:
				cout << "\nOut of capacitors!" << endl;
				break;

			}CapNum++;

			PincerControl(2);				Sleep(1000);
			initialMove();					Sleep(1000);
			moveToBoard(i + 1);				Sleep(1200);
			PincerControl(1);				Sleep(1000);
			initialMove();					Sleep(1000);

			break;

		case 'L':

			printf("\nGETTING LED\n");
			PincerControl(1);				Sleep(1000);

			switch (LEDnum)
			{
			case 0:
				moveToComponents(7);		Sleep(1200);
				break;
			case 1:
				moveToComponents(8);		Sleep(1200);
				break;
			case 2:
				moveToComponents(9);		Sleep(1500);
				break;
			default:
				cout << "\nOut of LEDs!" << endl;
				break;

			}LEDnum++;

			PincerControl(2);				Sleep(1000);
			initialMove();					Sleep(1000);
			moveToBoard(i + 1);				Sleep(1300);
			PincerControl(1);				Sleep(1000);
			initialMove();					Sleep(1000);

			break;


		}
		i++;
		if (i > 9) enable = 0;
	}
	//disconnectPort();
	ResNum = 0;
	CapNum = 0;
	LEDnum = 0;
	printf("\n%i  %i  %i", ResNum, CapNum, LEDnum);
	}
	else
	{
		cout << "NOT CONNECTED :(((" << endl;
	}
	return false;
}