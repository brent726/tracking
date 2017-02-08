
#include "stdafx.h"
#include "Roboto.h"


Roboto::Roboto(void)
{
	hComm = NULL;
	activated = false;
};


Roboto::~Roboto(void)
{

};

bool Roboto::connectPort(void)
{
	activated = comm.SetPort(&hComm, 115200);
	if (activated) cout << "Roboto Successfully Connected!" << endl;
	return activated;
}

bool Roboto::disconnectPort(void)
{
	activated = comm.ClosePort(&hComm);
	if (!activated) cout << "Roboto Successfully Disconnected!" << endl;
	return activated;
}

bool Roboto::initialMove()
{
	DWORD sizeWritten;
	char DefaultPosition[200] = "#0 P1500 #1 P2100 #2 P2400 #3 P700 T1000 <cr>\r\n";
	Sleep(1000);

	printf("Moving to initial position...\n");
	cout << DefaultPosition << endl;
	return comm.SendData(&hComm, &sizeWritten, DefaultPosition);
}


bool Roboto::PincerControl(int choice)
{
	DWORD sizeWritten;
	char closePincer[200] = "#4 P1900 T500 <cr>\r\n";
	char openPincer[200] = "#4 P1200 T500 <cr>\r\n";

	switch (choice)
	{
	case 1:
		printf("Opening end effector...\n");
		cout << closePincer << endl;
		return comm.SendData(&hComm, &sizeWritten, closePincer);
		break;

	case 2:
		printf("Closing end effector...\n");
		cout << openPincer << endl;
		return comm.SendData(&hComm, &sizeWritten, openPincer);
		break;
	}
}




bool Roboto::moveToBoard(int slot)
{
	DWORD sizeWritten;
	char instruction[200] = "";
	Roboto robot;

	switch(slot)
	{
	case 1:		strcat_s(instruction, " #0 P1900 #1 P1400 #2 P2400 #3 P750 T1000 <cr>\r\n");
		break;
	case 2:		strcat_s(instruction, " #0 P1500 #1 P1400 #2 P2400 #3 P650 T1000 <cr>\r\n");
		break;
	case 3:		strcat_s(instruction, " #0 P1125 #1 P1400 #2 P2400 #3 P750 T1000 <cr>\r\n");
		break;
	case 4:		strcat_s(instruction, " #0 P1750 #1 P1100 #2 P2000 #3 P750 T1000 <cr>\r\n");
		break;
	case 5:		strcat_s(instruction, " #0 P1500 #1 P1100 #2 P2000 #3 P650 T1000 <cr>\r\n");
		break;
	case 6:		strcat_s(instruction, " #0 P1300 #1 P1100 #2 P2000 #3 P750 T1000 <cr>\r\n");
		break;
	case 7:		strcat_s(instruction, " #0 P1675 #1 P950  #2 P1700 #3 P800 T1200 <cr>\r\n");
		break;
	case 8:		strcat_s(instruction, " #0 P1500 #1 P920  #2 P1700 #3 P850 T1200 <cr>\r\n");
		break;
	case 9:		strcat_s(instruction, " #0 P1350 #1 P1000  #2 P1850 #3 P950 T1200 <cr>\r\n");
		break;


	default:	printf("\nInvalid Slot Number...\n");
		break;
	}

	//MOVING THE ARM TO BOARD COORDINATES, RELEASING THE END EFFECTOR, AND RETURNING TO BASE POSITION

	if (activated)
	{
		Sleep(1000);
		printf("Placing component...\n");
		cout << instruction << endl;
		return comm.SendData(&hComm, &sizeWritten, instruction);
	}
	else
	{
		return activated;
	}

}



//MOVING THE ARM TO COMPONENT COORDINATES


bool Roboto::moveToComponents(int caps)
{
	DWORD sizeWritten;
	char instruction[200] = "";

	switch (caps) {

	case 1:		strcat_s(instruction, " #0 P2450 #1 P1225 #2 P2250 #3 P850 T1000 <cr>\r\n");
		break;
	case 2:		strcat_s(instruction, " #0 P2450 #1 P1050 #2 P2000 #3 P850 T1000 <cr>\r\n");
		break;
	case 3:		strcat_s(instruction, " #0 P2450 #1 P850  #2 P1700 #3 P875 T1200 <cr>\r\n");
		break;
	case 4:		strcat_s(instruction, " #0 P2200 #1 P1175 #2 P2200 #3 P850 T1000 <cr>\r\n");
		break;
	case 5:		strcat_s(instruction, " #0 P2250 #1 P1050 #2 P2000 #3 P900 T1000 <cr>\r\n");
		break;
	case 6:		strcat_s(instruction, " #0 P2300 #1 P800  #2 P1600 #3 P875 T1200 <cr>\r\n");
		break;
	case 7:		strcat_s(instruction, " #0 P2025 #1 P950  #2 P1800 #3 P675 T1200 <cr>\r\n");
		break;
	case 8:		strcat_s(instruction, " #0 P2125 #1 P850  #2 P1650 #3 P775 T1200 <cr>\r\n");
		break;
	case 9:		strcat_s(instruction, " #0 P2175 #1 P800  #2 P1650 #3 P1000 T1400 <cr>\r\n");
		break;




	default:	printf("\nInvalid Cap Number...\n");
		break;
	}

	//MOVING THE ARM TO CAP COORDINATES

	if (activated)
	{
		Sleep(1000);

		printf("Getting component...\n");
		cout << instruction << endl;
		return comm.SendData(&hComm, &sizeWritten, instruction);

	}
	else
	{
		return activated;
	}
}
