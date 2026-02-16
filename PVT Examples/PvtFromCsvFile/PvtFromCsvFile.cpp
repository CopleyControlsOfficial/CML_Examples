/*

PvtFromCsvFile.cpp

The following is an example of how to load PVT points from a CSV file.
The points are for a three axis network connected in a linkage.

The commanded position data for each axis is stored in an CSV file
named "XyzPoints.csv."

The first row of the CSV file contains the names of each axis.
Example of CSV contents:

Time, Axis A Positions, Axis B Positions, Axis C Positions
250 ,	  100         ,      150        ,       100
200 ,	  200         ,      250        ,       100
200 ,	  230         ,      250        ,       200
250 ,	  250         ,      250        ,       250
250 ,	  250         ,      250        ,       250
200 ,	  300         ,      256        ,       289
200 ,	  300         ,      300        ,       300

The PVT linkage will attempt to achieve these commanded positions
using a PVT algorithm in the PvtConstAccelTrj class.

The algorithm calculates velocities which will produce continuous
accel/decel values. The velocities are calculated using the
positions in the CSV file and the time between each PVT point.

*/

// Comment this out to use EtherCAT
#define USE_CAN

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string> 
#include<vector> // for vectors 
#include <list>
#include <sstream>
#include "CML.h"

using std::list;

#if defined( USE_CAN )
#include "can/can_copley.h"
#elif defined( WIN32 )
#include "ecat/ecat_winudp.h"
#else
#include "ecat/ecat_linux.h"
#endif

// If a namespace has been defined in CML_Settings.h, this
// macros starts using it. 
CML_NAMESPACE_USE();

/* local functions */
static void showerr(const Error* err, const char* str);

/* local data */
int32 canBPS = 1000000;             // CAN network bit rate
int16 canNodeID = 1;                // CANopen node ID

// the number of axes in the move.
const int axisNum = 3;

using namespace std;

// load the PVT data from the passed CSV file into the PvtConstAccelTrj class.
void loadPvtPointsFromFile(PvtConstAccelTrj& pvtConstTrjObj, const char* inputExcelFile) {
	try 
	{
		ifstream ifstrObj;

		const Error* err{ 0 };
		uint8 timeValue = 0;
		vector<double> tempVec;
		
		// initialize the temporary position vector with 0.0 for each axis
		for (int i = 0; i < axisNum; i++) 
		{
			tempVec.push_back(0.0);
		}

		vector<double>* tempVecPntr = &tempVec;
		uint8 time = 0;

		ifstrObj.open(inputExcelFile);

		if (ifstrObj.is_open()) {
			int count{ 0 };
			string lineOfData{ NULL };
			while (!ifstrObj.eof()) {

				// get a line of data from the excel file.
				getline(ifstrObj, lineOfData);

				// ignore first row, which is just the title row. (Time, X Coordinate, Y Coordinate, Z Coordinate).
				if (count != 0) 
				{
					vector<string> allNumsVec;
					string tempStr;
					stringstream strStream(lineOfData);

					if (lineOfData != "") 
					{
						// seperate the line using the commas. The string looks like "250,1034,120345,123" which is [time, pos1, pos2, pos3]
						while (getline(strStream, tempStr, ',')) 
						{
							allNumsVec.push_back(tempStr);
						}

						// use the stoi method to convert from string to int type
						time = std::stoi(allNumsVec[0]);

						printf("Time: %d ; ", time);
						printf("Position: ");

						// use the stod method to convert string to double type.
						for (int i = 0; i < axisNum; i++) 
						{
							tempVec[i] = std::stod(allNumsVec[i+1]);
							printf("%f, ", tempVec[i]);
						}

						printf("\n");

						// add the PVT point to the PvtConstAccelTrj object.
						err = pvtConstTrjObj.addPvtPoint(tempVecPntr, &time);
						showerr(err, "adding points to the PVT object");
					}
				}

				// used to ignore first row. (Title header)
				else 
				{
					count = 1;
				}
			}
		}

		ifstrObj.close();
	}
	// catch any exception here
	catch (...) {
		cout << "\nException occured in loadPvtPointsFromFile.\n";
		throw;
	}
}

int main(void)
{
	// The libraries define one global object of type
	// CopleyMotionLibraries named cml.
	//
	// This object has a couple handy member functions
	// including this one which enables the generation of
	// a log file for debugging
	cml.SetDebugLevel(LOG_DEBUG);

	// Create an object used to access the low level CAN network.
	// This examples assumes that we're using the Copley PCI CAN card.
#if defined( USE_CAN )
	CopleyCAN hw("CAN0");
	hw.SetBaud(canBPS);
#elif defined( WIN32 )
	WinUdpEcatHardware hw("192.168.0.100");
#else
	LinuxEcatHardware hw("eth0");
#endif

	// Open the network object
#if defined( USE_CAN )
	CanOpen net;
#else
	EtherCAT net;
#endif
	const Error* err{ NULL };
	err = net.Open(hw);
	showerr(err, "Opening network");

	// Initialize the amplifiers using default settings
	Amp ampArray[axisNum];

	for (int i = 0; i < axisNum; i++) 
	{
		printf("Initing Node %d\n", i+1);
		err = ampArray[i].Init(net, i+1);
		showerr(err, "Initializing node");
	}

	// Create a linkage object holding these amps
	Linkage link;
	err = link.Init(axisNum, ampArray);
	showerr(err, "Linkage init");

	double pathMaxVel{ 160000 };
	double pathMaxAccel{ 960000 };
	double pathMaxDecel{ 960000 };
	double pathMaxJerk{ 200000 };

	// Set the limits for the linkage object. 
	// These trajectory limits are used when using the Linkage::MoveTo method.
	// WARNING: These move limits are ignored when using the PvtConstAccelTrj or PvtTrj classes. 
	// This is because the PvtConstAccelTrj and PvtTrj classes generate the trajectory.
	err = link.SetMoveLimits(pathMaxVel, pathMaxAccel, pathMaxDecel, pathMaxJerk); showerr(err, "Setting Linkage Move Limits");

	// create an instance of the PvtConstAccelTrj class.
	PvtConstAccelTrj pvtConstTrjObj;

	// initialize the object with the number of dimensions in the trajectory.
	err = pvtConstTrjObj.Init(axisNum);
	showerr(err, "initializing the PvtConstAccelTrj object");

	int numberOfCycles{ 1 };
	while (numberOfCycles != 0) {

		cout << "\nPlease enter number of cycles. Enter 0 to quit: ";
		cin >> numberOfCycles;

		for (int i = 0; i < numberOfCycles; i++) {

			loadPvtPointsFromFile(pvtConstTrjObj, "XyzPoints.csv");

			Point<axisNum> startingPoint;
			vector<list<double>>* posPntr = pvtConstTrjObj.getPositionsPntr();

			// extract the starting position
			for (int j = 0; j < axisNum; j++) 
			{
				startingPoint[j] = (*posPntr)[j].front();
			}

			// move to the starting position
			link.MoveTo(startingPoint);
			showerr(err, "moving to the starting position (first PVT point)");
			link.WaitMoveDone(-1);
			showerr(err, "waiting for the move to the starting position to finish");

			printf("Sending trajectory to drives\n");

			// send the trajectory to the linkage.
			err = link.SendTrajectory(pvtConstTrjObj);
			showerr(err, "starting PVT move");

			// Set to -1 to wait indefinitely.
			err = link.WaitMoveDone(-1);
			showerr(err, "waiting for the PVT move to finish");
		}
	}

	printf("Program finished. Hit any key to quit\n");
	getchar();

	return 0;

}

static void showerr(const Error* err, const char* str)
{
	if (err)
	{
		printf("Error %s: %s\n", str, err->toString());
		exit(1);
	}
}

