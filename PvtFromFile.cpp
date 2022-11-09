/** \file

The following is an example of how to load PVT points from a CSV file. 
The points are for a three axis network connected in a linkage. 

The position data for each axis is stored in an CSV file. 
The first row of the CSV file contains the names of each axis.

Example of CSV contents:

Axis A Positions, Axis B Positions, Axis C Positions
      100       ,      150        ,       100
      200       ,      250        ,       100
      230       ,      250        ,       200
      250       ,      250        ,       250
      250       ,      250        ,       250
      300       ,      256        ,       289
      300       ,      300        ,       300

The PVT linkage will attempt to achieve these commanded positions
using a PVT algorithm in the PvtConstAccelTrj class.

The algorithm calculates velocities which will produce constant 
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
int axisNum{ 3 };

// Time between PVT Points
uint8 timeBetweenPoints = 10; // 10ms between points

using namespace std;

// load the PVT data from the passed CSV file into the PvtConstAccelTrj class.
void loadPvtPointsFromFile(PvtConstAccelTrj& pvtConstTrjObj, const char* inputExcelFile) {
	try {
		ifstream ifstrObj;
		string xPos{ NULL };
		string yPos{ NULL };
		string zPos{ NULL };

		const Error* err{ 0 };
		vector<double> tempVec = { 0.0, 0.0, 0.0 };
		vector<double>* tempVecPntr = &tempVec;

		vector<double> xPosVec;
		vector<double> yPosVec;
		vector<double> zPosVec;

		vector<uint8> constantTimeVec;
		constantTimeVec.push_back(timeBetweenPoints);

		ifstrObj.open(inputExcelFile);

		if (ifstrObj.is_open()) {
			int count{ 0 };
			string lineOfData{ NULL };
			while (!ifstrObj.eof()) {

				// get a line of data from the excel file.
				getline(ifstrObj, lineOfData);

				// ignore first row, which is just the title row. (X Coordinate, Y Coordinate, Z Coordinate, Time).
				if (count != 0) {
					
					vector<string> allNumsVec;
					string tempStr;
					stringstream strStream(lineOfData);

					if (lineOfData != "") {
						// seperate the line using the commas. The string looks like "1034,120345,123"
						while (getline(strStream, tempStr, ',')) {
							allNumsVec.push_back(tempStr);
						}

						// use the stod method to convert string to double type.
						tempVec[0] = std::stod(allNumsVec[0]);
						tempVec[1] = std::stod(allNumsVec[1]);
						tempVec[2] = std::stod(allNumsVec[2]);

						// add the PVT point to the PvtConstAccelTrj object.
						err = pvtConstTrjObj.addPvtPoint(tempVecPntr, &constantTimeVec[0]);
						showerr(err, "adding points to the PVT object");
					}
				}

				// used to ignore first row. (Title header)
				else {
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
	Amp amp[3];
	AmpSettings amp_settings;

	// How to set the guard time. Guard time is 200ms by default.
	amp_settings.guardTime = 0; // set guardtime to 0 to disable Node Guarding.

	// ME3 synchPeriod is 2000 ms (2 seconds)
	//amp_settings.synchPeriod = 2000;

	printf("Initing Node %d\n", 1);
	err = amp[0].Init(net, 1, amp_settings);
	showerr(err, "Initing Node 1");

	printf("Initing Node %d\n", 2);
	err = amp[1].Init(net, 2, amp_settings);
	showerr(err, "Initing Node 2");

	printf("Initing Node %d\n", 3);
	err = amp[2].Init(net, 3, amp_settings);
	showerr(err, "Initing Node 3");

	// Create a linkage object holding these amps
	Linkage link;
	err = link.Init(3, amp);
	showerr(err, "Linkage init");

	double pathMaxVel{ 160000 };
	double pathMaxAccel{ 960000 };
	double pathMaxDecel{ 960000 };
	double pathMaxJerk{ 200000 };

	// set the limits for the linkage object
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

			Point<3> startingPoint;
			vector<list<double>>* posPntr = pvtConstTrjObj.getPositionsPntr();
			startingPoint[0] = (*posPntr)[0].front();
			startingPoint[1] = (*posPntr)[1].front();
			startingPoint[2] = (*posPntr)[2].front();
			link.MoveTo(startingPoint);
			link.WaitMoveDone(-1);

			printf("Sending trajectory to drives\n");

			// send the trajectory to the linkage.
			err = link.SendTrajectory(pvtConstTrjObj);
			showerr(err, "sending trajectory");

			// Set to -1 to wait indefinitely.
			err = link.WaitMoveDone(-1);
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