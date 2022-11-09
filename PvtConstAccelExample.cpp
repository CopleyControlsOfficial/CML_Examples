/** \file

The following is an example of how to load PVT points to a dual axis drive
to start a linkage move. The points are generated using the PvtConstAccelTrj
class. 

The PvtConstAccelTrj class contains an algorithm that calculates velocities 
which will produce constant accel/decel values. The velocities are calculated 
using the positions in the lists and the times between each PVT point.

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

// global variables used to load PVT points
int axisNum = 2;
int numberOfPvtPoints = 500;

vector<vector<double>> multiDimensionalPositionData;
vector<uint8> timeVector;

using namespace std;

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
	const Error* err = net.Open(hw);
	showerr(err, "Opening network");

	// Initialize the amplifiers using default settings
	Amp amp[2];
	AmpSettings amp_settings;

	// How to set the guard time. Guard time is 200ms by default.
	//amp_settings.guardTime = 0; // set guardtime to 0 to disable Node Guarding.

	// ME3 synchPeriod is 2000 ms (2 seconds)
	//amp_settings.synchPeriod = 2000;

	// Initializing the first axis of a two axis drive
	printf("Initing Axis %d\n", 1);
	err = amp[0].Init(net, 1, amp_settings); // address is 1 for first drive on CANopen Network

	showerr(err, "Initing Axis A");

	// Initializing the second axis of a two axis drive
	printf("Initing Axis %d\n", 2);
	//err = amp[1].InitSubAxis(amp[0], 2); // uncomment to initialize a sub-axis of an EtherCAT multi-axis 
	err = amp[1].Init(net, 2, amp_settings); // init axis 2
	showerr(err, "Initing Axis B");

	// Create a linkage object holding these amps
	Linkage link;
	err = link.Init(2, amp);
	showerr(err, "Linkage init");

	// Time between PVT Points
	uint8 timeBetweenPoints = 10; // 10ms between points

	// initialize the time vector with time value. Slow for first half.
	for (int i = 0; i < numberOfPvtPoints / 2; i++) {
		timeVector.push_back(timeBetweenPoints * 2);
	}

	// initialize second half of time values. Faster move for second half.
	for (int i = numberOfPvtPoints / 2; i < numberOfPvtPoints; i++) {
		timeVector.push_back(timeBetweenPoints);
	}

	// initialize the position data
	for (int i = 0; i < axisNum; i++) {

		double commandedPosition{ 0.0 };
		amp[i].GetPositionActual(commandedPosition);

		// temporary vector for loading points
		vector<double> tempVec;
		for (int j = 0; j < numberOfPvtPoints; j++) {
			double positionValue = commandedPosition;
			tempVec.push_back(positionValue);
			commandedPosition = positionValue + 100;
		}

		// load the temporary vector's position data into the multidimensional position vector
		multiDimensionalPositionData.push_back(tempVec);
	}

	// pointer to position data
	vector<vector<double>>* pointsPntr = &multiDimensionalPositionData;

	// pointer to time data
	vector<uint8>* pntrTimeData = &timeVector;

	// create an instance of the PvtConstAccelTrj class.
	PvtConstAccelTrj pvtConstTrjObj;

	// initialize the object with the number of dimensions in the trajectory.
	pvtConstTrjObj.Init(2);
	showerr(err, "initializing the PvtConstAccelTrj object");

	vector<double> tempVec = { 0.0, 0.0 };
	vector<double>* tempVecPntr = &tempVec;

	for (int i = 0; i < timeVector.size(); i++) {
		tempVec[0] = multiDimensionalPositionData[0][i];
		tempVec[1] = multiDimensionalPositionData[1][i];
		err = pvtConstTrjObj.addPvtPoint(tempVecPntr, &timeVector[i]);
		showerr(err, "adding points to the PVT object");
	}

	// Create my trajectory and send it to the linkage object
	printf("Sending 1st trajectory to drives\n");

	// send the trajectory to the linkage.
	err = link.SendTrajectory(pvtConstTrjObj);
	showerr(err, "sending trajectory");

	// Wait 20 seconds for the move to finish. Set to -1 to wait indefinitely.
	err = link.WaitMoveDone(-1);
	
	////////////////////////
	//   2nd Move
	////////////////////////

	multiDimensionalPositionData.clear();
	timeVector.clear();

	// initialize the time vector with time value. Slow for first half.
	for (int i = 0; i < numberOfPvtPoints / 2; i++) {
		timeVector.push_back(timeBetweenPoints * 2);
	}

	// initialize second half of time values. Faster move for second half.
	for (int i = numberOfPvtPoints / 2; i < numberOfPvtPoints; i++) {
		timeVector.push_back(timeBetweenPoints);
	}

	// initialize the position data
	for (int i = 0; i < axisNum; i++) {

		double commandedPosition{ 0.0 };
		amp[i].GetPositionActual(commandedPosition);

		// temporary vector for loading points
		vector<double> tempVec;
		for (int j = 0; j < numberOfPvtPoints; j++) {
			double positionValue = commandedPosition;
			tempVec.push_back(positionValue);
			commandedPosition = positionValue + 100;
		}

		// load the temporary vector's position data into the multidimensional position vector
		multiDimensionalPositionData.push_back(tempVec);
	}

	// add the new PVT points to the pvtConstTrj object.
	for (int i = 0; i < timeVector.size(); i++) {
		tempVec[0] = multiDimensionalPositionData[0][i];
		tempVec[1] = multiDimensionalPositionData[1][i];
		err = pvtConstTrjObj.addPvtPoint(tempVecPntr, &timeVector[i]);
		showerr(err, "adding points to the PVT object");
	}

	// Create my trajectory and send it to the linkage object
	printf("Sending 2nd trajectory to drives\n");

	// send the trajectory to the linkage.
	err = link.SendTrajectory(pvtConstTrjObj);
	showerr(err, "sending trajectory");
	
	// Wait 20 seconds for the move to finish. Set to -1 to wait indefinitely.
    err = link.WaitMoveDone(-1);
	showerr(err, "waiting on move");
	//}

	printf("Move finished, hit enter to quit\n");
	getchar();

	return 0;
	
}
/**************************************************/


static void showerr(const Error* err, const char* str)
{
	if (err)
	{
		printf("Error %s: %s\n", str, err->toString());
		exit(1);
	}
}