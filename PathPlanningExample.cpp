/*

PathPlanningExample.cpp

The following example illustrates how to plan a two dimensional 
path and send it to a dual axis drive using CML's path planning
class.

The example will create a path, then extract and display all of
the commanded positions and velocities for both axes. The 
position data will be saved to a CSV file. The path will then be 
sent to the drive for execution.

*/

// Comment this out to use EtherCAT
#define USE_CAN

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include "CML.h"
#include <math.h>

using std::ofstream;
using std::to_string;

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

int main(void)
{
	const Error* err = NULL;

	// Time between PVT Points
	uint8 Time_between_points = 250; // 250ms between points

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
    err = net.Open(hw);
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

	double maxVel{ 340777 };
	double maxAcc{ 340777 };
	double maxDec = maxAcc;
	double maxJrk = maxAcc * 10;

	// set the move limits for the linkage object.
	link.SetMoveLimits(maxVel, maxAcc, maxDec, maxJrk);
	
	// Create a new path object
	Path path(2);

	err = path.SetVel(maxVel);  showerr(err, "Setting velocity");
	err = path.SetAcc(maxAcc);  showerr(err, "Setting acceleration");
	err = path.SetDec(maxDec);  showerr(err, "Setting acceleration");
	err = path.SetJrk(maxJrk);  showerr(err, "Setting jerk");

	Point<2> p;
	p[0] = 50000;
	p[1] = 50000;
	err = path.SetStartPos(p); showerr(err, "Setting start position");

	Point<2> startingPos;
	startingPos[0] = p[0];
	startingPos[1] = p[1];

	p[0] = 70000;
	p[1] = 70000;
	err = path.AddArc(p, 3.1419359); showerr(err, "Adding arc");

	err = path.AddLine(p);  showerr(err, "Adding line");
	err = path.AddLine(p);  showerr(err, "Adding line");
	err = path.AddLine(p);  showerr(err, "Adding line");

	p[0] = 50000;
	p[1] = 50000;
	err = path.AddLine(p);  showerr(err, "Adding line");

	// Now, print out the positions and velocities along the path.
	// We need to reset the path before we starting playing it back.
	path.Reset();

	err = path.StartNew(); showerr(err, "Start new");

	path.Reset();

	ofstream excelFile;
	excelFile.open("PositionData.csv");
	excelFile << "X Coordinate,Y Coordinate\n";

	bool done = false;
	do
	{

		double pos[2], vel[2];
		done = path.PlayPath(0.01, pos, vel);

		printf("%9.5lf  %9.5lf  %9.5lf  %9.5lf\n", pos[0], vel[0], pos[1], vel[1]);
		excelFile << to_string(pos[0]) << ',' << to_string(pos[1]) << '\n';


	} while (!done);

	excelFile.close();

	// first, move to the starting position of the path.
	err = link.MoveTo(startingPos);
	showerr(err, "moving to starting position");

	err = link.WaitMoveDone(-1);
	showerr(err, "waiting for move to starting position to finish");

	err = link.SendTrajectory(path, true);
	showerr(err, "beginning linkage move");

	err = link.WaitMoveDone(-1);
	showerr(err, "waiting for the linkage move to finish");

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
