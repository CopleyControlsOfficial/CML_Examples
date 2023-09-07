/*

The following C++ script is used to transition a single node 
through either the CANopen or EtherCAT state machines. Simply
comment out the "#define USE_CAN" below to use EtherCAT.
The script assumes that a Copley CAN card is in use. 
If using an EtherCAT connection, please change the IP address
in line 60.

*/


// Comment this out to use EtherCAT
//#define USE_CAN

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include "CML.h"

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
	WinUdpEcatHardware hw("192.168.0.98");
#else
	LinuxEcatHardware hw("eth0");
#endif

	// Open the network object
#if defined( USE_CAN )
	CanOpen net;
	printf("CAN network test is running.\n");
#else
	EtherCAT net;
	printf("EtherCAT network test is running.\n");
#endif
	const Error* err = net.Open(hw);
	showerr(err, "Opening network");

	// Initialize the amplifiers using default settings
	Amp amp[1];
	AmpSettings amp_settings;

	// How to set the guard time. Guard time is 200ms by default.
	//amp_settings.guardTime = 0; // set guardtime to 0 to disable Node Guarding.

	// ME3 synchPeriod is 2000 ms (2 seconds)
	amp_settings.enableOnInit = false;

	printf("Initing axis %d\n", canNodeID);

#if defined( USE_CAN )

	// Initializing the drive on the network.
	err = amp[0].Init(net, canNodeID, amp_settings);

	// check for error when initializing drive on the network.
	showerr(err, "Initailizing amp");

	// reset the node to force NMT state machine back to init state.
	err = amp[0].ResetNode();

	// check for error when reseting node.
	showerr(err, "Reseting node");

	// send message to user to hit any key to continue.
	printf("Amp is in pre-op state.\n");
	printf("The green network LED should be blinking 200ms ON and 200ms OFF.\n");
	printf("Press any key to continue to stopped state.\n");

	// user must press any key to continue.
	getchar();

	// change NMT state machine to stopped
	err = amp[0].StopNode();

	// check for error when changing NMT state machine to stopped.
	showerr(err, "Changing NMT state machine to stopped");

	// send message to user to hit any key to continue.
	printf("Amp is in stopped state.\n");
	printf("The green network LED should be blinking 200ms ON and 1 second OFF.\n");
	printf("Press any key to continue to operational state.\n");

	// user must press any key to continue.
	getchar();

	// change NMT state machine to operational
	err = amp[0].StartNode();

	// check for error when changing NMT state machine to stopped.
	showerr(err, "Changing NMT state machine to Operational");

	// send message to user to hit any key to continue.
	printf("Amp is in operational state.\n");
	printf("The green network LED should be solid (not blinking).\n");



#else

	// Initializing the drive on the network.
	err = amp[0].Init(net, -1, amp_settings);
	showerr(err, "Initailizing amp");

	// change node state to INIT in EtherCAT state machine.
	err = net.StopNode(&amp[0]);
	showerr(err, "changing node state to INIT in EtherCAT state machine");
	printf("EtherCAT node is in INIT state. Press any key to continue to PRE-OP state.\n");
	getchar();

	// change node state to PRE-OP in EtherCAT state machine.
	err = net.PreOpNode(&amp[0]);
	showerr(err, "changing node state to PRE-OP in EtherCAT state machine");
	printf("EtherCAT node is in PRE-OP state. Press any key to continue to SAFE-OP state.\n");
	getchar();

	// change node state to SAFE-OP in EtherCAT state machine.
	err = net.SafeOpNode(&amp[0]);
	showerr(err, "changing node state to SAFE-OP in EtherCAT state machine");
	printf("EtherCAT node is in SAFE-OP state. Press any key to continue to OP state.\n");
	getchar();

	// change node state to OP in EtherCAT state machine.
	err = amp[0].StartNode();
	showerr(err, "changing node state to OP in EtherCAT state machine");
	printf("EtherCAT node is in OP state. Press any key to continue to BOOT MODE state.\n");
	getchar();

	// change node state to BOOT MODE in EtherCAT state machine.
	err = net.BootModeNode(&amp[0]);
	showerr(err, "changing node state to BOOT MODE in EtherCAT state machine");
	printf("EtherCAT node is in BOOT MODE state.\n");

#endif

	// prompt the user to press any key to end the program.
	printf("Hit enter to quit\n");

	// user must press any key to continue.
	getchar();

	// return 0 (success)
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