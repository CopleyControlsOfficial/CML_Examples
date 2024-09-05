/*

CcdFileExample.cpp

Example of how to upload & download a CCD file using CML.

CCD files contain all of the contents of the drive's flash
memory. The flash memory contains all flash parameters and
the contents of the CVM filing system (CVM programs, CAM 
tables, gain scheduling tables, encoder correction tables,
etc.). The CCD file is in XML file format and is generated
by the drive's firmware.

*/

// Comment this out to use EtherCAT
//#define USE_CAN

#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <thread>

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
static void showerr(const Error *err, const char *str);

/* local data */
int32 canBPS = 1000000;             // CAN network bit rate
int16 nodeID = -1;                // CANopen node ID

int main(void)
{
	// The libraries define one global object of type
	// CopleyMotionLibraries named cml.
	//
	// This object has a couple handy member functions
	// including this one which enables the generation of
	// a log file for debugging
	cml.SetDebugLevel(LOG_EVERYTHING);

	// Create an object used to access the low level CAN network.
	// This examples assumes that we're using the Copley PCI CAN card.
#if defined( USE_CAN )
	CopleyCAN hw("CAN0");
	hw.SetBaud(canBPS);
#elif defined( WIN32 )
	WinUdpEcatHardware hw("192.168.0.40");
#else
	LinuxEcatHardware hw("eth0");
#endif

#define MAX_LINE_LEN  200

	// Open the network object
#if defined( USE_CAN )
	CanOpen net;
#else
	EtherCAT net;
#endif
	const Error *err = net.Open(hw);
	showerr(err, "Opening network");

	// Initialize the amplifier using default settings
	Amp amp;
	printf("Doing init\n");
	
	err = amp.Init(net, nodeID);
	showerr(err, "Initting amp");

	err = amp.WriteCCDToFile("TestFile.ccd", net);
	showerr(err, "creating the CCD file by reading the flash memory from the drive");

	err = amp.LoadCCDFromFile("TestFile.ccd", net);
	showerr(err, "writing from the newly created CCD file to the flash memory of the drive");

	printf("Program successful\n");
	return 0;
}

static void showerr(const Error *err, const char *str)
{
	if (err)
	{
		printf("Error %s: %s\n", str, err->toString());
		exit(1);
	}
}

