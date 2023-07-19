/** \file

The following is an example of how start/stop the function generator
using the CML library. CML sends SDO's over the EtherCAT or CANopen 
networks to achieve this task.

*/

// Comment this out to use EtherCAT
#define USE_CAN

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string> 
#include <vector> // for vectors 
#include <list>
#include "CML.h"

using std::string;

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
void StartFunctionGenerator(Amp& amp, string mode);
void StopFunctionGenerator(Amp& amp, string mode);

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
#else
	EtherCAT net;
#endif
	const Error* err{ NULL };
	err = net.Open(hw);
	showerr(err, "Opening network");

	// Initialize the amplifiers using default settings
	Amp amp;
	AmpSettings ampSettingsObj;

	// How to set the guard time. Guard time is 200ms by default.
	ampSettingsObj.guardTime = 0; // set guardtime to 0 to disable Node Guarding.

#ifdef USE_CAN
	err = amp.Init(net, canNodeID, ampSettingsObj); // address is 1 for first drive on CANopen Network
	showerr(err, "initing amp");
#else
	// Initializing the axis
	err = amp.Init(net, -1, ampSettingsObj); // address is -1 for first drive on EtherCAT Network
	showerr(err, "Initing axis A");
#endif

	// Configure a square wave
	FuncGenConfig funcGenConfigObj;
	funcGenConfigObj.amp = 2000;  // amplitude = 2000 counts
	funcGenConfigObj.freq = 4;    // frequency = 2 Hz
	funcGenConfigObj.duty = 1000; // duty cycle = 1000. Units are 0.1% so 1000 = 100%
	funcGenConfigObj.cfg = 8193;  // config = bits 0-2 = 1 = Square wave output. Bit 13 set = Invert every other period.

	err = amp.SetFuncGenConfig(funcGenConfigObj);
	showerr(err, "configuring function generator");

	printf("Starting the function generator. Driving the P-loop.\n");

	// have function generator drive the p-loop in servo mode
	StartFunctionGenerator(amp, "FuncGenDrivesPloop");

	Thread::sleep(5000); // run in this mode for 5 seconds

	printf("Stopping the function generator.\n");

	// stop the function generator and set it back to CANopen servo mode.
	StopFunctionGenerator(amp, "CANopenInterfaceControlsAmp");

	// Configure a sine wave
	funcGenConfigObj.amp = 26667;  // amplitude = 20 rpm. (specific for my motor)
	funcGenConfigObj.freq = 5;     // frequency = 5 Hz
	funcGenConfigObj.duty = 1000;  // duty cycle = 1000 = 100%.
	funcGenConfigObj.cfg = 2;      // config = bits 0-2 = 2 = Sine Wave

	err = amp.SetFuncGenConfig(funcGenConfigObj);
	showerr(err, "configuring function generator");

	printf("Starting the function generator. Driving the V-loop.\n");

	// have function generator drive the velocity loop
	StartFunctionGenerator(amp, "FuncGenDrivesVloop");

	Thread::sleep(5000); // run in this mode for 5 seconds

	printf("Stopping the function generator.\n");

	// stop the function generator and set it back to CANopen servo mode.
	StopFunctionGenerator(amp, "CANopenInterfaceControlsAmp");

	// Configure a square wave
	funcGenConfigObj.amp = 5; // amplitude = 0.05 Amps
	funcGenConfigObj.freq = 200; // frequency = 200 Hz
	funcGenConfigObj.duty = 1000; // duty cycle = 1000 = 100%
	funcGenConfigObj.cfg = 8193;  // config = bits 0-2 = 1 = Square wave output. Bit 13 set = Invert every other period.

	err = amp.SetFuncGenConfig(funcGenConfigObj);
	showerr(err, "configuring function generator");

	printf("Starting the function generator. Driving the I-loop.\n");

	// have function generator drive the current loop
	StartFunctionGenerator(amp, "FuncGenDrivesIloop");

	Thread::sleep(5000); // run in this mode for 5 seconds

	printf("Stopping the function generator.\n");

	// stop the function generator and set it back to CANopen servo mode.
	StopFunctionGenerator(amp, "CANopenInterfaceControlsAmp");

	printf("Example finished, hit enter to quit\n");
	getchar();

	return 0;
	
}
/**************************************************/

// set the ampmode to start the function generator
void StartFunctionGenerator(Amp& amp, string mode) {
	const Error* err = 0;
	if (mode == "FuncGenDrivesPloop") {
		err = amp.SetAmpMode(AMPMODE_FGEN_POS);
		showerr(err, "setting ampmode to function generator drives position loop");
	}
	if (mode == "FuncGenDrivesMicrostepper") {
		err = amp.SetAmpMode(AMPMODE_FGEN_USTEP);
		showerr(err, "setting ampmode to function generator drives microstepper");
	}
	if (mode == "FuncGenDrivesVloop") {
		err = amp.SetAmpMode(AMPMODE_FGEN_VEL);
		showerr(err, "setting ampmode to function generator drives velocity loop");
	}
	if (mode == "FuncGenDrivesIloop") {
		err = amp.SetAmpMode(AMPMODE_FGEN_CRNT);
		showerr(err, "setting ampmode to function generator drives current loop");
	}
}

// method to stop the function generator
void StopFunctionGenerator(Amp& amp, string mode) {
	const Error* err = 0;

	AMP_MODE ampMode;
	err = amp.GetAmpMode(ampMode);
	showerr(err, "reading amp mode");

	// if we are driving the p-loop with the function generator, we will 
	// need to abort the move first
	if ((ampMode == AMPMODE_FGEN_POS) || (ampMode == AMPMODE_FGEN_USTEP)) {
		// Send the trajectory command to abort the move
		short index = 0x2000;
		short subIndex = 0;
		int size = 3;
		byte data[3] = { 0x11, 0x00, 0x00 }; // 0x11 is trajectory op-code. 0x0000 is abort move command.
		err = amp.Download(index, subIndex, size, data);
		showerr(err, "aborting the function generator move");
	}

	// Set the amp mode (desired state) back to the mode passed into the method
	if (mode == "CANopenInterfaceControlsAmp") {
		short desiredStateCanServo = 30;
		err = amp.sdo.Dnld16(0x2300, 0, desiredStateCanServo);
		showerr(err, "setting desired state to CANopen interface controls amplifier");
	}

	if (mode == "CANopenInterfaceControlsMicrostepper") {
		short desiredStateCanUstep = 40;
		err = amp.sdo.Dnld16(0x2300, 0, desiredStateCanUstep);
		showerr(err, "setting ampmode to CANopen interface controls microstepper");
	}

	// set the function generator amplitude to 0
	err = amp.sdo.Dnld32(OBJID_FGEN_AMP, 0, 0);
	showerr(err, "setting amplitude to zero");
}

static void showerr(const Error* err, const char* str)
{
	if (err)
	{
		printf("Error %s: %s\n", str, err->toString());
		exit(1);
	}
}