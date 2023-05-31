/*

ME4Init.cpp

The following is a simple example of how to initialize an ME4 drive on an EtherCAT network.
Note: The p-loop and v-loop update rates of the ME3/ME4 drives are 2.5kHz (400 usec). The 
      EtherCAT loop update rate must be an integer multiple of this number. Therefore, a 
      value of 2ms must be configured for these drives using the AmpSettings object. 
*/

// Comment this out to use EtherCAT
//#define USE_CAN

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
int16 canNodeID = -1;                // CANopen node ID

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
    const Error* err = net.Open(hw);
    showerr(err, "Opening network");

    // Initialize the amplifiers using default settings
	Amp amp[4];
	AmpSettings amp_settings;

	// ME3/ME4 synchPeriod is 2000 ms (2 seconds)
	//amp_settings.enableOnInit = false;
	amp_settings.synchPeriod = 2000;

	// Initializing the first axis of a four axis drive
	printf("Initing axis %d\n", 1);
	err = amp[0].Init(net, -1, amp_settings); // address is -1 for first drive on EtherCAT Network
	showerr(err, "Initing axis a");

	// Initializing the second axis of a four axis drive
	printf("Initing axis %d\n", 2);
	err = amp[1].InitSubAxis(amp[0], 2);
	showerr(err, "Initing axis b");

	// Initializing the third axis of a four axis drive
	printf("Initing axis %d\n", 3);
	err = amp[2].InitSubAxis(amp[0], 3);
	showerr(err, "Initing axis c");

	// Initializing the fourth axis of a four axis drive
	printf("Initing axis %d\n", 4);
	err = amp[3].InitSubAxis(amp[0], 4);
	showerr(err, "Initing axis d");

	printf("Hit enter to quit\n");
	getchar();

    return 0;
}

/**********************************************
   Print the error to the console and exit the 
   application.                                
**********************************************/
static void showerr(const Error* err, const char* str)
{
    if (err)
    {
        printf("Error %s: %s\n", str, err->toString());
        exit(1);
    }
}
