/*

ProgrammedCurrentMove.cpp

The following program demonstrates one way of performing the
sequence of events shown below on axis C of an MP4 drive:

1) Initiate a programmed current move
2) Wait for 100 milliseconds
3) Wait for the actual velocity to be below a specific value.

*/

// Comment this out to use EtherCAT
#define USE_CAN

#include <cstdio>
#include <cstdlib>

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

/**************************************************
* Just home the motor and do a bunch of random
* moves.
**************************************************/
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
    WinUdpEcatHardware hw("192.168.0.205");
#else
    LinuxEcatHardware hw("eth0");
#endif

    // Open the network object
#if defined( USE_CAN )
    CanOpen net;
#else
    EtherCAT net;
#endif

    AmpInfo test;
    test.model;

    const Error* err = net.Open(hw);
    showerr(err, "Opening network");

    // Initialize the amplifiers using default settings
    Amp amp[4];

#if defined( USE_CAN )
    // Initializing the first axis of a four axis drive
    printf("Initing axis %d\n", 1);
    err = amp[0].Init(net, canNodeID);
    showerr(err, "Initing axis a");

    // Initializing the second axis of a four axis drive
    printf("Initing axis %d\n", 2);
    err = amp[1].Init(net, canNodeID + 1);
    showerr(err, "Initing axis b");

    // Initializing the third axis of a four axis drive
    printf("Initing axis %d\n", 3);
    err = amp[2].Init(net, canNodeID + 2);
    showerr(err, "Initing axis c");

    // Initializing the fourth axis of a four axis drive
    printf("Initing axis %d\n", 4);
    err = amp[3].Init(net, canNodeID + 3);
    showerr(err, "Initing axis d");
#else
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
#endif

    short desiredStateProgCurrentMode = 1;
    err = amp[2].sdo.Dnld16(0x2300, 0, desiredStateProgCurrentMode);
    showerr(err, "setting desired state to programmed current mode");

    short currentLoopProgrammedValue = 50; // 0.5 A 
    err = amp[2].sdo.Dnld16(0x2340, 0, currentLoopProgrammedValue);
    showerr(err, "setting current loop programmed value");

    short currentLoopSlopeValue = 0;
    err = amp[2].sdo.Dnld32(0x2113, 0, currentLoopSlopeValue);
    showerr(err, "setting the current loop slope value");

    short currentLoopOffset;
    err = amp[2].sdo.Upld16(0x60F6, 3, currentLoopOffset);
    showerr(err, "getting the current loop offset");

    short commandedCurrent;
    err = amp[2].sdo.Upld16(0x221D, 0, commandedCurrent);
    showerr(err, "getting the commanded current");

    // wait for at commanded current
    while (commandedCurrent < currentLoopOffset + currentLoopProgrammedValue) 
    {
        err = amp[2].sdo.Upld16(0x221D, 0, commandedCurrent);
        showerr(err, "getting the commanded current");
    }

    // wait for a delay time 100 milliseconds
    CML::Thread::sleep(100);

    int actualVelocity;
    err = amp[2].sdo.Upld32(0x6069, 0, actualVelocity);
    showerr(err, "getting the actual velocity");

    // wait for actual velocity to be less than or equal to 5
    while (actualVelocity > 5) 
    {
        err = amp[2].sdo.Upld32(0x6069, 0, actualVelocity);
        showerr(err, "getting the actual velocity");
    }

    printf("Program finished. Hit enter to quit\n");
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
