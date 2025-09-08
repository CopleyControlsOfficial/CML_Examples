/*

ProgrammedCurrentMove.cpp

The following program demonstrates one way of performing the
sequence of events shown below on a Copley Controls drive:

1) Initiate a programmed current move
2) Wait for "at commanded current"
3) Wait for 3 seconds
4) End the programmed current move

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
int16 canNodeID = -1;                // CANopen node ID

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
    canNodeID = 1;
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

    const Error* err = net.Open(hw);
    showerr(err, "Opening network");

    // Initialize the amplifiers using default settings
    Amp amp;

    // Initializing the first axis of a four axis drive
    printf("Initing axis %d\n", 1);
    err = amp.Init(net, canNodeID);
    showerr(err, "Initing axis a");

    err = amp.SetAmpMode(AMPMODE_PROG_CRNT);
    showerr(err, "setting desired state to programmed current mode");

    short currentLoopProgrammedValue = 5; // 0.05 A 
    err = amp.sdo.Dnld16(OBJID_PROG_CRNT, 0, currentLoopProgrammedValue);
    showerr(err, "setting current loop programmed value");

    short currentLoopSlopeValue = 0;
    err = amp.sdo.Dnld32(OBJID_CRNT_SLOPE, 0, currentLoopSlopeValue);
    showerr(err, "setting the current loop slope value");

    short currentLoopOffset;
    err = amp.sdo.Upld16(OBJID_CRNTLOOP, 3, currentLoopOffset);
    showerr(err, "getting the current loop offset");

    short commandedCurrent;
    err = amp.sdo.Upld16(OBJID_CRNT_CMD, 0, commandedCurrent);
    showerr(err, "getting the commanded current");

    // wait for at commanded current
    while (commandedCurrent < currentLoopOffset + currentLoopProgrammedValue)
    {
        err = amp.sdo.Upld16(OBJID_CRNT_CMD, 0, commandedCurrent);
        showerr(err, "getting the commanded current");
    }

    // wait for a delay time 3000 milliseconds
    CML::Thread::sleep(3000);

    currentLoopProgrammedValue = 0;
    err = amp.sdo.Dnld16(OBJID_PROG_CRNT, 0, currentLoopProgrammedValue);
    showerr(err, "setting programming current to zero amps, ending move");

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

