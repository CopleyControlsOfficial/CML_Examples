/*

ProfileTorqueMode.cpp

Simple example of how to start and stop a Profile Torque Mode move.

*/

// Comment this out to use EtherCAT
//#define USE_CAN

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include "CML.h"

using std::cout;
using std::endl;

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

/**************************************************
* Start a profile torque mode move, wait for 3
* seconds, then stop the move.
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

    /* local data */
    int32 canBPS = 1000000;             // CAN network bit rate
    int16 canNodeID = -1;                // CANopen node ID

    // Create an object used to access the low level CAN network.
    // This examples assumes that we're using the Copley PCI CAN card.
#if defined( USE_CAN )
    CopleyCAN hw("CAN0");
    hw.SetBaud(canBPS);
    canNodeID = 1;
#elif defined( WIN32 )
    WinUdpEcatHardware hw("192.168.0.244");
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

    // Initialize the amplifier using default settings
    Amp amp;
    printf("Doing init\n");
    err = amp.Init(net, canNodeID);
    showerr(err, "Initting amp");

    err = amp.SetAmpMode(AMPMODE_CAN_TORQUE);
    showerr(err, "setting mode of operation to profile torque mode (mode 4)");

    // The rate of change specified in thousandths of the total rated torque
    // per second.For example, setting to 1000 would specify a slope of the full
    // rated torque of the motor every second.
    err = amp.SetTorqueSlope(500);
    showerr(err, "setting torque slope");

    // The torque value to be set.This is specified in thousandths
    // of the motor rated torque.
    err = amp.SetTorqueTarget(100);
    showerr(err, "setting target torque");

    CML::Thread::sleep(3000); // move for 3 seconds. 

    // set the target torque to zero, ending the move.
    amp.SetTorqueTarget(0);

    printf("Profile Torque Mode move complete.\n");

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

