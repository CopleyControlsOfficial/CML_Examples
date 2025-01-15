/*

ProfileVelocityMode.cpp

Simple example of how to start and stop a Profile Velocity Mode move.

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
* Start a profile velocity mode move, wait for 3
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

    // Initialize the amplifier using default settings
    Amp amp;
    printf("Doing init\n");
    err = amp.Init(net, canNodeID);
    showerr(err, "Initting amp");

    err = amp.sdo.Dnld8(0x6060, 0, (int8)CML::AMPMODE_CAN_VELOCITY);
    showerr(err, "setting mode of operation to profile velocity mode (mode 3)");

    amp.SetProfileAcc(1000); // units are 10 counts/sec^2
    amp.SetProfileDec(1000); // units are 10 counts/sec^2
    
    // set the target velocity to 1000 in units of 0.1 counts/sec
    amp.SetTargetVel(1000);

    CML::Thread::sleep(3000); // jog for 3 seconds. 

    // set the target velocity to zero, ending the move.
    amp.SetTargetVel(0);

    printf("Profile Velocity Mode move complete.\n");

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
