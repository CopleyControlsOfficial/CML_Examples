/*

SimpleLinkageMove.cpp

A basic example of how to command a linkage move. 

All linked axes will start and end the move at exactly the same time. 
Simply specify the n-dimensional target position to travel to. 
The linkage will move in an n-dimensional straight line to that target
position. The trajectory will resemble an s-curve.  If a trapezoidal 
profile is desired, simply increase the acceleration, deceleration, 
and jerk values in the trajectory limits for the linkage. 

*/

// Comment this out to use EtherCAT
#define USE_CAN

#include <cstdio>
#include <cstdlib>
#include <iostream>

#include "CML.h"

#if defined( USE_CAN )
#include "can/can_copley.h"
#elif defined( WIN32 )
#include "ecat/ecat_winudp.h"
#else
#include "ecat/ecat_linux.h"
#endif

using std::cout;
using std::endl;

// If a namespace has been defined in CML_Settings.h, this
// macros starts using it. 
CML_NAMESPACE_USE();

/* local functions */
static void showerr(const Error* err, const char* str);

/* local defines */
#define AMPCT 3

/* local data */
int32 canBPS = 1000000;                // CAN network bit rate
const char* canDevice = "CAN0";        // Identifies the CAN device, if necessary
int16 canNodeID = 1;                   // CANopen node ID of first amp.  Second will be ID+1, etc.


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
    WinUdpEcatHardware hw("eth0");
#else
    LinuxEcatHardware hw("eth0");
#endif

    // Open the network object
#if defined( USE_CAN )
    CanOpen net;
#else
    EtherCAT net;
#endif

    const Error* err = NULL;

    err = net.Open(hw);
    showerr(err, "Opening network");

    // Initialize the amplifiers using default settings
    Amp amp[AMPCT];
    printf("Doing init\n");
    for (int i = 0; i < AMPCT; i++)
    {
        printf("Initing %d\n", canNodeID + i);
        err = amp[i].Init(net, canNodeID + i);
        showerr(err, "Initting amp");

        MtrInfo mtrInfo;
        err = amp[i].GetMtrInfo(mtrInfo);
        showerr(err, "Getting motor info\n");

        // configuring non-default user units. user unit value of 1.0 will be equal to 1 millimeter (ASCII parameter 0x62 in flash).
        err = amp[i].SetCountsPerUnit(mtrInfo.ctsPerRev);
        showerr(err, "Setting cpr\n");
    }

    // begin homing all axes
    for (int i = 0; i < AMPCT; i++)
    {
        amp[i].GoHome();
        showerr(err, "commanding home");
    }

    // wait for all axes to finish homing routine
    for (int i = 0; i < AMPCT; i++)
    {
        amp[i].WaitHomeDone(-1);
        showerr(err, "waiting for home to complete");
    }
    
    // Create a linkage object holding these amps
    Linkage link;
    err = link.Init(AMPCT, amp);
    showerr(err, "Linkage init");

    double velocity = 0.5; // this is the max velocity achievable for the system in terms of user units. 
    double accel = 5.0;
    double decel = 1.0;
    double jerk = 10.0;

    // Setup the velocity, acceleration, deceleration & jerk limits
    // for multi-axis moves using the linkage object
    err = link.SetMoveLimits(velocity, accel, decel, jerk);
    showerr(err, "setting move limits");

    // Create an arbitrary N dimensional target position.
    Point<AMPCT> targetPosition;

    double cmdPos2 = 0;
    amp[2].GetPositionCommand(cmdPos2);

    targetPosition[0] = 0.25; // one-quarter millimeter
    targetPosition[1] = 0.25; // one-quarter millimeter
    targetPosition[2] = cmdPos2; // do not move.  Hold position. 

    link.MoveTo(targetPosition);
    showerr(err, "starting linkage move");

    err = link.WaitMoveDone(-1);
    showerr(err, "waiting for move to finish");
    
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