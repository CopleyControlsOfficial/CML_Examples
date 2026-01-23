/*

TouchProbe.cpp

Simple-minded example of how to configure a general 
purpose input as a CANopen Touch Probe. The position
and time will be captured on the rising edge of the 
configured touch probe input. 

This example configures IN1 as the touch probe input.
This example will start a move in the positive direction, 
then poll the touch probe status. When touch probe 1 has 
triggered, the captured position and time are printed
to the console.

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
    const Error* err = net.Open(hw);
    showerr(err, "Opening network");

    // Initialize the amplifier using default settings
    Amp amp;
    printf("Doing init\n");
    err = amp.Init(net, canNodeID);
    showerr(err, "Initting amp");

    // start at absolute position 0
    err = amp.MoveAbs(0);
    showerr(err, "moving to the zero position");
    err = amp.WaitMoveDone();
    showerr(err, "waiting for the move to the zero position to finish");

    // Configure the touch probe function object (0x60B8) as 0x001B
    // Bit 0 = Enable touch probe 1
    // Bit 1 = Overwrite old captured position
    // Bits 2-3 = 2 = Use input pin selected by 0x60D0.1
    // Bit 4 = Capture on rising edge of touch probe 1.
    uint16 touchProbeFunction = 0x001B;
    err = amp.sdo.Dnld16(0x60B8, 0, touchProbeFunction);
    showerr(err, "configuring the touch probe function object (0x60B8) as 0x001B");

    // select IN1 as the source for touch probe sensor 1
    int16 touchProbeInputSelect = -1; // -1 = IN1, -2 = IN2, etc.
    err = amp.sdo.Dnld16(0x60D0, 1, touchProbeInputSelect);
    showerr(err, "configuring IN1 as the source for touch probe sensor 1");

    err = amp.MoveAbs(1310720);
    showerr(err, "starting move to 1310720 counts");

    printf("Making move\n");

    // Bit 1 of touch probe status is set if a position 
    // has been captured on a positive edge of touch probe 1
    uint16 positionCapturedMask = 0x0002;
    uint16 touchProbeStatus = 0;
    err = amp.sdo.Upld16(0x60B9, 0, touchProbeStatus);
    showerr(err, "reading the touch probe status");

    bool positionCaptured = ((touchProbeStatus & positionCapturedMask) == positionCapturedMask);
    while( positionCaptured == false )
    {
        err = amp.sdo.Upld16(0x60B9, 0, touchProbeStatus);
        showerr(err, "reading the touch probe status");
        positionCaptured = ((touchProbeStatus & positionCapturedMask) == positionCapturedMask);
        CML::Thread::sleep(100);
    }

    // 0x60BA = position captured on rising edge of touch probe 1
    int32 capturedPosition = 0;
    err = amp.sdo.Upld32(0x60BA, 0, capturedPosition);
    showerr(err, "reading touch probe 1 captured position");
    printf("Touch probe 1 rising edge captured position: %d\n", capturedPosition);

    // 0x60D1 = time captured on rising edge of touch probe 1
    uint32 capturedTime = 0;
    err = amp.sdo.Upld32(0x60D1, 0, capturedTime);
    showerr(err, "reading touch probe 1 captured time");
    printf("Touch probe 1 rising edge captured time: %d\n", capturedTime);

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
