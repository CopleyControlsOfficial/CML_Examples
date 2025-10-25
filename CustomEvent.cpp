/*

CustomEvent.cpp

The following is an example of how to use the EventMap class
in CML to wait for a specific event. In this example, we will
wait for an input (configured as home switch) to capture the 
home position. When the home position has been captured, the 
event will be triggered. We will do this three times in a row. 
The event is reset by clearing the event mask and calling 
EventMap::Wait again.

*/

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "CML.h"

#define USE_CAN
#if defined( USE_CAN )
#include "can/can_copley.h"
#elif defined( WIN32 )
#include "ecat/ecat_winudp.h"
#else
#include "ecat/ecat_linux.h"
#endif

using std::cout;

// If a namespace has been defined in CML_Settings.h, this
// macros starts using it. 
CML_NAMESPACE_USE();

/* local functions */
static void showerr(const Error* err, const char* str);

class CmdThread : public Thread
{
    uint32 maskForThisThread; // set the global event mask to this value when the event occurs.
    Amp* ampObjPntr;
    EventMap* eventMapPntr;     // a reference to the EventMap object
    int lastCapturedHomePosition = 0;
    bool quit;
public:
    void Init(Amp* ampPntrIn, uint32 maskForThisThreadIn, EventMap* eventMapObjIn);
    void run();
    void Quit() { quit = true; }
    int GetLastCapturedHomePosition();
};

int main(void)
{
    // The libraries define one global object of type
    // CopleyMotionLibraries named cml.
    //
    // This object has a couple handy member functions
    // including this one which enables the generation of
    // a log file for debugging
    cml.SetDebugLevel(LOG_EVERYTHING);
    //cml.SetFlushLog( true );

    // Create an object used to access the low level CAN network.
    // This examples assumes that we're using the Copley PCI CAN card.
#ifdef USE_CAN
    CopleyCAN hw("CAN0");
    int32 canBPS = 1000000;             // CAN network bit rate
    hw.SetBaud(canBPS);
    CanOpen net;
    int nodeId = 1;
#elif defined( WIN32 )
    WinUdpEcatHardware hw("192.168.0.205");
    EtherCAT net;
    int nodeId = -1;
#else
    LinuxEcatHardware hw("eth0");
    EtherCAT net;
    int nodeId = -1;
#endif

    const Error* err = net.Open(hw);
    showerr(err, "Opening CANopen network");

    EventMap homePositionCapturedEvent;
    Amp amp;

    // Initializing the first axis
    err = amp.Init(net, nodeId);
    showerr(err, "Initting axis A");

    // set the homing method to "set current position as home"
    err = amp.SetHomeMethod(CHM_NONE);
    showerr(err, "setting homing method");

    uint16 inputConfig = 14; // home switch - active hi
    err = amp.sdo.Dnld16(OBJID_INPUT_CFG, 1, inputConfig);
    showerr(err, "configuring IN1 as home switch - active hi");

    // Create an mask value that signifies when the home
    // position has been captured in the drive.
    EventAll event = 1;

    // Set the mask for this thread equal to the mask for 
    // the event so that the thread will trigger the event
    // using its mask.
    uint32 maskForThisThread = 1; 
    CmdThread cmdThread;
    cmdThread.Init(&amp, maskForThisThread, &homePositionCapturedEvent);
    cmdThread.start();

    // Clear the event mask.
    homePositionCapturedEvent.setMask(0);

    // In the main loop, I simply wait on an event which will occur when 
    // the home position has been captured. Once the data has been received,
    // I will print the new home captured position to the console.
    for (int i = 0; i < 3; i++)
    {
        err = amp.DoMove((i + 1) * 300);
        showerr(err, "making move");

        err = amp.WaitMoveDone(-1);
        showerr(err, "waiting for move to finish");

        // BLOCKING CALL: WAIT FOR HOME SWITCH
        // Timeout = -1 = Wait forever
        err = event.Wait(homePositionCapturedEvent, -1);
        if (err) showerr(err, "Waiting on events");

        // Clear the event mask. 
        homePositionCapturedEvent.setMask(0);

        printf("Captured Position: %d\n", cmdThread.GetLastCapturedHomePosition());
    }

    cmdThread.Quit();

    printf("Finished. Press any key to quit.\n");
    return 0;
}

void CmdThread::Init(Amp* ampPntrIn, uint32 maskForThisThreadIn, EventMap* eventMapPntrIn)
{
    ampObjPntr = ampPntrIn;
    maskForThisThread = maskForThisThreadIn;
    eventMapPntr = eventMapPntrIn;
}

void CmdThread::run()
{
    uint32 networkReference = (*ampObjPntr).GetNetworkRef();

    // acquire a reference to the network.
    RefObjLocker<CanOpen> net(networkReference);

    // check that the network is available.
    if (!net)
    {
        printf("Network object was unavailable\n");
        quit = true;
    }

    const Error* err = 0;

    while (!quit)
    {
        uint16 positionCaptureStatus = 0;
        err = (*ampObjPntr).sdo.Upld16(OBJID_CAP_STAT, 0, positionCaptureStatus);
        showerr(err, "reading position capture status");

        uint16 positionCapturedMask = 0x10; // bit 4 = home position captured
        if (positionCaptureStatus & positionCapturedMask) 
        {
            err = (*ampObjPntr).sdo.Upld32(OBJID_CAP_HOME, 0, lastCapturedHomePosition);
            showerr(err, "reading the last captured position");
            (*eventMapPntr).setBits(maskForThisThread);
        }
    }
}

int CmdThread::GetLastCapturedHomePosition() 
{
    return lastCapturedHomePosition;
}

// show any errors to the user.
static void showerr(const Error* err, const char* str)
{
    if (err)
    {
        printf("Error %s: %s\n", str, err->toString());
        exit(1);
    }
}