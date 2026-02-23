/*

LinkageMonitor.cpp

The following example starts a profile velocity mode move on a dual-axis 
network. It then creates a Linkage object and uses this object to 
passively monitor all the axes in the linkage using the Linkage::WaitEvent 
method. The events that can be waited on are defined in the LINK_EVENT 
enumerator. The linkage is monitored in a separate thread and if any of
the axes in the linkage encounter an error, it will halt all the axes in 
the linkage.

This example triggers the halt command to all axes in the linkage if any
axis encounters an abort, fault, or error condition.

// events that we want to trigger the halt command.
EventAny event = ( LINK_EVENT::LINKEVENT_ABORT | LINK_EVENT::LINKEVENT_ERROR | LINK_EVENT::LINKEVENT_FAULT );

If this event occurs, we want to make sure that all axes decelerate at the
same rate. Below are important settings to ensure that this happens:

    1) Set the Halt Option Code (0x605A) to a value of 2 to use the Quick Stop
    Deceleration (0x6085) when a halt command is issued.
    
    2) Set the Abort Option Code (0x6007) to 3 to issue a quick stop command in 
    the event of an abort.
    
    3) Set the Quick Stop Option Code (0x605D) to a value of 2 to use the Quick
    Stop Deceleration (0x6085) when a quick stop command is issued.

*/

// Comment this out to use EtherCAT
#define USE_CAN

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

#define AxisCount 2

// If a namespace has been defined in CML_Settings.h, this
// macros starts using it. 
CML_NAMESPACE_USE();

/* local functions */
static void showerr(const Error* err, const char* str);

class LinkageMonitorThread : public Thread
{
    Linkage* linkObjPntr;
    bool quit;
public:
    void Init( Linkage* linkPntrIn );
    void run();
    void Quit() { quit = true; }
};

/**************************************************
* Start a profile velocity mode move, wait for 3
* seconds, then stop the move.
**************************************************/
int main( void )
{
    // The libraries define one global object of type
    // CopleyMotionLibraries named cml.
    //
    // This object has a couple handy member functions
    // including this one which enables the generation of
    // a log file for debugging
    cml.SetDebugLevel( LOG_EVERYTHING );

    /* local data */
    int32 canBPS = 1000000;             // CAN network bit rate
    int16 canNodeID = -1;                // CANopen node ID

    // Create an object used to access the low level CAN network.
    // This examples assumes that we're using the Copley PCI CAN card.
#if defined( USE_CAN )
    CopleyCAN hw( "CAN0" );
    hw.SetBaud( canBPS );
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

    const Error* err = net.Open( hw );
    showerr( err, "Opening network" );

    // Initialize the amplifier using default settings
    Amp amp[AxisCount];
    printf( "Doing init\n" );
    for ( int i = 0; i < AxisCount; i++ ) 
    {
        err = amp[i].Init( net, canNodeID + i );
        showerr( err, "Initting amp" );

        // if we command an axis to halt, it should decelerate using the quick stop deceleration (0x6085)
        err = amp[i].SetHaltMode( CML::HALT_MODE::HALT_QUICKSTOP );
        showerr( err, "Setting halt option code (0x605D) to 2 (use the quick stop deceleration (0x6085))" );

        // if one of the drives aborts, it should perform a quick stop command
        short abortOptionCodeValue = 3; // quickstop command
        err = amp[i].sdo.Dnld16( 0x6007, 0, abortOptionCodeValue );
        showerr( err, "Setting the abort option code to 3 (perform quick stop command)" );

        // if performing a quick stop, use the quick stop deceleration (0x6085)
        err = amp[i].SetQuickStop( CML::QUICK_STOP_MODE::QSTOP_QUICKSTOP );
        showerr( err, "Setting the quick stop option code (0x605A) to 2 (use quick stop decel - CANopen object index 0x6085)" );
    }

    Linkage link;
    err = link.Init( 2, amp );
    showerr( err, "initializing linkage object" );

    for( int i = 0; i < AxisCount; i++ )
    {
        err = amp[i].SetAmpMode( AMPMODE_CAN_VELOCITY );
        showerr( err, "setting mode of operation to profile velocity mode (mode 3)" );
        err = amp[i].SetProfileAcc( 1000 ); // units are 10 counts/sec^2
        showerr( err, "setting profile accel" );
        err = amp[i].SetProfileDec( 1000 ); // units are 10 counts/sec^2
        showerr( err, "setting profile decel" );
    }

    LinkageMonitorThread linkMonitorThread;
    linkMonitorThread.Init( &link );

    for( int i = 0; i < AxisCount; i++ )
    {
        // set the target velocity to 1000 in units of 0.1 counts/sec
        err = amp[i].SetTargetVel( 1000 );
        showerr( err, "setting target velocity (starting move)" );
    }

    // wait for the moves to begin on all the nodes
    for( int i = 0; i < AxisCount; i++ )
    {
        CML::EVENT_STATUS eventStatus;
        err = amp[i].GetEventStatus( eventStatus );
        showerr(err, "reading event status word");
        while( ( eventStatus & CML::EVENT_STATUS::ESTAT_MOVING ) != CML::EVENT_STATUS::ESTAT_MOVING )
        {
            err = amp[i].GetEventStatus( eventStatus );
            showerr(err, "reading event status word");
        }
        printf( "node %d is moving\n", i+1 );
    }

    linkMonitorThread.start();

    CML::Thread::sleep( 10000 ); // jog for 10 seconds. 

    for (int i = 0; i < AxisCount; i++)
    {
        // set the target velocity to zero, ending the move.
        err = amp[i].SetTargetVel( 0 );
        showerr( err, "setting target velocity to 0 (ending move)" );
    }

    printf( "Profile Velocity Mode move complete.\n" );

    linkMonitorThread.Quit();
    CML::Thread::sleep(250); // wait 250ms for linkMonitorThread to die.

    return 0;
}

void LinkageMonitorThread::Init( Linkage* linkPntrIn )
{
    linkObjPntr = linkPntrIn;
}

void LinkageMonitorThread::run()
{
    const Error* err = 0;

    // events that we want to trigger the halt command.
    EventAny event = ( LINK_EVENT::LINKEVENT_ABORT | LINK_EVENT::LINKEVENT_ERROR | LINK_EVENT::LINKEVENT_FAULT );
    float timeoutMilliseconds = 1.0; 
    LINK_EVENT linkEventReturned;
    err = ( *linkObjPntr ).WaitEvent( event, timeoutMilliseconds, linkEventReturned );
    while( ( err == &ThreadError::Timeout ) && ( !quit ) )
    {
        err = ( *linkObjPntr ).WaitEvent( event, 1, linkEventReturned );
    }

    // I'll return an error if it's not a timeout error.
    if( err != &ThreadError::Timeout )
    {
        showerr( err, "error waiting on linkage event" );
    }

    // If the main thread requests this thread to quit, just return.
    if( quit )
    {
        printf( "LinkageMonitorThread::Quit() was called.\n" );
        printf( "Killing LinkageMonitorThread\n" );
        return;
    }
    // The event occurred. Halt the linkage.
    else 
    {
        err = ( *linkObjPntr ).HaltMove();
        showerr( err, "commanding each amp in the linkage to halt" );

        if( linkEventReturned & LINK_EVENT::LINKEVENT_ABORT ){ printf( "One of the amps in the linkage encountered an abort.\n" ); }
        if( linkEventReturned & LINK_EVENT::LINKEVENT_ERROR ){ printf( "One of the amps in the linkage encountered an error.\n" ); }
        if( linkEventReturned & LINK_EVENT::LINKEVENT_FAULT ){ printf( "One of the amps in the linkage encountered an fault.\n" ); }
        printf( "All amps in the linkage are commanded to halt.\n" );
    }
}

/**************************************************/

static void showerr(const Error* err, const char* str)
{
    if( err )
    {
        printf( "Error %s: %s\n", str, err->toString() );
        exit(1);
    }
}
