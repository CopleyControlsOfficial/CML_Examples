/*

InputTriggersLinkageMove.cpp

The following is an example of how to read a digital input pin and use it to 
trigger a linkage move. We will wait for IN1 to go LO, then make a linkage 
move to position < 0, 0, 0 >. Then we will wait for IN1 to go HI, then make 
a linkage move to position < 10, 10, 10 >. 

This example initializes an ME3 drive on an EtherCAT network.
The p-loop and v-loop update rates of the ME3/ME4 drives are 2.5kHz (400 usec). 
The EtherCAT loop update rate (sync0 pulse period) must be an integer multiple 
of this number. Therefore, a value of 2ms must be configured for the sync0 
pulse period in order to command these drives over EtherCAT using CML. Set the
sync0 pulse period using the AmpSettings object.

*/

// Comment this out to use EtherCAT
//#define USE_CAN

#include "CML.h"
#include <iostream>

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
    WinUdpEcatHardware hw("192.168.0.40");
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
    Amp amp[3];
    AmpSettings amp_settings;

    // ME3/ME4 synchPeriod is 2000 usec (2 ms)
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

    // Create a linkage object holding these amps
    Linkage link;
    err = link.Init(3, amp);
    showerr(err, "Linkage init");

    double pathMaxVel{ 160000 };
    double pathMaxAccel{ 960000 };
    double pathMaxDecel{ 960000 };
    double pathMaxJerk{ 200000 };

    // set the limits for the linkage object
    err = link.SetMoveLimits(pathMaxVel, pathMaxAccel, pathMaxDecel, pathMaxJerk); showerr(err, "Setting Linkage Move Limits");
    showerr(err, "setting the linkage move trajectory limits");

    uint32 digitalInputPinStates = 0;  
    err = amp[0].GetInputs32(digitalInputPinStates);
    showerr(err, "reading input pin states");

    cout << "Digital input pin states are: " << digitalInputPinStates << endl; 
    cout << "Pull IN1 low to make a move to < 0, 0, 0 >" << endl;

    // In my case, the input pin states are reading "1863807".
    // I'll wait for IN1 to go LO. I'll toggle it using a switch.
    int in1Mask = 1; // bit 0 represents IN1
    err = amp[0].WaitInputLow(in1Mask);
    showerr(err, "waiting for IN1 to go low");

    Point<3> targetPosition;
    targetPosition[0] = 0; // X coordinate
    targetPosition[1] = 0; // Y coordinate
    targetPosition[2] = 0; // Z coordinate

    err = link.MoveTo(targetPosition);
    showerr(err, "starting move to target position 1");

    err = link.WaitMoveDone(-1);
    showerr(err, "waiting for move to finish to target position 1");

    cout << "Pull IN1 high to make a move to < 10, 10, 10 >" << endl;

    err = amp[0].WaitInputHigh(in1Mask);
    showerr(err, "waiting for IN1 to go high");

    targetPosition[0] = 10; // X coordinate
    targetPosition[1] = 10; // Y coordinate
    targetPosition[2] = 10; // Z coordinate

    err = link.MoveTo(targetPosition);
    showerr(err, "starting move to target position 2");

    err = link.WaitMoveDone(-1);
    showerr(err, "waiting for move to finish to target position 2");

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