/*

CANopenFactorGroups.cpp

Simple-minded example of how to use the CANopen factor groups in position-based control. Factor groups are used to provide user-defined 
units. 

In this example, one user unit will be equal to one motor revolution. To do this, simply set CANopen object 0x608F.1 equal to the counts 
per rev of the encoder connected to the motor shaft.

The CANopen Factor Group objects are listed below.
    •	Position Encoder Increments (0x608F.1)
    •	Position Encoder Revolutions (0x608F.2)
    •	Gear Ratio Motor Revolutions (0x6091.1)
    •	Gear Ratio Shaft Revolutions (0x6091.2)
    •	Feed Constant Feed (0x6092.1)
    •	Feed Constant Shaft Revolutions (0x6092.2)

CANopen objects that are affected by the CANopen Factor Groups are listed below:

Sent from master to drive: 
    •	Target Position (0x607a)
    •	Profile Velocity (0x6081)
    •	Profile Acceleration (0x6083)
    •	Profile Deceleration (0x6084)
    •	Profile Jerk (0x60A4.1)

Sent from drive to master: 
    •	Actual Position (0x6064)
    •	Actual Velocity (0x606C)

The Actual Position (0x6064) is scaled using the factor group objects using the equation below:

Actual Position = ( Position Internal Value * Feed Constant ) / ( Position Encoder Resolution * Gear Ratio )

In terms of CANopen Objects, this formula is:
0x6064 = ( ( Position Internal Value in counts ) * ( 0x6092.1 / 0x6092.2 ) ) / ( ( 0x608F.1 / 0x608F.2 ) * ( 0x6091.1 / 0x6091.2 ) )

All the factor group objects default to a value of 1, so by default, the Position Actual Value is equal to the Position Internal Value.

In this example, one rev was equal to 131,072 encoder counts. 

To set the user units in terms of motor revolutions instead of counts, simply set the position encoder resolution using CANopen object 0x608F.
    •	Position Encoder Increments (0x608F.1) = 131,072
    •	Position Encoder Revolutions (0x608F.2) = 1

These settings yield this calculation being performed by the Copley firmware to calculate actual position in user-defined units:
Position actual value (user units) = ( position internal value in counts * (1 / 1) )  /  (  (131,072 / 1) * (1 / 1)  ) )

So, if the position internal value is 131,072 counts, the Actual Position (0x6064) will read a value of 1 over the CANopen network in user 
units of motor revolutions. 

Note that the other trajectory parameters are also scaled using this value.

Profile Velocity = ( input value sent over CANopen ) * ( ( position encoder resolution * gear ratio ) / feed constant ) * ( 0.1 counts/sec )
Profile Velocity = ( input value sent over CANopen ) * ( 131,072 ) * (  0.1 counts/sec )

So, a value of 10 set in Profile Velocity (0x6081) over CANopen would provide:

Profile Velocity = 10 * ( 131,072 ) * ( 0.1 counts/sec )
Profile Velocity = 131,072 counts/sec

Converted to RPM is:
( 131,072 counts / sec ) * ( 60 sec / 1 min ) * ( 1 rev / 131,072 counts ) = 60 RPM

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
int16 canNodeID = 2;                // CANopen node ID

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

    // read the counts per rev of the encoder. In this example, this
    // value was read as 131,072 encoder counts. 
    MtrInfo infoObj;
    amp.GetMtrInfo(infoObj);
    double countsPerRev = infoObj.ctsPerRev;

    // set mode of operation to Profile Position Mode (1)
    int8 modeOfOperation = 1;
    err = amp.sdo.Dnld8(0x6060, 0, modeOfOperation);
    showerr(err, "configuring profile position mode");

    // set move type to s-curve profile
    int16 profileType = 3;
    err = amp.sdo.Dnld16(0x6086, 0, profileType);
    showerr(err, "configuring s-curve profile type");
    
    uint32 positionEncoderIncrements = (uint32)countsPerRev;
    err = amp.sdo.Dnld32(0x608F, 1, positionEncoderIncrements);
    showerr(err, "setting the position encoder increments (factor group object 0x608F.1)");

    uint32 positionEncoderRevolutions = 1;
    err = amp.sdo.Dnld32(0x608F, 2, positionEncoderRevolutions);
    showerr(err, "setting the position encoder revolutions (factor group object 0x608F.2)");
    
    // we will move two motor revolutions
    int32 targetPosition = 2;
    err = amp.sdo.Dnld32(0x607a, 0, targetPosition);
    showerr(err, "setting the target position to 2 revs");

    uint32 profileVelocity = 10; // in units of 0.1 counts/sec * ( 131,072 / 1 )
    err = amp.sdo.Dnld32(0x6081, 0, profileVelocity);
    showerr(err, "setting the profile velocity");

    uint32 profileAccel = 13; // in units of 10 counts/sec^2 * ( 131,072 / 1 )
    err = amp.sdo.Dnld32(0x6083, 0, profileAccel);
    showerr(err, "setting the profile accel");

    uint32 profileDecel = 13; // in units of 10 counts/sec^2 * ( 131,072 / 1 )
    err = amp.sdo.Dnld32(0x6084, 0, profileDecel);
    showerr(err, "setting the profile decel");

    uint32 profileJerk = 131; // in units of 100 counts/sec^3 * ( 131, 072 / 1 )
    err = amp.sdo.Dnld32(0x60A4, 1, profileJerk);
    showerr(err, "setting the profile jerk");

    // start the move
    uint16 controlWordEnable = 0x000F;
    err = amp.sdo.Dnld16(0x6040, 0, controlWordEnable);
    showerr(err, "enabling the drive");

    uint16 controlWordStartMove = 0x003F;
    err = amp.sdo.Dnld16(0x6040, 0, controlWordStartMove);
    showerr(err, "starting move");

    CML::Thread::sleep(10); // wait a little bit for the move to begin

    // Bit 27 of event status register is the in-motion bit.
    // High if moving. Low if not moving and settled in 
    // position window. 
    uint32 inMotionBit = 0x8000000;
    uint32 eventStatusWord = 0;
    err = amp.sdo.Upld32(0x1002, 0, eventStatusWord);
    showerr(err, "reading the event status word");

    bool inMotion = ((eventStatusWord & inMotionBit) == inMotionBit);
    while( inMotion == true ) 
    {
        int32 actualPosition = 0; 
        err = amp.sdo.Upld32(0x6064, 0, actualPosition);
        showerr(err, "reading the actual position");
        printf("actual position: %d\n", actualPosition);

        int32 actualVelocity = 0;
        err = amp.sdo.Upld32(0x606C, 0, actualVelocity);
        showerr(err, "reading the actual velocity");
        printf("actual velocity: %d\n", actualVelocity);

        err = amp.sdo.Upld32(0x1002, 0, eventStatusWord);
        showerr(err, "reading the event status word");
        inMotion = ((eventStatusWord & inMotionBit) == inMotionBit);
    }

    printf("Move complete\n");

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