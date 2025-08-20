/*

ProfilePositionMode.cpp
Trapezoidal moves.  Update the Target Position (0x607a) using an RPDO. 
Trapezoidal moves support dynamic trajectory updating.  This means that the move
will update to a new target position even if the previous move was not complete.

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

#define numberOfAxes  1

// Target Position RxPDO.  This class is used
// to update the target position of the amp.
class RpdoTargetPosition : public RPDO
{
    uint32 networkReference; // used to transmit the RxPDO on the network
    Pmap32 targetPosition;  // used to update the target position parameter
    Pmap16 controlWord1;
    Pmap16 controlWord2; 

public:
    // default constructor
    RpdoTargetPosition() {}

    // initialize the RxPDO
    const Error* Init(Amp& amp1, int nodeId, int slotNumber);

    // transmit the RxPDO
    const Error* Transmit(int32 targetPositionIn);
};

/* local data */
int32 canBPS = 1000000; // CAN network bit rate
Amp amp[numberOfAxes];
RpdoTargetPosition rpdoTargetPosition;

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
    hw.SetBaud(canBPS);
    CanOpen net;
    int node = 1;
#elif defined( WIN32 )
    WinUdpEcatHardware hw("192.168.0.100");
    EtherCAT net;
    int node = -1;
#else
    LinuxEcatHardware hw("eth0");
    EtherCAT net;
    int node = -1;
#endif

    const Error* err = net.Open(hw);
    showerr(err, "Opening CANopen network");

    for (int i = 0; i < numberOfAxes; i++)
    {
        err = amp[i].Init(net,  i + 1);
        showerr(err, "Initting amp");

        err = amp[i].PreOpNode();
        showerr(err, "Preopping node");

        // use slot 2
        err = rpdoTargetPosition.Init(amp[i], i + 1, 2);
        showerr(err, "Initting target position rpdo");
    }

    // starting all nodes
    for (int i = 0; i < numberOfAxes; i++)
    {
        err = amp[i].StartNode();
        showerr(err, "Starting node");
    }

    int16 trapezoidalProfile = 0;
    for (int k = 0; k < numberOfAxes; k++)
    {
        amp[k].SetProfileVel(50000);
        amp[k].SetProfileAcc(50000);
        amp[k].SetProfileDec(50000);
        amp[k].SetProfileJerk(500000);
        amp[k].SetAmpMode(AMPMODE_CAN_PROFILE);
        amp[k].sdo.Dnld16(OBJID_PROFILE_TYPE, 0, trapezoidalProfile);
    }

    int polarity = 1;

    // send a target position 10 times. 
    for (int i = 0; i < 10; i++)
    {
        if (i % 2) { polarity *= -1; }

        int targetPos = i * 5000 * polarity;
        err = rpdoTargetPosition.Transmit(targetPos);
        showerr(err, "Sending PDO");

        CML::Thread::sleep(1000);
    }

    for (int i = 0; i < numberOfAxes; i++) 
    {
        err = amp[i].WaitMoveDone(-1);
        showerr(err, "waiting for the last move to complete");
    }

    printf("Moves finished.\n");
    return 0;
}

/// <summary>
/// Method to initialize the Profile Position RPDO containing target position data.
/// </summary>
/// <param name="amp">amp object</param>
/// <param name="canId">Node ID for the drive</param>
/// <param name="slotNumber">the slot number of the PDO to use. CML uses slots 0 and 1. Slots 2 and 3 are available per drive.</param>
/// <returns>an error code indicating an error or success</returns>
const Error* RpdoTargetPosition::Init(Amp& amp, int canId, int slotNumber)
{
    networkReference = amp.GetNetworkRef();

    // Init the base class
    uint32 canMessageId = 0x200 + slotNumber * 0x100 + canId;
    const Error* err = RPDO::Init(canMessageId);

    // Init the mapping objects that describe the data mapped to this PDO
    if (!err) err = targetPosition.Init(OBJID_PROFILE_POS);
    if (!err) err = controlWord1.Init(OBJID_CONTROL);
    if (!err) err = controlWord2.Init(OBJID_CONTROL);

    // Add these variables to the PDO
    if (!err) err = AddVar(targetPosition);
    if (!err) err = AddVar(controlWord1);
    if (!err) err = AddVar(controlWord2);

    // load the control word values to automatically update the target position every time this RPDO is transmitted.
    controlWord1.Write(0x002F);
    controlWord2.Write(0x003F);

    // Set the PDO type so that it's data will be acted on immediately
    if (!err) err = SetType(255);

    // Program this PDO
    if (!err) err = amp.PdoSet(slotNumber, *this);

    return err;
}

/// <summary>
/// Method to transmit the RPDO on the network.
/// </summary>
/// <param name="targetPosIn">The target position to move to.</param>
/// <returns>An error code indicating whether or not the PDO transmission was successful.</returns>
const Error* RpdoTargetPosition::Transmit(int32 targetPosIn)
{
    targetPosition.Write(targetPosIn);

    // acquire a reference to the network.
    RefObjLocker<Network> net(networkReference);

    // check that the network is available.
    if (!net) return &NodeError::NetworkUnavailable;

    // tranmit the RxPDO on the network.
    return RPDO::Transmit(*net);
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