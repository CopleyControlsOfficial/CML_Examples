/*

ProfileVelocityMode.cpp

Simple example of how to start and stop a Profile Velocity move.
Profile Velocity Mode is mode 3.  Mode of operation (0x6060) = 3. 
It is a defined mode in the DS402 CANopen Protocol for Motion Control.

This example is written for EtherCAT networks using a multi-axis drive
(BE2, XE2, ME3, ME4, etc).

*/

// Comment this out to use EtherCAT
//#define USE_CAN

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include "CML.h"

using std::cout;
using std::endl;

#define AxisCount 2

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

// Position/Velocity transmit PDO.  This class is used
// to receive position and velocity updates that are sent 
// automatically every EtherCAT PDO cycle.
class TpdoEcatActVelActPosDualAxis : public TPDO
{
public:
    Pmap32 actualPosAxisA; // 32 bits
    Pmap32 actualVelAxisA; // 32 bits
    Pmap32 actualPosAxisB; // 32 bits
    Pmap32 actualVelAxisB; // 32 bits

    /// Default constructor for this PDO
    TpdoEcatActVelActPosDualAxis() {}

    // initialize the TxPDO
    const Error* Init(Amp& amp, int slot);

    // virtual function signifying reception of TxPDO
    virtual void Received(void);
};

// Profile Velocity RxPDO.  This class is used
// to update the profile velocity of the amp.
class RpdoEcatProfileVelDualAxis : public RPDO
{
    uint32 networkReference;
    Pmap32 profileVelocityAxisA;
    Pmap32 profileVelocityAxisB;

public:
    // default constructor
    RpdoEcatProfileVelDualAxis() {}

    // initialize the RxPDO
    const Error* Init(Amp& amp, uint16 slot);

    // transmit the RxPDO
    const Error* Transmit(int32 profileVelocityAxisA, int32 profileVelocityAxisB);
};

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
    WinUdpEcatHardware hw("192.168.0.54");
#else
    LinuxEcatHardware hw("eth0");
#endif

    // Open the network object
#if defined( USE_CAN )
    CanOpen net;
#else
    EtherCAT net;
#endif

    EtherCatSettings ecatSettingsObj;
    //// uncomment the following line if using a non real-time operating system (i.e. Windows)
    //ecatSettingsObj.cyclePeriod = 20; // 20 ms.

    const Error* err = net.Open(hw, ecatSettingsObj);
    showerr(err, "Opening network");

    TpdoEcatActVelActPosDualAxis tpdo;
    RpdoEcatProfileVelDualAxis rpdo;

    AmpSettings ampSettingsObj;
    //// uncomment the following line is using an ME3 or ME4 multi-axis drive.
    //ampSettingsObj.synchPeriod = 2000;

    // Initialize the amplifier using default settings
    Amp amp[AxisCount];
    printf("Doing init\n");
    err = amp[0].Init(net, canNodeID, ampSettingsObj);
    showerr(err, "Initting amp");

    err = amp[1].InitSubAxis(amp[0], 2);
    showerr(err, "Initting sub-axis of multi-axis drive");

    err = amp[0].PreOpNode();
    showerr(err, "Preopping node");

    err = tpdo.Init(amp[0], 2);
    showerr(err, "Initting tpdo");

    err = rpdo.Init(amp[0], 1);
    showerr(err, "Initting rpdo");

    err = amp[0].StartNode();
    showerr(err, "Starting node");

    err = amp[0].sdo.Dnld8(0x6060, 0, (int8)CML::AMPMODE_CAN_VELOCITY);
    showerr(err, "setting mode of operation to profile velocity mode (mode 3) on Axis A");

    err = amp[0].sdo.Dnld8(0x6060 + 0x800, 0, (int8)CML::AMPMODE_CAN_VELOCITY);
    showerr(err, "setting mode of operation to profile velocity mode (mode 3) on Axis B");

    for (int i = 0; i < AxisCount; i++) 
    {
        amp[i].SetProfileAcc(1000); // units are 10 counts/sec^2
        amp[i].SetProfileDec(1000); // units are 10 counts/sec^2
    }

    // Jog each axis 3000 in units of 0.1 counts/sec 
    err = rpdo.Transmit(3000, 3000);
    showerr(err, "Sending RPDO");

    // jog for 3 seconds.
    for (int i = 0; i < 30; i++) 
    {
        cout << "Act Pos Axis A: " << (double)tpdo.actualPosAxisA.Read() << endl;
        cout << "Act Pos Axis B: " << (double)tpdo.actualPosAxisB.Read() << endl;
        cout << "Act Vel Axis A: " << (double)tpdo.actualVelAxisA.Read() << endl;
        cout << "Act Vel Axis B: " << (double)tpdo.actualVelAxisB.Read() << endl;
        CML::Thread::sleep(100); // sleep for 100 milliseconds
    }

    // Stop jogging. Set the target velocity to 0.
    err = rpdo.Transmit(0, 0);
    showerr(err, "Sending RPDO");

    printf("Profile Velocity Mode move complete.\n");

    return 0;
}

/**************************************************/

/**
 * Transmit PDO handling.
 * This class handles receiving these TPDOs from the
 * amplifier object. TPDOs have important feedback 
 * data inside of them, like Actual Position and 
 * Actual Velocity, etc. 
 *
 * @param amp   The amplifier to map this PDO.
 * @param slot  The slot to use for this PDO.
 * @return NULL on success, or an error object on failure.
 */
const Error* TpdoEcatActVelActPosDualAxis::Init(Amp& amp, int slotNumber)
{
    const Error* err = 0;

    // Init the position variable using the CANopen object ID 
    if (!err) err = actualPosAxisA.Init(OBJID_POS_LOAD, 0);
    if (!err) err = actualVelAxisA.Init(OBJID_VEL_ACT, 0);
    if (!err) err = actualPosAxisB.Init(OBJID_POS_LOAD + 0x800, 0);
    if (!err) err = actualVelAxisB.Init(OBJID_VEL_ACT + 0x800, 0);

    // Add the mapped variable
    if (!err) err = AddVar(actualPosAxisA);
    if (!err) err = AddVar(actualVelAxisA);
    if (!err) err = AddVar(actualPosAxisB);
    if (!err) err = AddVar(actualVelAxisB);

    // Program this PDO in the amp, and enable it
    if (!err) err = amp.PdoSet(slotNumber, *this);

    return err;
}

/**
 * This function will be called by the high priority CANopen receive
 * thread when the TPDO is received.
 */
void TpdoEcatActVelActPosDualAxis::Received(void)
{
    // nothing to do at this time
}

/// <summary>
/// Initialize the RPDO using the amp object and the slot (PDO) number
/// </summary>
/// <param name="amp">Amplifier object reference</param>
/// <param name="slotNumber">PDO number to use</param>
/// <returns>CML error code</returns>
const Error* RpdoEcatProfileVelDualAxis::Init(Amp& amp, uint16 slotNumber)
{
    networkReference = amp.GetNetworkRef();

    const Error* err = 0;

    // Init the mapping objects that describe the data mapped to this PDO
    if (!err) err = profileVelocityAxisA.Init(OBJID_TARGET_VEL);
    if (!err) err = profileVelocityAxisB.Init(OBJID_TARGET_VEL + 0x800);

    // Add these variables to the PDO
    if (!err) err = AddVar(profileVelocityAxisA);
    if (!err) err = AddVar(profileVelocityAxisB);

    // Program this PDO
    if (!err) err = amp.PdoSet(slotNumber, *this);

    return err;
}

// transmit the RxPDO on the network.
const Error* RpdoEcatProfileVelDualAxis::Transmit(int32 profileVelAxisAIn, int32 profileVelAxisBIn)
{
    // update the programmed velocity with the user value.
    profileVelocityAxisA.Write(profileVelAxisAIn);
    profileVelocityAxisB.Write(profileVelAxisBIn);

    // acquire a reference to the network.
    RefObjLocker<Network> net(networkReference);

    // check that the network is available.
    if (!net) return &NodeError::NetworkUnavailable;

    // tranmit the RxPDO on the network.
    return RPDO::Transmit(*net);
}


static void showerr(const Error* err, const char* str)
{
    if (err)
    {
        printf("Error %s: %s\n", str, err->toString());
        exit(1);
    }
}