/*

ProfileVelocityMode.cpp

*/

// Comment this out to use EtherCAT
#define USE_CAN

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
// to receive position and velocity updates
class TpdoActVelActPos : public TPDO
{
public:
    Pmap32 actualPos; // 32 bits
    Pmap32 actualVel; // 32 bits

    /// Default constructor for this PDO
    TpdoActVelActPos() {}

    // initialize the TxPDO
    const Error* Init(Amp& amp, int slot);

    // virtual function signifying reception of TxPDO
    virtual void Received(void);
};

// Profile Velocity RxPDO.  This class is used
// to update the profile velocity of the amp.
class RpdoProfileVel : public RPDO
{
    uint32 networkReference;
    Pmap16 controlWord;
    Pmap32 profileVelocity;

public:
    // default constructor
    RpdoProfileVel() {}

    // initialize the RxPDO
    const Error* Init(Amp& amp, uint16 slot);

    // transmit the RxPDO
    const Error* Transmit(int16 controlWordIn, int32 profileVelocityIn);
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
    int16 canNodeID = 1;                // CANopen node ID

    // Create an object used to access the low level CAN network.
    // This examples assumes that we're using the Copley PCI CAN card.
#if defined( USE_CAN )
    CopleyCAN hw("CAN0");
    hw.SetBaud(canBPS);
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

    const Error* err = net.Open(hw);
    showerr(err, "Opening network");

    TpdoActVelActPos tpdo[AxisCount];
    RpdoProfileVel rpdo[AxisCount];

    // Initialize the amplifier using default settings
    Amp amp[AxisCount];

    for (int i = 0; i < AxisCount; i++) 
    {
        printf("Doing init\n");
        err = amp[i].Init(net, canNodeID + i);
        showerr(err, "Initting amp");

        err = amp[i].PreOpNode();
        showerr(err, "Preopping node");

        err = tpdo[i].Init(amp[i], 2);
        showerr(err, "Initting tpdo");

        err = rpdo[i].Init(amp[i], 1);
        showerr(err, "Initting rpdo");

        err = amp[i].StartNode();
        showerr(err, "Starting node");

        err = amp[i].sdo.Dnld8(0x6060, 0, (int8)CML::AMPMODE_CAN_VELOCITY);
        showerr(err, "setting mode of operation to profile velocity mode (mode 3) on Axis A");
    }

    for (int i = 0; i < AxisCount; i++)
    {
        amp[i].SetProfileAcc(1000); // units are 10 counts/sec^2
        showerr(err, "setting accel");
        amp[i].SetProfileDec(1000); // units are 10 counts/sec^2
        showerr(err, "setting decel");
    }

    for (int i = 0; i < AxisCount; i++)
    {
        // Jog each axis 3000 in units of 0.1 counts/sec 
        err = rpdo[i].Transmit(15, 3000);
        showerr(err, "Sending RPDO");
    }

    // jog for 3 seconds.
    for (int i = 0; i < 30; i++)
    {
        cout << "Act Pos Axis A: " << (double)tpdo[0].actualPos.Read() << endl;
        cout << "Act Pos Axis B: " << (double)tpdo[1].actualPos.Read() << endl;
        cout << "Act Vel Axis A: " << (double)tpdo[0].actualVel.Read() << endl;
        cout << "Act Vel Axis B: " << (double)tpdo[1].actualVel.Read() << endl;
        CML::Thread::sleep(100); // sleep for 100 milliseconds
    }

    // Stop jogging. Set the target velocity to 0.
    for (int i = 0; i < AxisCount; i++)
    {
        // Jog each axis 3000 in units of 0.1 counts/sec 
        err = rpdo[i].Transmit(15, 0);
        showerr(err, "Sending RPDO");
    }

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
const Error* TpdoActVelActPos::Init(Amp& amp, int slotNumber)
{
    // Initialize the transmit PDO
    const Error* err = TPDO::Init(0x180 + (slotNumber * 0x100) + amp.GetNodeID());

    // Set transmit type to transmit on events
    if (!err) err = SetType(10);

    // Init the position variable using the CANopen object ID 
    if (!err) err = actualPos.Init(OBJID_POS_LOAD, 0);
    if (!err) err = actualVel.Init(OBJID_VEL_ACT, 0);

    // Add the mapped variable
    if (!err) err = AddVar(actualPos);
    if (!err) err = AddVar(actualVel);

    // Program this PDO in the amp, and enable it
    if (!err) err = amp.PdoSet(slotNumber, *this);

    return err;
}

/**
 * This function will be called by the high priority CANopen receive
 * thread when the TPDO is received.
 */
void TpdoActVelActPos::Received(void)
{
    // nothing to do at this time
}

/// <summary>
/// Initialize the RPDO using the amp object and the slot (PDO) number
/// </summary>
/// <param name="amp">Amplifier object reference</param>
/// <param name="slotNumber">PDO number to use</param>
/// <returns>CML error code</returns>
const Error* RpdoProfileVel::Init(Amp& amp, uint16 slotNumber)
{
    networkReference = amp.GetNetworkRef();

    // Initialize the transmit PDO
    const Error* err = RPDO::Init(0x200 + (slotNumber * 0x100) + amp.GetNodeID());

    // Set transmit type to transmit on events
    if (!err) err = SetType(255);

    // Init the mapping objects that describe the data mapped to this PDO
    if (!err) err = controlWord.Init(OBJID_CONTROL);
    if (!err) err = profileVelocity.Init(OBJID_TARGET_VEL);

    // Add these variables to the PDO
    if (!err) err = AddVar(controlWord);
    if (!err) err = AddVar(profileVelocity);

    // Program this PDO
    if (!err) err = amp.PdoSet(slotNumber, *this);

    return err;
}

// transmit the RxPDO on the network.
const Error* RpdoProfileVel::Transmit(int16 controlWordIn, int32 profileVelIn)
{
    // update the programmed velocity with the user value.
    controlWord.Write(controlWordIn);
    profileVelocity.Write(profileVelIn);

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