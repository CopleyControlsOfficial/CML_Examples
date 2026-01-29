/*

ProfilePositionSpecialVelocityMode.cpp

The following is an example of a 3 node network.
The 3 nodes will be jogged in Profile Position Special 
Velocity Mode.

The profile velocity parameter will be updated using
RxPDO's. The actual position and velocity data from each
node will be sent using TxPDO's and displayed to the 
console in real-time.

To save bandwidth, the commanded profile velocities will
be paired in groups of two and will share one RxPDO.
If there is an odd number of nodes on the network, the 
last node will have an RxPDO of its own (not shared). 

If an RxPDO has data that is intended for two different
nodes, then the shared RxPDO needs to use the same CAN
message ID. 

For example, we will use the Control Word mapped to 
one RxPDO to start and stop the velocity moves for all 
axes at the same time (determinism). The Control Word is 
only mapped to one RxPDO, so all the nodes share this PDO. 
Therefore, all the nodes are configured to recognize the 
same CAN message ID for this PDO.

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

#define numberOfAxes     3

// Position/Velocity transmit PDO.  This class is used
// to receive position and velocity updates that are sent 
// automatically by an amplifier every SYNC 
// period.
class TpdoActVelActPos : public TPDO
{
    uint32 globalMask;
public:
    Pmap32 actualPosition; // 32-bits
    Pmap32 actualVelocity; // 32-bits

    bool displayTpdoInfo{ false };

    /// Default constructor for this PDO
    TpdoActVelActPos() {}

    // initialize the TxPDO
    const Error* Init(Amp& amp, int slot, int index);

    // virtual function signifying reception of TxPDO
    virtual void Received(void);
};

// Profile Velocity RxPDO.  This class is used
// to update the profile velocity of the amp.
class RpdoProfileVelocity : public RPDO
{
    uint32 networkReference; // used to transmit the RxPDO on the network
    Pmap32 profileVelocity;  // used to update the Profile Velocity parameter
    Pmap32 unusedRegister;   // used as filler data to be ignored in the PDO

public:
    // default constructor
    RpdoProfileVelocity() {}

    // initialize the RxPDO
    const Error* Init(Amp& amp1, int canId, int slotNumber, bool isFirstDriveInPair);
    
    // transmit the RxPDO
    const Error* Transmit(int32 profileVelIn, int32 profileVelIn2);
};

// Control Word RxPDO.  This class is used
// to update the control word of all axes on the network.
class RpdoControlWord : public RPDO
{
    uint32 networkReference;
    Pmap16 controlWord;
    Pmap32 unusedRegister;

public:
    // default constructor
    RpdoControlWord() {}

    // initialize the RxPDO
    const Error* Init(Amp* amp, int slotNumber);

    // transmit the RxPDO
    const Error* Transmit(int16 controlWordIn);
};

/* local data */
int32 canBPS = 1000000;             // CAN network bit rate
EventMap eventDataReceived;
Amp amp[numberOfAxes];
TpdoActVelActPos tpdoArray[numberOfAxes];
RpdoProfileVelocity rpdoProfileVelArray[numberOfAxes];
RpdoControlWord rpdoControlWord;

int main(void)
{
    // The libraries define one global object of type
    // CopleyMotionLibraries named cml.
    //
    // This object has a couple handy member functions
    // including this one which enables the generation of
    // a log file for debugging
    cml.SetDebugLevel( LOG_EVERYTHING );
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

    // Set guard time to 0 to disable node guarding
    AmpSettings settings;
    settings.guardTime = 0;

    for (int i = 0; i < numberOfAxes; i++)
    {
        err = amp[i].Init(net, node * (i + 1), settings);
        showerr(err, "Initting amp");

        err = amp[i].PreOpNode();
        showerr(err, "Preopping node");

        // display the tpdo info to the console.
        tpdoArray[i].displayTpdoInfo = true;

        err = tpdoArray[i].Init(amp[i], 2, 1 << i);
        showerr(err, "Initting tpdo");

        int canIdLocal = 0;
        bool isFirstDriveInPair;
        if (i % 2 != 0) 
        {
            isFirstDriveInPair = false;
            canIdLocal = amp[i - 1].GetNodeID();
        }
        else
        {
            isFirstDriveInPair = true;
            canIdLocal = amp[i].GetNodeID();
        }

        // use slot 2
        err = rpdoProfileVelArray[i].Init(amp[i], canIdLocal, 2, isFirstDriveInPair);
        showerr(err, "Initting profile velocity rpdo");
    }

    // use slot 3
    err = rpdoControlWord.Init(amp, 3);
    showerr(err, "Initting control word rpdo");

    // starting all nodes
    for (int i = 0; i < numberOfAxes; i++)
    {
        err = amp[i].StartNode();
        showerr(err, "Starting node");
    }

    int16 specialVelMode = -1;
    for (int k = 0; k < numberOfAxes; k++) 
    {
        // 1 is for the positive direction
        amp[k].SetTargetPos(1);
        amp[k].SetProfileVel(5000);
        amp[k].SetProfileAcc(50000);
        amp[k].SetProfileDec(50000);
        amp[k].SetProfileJerk(500000);
        amp[k].SetAmpMode(AMPMODE_CAN_PROFILE);
        amp[k].sdo.Dnld16(OBJID_PROFILE_TYPE, 0, specialVelMode);
    }

    // In the main loop, I simply wait on an event which will occur when all my 
    // amplifiers have sent a position and velocity data update.  Once the data has been received,
    // I will send out new commanded profile velocities.
    for (int i = 0; i < 100; i++)
    {
        // Clear the event mask.  Each amplifier will
        // set it's own bit when it receives a data update.
        eventDataReceived.setMask(0);

        // Create an event which will wait for all data
        // to be received.
        EventAll event = (1 << numberOfAxes) - 1; // so if it were 2 axis = 1 left shifted twice = 100 (binary). Subtract 1 to get 011 which is what we want (a bit mask for both axis).

        // Wait on this with a 2 second timeout.
        err = event.Wait(eventDataReceived, 2000);
        if (err) showerr(err, "Waiting on events");

        int commandedVel = i * 50;

        // transmit this new profile velocity to each node on the network
        for (int j = 0; j < numberOfAxes; j++) 
        {
            if ((j % 2) == 0)
            {
                err = rpdoProfileVelArray[j].Transmit(commandedVel, commandedVel);
                showerr(err, "Sending PDO");
            }
        }

        // toggle bit 4 of the control word to update the velocity
        rpdoControlWord.Transmit(0x002F);
        rpdoControlWord.Transmit(0x003F);
    }

    // end the move by sending a commanded velocity of zero.
    for (int j = 0; j < numberOfAxes; j++)
    {
        if ((j % 2) == 0)
        {
            err = rpdoProfileVelArray[j].Transmit(0, 0);
            showerr(err, "Sending PDO");
        }
    }

    // toggle bit 4 of the control word to update the velocity
    rpdoControlWord.Transmit(0x002F);
    rpdoControlWord.Transmit(0x003F);

    // stop displaying the TxPDO info to the console
    for (int j = 0; j < numberOfAxes; j++) 
    {
        tpdoArray[j].displayTpdoInfo = false;
    }

    printf("Finished. Press any key to quit.\n");
    return 0;
}

/**************************************************/

/**
 * Transmit PDO handling.
 * Each amplifier is being configured to send out a CAN message
 * once every SYNC period.  This class handles receiving these
 * PDOs.  When the new position data is received, this class will
 * indicate it in an EventMap object that is shared by all amps.
 * This allows a thread to be triggered when all positions and velocities have
 * been received.
 *
 * @param amp   The amplifier to map this PDO.
 * @param slot  The slot to use for this PDO.
 * @param m     The mask used to differentiate between axes.
 *              For example, axis A is 001, axis B is 010, axis C is 100, etc.
 * @return NULL on success, or an error object on failure.
 */
const Error* TpdoActVelActPos::Init(Amp& amp, int slotNumber, int inputMask)
{
    globalMask = inputMask;

    // Initialize the transmit PDO
    uint32 canMessageId = 0x180 + (slotNumber * 0x100) + amp.GetNodeID();
    const Error* err = TPDO::Init(canMessageId);

#ifdef USE_CAN
    // Set transmit type to transmit every 10th SYNC pulse
    if (!err) err = SetType(10);
#endif

    // Init the position variable using the CANopen object ID 
    if (!err) err = actualPosition.Init(OBJID_POS_LOAD, 0);
    if (!err) err = actualVelocity.Init(OBJID_VEL_ACT, 0);

    // Add the mapped variable
    if (!err) err = AddVar(actualPosition);
    if (!err) err = AddVar(actualVelocity);

    // Program this PDO in the amp, and enable it
    if (!err) err = amp.PdoSet(slotNumber, *this);

    return err;
}

/**
 * This function will be called by the high priority CANopen receive
 * thread when the PDO is received.
 *
 * All I do here is set a bit in the global event map.  This bit indicates
 * that the data has been received.
 */
void TpdoActVelActPos::Received(void)
{
    eventDataReceived.setBits(globalMask);
    if (displayTpdoInfo) {
        printf("TPDO: pos %9d  vel: %9d\n", actualPosition.Read(), actualVelocity.Read());
    }
}

/// <summary>
/// Method to initialize the Profile Velocity RPDO containing commanded profile velocity data for two axes.
/// Make sure that the PDO uses the same CAN message ID for both axes.
/// </summary>
/// <param name="amp">one of the axes to initialize in the two-axis PDO pair</param>
/// <param name="slotNumber">the slot number of the PDO to use. CML uses slots 0 and 1. Slots 2 and 3 are available per drive.</param>
/// <param name="isFirstDriveInPair">is this the first drive in the PDO mapping pair?</param>
/// <returns>an error code indicating an error or success</returns>
const Error* RpdoProfileVelocity::Init(Amp& amp, int canId, int slotNumber, bool isFirstDriveInPair)
{
    networkReference = amp.GetNetworkRef();

    // Init the base class
    uint32 canMessageId = 0x200 + slotNumber * 0x100 + canId;
    const Error* err = RPDO::Init(canMessageId);
            
    // Init the mapping objects that describe the data mapped to this PDO
    if (!err) err = profileVelocity.Init(OBJID_PROFILE_VEL);
    if (!err) err = unusedRegister.Init(0x0004);

    if (isFirstDriveInPair) 
    {
        // Add these variables to the PDO
        if (!err) err = AddVar(profileVelocity);
        if (!err) err = AddVar(unusedRegister);
    }
    else 
    {
        // Add these variables to the PDO
        if (!err) err = AddVar(unusedRegister);
        if (!err) err = AddVar(profileVelocity);
    }

    // Set the PDO type so that it's data will be acted on immediately
    if (!err) err = SetType(255);

    // Program this PDO
    if (!err) err = amp.PdoSet(slotNumber, *this);

    return err;
}

/// <summary>
/// Method to transmit the two profile velocities on the network. Both drives use the same RPDO.
/// </summary>
/// <param name="profileVelIn1">the commanded profile velocity of the first node</param>
/// <param name="profileVelIn2">the commanded profile velocity of the second node</param>
/// <returns>an error code indicating whether or not the PDO transmission was successful</returns>
const Error* RpdoProfileVelocity::Transmit(int32 profileVelIn1, int32 profileVelIn2)
{
    profileVelocity.Write(profileVelIn1);
    unusedRegister.Write(profileVelIn2);

    // acquire a reference to the network.
    RefObjLocker<Network> net(networkReference);

    // check that the network is available.
    if (!net) return &NodeError::NetworkUnavailable;

    // tranmit the RxPDO on the network.
    return RPDO::Transmit(*net);
}

/// <summary>
/// Method to initialize the Control Word PDO shared by all axes on the network.
/// </summary>
/// <param name="amp">the array of all axes on the network</param>
/// <param name="slotNumber">the slot number of the PDO to use. CML uses slots 0 and 1. Slots 2 and 3 are available per drive.</param>
/// <returns>an error code indicating an error or success</returns>
const Error* RpdoControlWord::Init(Amp* amp, int slotNumber)
{
    networkReference = amp[0].GetNetworkRef();

    // Init the base class
    uint32 canMessageId = 0x200 + slotNumber * 0x100 + amp[0].GetNodeID();
    const Error* err = RPDO::Init(canMessageId);

    // Init the mapping objects that describe the data mapped to this PDO
    if (!err) err = controlWord.Init(OBJID_CONTROL);

    // Add these variables to the PDO
    if (!err) err = AddVar(controlWord);

    // Set the PDO type so that it's data will be acted on immediately
    if (!err) err = SetType(255);

    for (int i = 0; i < numberOfAxes; i++) 
    {
        // Program this PDO
        if (!err) err = amp[i].PdoSet(slotNumber, *this);
    }

    return err;
}

// transmit the RxPDO on the network.
const Error* RpdoControlWord::Transmit(int16 controlWordIn)
{
    controlWord.Write(controlWordIn);

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

