/*

EcatDualAxisProgrammedVelMode.cpp

The following is an example of a dual-axis EtherCAT network.
The 2 axes will be jogged in Programmed Velocity Mode.
The programmed velocity parameter will be updated using
RxPDO's. The actual position and velocity data from each
node will be sent using TxPDO's and displayed to the 
console in real-time.

Programmed Velocity Mode is not a mode defined in the DS402.
It is a velocity mode that allows pure V-Loop control. 
Simply update the programmed velocity value to command motion.

To set this mode, set the Desired State (0x24 ; 0x2300) to a 
value of 11. 

*/

#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#include "CML.h"

//#define USE_CAN
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

#define numberOfAxes     2

// Position/Velocity transmit PDO.  This class is used
// to receive position and velocity updates that are sent 
// automatically by an amplifier every SYNC 
// period.
class TpdoEcatActVelActPosDualAxis : public TPDO
{
    uint32 maskForThisThread;
public:
    Pmap32 actualPositionAxisA; // 32 bits
    Pmap32 actualVelocityAxisA; // 32 bits
    Pmap32 actualPositionAxisB; // 32 bits
    Pmap32 actualVelocityAxisB; // 32 bits

    bool displayTpdoInfo{ false };

    /// Default constructor for this PDO
    TpdoEcatActVelActPosDualAxis() {}

    // initialize the TxPDO
    const Error* Init(Amp& amp, int slot, int inputMask);
    
    // virtual function signifying reception of TxPDO
    virtual void Received(void);
};

// Programmed Velocity RxPDO.  This class is used
// to update the programmed velocity of the amp.
class RpdoEcatProgrammedVelocityDualAxis : public RPDO
{
    uint32 networkReference;
    Pmap32 programmedVelocityAxisA;
    Pmap32 programmedVelocityAxisB;

public:
    // default constructor
    RpdoEcatProgrammedVelocityDualAxis() {}

    // initialize the RxPDO
    const Error* Init(Amp& amp, uint16 slot);
    
    // transmit the RxPDO
    const Error* Transmit(int32 programmedVelocityAxisA, int32 programmedVelocityAxisB);
};

/* local data */
int32 canBPS = 1000000;             // CAN network bit rate
EventMap eventDataReceived;
Amp amp[numberOfAxes];
TpdoEcatActVelActPosDualAxis tpdo;
RpdoEcatProgrammedVelocityDualAxis rpdo;

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
    WinUdpEcatHardware hw("192.168.0.205");
    EtherCAT net;
    int nodeId = -1;
#else
    LinuxEcatHardware hw("eth0");
    EtherCAT net;
    int node = -1;
#endif

    const Error* err = net.Open(hw);
    showerr(err, "Opening CANopen network");

    // I'll use some custom amplifier settings, in particular
    // I'll use a SYNC period of 10ms
    AmpSettings settings;
    settings.synchPeriod = 10000;
    settings.guardTime = 0;

    // Initializing the first axis
    err = amp[0].Init(net, nodeId, settings);
    showerr(err, "Initting axis A");

    // Initializing the second axis
    err = amp[1].InitSubAxis(amp[0], 2);
    showerr(err, "Initing axis b");

    err = amp[0].PreOpNode();
    showerr(err, "Preopping node");

    // display the tpdo info to the console.
    tpdo.displayTpdoInfo = true;

    err = tpdo.Init(amp[0], 2, 1);
    showerr(err, "Initting tpdo");

    err = rpdo.Init(amp[0], 2);
    showerr(err, "Initting rpdo");

    err = amp[0].StartNode();
    showerr(err, "Starting node");

    for (int k = 0; k < numberOfAxes; k++) 
    {
        amp[k].SetAmpMode(CML::AMPMODE_PROG_VEL);
    }

    // Create an event which will wait for all data
    // to be received.
    EventAll event = 1;

    // In the main loop, I simply wait on an event which will occur when all my 
    // amplifiers have sent a position and velocity data update.  Once the data has been received,
    // I will send the next set of programmed velocities.
    for (int i = 0; i < 100; i++)
    {
        // Clear the event mask.  The drive will set this to 1 when it updates CML with TPDO data.
        eventDataReceived.setMask(0);

        // Wait on this with a 2 second timeout.
        err = event.Wait(eventDataReceived, 2000);
        if (err) showerr(err, "Waiting on events");

        // Calculate the new commanded velocities and 
        // transmit them to the amplifiers
        err = rpdo.Transmit(3000 + (i * 100), 3000 + (i * 100));
        showerr(err, "Sending PDO");
    }

    // Clear the event mask
    eventDataReceived.setMask(0);

    // Wait on this with a 2 second timeout.
    err = event.Wait(eventDataReceived, 2000);
    if (err) showerr(err, "Waiting on events");

    CML::Thread::sleep(1000); // wait for 1000 milliseconds

    // end the move by setting the programmed velocities to zero.
    err = rpdo.Transmit(0, 0);
    showerr(err, "Sending PDO");

    // Clear the event mask
    eventDataReceived.setMask(0);

    // Wait on this with a 2 second timeout.
    err = event.Wait(eventDataReceived, 2000);
    if (err) showerr(err, "Waiting on events");

    tpdo.displayTpdoInfo = false;

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
 * @param inputMask     The mask indicating whether the TPDO data has been updated. This value is OR-ed with the EventMap by the TPDO thread when fresh data is received.
 * @return NULL on success, or an error object on failure.
 */
const Error* TpdoEcatActVelActPosDualAxis::Init(Amp& amp, int slotNumber, int inputMask)
{
    maskForThisThread = inputMask;

    // Initialize the transmit PDO
    const Error* err = TPDO::Init(0x280 + slotNumber * 0x100 + amp.GetNodeID());

#ifdef USE_CAN
    // Set transmit type to transmit every 10th SYNC pulse
    if (!err) err = SetType(10);
#endif

    // Init the position variable using the CANopen object ID 
    if (!err) err = actualPositionAxisA.Init(OBJID_POS_LOAD, 0);
    if (!err) err = actualVelocityAxisA.Init(OBJID_VEL_ACT, 0);
    if (!err) err = actualPositionAxisB.Init(OBJID_POS_LOAD + 0x800, 0);
    if (!err) err = actualVelocityAxisB.Init(OBJID_VEL_ACT + 0x800, 0);

    // Add the mapped variable
    if (!err) err = AddVar(actualPositionAxisA);
    if (!err) err = AddVar(actualVelocityAxisA);
    if (!err) err = AddVar(actualPositionAxisB);
    if (!err) err = AddVar(actualVelocityAxisB);

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
void TpdoEcatActVelActPosDualAxis::Received(void)
{
    eventDataReceived.setBits(maskForThisThread);
    if (displayTpdoInfo) {
        printf("TPDO: posA %9d  velA: %9d  posB %9d  velB: %9d\n", actualPositionAxisA.Read(), actualVelocityAxisA.Read(), actualPositionAxisB.Read(), actualVelocityAxisB.Read());
    }
}

const Error* RpdoEcatProgrammedVelocityDualAxis::Init(Amp& amp, uint16 slotNumber)
{
    networkReference = amp.GetNetworkRef();

    // Init the base class
    const Error* err = RPDO::Init(0x200 + slotNumber * 0x100 + amp.GetNodeID()); //0x200+slot*0x100+amp.GetNodeID() );

    // Init the mapping objects that describe the data mapped to this PDO
    if (!err) err = programmedVelocityAxisA.Init(OBJID_PROG_VEL);
    if (!err) err = programmedVelocityAxisB.Init(OBJID_PROG_VEL + 0x800);

    // Add these variables to the PDO
    if (!err) err = AddVar(programmedVelocityAxisA);
    if (!err) err = AddVar(programmedVelocityAxisB);

    // Set the PDO type so that it's data will be acted on immediately
#ifdef USE_CAN
    if (!err) err = SetType(255);
#endif

    // Program this PDO
    if (!err) err = amp.PdoSet(slotNumber, *this);

    return err;
}

// transmit the RxPDO on the network.
const Error* RpdoEcatProgrammedVelocityDualAxis::Transmit(int32 programmedVelocityValueAxisA, int32 programmedVelocityValueAxisB)
{
    // update the programmed velocity with the user value.
    programmedVelocityAxisA.Write(programmedVelocityValueAxisA);
    programmedVelocityAxisB.Write(programmedVelocityValueAxisB);

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
