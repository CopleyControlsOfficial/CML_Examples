/*

ProgrammedVelMode.cpp

The following is an example of a 3 node network.
The 3 nodes will be jogged in Programmed Velocity Mode.
The programmed velocity parameter will be updated using
RxPDO's. The actual position and velocity data from each
node will be sent using TxPDO's and displayed to the 
console in real-time.

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

// Programmed Velocity RxPDO.  This class is used
// to update the programmed velocity of the amp.
class RpdoProgrammedVelocity : public RPDO
{
    uint32 networkReference;
    Pmap32 programmedVelocity;

public:
    // default constructor
    RpdoProgrammedVelocity() {}

    // initialize the RxPDO
    const Error* Init(Amp& amp, uint16 slot);
    
    // transmit the RxPDO
    const Error* Transmit(int32 programmedVelocity);
};


/* local data */
int32 canBPS = 1000000;             // CAN network bit rate
EventMap eventDataReceived;
Amp amp[numberOfAxes];
TpdoActVelActPos tpdo[numberOfAxes];
RpdoProgrammedVelocity rpdo[numberOfAxes];

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

    // I'll use some custom amplifier settings, in particular
    // I'll use a SYNC period of 5ms rather then the default 10ms
    AmpSettings settings;
    settings.synchPeriod = 10000;
    settings.guardTime = 0;

    //printf("Doing init\n");
    int i;
    for (i = 0; i < numberOfAxes; i++)
    {
        err = amp[i].Init(net, node * (i + 1), settings);
        showerr(err, "Initting amp");

        err = amp[i].PreOpNode();
        showerr(err, "Preopping node");

        // display the tpdo info to the console.
        tpdo[i].displayTpdoInfo = true;

        err = tpdo[i].Init(amp[i], 2, 1 << i);
        showerr(err, "Initting tpdo");

        err = rpdo[i].Init(amp[i], 2);
        showerr(err, "Initting rpdo");

        err = amp[i].StartNode();
        showerr(err, "Starting node");
    }

    for (int k = 0; k < numberOfAxes; k++) {
        amp[k].SetAmpMode(CML::AMPMODE_PROG_VEL);
    }

    // In the main loop, I simply wait on an event which will occur when all my 
    // amplifiers have sent a position and velocity data update.  Once the data has been received,
    // I will use them to calculate the next set of programmed velocities, which I will
    // send out.
    for (i = 0; i < 100; i++)
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

        // Calculate the new commanded velocities and 
        // transmit them to the amplifiers
        for (int j = 0; j < numberOfAxes; j++)
        {
            err = rpdo[j].Transmit(33389);
            showerr(err, "Sending PDO");
        }
    }

    // end the move by sending a commanded velocity of zero.
    for (int j = 0; j < numberOfAxes; j++)
    {
        err = rpdo[j].Transmit(0);
        showerr(err, "Sending PDO");
    }

    // stop displaying the TxPDO info to the console
    for (int j = 0; j < numberOfAxes; j++) {
        tpdo[j].displayTpdoInfo = false;
    }

    printf("Finished. Press any key to quit.\n");
    getchar();
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
    const Error* err = TPDO::Init(0x280 + slotNumber * 0x100 + amp.GetNodeID());

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

const Error* RpdoProgrammedVelocity::Init(Amp& amp, uint16 slotNumber)
{
    networkReference = amp.GetNetworkRef();

    // Init the base class
    const Error* err = RPDO::Init(0x200 + slotNumber * 0x100 + amp.GetNodeID()); //0x200+slot*0x100+amp.GetNodeID() );

    // Init the mapping objects that describe the data mapped to this PDO
    if (!err) err = programmedVelocity.Init(OBJID_PROG_VEL);

    // Add these variables to the PDO
    if (!err) err = AddVar(programmedVelocity);

    // Set the PDO type so that it's data will be acted on immediately
#ifdef USE_CAN
    if (!err) err = SetType(255);
#endif

    // Program this PDO
    if (!err) err = amp.PdoSet(slotNumber, *this);

    return err;
}

// transmit the RxPDO on the network.
const Error* RpdoProgrammedVelocity::Transmit(int32 programmedVelocityValue)
{
    // update the programmed velocity with the user value.
    programmedVelocity.Write(programmedVelocityValue);

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