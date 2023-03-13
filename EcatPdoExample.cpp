/** \file

EcatPdoExample.cpp

The following example demonstrates how to map PDO's on a two-axis
Copley Controls EtherCAT drive (XE2). The PDO's are user-mappable,
meaning that their contents are configured by the user. 

Receive Process Data Object (RxPDO): The drive receives commands
from the master (CML). 

Transmit Process Data Object (TxPDO): The drive transmits data 
to the master (CML).

In CML, the user inherits from the TPDO or RPDO base classes to 
define their custom TPDO or RPDO mapping. Each EtherCAT PDO can 
store up to 256 bits of data. There are 4 TPDO's and 4 RPDO's
supported in the EtherCAT protocol. Therefore, a total mapping of
4*256 = 1024 bits is available for RPDO's and 1024 bits is available
for TPDO's. However, CML is using the first 2 TPDO's and the first 
RPDO under the hood, so the user is really left with 2*256 = 512
bits available for TPDO data and 3*256 = 768 bits available for 
RPDO data over the EtherCAT network. 

err = tpdo.Init(amp, 2); // use slot 2 or 3. (slots 0 and 1 are 
being used by CML, so they are unavailable)

err = rpdo.Init(amp, 1); // use slot 1, 2, or 3. (slot 0 is being 
used by CML, so it is unavailable)

The EtherCAT PDO mapping capacity is an improvement over the CANopen 
PDO mapping capacity. The CANopen protocol supports 8 TPDO's and 8 
RPDO's, but each PDO holds 64 bits of data. CML uses the first 2 
TPDO's and the first RPDO, so the user is left with 6*64 = 384 bits
of TPDO data and 7*64 = 448 bits of RPDO data over the CANopen 
network.

EtherCAT PDO's behave differently than CANopen PDO's. CANopen PDO's
can be transmitted synchronously or asynchronously (event-driven). 
EtherCAT PDO's are always synchronous. They will transmit constantly
after the node has been initialized on the network. The rate at 
which EtherCAT PDO's transmit is based on the Sync 0 period. To 
configure the Sync 0 period in CML, use net.SetSync0Period().

Note: amp.PreOpNode() must be called before PDO.Init(). After 
PDO.Init() has been called, amp.StartNode() must be called to begin 
PDO transmission.

Note: DO NOT MAP the same object to two different RPDO's. The RPDO's
will overwrite each other, resulting in a race condition (undefined
behavior). For example, do not map the digital output states to two
RPDO's.

*/

// Comment this out to use EtherCAT
//#define USE_CAN

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

#define numberOfAxesXe2 2

/* local functions */
static void showerr( const Error *err, const char *str );

/* local data */
int32 canBPS = 1000000;             // CAN network bit rate
int16 canNodeID = -1;                // CANopen node ID

// Define a class based on the TPDO base class.
// A Transmit PDO is one that's transmitted by the
// CANopen node and received by the master.
// This PDO will be used to send the motor position,
// actual current, and digital input states from the
// drive to the master.
class NonFixedTpdoEventStatusAndOutputsXe2 : public TPDO
{

public:

    // Default constructor does nothing
    NonFixedTpdoEventStatusAndOutputsXe2() {}

    // Called once at startup to map the PDO and configure CML to 
    // listen for it.
    const Error* Init(Amp& ampObj, int slot);

    // This function is called when the PDO is received
    virtual void Received(void);

    bool display{ false };

private:

    // These variables are used to map objects to the PDO.
    // Different mapping objects are used for different data sizes.
    Pmap32 eventStatusAxisA;
    Pmap32 eventStatusAxisB;
    Pmap16 digitalOutputs;
};

/**
 * Non-Fixed Transmit PDO handling.
 * Each axis is being configured to send out a PDO message
 * once every SYNC period.  This class handles receiving these
 * PDOs.
 * 
 * Each non-fixed PDO holds a max of 32 bytes (256 bits) of data.
 *
 * @param ampObj The axis to map this TxPDO on.
 * @param slotNumber  The slot number used by this TxPDO.
 * @return NULL on success, or an error object on failure.
 */
const Error * NonFixedTpdoEventStatusAndOutputsXe2::Init( Amp &ampObj, int slotNumber)
{
    const Error* err{ NULL };

    // Initialize the transmit PDO
    // err = TPDO::Init(0x280 + slotNumber * 0x100 + ampObj.GetNodeID());

    // Init the variables using the CANopen object ID 
    if (!err) err = eventStatusAxisA.Init(0x2185, 0);              //   32 bits
    if (!err) err = eventStatusAxisB.Init(0x2185 + 0x800, 0);      //   32 bits
    if (!err) err = digitalOutputs.Init(OBJID_OUTPUTS);            // + 16 bits
                                                                   // ---------
                                                                   //   80 bits
    // Add the mapped variables to the PDO
    if (!err) err = AddVar(eventStatusAxisA);
    if (!err) err = AddVar(eventStatusAxisB);
    if (!err) err = AddVar(digitalOutputs);

    // Program this PDO in the amp, and enable it
    if (!err) err = ampObj.PdoSet(slotNumber, *this);

   return err;
}

/**
 * This function will be called by the high priority EtherCAT receive
 * thread when the PDO is received.  
 *
 * By the time this function is called, the data from the two mapped objects will
 * have been parsed from the input message.  It can be accessed by the Pmap objects
 * that we created.
 *
 * Keep in mind that this function is called from the same thread that receives all
 * EtherCAT messages.  Keep any processing here short and don't try to do any SDO access.
 * Often it's best to simply post a semaphore here and have another thread handle the data.
 *
 */
void NonFixedTpdoEventStatusAndOutputsXe2::Received( void )
{
   if (display) {
       printf("evntStatA: 0x%04x evntStatB: 0x%04x DOUT: 0x%04x \n", eventStatusAxisA.Read(), eventStatusAxisB.Read(), digitalOutputs.Read());
       CML::Thread::sleep(1000);
   }
}

// This represents the non-fixed receive PDO to map the profile position.
class NonFixedRpdoProfilePosition : public RPDO
{
    uint32 netRef;

public:
    Pmap32 profilePosA;
    Pmap32 profilePosB;

    /// Default constructor for this PDO
    NonFixedRpdoProfilePosition() { SetRefName("RPDO_ProfilePosition"); }
    virtual ~NonFixedRpdoProfilePosition() { KillRef(); }
    const Error* Init(Node& node, uint16 slot = 1)
    {
        netRef = node.GetNetworkRef();
        const Error* err = 0;

        // Let the various mapped variables know which 
        // objects in the amp's object dictionary they
        // are linked to.
        if (!err) err = profilePosB.Init(OBJID_PROFILE_POS, 0);
        if (!err) err = profilePosB.Init(OBJID_PROFILE_POS + 0x800, 0);

        // Add the mapped variables
        if (!err) err = AddVar(profilePosA);
        if (!err) err = AddVar(profilePosB);

        // Program this PDO in the amp, and enable it
        if (!err) err = node.PdoSet(slot, *this);

        return err;
    }

    const Error* Send(int32 commandedPositionA, int32 commandedPositionB)
    {
        profilePosA.Write(commandedPositionA);
        profilePosA.Write(commandedPositionB);

        RefObjLocker<Network> net(netRef);
        if (!net) return &NodeError::NetworkUnavailable;

        return Transmit(*net);
    }
};


/**************************************************
* Just home the motor and do a bunch of random
* moves.
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

   // Create an object used to access the low level CAN network.
   // This examples assumes that we're using the Copley PCI CAN card.
#if defined( USE_CAN )
   CopleyCAN hw( "CAN0" );
   hw.SetBaud( canBPS );
#elif defined( WIN32 )
   WinUdpEcatHardware hw( "192.168.0.98" );
#else
   LinuxEcatHardware hw( "eth0" );
#endif

   auto err = hw.Open(); // maybe not needed
   showerr(err, "Opening ecat hardware");

   CML::cml.SetLogFile("cml.log");
   CML::cml.SetDebugLevel(CML::LOG_EVERYTHING);

   // Open the network object
#if defined( USE_CAN )
   CanOpen net;
#else
   EtherCAT net;
#endif
   err = net.Open( hw );
   showerr( err, "Opening network" );

   // create TxPDO objects
   NonFixedTpdoEventStatusAndOutputsXe2 nonFixedTpdo;
   NonFixedRpdoProfilePosition nonFixedRxPDO;

   // Initialize the amplifier using default settings
   Amp ampArray[2];
   AmpSettings ampSettings;
   ampSettings.synchPeriod = 2000;

   printf( "Doing init\n" );
   err = ampArray[0].Init(net, canNodeID, ampSettings);
   showerr( err, "Initting XE2 axis A" );
   err = ampArray[1].InitSubAxis(ampArray[0], 2);
   showerr(err, "Initting XE2 axis B");

   // pre-op the XE2 drive (only need to do it to axis a)
   err = ampArray[0].PreOpNode();
   showerr(err, "Preopping node");

   // initializing the non-fixed TxPDO
   err = nonFixedTpdo.Init(ampArray[0], 2); // use slot 2 or 3. (slots 0 and 1 are being used by CML, so they are unavailable)
   showerr(err, "Initting non-fixed tpdo");

   nonFixedTpdo.display = true;

   err = nonFixedRxPDO.Init(ampArray[0], 1); // use slot 1, 2, or 3. (slot 0 is being used by CML, so it is unavailable)
   showerr(err, "Initting non-fixed rpdo");

   printf("Setting heartbeat\n");
   err = ampArray[0].StartHeartbeat(100, 0);
   showerr(err, "Setting heartbeat");

   printf("Setting SYNC0\n");
   err = net.SetSync0Period(&ampArray[0], 1000000); // set SYNC0 pulse to trigger every 1ms
   showerr(err, "Setting SYNC0 period");

   // starting node
   err = ampArray[0].StartNode();
   showerr(err, "Starting node");

   getchar();

   return 0;
}

/**************************************************/

static void showerr( const Error *err, const char *str )
{
   if( err )
   {
      printf( "Error %s: %s\n", str, err->toString() );
      exit(1);
   }
}

