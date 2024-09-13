/*

TE2_AnalogInputsTpdo.cpp

The following example maps two analog inputs of a TE2 drive 
to a TPDO for easy reading.

*/

// Comment this out to use EtherCAT
//#define USE_CAN

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

// If a namespace has been defined in CML_Settings.h, this
// macros starts using it. 
CML_NAMESPACE_USE();

#define numberOfAxesXe2 2

/* local functions */
static void showerr( const Error *err, const char *str );

// Define a class based on the TPDO base class. A Transmit PDO is 
// one that's transmitted by the CANopen node and received by the 
// master. This PDO will be used to send the analog input states 
// from the drive to the master (CML).
class TpdoAnalogInputs : public TPDO
{

public:

    // Default constructor does nothing
    TpdoAnalogInputs() {}

    // Called once at startup to map the PDO and configure CML to 
    // listen for it.
    const Error* Init(Amp& ampObj, int slot);

    // This function is called when the PDO is received
    virtual void Received(void);

    bool display{ false };

private:

    // These variables are used to map objects to the PDO.
    // Different mapping objects are used for different data sizes.
    Pmap16 analogInputAxisA;
    Pmap16 analogInputAxisB;
};

/**
 * Non-Fixed Transmit PDO handling.
 * Each axis is being configured to send out a PDO message
 * once every SYNC period.  This class handles receiving these
 * PDOs.
 *
 * @param ampObj The axis to map this TxPDO on.
 * @param slotNumber  The slot number used by this TxPDO.
 * @return NULL on success, or an error object on failure.
 */
const Error * TpdoAnalogInputs::Init( Amp &ampObj, int slotNumber)
{
    const Error* err{ NULL };

    // Initialize the transmit PDO
    // err = TPDO::Init(0x280 + slotNumber * 0x100 + ampObj.GetNodeID());

    // Init the variables using the CANopen object ID 
    if (!err) err = analogInputAxisA.Init(0x2200, 0);              //   16 bits
    if (!err) err = analogInputAxisB.Init(0x2200 + 0x800, 0);      //   16 bits
                                                                   // ---------
                                                                   //   32 bits
    // Add the mapped variables to the PDO
    if (!err) err = AddVar(analogInputAxisA);
    if (!err) err = AddVar(analogInputAxisB);

    // Program this PDO in the amp, and enable it
    if (!err) err = ampObj.PdoSet(slotNumber, *this);

   return err;
}

/**
 * This function will be called by the high priority CANopen receive
 * thread when the PDO is received.  
 *
 * By the time this function is called, the data from the two mapped objects will
 * have been parsed from the input message.  It can be accessed by the Pmap objects
 * that we created.
 *
 * Keep in mind that this function is called from the same thread that receives all
 * CANopen messages.  Keep any processing here short and don't try to do any SDO access.
 * Often it's best to simply post a semaphore here and have another thread handle the data.
 *
 */
void TpdoAnalogInputs::Received( void )
{
   if (display) {
       cout << "AnalogInputA: " << (int16)analogInputAxisA.Read() << " AnalogInputB: " << (int16)analogInputAxisB.Read() << endl;
       CML::Thread::sleep(100);
   }
}


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
   WinUdpEcatHardware hw( "192.168.0.52" );
#else
   LinuxEcatHardware hw( "eth0" );
#endif

   auto err = hw.Open(); // maybe not needed
   showerr(err, "Opening ecat hardware");

   // Open the network object
#if defined( USE_CAN )
   CanOpen net;
#else
   EtherCAT net;
#endif
   err = net.Open( hw );
   showerr( err, "Opening network" );

   /* local data */
   int32 canBPS = 1000000;             // CAN network bit rate
   int16 canNodeID = -1;                // CANopen node ID

   // create TxPDO objects
   TpdoAnalogInputs analogInputsTpdo;

   // Initialize the amplifier using default settings
   Amp ampArray[2];
   AmpSettings ampSettings;
   ampSettings.enableOnInit = false;

   printf( "Doing init\n" );
   err = ampArray[0].Init(net, canNodeID, ampSettings);
   showerr( err, "Initting TE2 axis A" );
   err = ampArray[1].InitSubAxis(ampArray[0], 2);
   showerr( err, "Initting TE2 axis B" );

   // pre-op the TE2 drive (only need to do it to axis a)
   err = ampArray[0].PreOpNode();
   showerr(err, "Preopping node");

   // initializing the non-fixed TxPDO
   err = analogInputsTpdo.Init(ampArray[0], 2); // use slot 2 or 3.
   showerr(err, "Initting non-fixed tpdo");

   analogInputsTpdo.display = true;

   // starting node
   err = ampArray[0].StartNode();
   showerr(err, "Starting node");

   // sleep for 2 seconds so that we can read the analog inputs being printed to the console.
   CML::Thread::sleep(2000);

   // turn off printing to console
   analogInputsTpdo.display = false;
   CML::Thread::sleep(100);

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

