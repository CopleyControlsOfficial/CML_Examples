/*

BeckhoffAsicChipPdo16.cpp

Drive a non-CoE EtherCAT I/O module through PDOs, so the cycle thread
carries its process data at the network cycle rate.  No NodeRead/NodeWrite.

Hard-coded for the 2-byte-in / 2-byte-out variant:

    SM2  0x0f02  ctrl 0x44  output   2 byte(s)
    SM3  0x1000  ctrl 0x00  input    2 byte(s)

One 16-bit object each way, because the process image is one 16-bit word
each way.  The TOTAL MAPPED WIDTH is what matters: once PDOs are mapped,
their combined width is what CML configures the SyncManager to, and it has
to match what the device presents.  Two 8-bit objects in one PDO would do
just as well; two 8-bit objects in two PDOs at the same slot would NOT -
the second replaces the first and you end up with one byte mapped against
a two-byte image.

Requires CML 2.20.03 or later.

Build (Linux):
    g++ -O1 -g -I inc -I inc/can -I inc/ecat -I inc/pugixml \
        -o BeckhoffAsicChipPdo16 BeckhoffAsicChipPdo16.cpp \
        libMotionLib.a -lpthread -lrt -ldl

*/

#include <cstdio>
#include <cstdlib>
#include <iostream>

#include "CML.h"

using std::cout;
using std::endl;

#if defined( WIN32 )
#include "ecat/ecat_winudp.h"
#else
#include "ecat/ecat_linux.h"
#endif

CML_NAMESPACE_USE();

static void showerr( const Error *err, const char *str );
static void report( EtherCAT &net, Node &n, int sm );

/* Set these two for your machine. */
int16       etherCatNodeID = 128;             /* EtherCAT node alias     */
const char *networkAddress = "192.168.0.33";  /* or the NIC name on Linux */

class BeckhoffAsicChip: public Node {};

int main( void )
{
   cml.SetDebugLevel( LOG_EVERYTHING );

#if defined( WIN32 )
   WinUdpEcatHardware hw( networkAddress );
#else
   LinuxEcatHardware hw( "eth0" );
#endif

   EtherCAT net;
   EtherCatSettings ecatSettings;
   ecatSettings.cyclePeriod = 20;

   const Error *err = net.Open( hw, ecatSettings );
   showerr( err, "opening the network" );

   BeckhoffAsicChip ioModule;

   /* Node::Init, not EtherCAT::AttachNode.  Init grabs the network
      reference first, and the sync manager configuration needs it. */
   err = ioModule.Init( net, etherCatNodeID );
   showerr( err, "initializing the I/O module" );

   cout << "Node alias: " << ioModule.GetNodeID() << endl;

   /*-----------------------------------------------------------------------
     What this module presents, before anything is mapped.

     Printed rather than assumed, so the program says which variant it is
     talking to.  If these are not 2 and 2, the mapping below is wrong for
     this module and the widths have to change to match.
    -----------------------------------------------------------------------*/
   report( net, ioModule, 2 );
   report( net, ioModule, 3 );

   /*-----------------------------------------------------------------------
     The outputs: one 16-bit word in SM2.
    -----------------------------------------------------------------------*/
   Pmap16 outputs;
   RPDO   rpdo;

   err = outputs.Init( 0x7000, 1 );
   showerr( err, "binding the output word" );

   /* The module has no CoE, so there is no PDO assignment in an object
      dictionary for CML to upload and compare against.  This tells it to
      take the mapping as given rather than verifying it. */
   rpdo.SetVerifyFixedPdoMapping( false );

   err = rpdo.AddVar( outputs );
   showerr( err, "adding the output word to the receive PDO" );

   err = ioModule.PdoSet( 0, rpdo, true );
   showerr( err, "mapping the receive PDO" );

   /*-----------------------------------------------------------------------
     The inputs: one 16-bit word in SM3.

     Transmit slot 0, which is NOT the same slot space as receive slot 0 -
     a PDO's dictionary index is based on its direction.
    -----------------------------------------------------------------------*/
   Pmap16 inputs;
   TPDO   tpdo;

   err = inputs.Init( 0x6000, 1 );
   showerr( err, "binding the input word" );

   tpdo.SetVerifyFixedPdoMapping( false );

   err = tpdo.AddVar( inputs );
   showerr( err, "adding the input word to the transmit PDO" );

   err = ioModule.PdoSet( 0, tpdo, true );
   showerr( err, "mapping the transmit PDO" );

   /*-----------------------------------------------------------------------
     Up to OPERATIONAL.

     THIS IS THE STEP A REGISTER-WRITE APPLICATION SKIPS, and it is why
     the register writes fail: the process data SyncManagers are INACTIVE
     below safe-op.  Until the node is started there is nothing at those
     addresses to read or write, by any mechanism.
    -----------------------------------------------------------------------*/
   err = ioModule.PreOpNode();
   showerr( err, "pre-opping the node" );

   err = ioModule.StartNode();
   showerr( err, "starting the node" );

   /* And the mapped widths, which is what the SyncManagers now carry. */
   report( net, ioModule, 2 );
   report( net, ioModule, 3 );

   /*-----------------------------------------------------------------------
     From here it is just reads and writes of two local variables.  The
     cycle thread carries them; nothing in this loop touches the network
     except the two waits.
    -----------------------------------------------------------------------*/
   for( uint16 bit = 1; bit; bit = (uint16)( bit << 1 ) )
   {
      outputs.Write( bit );

      err = rpdo.Transmit( net );
      showerr( err, "publishing the output word" );

      /* One cycle to carry the outputs out... */
      err = net.WaitCycleUpdate( -1 );
      showerr( err, "waiting one cycle" );

      /* ...and one more to carry the resulting inputs back. */
      err = net.WaitCycleUpdate( -1 );
      showerr( err, "waiting one cycle" );

      printf( "  out 0x%04X -> in 0x%04X\n",
              (unsigned)bit, (unsigned)inputs.Read() );
   }

   /* Leave the outputs clear rather than wherever the loop ended. */
   outputs.Write( 0 );
   err = rpdo.Transmit( net );
   showerr( err, "clearing the outputs" );

   err = net.WaitCycleUpdate( -1 );
   showerr( err, "waiting one cycle" );

   return 0;
}

/**************************************************/

/* What one SyncManager says about itself.  eepromLen is what the device
   declares; cfgLen is what CML has configured, which after mapping is the
   PDOs' total width. */
static void report( EtherCAT &net, Node &n, int sm )
{
   EcatSyncMgrInfo info;

   const Error *err = net.GetSyncMgrInfo( &n, sm, info );

   if( err )
   {
      printf( "  SM%d: could not be read (%s)\n", sm, err->toString() );
      return;
   }

   printf( "  SM%d: 0x%04X ctrl 0x%02X %-7s eeprom %u byte(s), "
           "configured %u byte(s)\n",
           sm, (unsigned)info.ramAddr, (unsigned)info.smCtrl,
           info.isOutput ? "output" : "input",
           (unsigned)info.eepromLen, (unsigned)info.cfgLen );
}

/* Fatal: setup failed, there is nothing to run. */
static void showerr( const Error *err, const char *str )
{
   if( err )
   {
      printf( "Error %s: %s\n", str, err->toString() );
      exit( 1 );
   }
}
