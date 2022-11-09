/** \file

Simple minded example of reading/storing .cci file
Copley I/O Modules are available for EtherCAT or CANopen networks.

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
//int16 etherCatNodeID = -1;        // EtherCAT node ID
int16 canNodeID = 1;                // CANopen node ID

/**************************************************
* Read I/O from .cci file.
* Store to amp.
**************************************************/
int main( void )
{
    // The libraries define one global object of type
    // CopleyMotionLibraries named cml.
    //
    // This object has a couple handy member functions
    // including this one which enables the generation of
    // a log file for debugging
    cml.SetDebugLevel(LOG_DEBUG);

    // Create an object used to access the low level CAN network.
    // This examples assumes that we're using the Copley PCI CAN card.
#if defined( USE_CAN )
    CopleyCAN hw("CAN0");
    hw.SetBaud(canBPS);
#elif defined( WIN32 )
    WinUdpEcatHardware hw("192.168.0.96");
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
   CopleyIO ioModule;

   int line;

   err = ioModule.Init(net, canNodeID );
   showerr( err, "Initializing I/O module\n" );

   printf("IO Module Initialized.\n");

   CopleyIOInfo ioInfoObj;
   err = ioModule.GetIOInfo(ioInfoObj);
   showerr(err, "Retrieving IO Info object");

   CopleyIODigi ioDigiObj;
   err = ioModule.GetIODigi(ioDigiObj);
   showerr(err, "Retrieving IO Digi object");

   CopleyIOAnlg ioAnalog;
   err = ioModule.GetIOAnlg(ioAnalog);
   showerr(err, "Retrieving IO Analog object");

   CopleyIOPWM ioPWMObj;
   err = ioModule.GetIOPWM(ioPWMObj, ioInfoObj);
   showerr(err, "Retrieving IO PWM object");

   err = ioModule.SetIOAnlg(ioAnalog);
   showerr(err, "setting io analog");

   //insert load from file here
   err = ioModule.LoadFromFile( "IOFileExample.cci", line );
   showerr( err, "Loading from file\n" );

   err = ioModule.SaveIOConfig( );
   showerr( err, "Saving I/O config\n" );

   return 0;
}

/**************************************************/

static void showerr( const Error *err, const char *str )
{
   if( err )
   {
      printf( "Error %s: %s\n", str, err->toString() );
      getchar();
      exit(1);
   }
}

