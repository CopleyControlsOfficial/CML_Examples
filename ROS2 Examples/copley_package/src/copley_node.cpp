#include <functional>
#include <memory>
#include <cstdio>
#include <cstdlib>
#include "CML.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Comment this out to use EtherCAT
#define USE_CAN

#if defined( USE_CAN )
#include "can_copley.h"
#elif defined( WIN32 )
#include "ecat_winudp.h"
#else
#include "ecat_linux.h"
#endif

// If a namespace has been defined in CML_Settings.h, this
// macros starts using it. 
CML_NAMESPACE_USE();

/* local functions */
static void showerr( const Error *err, const char *str );

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world copley_package package\n");
  
  int16 canNodeID = 1;      // CAN node ID of this servo drive
  int32 canBPS = 1000000;   // CAN network bit rate
  
  // Create an object used to access the low level CAN network.
  // This examples assumes that we're using one of the Copley CAN cards.
#if defined( USE_CAN )
  CopleyCAN hw( "CAN0" );
  hw.SetBaud( canBPS );
#elif defined( WIN32 )
  WinUdpEcatHardware hw( "eth0" );
#else
  LinuxEcatHardware hw( "eth0" );
#endif
  
#if defined( USE_CAN )
  CanOpen net;
#else
  EtherCAT net;
#endif
  
  // Open the network object
  const Error* err = net.Open( hw );
  showerr( err, "Opening network" );
  
  // Initialize the amplifier using default settings
  Amp amp;
  err = amp.Init( net, canNodeID );
  showerr( err, "Initting amp" );
  
  return 0;
}

static void showerr( const Error *err, const char *str )
{
   if( err )
   {
      printf( "Error %s: %s\n", str, err->toString() );
      exit(1);
   }
}
