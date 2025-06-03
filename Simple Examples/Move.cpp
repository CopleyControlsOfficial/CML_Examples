/*

Move.cpp

Simple minded example of homing & moving a motor.

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
static void showerr( const Error *err, const char *str );

/* local data */
int32 canBPS = 1000000;             // CAN network bit rate
int16 canNodeID = 1;                // CANopen node ID

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
   WinUdpEcatHardware hw( "eth0" );
#else
   LinuxEcatHardware hw( "eth0" );
#endif

   // Open the network object
#if defined( USE_CAN )
   CanOpen net;
#else
   EtherCAT net;
#endif
   const Error *err = net.Open( hw );
   showerr( err, "Opening network" );

   // Initialize the amplifier using default settings
   Amp amp;
   printf( "Doing init\n" );
   err = amp.Init( net, canNodeID );
   showerr( err, "Initting amp" );

   printf( "Hit return to home\n" );
   getchar();

   // Home the motor.
   HomeConfig hcfg;
   hcfg.method  = CHM_NDX_POS;
   hcfg.velFast = 100000;
   hcfg.velSlow = 50000;
   hcfg.accel   = 90000;
   hcfg.offset  = 0;

   err = amp.GoHome( hcfg );
   showerr( err, "Going home" );

   printf( "Waiting for home to finish...\n" );
   err = amp.WaitMoveDone( 20000 ); 
   showerr( err, "waiting on home" );

   printf( "Hit return to start moves\n" );
   getchar();

   // Setup the move speeds.  For simplicity I'm just using the same
   // vel, acc & decel for all moves.
   printf( "Setting up moves...\n" );

   ProfileConfigTrap trap;
   ProfileConfigScurve scurve;

   trap.vel = 800000;
   trap.acc = 50000;
   trap.dec = 50000;

   scurve.vel = 800000;
   scurve.acc = 50000;
   scurve.jrk = 8000;

   // Do a bunch of moves to random locaitons.
   for( int i=0; i<50; i++ )
   {
      int x = rand(); x %= 100000;

      printf( "Moving to %d ", x );

      if( i & 1 )
      {
         printf( "using trap profile\n" );
         trap.pos = x;
         err = amp.DoMove( trap );
      }
      else
      {
         printf( "using s-curve profile\n" );
         scurve.pos = x;
         err = amp.DoMove( scurve );
      }

      showerr( err, "doing move" );

      err = amp.WaitMoveDone( 30000 ); 
      showerr( err, "waiting on move" );
   }
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

