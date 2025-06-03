/*

TwoAxisExample.cpp

Simple example of a two axis system using a linkage object.

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

/* local defines */
#define AMPCT 2

/* local data */
int32 canBPS = 1000000;             // CAN network bit rate
const char *canDevice = "CAN0";           // Identifies the CAN device, if necessary
int16 canNodeID = 1;                // CANopen node ID of first amp.  Second will be ID+1, etc.

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
   cml.SetDebugLevel( LOG_DEBUG );

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

   // Initialize the amplifiers using default settings
   Amp amp[AMPCT];
   AmpSettings set;
   set.guardTime = 0;
   printf( "Doing init\n" );
   int i;
   for( i=0; i<AMPCT; i++ )
   {
      printf( "Initing %d\n", canNodeID+i );
      err = amp[i].Init( net, canNodeID+i, set );
      showerr( err, "Initting amp" );

      MtrInfo mtrInfo;
      err = amp[i].GetMtrInfo( mtrInfo );
      showerr( err, "Getting motor info\n" );

      err = amp[i].SetCountsPerUnit( mtrInfo.ctsPerRev );
      showerr( err, "Setting cpr\n" );
   }

   // Create a linkage object holding these amps
   Linkage link;
   err = link.Init( AMPCT, amp );
   showerr( err, "Linkage init" );

   // Home the amps
   HomeConfig hcfg;
   hcfg.method  = CHM_NONE;
   hcfg.offset  = 0;

   for( i=0; i<AMPCT; i++ )
   {
      // Home the amp.  Notice that the linkage object may be used 
      // like an array to reference the amplifiers.
      err = link[i].GoHome( hcfg );
      showerr( err, "Going home" );
   }

   // Wait for all amplifiers to finish the home by waiting on the
   // linkage object itself.
   printf( "Waiting for home to finish...\n" );
   err = link.WaitMoveDone( 20000 ); 
   showerr( err, "waiting on home" );

   // Setup the velocity, acceleration, deceleration & jerk limits
   // for multi-axis moves using the linkage object
   err = link.SetMoveLimits( 10, 10, 10, 100 );
   showerr( err, "setting move limits" );

   // Do a bunch of random moves.
   for( int j=0; j<25; j++ )
   {
      // Create an N dimensional position to move to
      Point<AMPCT> pos;

      printf( "%d: moving to ", j );
      for( i=0; i<AMPCT; i++ )
      {
	 pos[i] = (rand() % 100000) / 2000.0;

         //if( i ) pos[i] = pos[0];

	 printf( "%.3lf ", pos[i] );
      }
      printf( "\n" );

      // This function will cause the linkage object to create a 
      // multi-axis S-curve move to the new position.  This 
      // trajectory will be passed down to the amplifiers using
      // the PVT trajectory mode
      err = link.MoveTo( pos );
      showerr( err, "Moving linkage" );

      // Wait for the move to finish
      err = link.WaitMoveDone( 1000 * 30 );
      showerr( err, "waiting on linkage done" );
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

