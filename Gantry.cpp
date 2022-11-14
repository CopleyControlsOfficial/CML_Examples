/*
 * Gantry.cpp
 *
 * This is a very simple example showing how to use CML to control two
 * motors that are connected in parallel in a gantry configuration.
 */

#include <CML.h>
#include <can/can_copley.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>


// This define is used below to set the move speed for the example moves in 
// encoder count / sec units.  Set it to something reasonable for your system.
#define MOVE_SPEED      40000

CML_NAMESPACE_USE();

// This is a custom linkage class that allows us to treat the two amplifiers on the gantry as a single
// axes of motion. This prevents the velocity from being scaled by a factor of sqrt(2).
class GantryLinkage : public Linkage
{
public:
   // Default constructor does nothing
   GantryLinkage(){};

   // Here I will override the function that would normally return the number of amplifiers
   // It now returns 1, the number of axes of motion associated with this linkage
   virtual uint16 GetAxesCount( void )
   {
      return 1;
   }

   // This virtual function allows us to implement our own kinematics for converting
   // the axis to amp frames.
   // 
   // The purpose here is to take a single dimension point, and set the position expected 
   // in Linkage::RequestNextTrjPoint() to be the same for each physical axis (ampCt)
   virtual const Error *ConvertAxisToAmp( uunit pos[], uunit vel[] )
   {
      // Set both points in the array equal to the point passed
      pos[1] = pos[0];
      vel[1] = vel[0];
      return 0;
   }
};
/* Local Functions */
static const Error *HomeGantry( GantryLinkage &link );
static void showerr( const Error *err, const char *msg );

int main( int argc, char **argv )
{
   const Error *err;

   // This will cause lots of debug information to be saved to a log file.
   // It's handy for debugging, but not necessary for a production system.
   cml.SetDebugLevel( LOG_EVERYTHING );

   // Open the low level CAN port that will be used to communicate to the
   // amplifiers.
   CopleyCAN can;
   err = can.Open();
   showerr( err, "Opening CAN port" );

   // Create the upper level CANopen object
   CanOpen canOpen;
   err = canOpen.Open( can );
   showerr( err, "Opening CANopen network" );

   // We use two Amp objects to control the two amplifiers in the system.
   Amp amp[2];
   err = amp[0].Init( canOpen, 1 );
   showerr( err, "Initting amp 1" );

   err = amp[1].Init( canOpen, 2 );
   showerr( err, "Initting amp 2" );

   // We use one upper level linkage object to control the two amplifiers 
   // in a coordinated mannor.
   // This uses the overridden Linkage object to allow 2 amps to control one axis of motion. This ensures that the
   // velocties used are what we expect, and not off by a factor of sqrt(2)
   GantryLinkage link;
   err = link.Init( 2, amp );
   showerr( err, "Initting linkage" );

   printf( "Finished initting amps and linkage.\n" );

   // Home the gantry
   err = HomeGantry( link );
   showerr( err, "Homing gantry" );

   // In a gantry type system it's normally a good idea to bring both axes to a halt
   // if either of them stop tracking position well.  The Linkage object can do this
   // automatically if it's configured to do so.  We will configure it to stop any 
   // move in progress on a 'position warning' or 'velocity window' event from either
   // amplifier.
   //
   // Note that for this to work correctly the position warning window and velocity tracking
   // window must be configured properly for the system.  This is best done through the 
   // CME interface.
   LinkSettings linkCfg;
   linkCfg.haltOnPosWarn = true;
   linkCfg.haltOnVelWin = true;
   link.Configure( linkCfg );

   // I'll disable both axes in the case of an error condition.  Other options here are
   // to bring the axes to a halt using some programmed acceleration value.
   link[0].SetHaltMode( HALT_DISABLE );
   link[1].SetHaltMode( HALT_DISABLE );

   // Now, we'll do a bunch of moves of the gantry.  We use the linkage object
   // to coordinate the moves on both axes.

   // First, set some reasonable move constraints
   double velLimit   = MOVE_SPEED;       // velocity (encoder counts / second)
   double accelLimit = MOVE_SPEED*10;    // acceleration (cts/sec/sec)
   double decelLimit = MOVE_SPEED*10;    // deceleration (cts/sec/sec)
   double jerkLimit  = MOVE_SPEED*50;    // jerk (cts/sec/sec/sec)
   err = link.SetMoveLimits( velLimit, accelLimit, decelLimit, jerkLimit );
   showerr( err, "Setting linkage move limits" );

   for( int i=0; i<1000; i++ )
   {
      // Find a random position between 0 and 100,000 encoder counts
      double pos = (double)rand() * 100000.0 / RAND_MAX;
      pos = floor(pos);

      printf( "Moving to position %.1lf\n", pos );

      // Convert to a single point
      Point<1> p;
      p[0] = pos;

      // Start a move to this point
      err = link.MoveTo( p );
      showerr( err, "Starting move" );

      // Wait for the move to finish
      err = link.WaitMoveDone( 10000 );
      showerr( err, "Waiting for move to finish" );
   }

   printf( "Finished\n");
   getchar();
   return 0;
}

// This simple macro just displays an error message and returns the error 
// object if the error was non-zero.
#define reterr( err, msg ) do{ if( err ){ printf( "Error: %s - %s\n", msg, err->toString() ); return err;} } while(0)

/** 
 * This function takes a linkage object which consists of two motors 
 * physically connected together on a parallel gantry configuration.
 * It find the home (zero) location of the gantry using this basic
 * procedure:
 *
 * 1) Disable one axis and home the other one to a home sensor.  The
 *    disabled axis will be dragged along by the first during this 
 *    home move.
 *
 * 2) Hold position on the axis that was homed in step 1.  Move the
 *    other axis in each direction with a limited current and find 
 *    the distance it is able to move.
 *
 * 3) Set the zero position on the second axis to the center point
 *    of it's two limited current moves.  The two axes should now 
 *    be properly aligned.
 */
static const Error *HomeGantry( GantryLinkage &link )
{
   const Error *err;

   // Step 1, disable one motor and home the other to a home 
   // sensor.
   err = link[1].Disable();
   reterr( err, "Disabling amp 1" );

   HomeConfig homeCfg;
   homeCfg.method  = CHM_NHOME;           // Home method is 'home in negative direction to home sensor
   homeCfg.velFast = MOVE_SPEED;          // Home velocity in cts/sec
   homeCfg.velSlow = MOVE_SPEED/10;       // Home velocity in cts/sec
   homeCfg.accel   = MOVE_SPEED*5;        // Home accel in cts/sec/sec

   printf( "Homing first axis\n" );
   err = link[0].GoHome( homeCfg );
   reterr( err, "homing amp 0" );

   // Wait for the home move to finish (max 30 seconds)
   err = link[0].WaitMoveDone( 30000 );
   reterr( err, "Waiting for amp 0 to finish homing" );

   // At this point, the first axis is holding position at it's 
   // zero position.  Now, I'll enable the second axis
   err = link[1].Enable();
   reterr( err, "Enabling axis 1" );

   // Now, I'll use a 'home to hard stop' method to push the other
   // axis in each direction.  This should allow me to align the 
   // two axes in the gantry.
   homeCfg.method = CHM_HARDSTOP_NEG;
   homeCfg.velFast = MOVE_SPEED/10;    // Home move velocity (cts/sec)
   homeCfg.velSlow = MOVE_SPEED/10;    // Home move velocity (cts/sec)
   homeCfg.accel   = MOVE_SPEED;       // Home move acceleration (cts/sec/sec)
   homeCfg.current = 100;              // push with 1.00 Amps of current.
   homeCfg.delay   = 400;              // Push for 400ms.

   printf( "Making first home move\n" );
   // Start the homing move.
   err = link[1].GoHome( homeCfg );
   reterr( err, "Starting first home move on amp 1" );

   // Wait for home move to finish (max of 5 seconds)
   err = link[1].WaitMoveDone( 5000 );
   reterr( err, "Wait for first home to finish" );

   // That last home just pushed one side of the gantry as far as possible
   // (with limited current) in one direction and set that end position as
   // zero.  Now, I'll make a similar move in the other direction.
   printf( "Making second home move\n" );
   homeCfg.method = CHM_HARDSTOP_POS;

   err = link[1].GoHome( homeCfg );
   reterr( err, "Starting second home move on amp 1" );
   err = link[1].WaitMoveDone( 5000 );
   reterr( err, "Wait for second home to finish" );

   // At this point, the second gantry axis was pushed as far as possible
   // in the positive direction (with limited current) and the resulting
   // position was set to zero.  I can read out the amount that the last
   // home caused the position to change, and the result should be the
   // distance between the two far points caused by these homing moves.
   double adjust;
   err = link[1].GetHomeAdjustment( adjust );
   reterr( err, "Getting home adjustment amount" );
   printf( "Total home adjustment: %0.1lf\n", adjust );

   // I'll adjust the actual position on the second axis by 1/2
   // of this amount, and move back to zero.  This should align
   // the two axes of the gantry.
   double pos;
   err = link[1].GetPositionActual( pos );
   reterr( err, "Getting position on amp 1" );

   err = link[1].SetPositionActual( pos+adjust/2 );
   reterr( err, "Setting adjusted position on amp 1" );

   // Move back to the adjusted zero position.
   ProfileConfigScurve moveCfg;
   moveCfg.pos = 0;               // Move destination position (encoder counts)
   moveCfg.vel = MOVE_SPEED;      // Move velocity (cts/sec)
   moveCfg.acc = MOVE_SPEED*10;   // Move accel (cts/sec/sec)
   moveCfg.jrk = MOVE_SPEED*50;   // Move accel (cts/sec/sec/sec)
   err = link[1].DoMove( moveCfg );
   reterr( err, "Starting move to zero on amp 1" );
   err = link[1].WaitMoveDone( 5000 );
   reterr( err, "Wait for move to zero to finish" );

   return 0;
}

// Just display the error (if there is one) and exit.
static void showerr( const Error *err, const char *msg )
{
   if( !err ) return;
   printf( "Error: %s - %s\n", msg, err->toString() );
   exit(1);
}

