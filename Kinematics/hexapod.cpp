/** \file

CML example: positioning hexapod sequence, PVT or CSP

A six-strut positioning hexapod with a platform of about 100 mm
(KinStewart), in mm and radians; its joints are the six strut lengths.  It
steps the platform through a short sequence from home: 8 mm each way in x
and y, 5 mm up and down, 4 degrees each way in roll and pitch, 5 degrees
each way in yaw, and back home.

The sequence is a positioning one rather than a motion cue, at 10 mm/s and
about 6 degrees/s: an amplitude and a speed an unloaded bench unit runs
safely.  Raise them only after a PLAN_ONLY run passes with your own limits.

HOW THIS EXAMPLE RUNS
---------------------
With PLAN_ONLY commented out, it brings up the network and one drive axis
per joint, gives each axis the joint's own unit (radians for a revolute
joint, millimeters for a prismatic one) and the joint's own limits, and
enables the axes where they stand.

It then reads where the joints are, plans a jerk-limited joint move from
there to the start pose, and runs it.  After that it runs the cycle until
you press a key.  Every move is planned from the end of the one before it,
checked against the keep-out scene before it is sent, and sent only if it
clears.  The next move is planned while the current one runs, so the robot
does not pause between moves.

The key halts the robot.  The example says whether the halt came in the
middle of a move or between two, where the robot came to rest and how the
run went, then disables the axes.

Each drive's position must read zero where the model's joint is zero, in
the joint unit set by COUNTS_PER_UNIT: home the robot, or set the position
offsets with CME, before the first run.

PVT OR CSP: ONE SWITCH
----------------------
STREAM_CSP, in YOUR MACHINE below, picks how the moves reach the drives.

    defined       cyclic synchronous position (DS402 mode 8).  The linkage
                  resamples each move onto the network cycle and sends every
                  drive one position per cycle.
    commented     PVT.  The drives interpolate the points themselves and the
    out           linkage keeps their buffers filled.

The plans are the same in both modes.  A Cartesian move is planned with
CartPathPlanner::PlanTimedSCurve into a PvtConstAccelTrj, and a joint move
with PlanJointMove into a LinkSampledTrj.  Both are trajectories that
Linkage::SendTrajectory streams in either mode, so this file has one set of
plans rather than a second set built with PlanCspSCurve for CSP.  The switch
changes Linkage::Init and, in PVT, turns on SetPvtStartCheck: every plan
here starts from the commanded position, so a trajectory that does not is a
mistake to refuse.

A halt differs between the two:

    CSP   the linkage ramps every joint to rest at its JOINT_HALT_DEC, all
          arriving at the same instant.
    PVT   each drive stops its own axis at its JOINT_HALT_DEC (halt mode
          HALT_DECEL, set below), so the joints do not stop together.

In both the tool leaves the planned path while it stops.  Neither is an
emergency stop: keep the machine's own safety circuit in place.

KEEP-OUT
--------
BuildGuard, below, describes the cell: the base plate it stands on, from
AddRobotZones, and a fixed probe over the platform.  Every planned move is
checked against it, including the motion between the planned points, and a
move that would come within KEEPOUT_MARGIN of any of it is not sent.  Edit
BuildGuard to match your cell before running anything.  The check is a
planning aid, not a safety-rated component.

CHECK YOUR EDITS FIRST: PLAN_ONLY
---------------------------------
PLAN_ONLY, just below the transport switch, is defined as shipped.  The
program then opens no network and moves nothing.  It plans the whole
program from PLAN_ONLY_SEED, the move to the start pose and then
PLAN_ONLY_CYCLES cycles, and checks every plan against the keep-out scene.
It streams each one through the same objects the real run sends, in the
mode STREAM_CSP selects: in PVT the points are evaluated as the drive
interpolates them, and in CSP they are resampled and checked as the linkage
does every cycle.  It prints each joint's peak velocity, acceleration and
jerk against its limit, how far the streamed tool strays from the planned
path, any CSP setpoint the per-cycle check would hold, and PASS or FAIL,
and exits 0 on a pass.  Run it after every change to YOUR MACHINE or to
BuildGuard, in the mode you will run, and comment PLAN_ONLY out only when
it passes.

RobotPlanCheck (CML_RobotCell.h) does the whole check: it plans each move,
checks it against the keep-out scene, measures how close it comes to each
obstacle and streams it as the drives would receive it.  A program of your
own can run the same check on its moves before it sends them.

WHICH TRANSPORT
---------------
USE_CAN is defined just below.  LEAVE IT IN for a CANopen network; COMMENT
IT OUT for EtherCAT.

    CANopen    one node per joint.  A multi-axis CANopen drive presents
               each axis as its own node, so every joint uses Amp::Init
               with its own node ID.
    EtherCAT   one node per drive.  The joint on axis 1 of a drive uses
               Amp::Init; the joints on its other axes use InitSubAxis on
               that joint's Amp.

EMBEDDED LINUX / PREEMPT_RT
---------------------------
Everything a real-time deployment needs is in the SetUpRealTime() section
below, commented out with the reasoning.  Uncomment what applies to your
system.  Nothing in this example requires it, but a cyclic stream on a
loaded machine drops cycles without it, and the statistics at the end are
how you tell.

BUILD
-----
On Linux, from this folder:

    make hexapod

which builds the CML library first if it needs building.  Or by hand, from
the top of the CML tree:

    g++ -O2 -g -I inc -I inc/can -I inc/ecat \
        -o hexapod examples/hexapod/hexapod.cpp libMotionLib.a -lpthread -lrt -ldl

On Windows, see readme.txt in this folder.

Run it from a terminal, from wherever your CML log should land; it writes
hexapod_cml.log.
*/

/***************************************************************************
 *  TRANSPORT.  Leave this defined for CANopen; comment it out for EtherCAT.
 ***************************************************************************/
#define USE_CAN

/***************************************************************************
 *  PLAN ONLY.  Leave this defined to plan and check the whole program with
 *  no network and no drives; comment it out to run the robot.
 ***************************************************************************/
#define PLAN_ONLY

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

#include "CML.h"
#include "CML_KinStewart.h"
#include "CML_KeepOut.h"
#include "CML_RobotCell.h"

#if defined( USE_CAN )
#  include "can/can_copley.h"        /* or the header for your CAN card */
#else
#  if defined( WIN32 )
#    include "ecat/ecat_winudp.h"
#  else
#    include "ecat/ecat_linux.h"
#  endif
#endif

#include <atomic>
#include <thread>
#include <mutex>

/* The key watcher's platform headers, after CML's own so that the Windows
   socket headers the EtherCAT transport needs come first. */
#if defined( _WIN32 )
#  ifndef WIN32_LEAN_AND_MEAN
#    define WIN32_LEAN_AND_MEAN
#  endif
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#  include <conio.h>
#else
#  include <termios.h>
#  include <unistd.h>
#endif

CML_NAMESPACE_USE();

/***************************************************************************
 *  YOUR MACHINE.  These are the only lines most people need to change.
 ***************************************************************************/

/* How the moves are streamed.  Leave this defined for cyclic synchronous
   position (mode 8); comment it out for PVT. */
#define STREAM_CSP

/* The robot's joint count and the coordinates in its Cartesian pose. */
#define JOINTS   6
#define POSE_DIM 6

#if defined( USE_CAN )

/* One CANopen node per joint, in joint order. */
static const int16 JOINT_NODE_ID[JOINTS] = { 1, 2, 3, 4, 5, 6 };

/* CAN port and bit rate.  The bit rate is in bits per second. */
static const char *CAN_PORT = "CAN0";
static const int32 CAN_BPS  = 1000000;

/* SYNC period, in MICROSECONDS.  In CSP this is the cyclic rate.  10000us
   = 100 Hz is a safe start for this many axes on 1 Mbit; come down from
   there once the bus load is known. */
static const uint32 SYNC_PERIOD_US = 10000;

#else

/* Each joint's drive and the axis it is on.  A drive is named by its
   position on the network: -1 is the first node, -2 the second, and so on.
   The joint on axis 1 of a drive must be listed before the joints on its
   other axes. */
static const int16 JOINT_DRIVE[JOINTS] = { -1, -1, -2, -2, -3, -3 };
static const int   JOINT_AXIS [JOINTS] = {  1,  2,  1,  2,  1,  2 };

/* The NIC.  On Windows this is the IP the UDP transport binds to; on Linux
   it is the interface name. */
#  if defined( WIN32 )
static const char *ECAT_WIN_IP    = "192.168.0.33";
#  else
static const char *ECAT_INTERFACE = "eth0";
#  endif

/* Cycle period in MILLISECONDS.  5 ms holds on a desktop Windows timer;
   a real-time system can go much lower. */
static const int ECAT_CYCLE_MS = 5;

#endif

/* Encoder counts per joint unit.  This makes every axis's user unit the
   joint's kinematic unit: radians for a revolute joint, millimeters for a
   prismatic one.  For a revolute joint it is counts per motor turn times
   the gear ratio, divided by 2*pi. */
static const double COUNTS_PER_UNIT[JOINTS] = { 10000, 10000, 10000, 10000, 10000, 10000 };

/* Each joint's travel, in its own unit.  The planner refuses any plan that
   needs a joint outside it. */
static const double JOINT_MIN[JOINTS] = {  95,  95,  95,  95,  95,  95 };
static const double JOINT_MAX[JOINTS] = { 119, 119, 119, 119, 119, 119 };

/* How far a strut's ball joints may tilt from their axes, radians. */
static const double JOINT_TILT_MAX = 40 * M_PI / 180;

/* Each joint's velocity, acceleration and jerk limit, per second, and the
   deceleration a halt uses.  Every plan stays inside the first three, and
   the linkage checks every setpoint against the velocity limit. */
static const double JOINT_VEL     [JOINTS] = {   15,   15,   15,   15,   15,   15 };
static const double JOINT_ACC     [JOINTS] = {  100,  100,  100,  100,  100,  100 };
static const double JOINT_JERK    [JOINTS] = { 1000, 1000, 1000, 1000, 1000, 1000 };
static const double JOINT_HALT_DEC[JOINTS] = {  150,  150,  150,  150,  150,  150 };

/* The platform's speed and acceleration along a Cartesian move, mm, and
   how fast it may turn, radians, per second.  The joint limits above slow
   a move further wherever they bind. */
static const double CART_VEL     = 10.0;        /* mm/s     */
static const double CART_ACC     = 50.0;        /* mm/s^2   */
static const double CART_ROT_VEL =  0.1;        /* rad/s    */
static const double CART_ROT_ACC =  0.5;        /* rad/s^2  */

/* The spacing of the planned points, in milliseconds: the PVT point time,
   and the spacing the CSP stream resamples from. */
static const int PVT_POINT_MS = 4;

/* The hexapod, mm: the base and platform anchor circles, how far either
   side of its pair's center each anchor sits, radians, the platform
   center's height at home, and the strut radius below which two struts
   count as touching.  READY_JOINTS is unused by the solution (a hexapod
   has one), and is home. */
static const double BASE_RADIUS  = 60.0;
static const double BASE_SPREAD  = 10 * M_PI / 180;
static const double PLAT_RADIUS  = 40.0;
static const double PLAT_SPREAD  = 10 * M_PI / 180;
static const double HOME_Z       = 100.0;
static const double STRUT_RADIUS =   4.0;
static const double READY_JOINTS[JOINTS] = { 107.345, 107.345, 107.345, 107.345, 107.345, 107.345 };

/* A sphere round the platform center that holds the platform and its
   payload, and the probe over it: its radius and its lowest point. */
static const double PAYLOAD_RADIUS = 45.0;
static const double PROBE_RADIUS   =  5.0;
static const double PROBE_TIP      = 170.0;

/* The base plate: its half-width and its thickness. */
static const double MOUNT_HALF      = 150.0;
static const double MOUNT_THICKNESS =  20.0;

/* The sequence's steps: mm across and up, degrees of tilt and of turn. */
static const double STEP_MM  = 8.0;
static const double LIFT_MM  = 5.0;
static const double TILT_DEG = 4.0;
static const double TURN_DEG = 5.0;

/* How close any part of the robot may come to anything in BuildGuard's
   scene, in mm. */
static const double KEEPOUT_MARGIN = 2.0;

/* PLAN_ONLY: where the plan starts, in joint units, and how many cycles
   it plans after the move to the start pose. */
static const double PLAN_ONLY_SEED[JOINTS] = { 104, 106, 108, 110, 108, 106 };
static const int    PLAN_ONLY_CYCLES = 2;


/***************************************************************************
 *  Moves
 ***************************************************************************/

typedef Point<JOINTS>   Joints;
typedef Point<POSE_DIM> Pose;

/* How every move is planned: the spacing of its points, and the tool's
   speed and acceleration along a Cartesian move.  The joint limits are the
   arm's own, set in SetUpArm. */
static const RobotMoveLimits MOVE_LIMITS =
   { PVT_POINT_MS / 1000.0, CART_VEL, CART_ACC, CART_ROT_VEL, CART_ROT_ACC };

static void ToJoints( const double q[], Joints &p )
{
   p.setDim( JOINTS );
   for( int i=0; i<JOINTS; i++ ) p[i] = q[i];
}

/* What AddRobotZones added, in the robot's length unit times 'scale' mm. */
static void ShowZones( const KeepOutRobotZones &z, bool selfCheck, double scale )
{
   const char *lead = "  zones :";

   if( z.mount )
   {
      printf( "%s a mount %s z = %.1f mm, %.0f mm square\n", lead,
              ( z.mountSide > 0 ) ? "below" : "above", z.mountZ * scale,
              2 * z.mountHalfSize * scale );
      lead = "         ";
   }
   if( z.column )
   {
      printf( "%s a column of radius %.1f mm from z = %.1f to %.1f mm\n", lead,
              z.columnRadius * scale, z.columnBottom * scale, z.columnTop * scale );
      lead = "         ";
   }
   printf( "%s self-collision checking %s\n\n", lead,
           selfCheck ? "ON" : "off: the helper does not enable it for this robot" );
}

/* argv[0], for finding files that ship beside the program. */
static const char *programPath = "";

/* Fatal: there is nothing useful to do after a setup failure, and carrying
   on with a half-configured linkage is how a test run damages hardware. */
static void showerr( const Error *err, const char *what )
{
   if( err )
   {
      printf( "ERROR %s: %s\n", what, err->toString() );
      exit( 1 );
   }
}


/***************************************************************************
 *  The platform, the cell and the sequence
 ***************************************************************************/
static KinStewart   arm;
static KeepOutGuard guard;

/* Lengths are in mm. */
static const double MM_PER_UNIT = 1.0;

static const char *JOINT_NAME[JOINTS]   = { "S1", "S2", "S3", "S4", "S5", "S6" };
static const char *JOINT_UNIT[JOINTS]   = { "mm", "mm", "mm", "mm", "mm", "mm" };
static const char *POSE_NAME[POSE_DIM]  = { "x", "y", "z", "yaw", "pitch", "roll" };
static const bool  POSE_ANGLE[POSE_DIM] = { false, false, false, true, true, true };

/* The anchors in three pairs, 120 degrees apart: a base pair BASE_SPREAD
   either side of 0, 120 and 240 degrees, and a platform pair between two
   base pairs, so each strut leans across to the next pair. */
static bool SetUpGeometry( void )
{
   StewartGeom g;
   StewartDefaultGeom( g );

   for( int c=0; c<3; c++ )
   {
      const double at = 2*M_PI*c/3;
      const double b[2] = { at - BASE_SPREAD, at + BASE_SPREAD };
      const double p[2] = { at - M_PI/3 + PLAT_SPREAD, at + M_PI/3 - PLAT_SPREAD };

      for( int s=0; s<2; s++ )
      {
         g.B[2*c+s][0] = BASE_RADIUS * cos( b[s] );
         g.B[2*c+s][1] = BASE_RADIUS * sin( b[s] );
         g.B[2*c+s][2] = 0;
         g.P[2*c+s][0] = PLAT_RADIUS * cos( p[s] );
         g.P[2*c+s][1] = PLAT_RADIUS * sin( p[s] );
         g.P[2*c+s][2] = 0;
      }
   }

   g.legMin      = JOINT_MIN[0];
   g.legMax      = JOINT_MAX[0];
   g.tiltMax     = JOINT_TILT_MAX;
   g.strutRadius = STRUT_RADIUS;
   arm.SetGeom( g );

   /* Forward kinematics is solved from a seed it keeps between calls, so
      this example uses the model from the main thread only. */
   Pose home;
   home.setDim( POSE_DIM );
   for( int i=0; i<POSE_DIM; i++ ) home[i] = 0;
   home[2] = HOME_Z;
   return !arm.SetSeedPose( home );
}

/* The body parts as KeepOutGuard numbers them: six struts, then the
   platform with its payload. */
static const char *BodyPartName( int part )
{
   static const char *names[] = { "strut 1", "strut 2", "strut 3", "strut 4",
                                  "strut 5", "strut 6", "platform" };
   return ( ( part >= 0 ) && ( part < 7 ) ) ? names[part] : "hexapod";
}

enum { OBST_PROBE = 1, OBST_MOUNT = 100, OBST_COLUMN = 101 };

static const char *ObstacleName( int id )
{
   switch( id )
   {
      case OBST_PROBE:  return "probe";
      case OBST_MOUNT:  return "base plate";
      case OBST_COLUMN: return "column";
   }
   return "obstacle";
}

/***************************************************************************
 *  KEEP-OUT
 *
 *  The hexapod is modeled as a capsule along each strut (STRUT_RADIUS)
 *  and a sphere round the platform center (PAYLOAD_RADIUS) that holds the
 *  platform and what is mounted on it.
 *
 *  AddRobotZones adds the plate the hexapod stands on, under its lowest
 *  base anchors, with the struts that stand on it exempt.  Self-collision
 *  checking is not supported for a hexapod; the struts are kept apart by
 *  the inverse kinematics' own interference check (STRUT_RADIUS).  The
 *  cell adds a fixed probe over the platform, PROBE_TIP its lowest point,
 *  that the payload must stay clear of.
 *
 *  Sweeping is on: between two planned points the guard subdivides until
 *  no part moves more than SWEEP_STEP.
 ***************************************************************************/
static const double SWEEP_STEP = 0.5;      /* mm */

static bool BuildGuard( void )
{
   Joints ref;
   ToJoints( READY_JOINTS, ref );

   guard.SetMargin( KEEPOUT_MARGIN );
   if( !guard.SetAllLinkRadii( STRUT_RADIUS ) || !guard.SetToolRadius( PAYLOAD_RADIUS )
    || !guard.SetMaxSweepStep( SWEEP_STEP ) )
      return false;

   KeepOutSceneAddCapsule( guard.Scene(), KeepOutV3( 0, 0, PROBE_TIP + PROBE_RADIUS ),
                           KeepOutV3( 0, 0, PROBE_TIP + 200 ), PROBE_RADIUS, OBST_PROBE );

   KeepOutRobotZones z;
   if( guard.DefaultRobotZones( arm, ref, z ) != KEEPOUT_OK ) return false;
   z.mountHalfSize  = MOUNT_HALF;
   z.mountThickness = MOUNT_THICKNESS;

   if( guard.AddRobotZones( arm, ref, z, OBST_MOUNT ) != KEEPOUT_OK )
   {
      printf( "ERROR: the base plate does not fit the hexapod at READY_JOINTS.\n" );
      return false;
   }
   ShowZones( z, guard.SelfCheck(), MM_PER_UNIT );
   return true;
}

/* The sequence: each row is a pose relative to home, mm and degrees, x y z
   then roll, pitch and yaw about the platform's own axes. */
static const double STEP[][6] =
{
   {  STEP_MM, 0, 0,  0, 0, 0 },
   { -STEP_MM, 0, 0,  0, 0, 0 },
   {  0,  STEP_MM, 0, 0, 0, 0 },
   {  0, -STEP_MM, 0, 0, 0, 0 },
   {  0, 0,  LIFT_MM, 0, 0, 0 },
   {  0, 0, -LIFT_MM, 0, 0, 0 },
   {  0, 0, 0,  TILT_DEG, 0, 0 },
   {  0, 0, 0, -TILT_DEG, 0, 0 },
   {  0, 0, 0,  0,  TILT_DEG, 0 },
   {  0, 0, 0,  0, -TILT_DEG, 0 },
   {  0, 0, 0,  0, 0,  TURN_DEG },
   {  0, 0, 0,  0, 0, -TURN_DEG },
   {  0, 0, 0,  0, 0, 0 }
};

static const char *STEP_NAME[] =
{
   "to +x", "to -x", "to +y", "to -y", "up", "down", "roll +", "roll -",
   "pitch +", "pitch -", "yaw +", "yaw -", "home"
};

static const int CYCLE_MOVES = (int)( sizeof( STEP ) / sizeof( STEP[0] ) );

static void StepPose( int s, double p[] )
{
   const double r = M_PI / 180.0;
   p[0] = STEP[s][0];
   p[1] = STEP[s][1];
   p[2] = HOME_Z + STEP[s][2];
   p[3] = STEP[s][5] * r;      /* yaw   */
   p[4] = STEP[s][4] * r;      /* pitch */
   p[5] = STEP[s][3] * r;      /* roll  */
}

static void CycleStartPose( double p[] )
{
   StepPose( CYCLE_MOVES - 1, p );
}

/* This example has no deliberately refused move. */
static bool RefusalSpec( const double from[], RobotMoveSpec &m )
{
   (void)from;
   (void)m;
   return false;
}

static void CycleMove( long k, const double from[], RobotMoveSpec &m )
{
   (void)from;

   const int s = (int)( k % CYCLE_MOVES );
   double a[POSE_DIM], b[POSE_DIM];
   StepPose( ( s + CYCLE_MOVES - 1 ) % CYCLE_MOVES, a );
   StepPose( s, b );
   m.Line( STEP_NAME[s], a, b );
}

/* The program: a joint move to the cycle's first pose, solved on the
   branch READY_JOINTS is on, the move the guard must refuse, then the
   cycle.  The plan-only check and the run both take their moves from it. */
static const RobotProgram PROGRAM =
   { "to the start pose", CycleStartPose, READY_JOINTS, RefusalSpec, CycleMove, CYCLE_MOVES };

/***************************************************************************
 *  Planning
 ***************************************************************************/

static bool SetUpArm( void )
{
   if( !SetUpGeometry() ) return false;

   Joints lo, hi, v, a, j;
   ToJoints( JOINT_MIN, lo );
   ToJoints( JOINT_MAX, hi );
   ToJoints( JOINT_VEL, v );
   ToJoints( JOINT_ACC, a );
   ToJoints( JOINT_JERK, j );

   arm.SetJointLimits( lo, hi );
   arm.SetJointVelLimits( v );
   arm.SetJointAccLimits( a );
   arm.SetJointJerkLimits( j );
   return true;
}

/* The arm in its cell: every move is planned with MOVE_LIMITS and checked
   against the guard, and the reports name things as below. */
static const RobotNames NAMES =
   { JOINT_NAME, JOINT_UNIT, POSE_NAME, POSE_ANGLE, BodyPartName, ObstacleName, MM_PER_UNIT };

static RobotCell cell;

/***************************************************************************
 *  STOPPING ON A KEY
 *
 *  The motion runs until a key is pressed.  A plain thread waits for the
 *  key, so nothing in the motion loop polls the keyboard.  It is a
 *  std::thread rather than a CML Thread on purpose: CML threads run at a
 *  real-time priority on POSIX, and a thread that waits on a person has no
 *  business there.
 *
 *  Ctrl+C counts as a key.  Killing the program would end the stream with
 *  no coordinated halt, so it halts instead.
 *
 *  On Linux the terminal is switched to single-key mode (no Enter, no
 *  echo) and switched back on the way out.  With no terminal on stdin, end
 *  of input counts as the key, so the run halts almost at once.
 ***************************************************************************/
static std::atomic<bool> keyPressed( false );
static void ( *onKey )( void ) = 0;

static void KeyPressed( void )
{
   /* The first key only: on Windows a Ctrl+C can arrive alongside it. */
   if( keyPressed.exchange( true ) ) return;
   if( onKey ) onKey();
}

#if defined( _WIN32 )

static BOOL WINAPI CtrlHandler( DWORD type )
{
   if( ( type != CTRL_C_EVENT ) && ( type != CTRL_BREAK_EVENT ) )
      return FALSE;

   KeyPressed();
   return TRUE;
}

static void WatchKeys( void )
{
   _getch();
   KeyPressed();
}

static void StartKeyWatcher( void ( *hook )( void ) )
{
   onKey = hook;
   SetConsoleCtrlHandler( CtrlHandler, TRUE );
   std::thread( WatchKeys ).detach();
}

#else

static struct termios    savedTerm;
static std::atomic<bool> termChanged( false );

static void RestoreTerminal( void )
{
   if( termChanged.exchange( false ) )
      tcsetattr( STDIN_FILENO, TCSANOW, &savedTerm );
}

static void WatchKeys( void )
{
   /* A key, end of input or an error: each one ends the run. */
   unsigned char c;
   ssize_t got = read( STDIN_FILENO, &c, 1 );
   (void)got;

   /* The rest of a multi-byte key, so it does not reach the shell. */
   tcflush( STDIN_FILENO, TCIFLUSH );
   RestoreTerminal();

   KeyPressed();
}

static void StartKeyWatcher( void ( *hook )( void ) )
{
   onKey = hook;

   struct termios t;
   if( isatty( STDIN_FILENO ) && ( tcgetattr( STDIN_FILENO, &t ) == 0 ) )
   {
      savedTerm = t;

      /* ISIG off is what makes Ctrl+C a key rather than a signal. */
      t.c_lflag &= ~( ICANON | ECHO | ISIG );
      t.c_cc[VMIN]  = 1;
      t.c_cc[VTIME] = 0;

      if( tcsetattr( STDIN_FILENO, TCSANOW, &t ) == 0 )
      {
         termChanged = true;
         atexit( RestoreTerminal );
      }
   }
   else
      printf( "stdin is not a terminal: end of input will halt the run.\n" );

   std::thread( WatchKeys ).detach();
}

#endif

/* The halt the key triggers.  The main loop sends every move holding
   moveMtx and marks it running there, and this takes moveMtx too, so the
   key either finds a move running and halts it, or finds none and the loop
   sends no other. */
static std::mutex   moveMtx;
static Linkage     *haltLink   = 0;
static const char  *runningMove = 0;     /* the move streaming now, or NULL */
static const char  *haltedMove  = 0;     /* the move the key halted, or NULL */

static void HaltOnKey( void )
{
   std::lock_guard<std::mutex> lock( moveMtx );

   if( !runningMove )
   {
      printf( "\nKey pressed between moves - no move is sent after this.\n" );
      return;
   }

   haltedMove = runningMove;
   printf( "\nKey pressed during '%s' - halting mid-move.\n", runningMove );

   const Error *err = haltLink->HaltMove();
   if( err ) printf( "WARNING: HaltMove failed: %s\n", err->toString() );
}

/***************************************************************************
 *  REAL-TIME SETUP FOR EMBEDDED LINUX
 *
 *  Every line in this function is commented out.  None of it is needed to
 *  make the example run; all of it matters on a real machine at a real
 *  cycle rate.  Uncomment what applies and leave the rest.
 *
 *  The numbers that tell you whether it worked are the ones the run
 *  statistics print.
 ***************************************************************************/
#if defined( USE_CAN )
static void SetUpRealTime( CanOpenSettings &settings, Linkage &link )
#else
static void SetUpRealTime( EtherCatSettings &settings, Linkage &link )
#endif
{
   /* Silence the unused-parameter warnings while everything below is
      commented out.  Delete these two lines as you uncomment. */
   (void)settings;
   (void)link;

   /*=======================================================================
     1.  KERNEL.  Not code - but nothing below helps without it.

         A PREEMPT_RT kernel, and a core reserved for the cyclic work:

             isolcpus=4,5  nohz_full=4,5  rcu_nocbs=4,5

         isolcpus keeps the general scheduler off those cores, nohz_full
         stops the periodic timer tick on them, and rcu_nocbs moves RCU
         callback processing away.
    =======================================================================*/

   /*=======================================================================
     2.  THREAD PRIORITY.  CML's threads run SCHED_FIFO when the process
         has the privilege for it (CAP_SYS_NICE, or run as root).  These
         are relative priorities inside CML's own scheme, not raw kernel
         numbers.

         Keep them BELOW any hard interrupt-thread priority on your system,
         and keep the cycle thread at or above the read thread: the cycle
         thread has a deadline, the read thread does not.
    =======================================================================*/
   /* settings.cycleThreadPriority = 9; */      /* the default */
   /* settings.readThreadPriority  = 9; */

   /*=======================================================================
     3.  AFFINITY.  Pin the cyclic work to the isolated core.  -1, the
         default, leaves placement to the scheduler, which will migrate the
         thread off a warm cache at the worst possible moment.
    =======================================================================*/
#if !defined( USE_CAN )
   /* settings.cycleThreadAffinity = 4; */      /* an isolated core */
   /* settings.readThreadAffinity  = -1; */     /* fine on the general pool */
#endif

   /*  CSP only: the setpoint stream's own thread, which is a Linkage thread
       rather than a network thread.  Takes effect at the start of the next
       move, because that is when the thread is created.  Pin it to a
       DIFFERENT core from the cycle thread unless that core is genuinely
       idle: the two run at the same rate and will fight. */
   /* link.SetCspThreadAffinity( 5 ); */

   /*=======================================================================
     4.  BUSY-WAIT THE LAST FEW MICROSECONDS.  EtherCAT only.

         Waking deliberately early and spinning the remainder takes the
         scheduler's wake-up latency out of the timing path.  It burns the
         core for the whole spin: pin the cycle thread first (step 3).

             > 0   sleep until this many us before the deadline, then spin.
             < 0   spin the ENTIRE interval; only on an isolated core.
    =======================================================================*/
#if !defined( USE_CAN )
   /* settings.cycleThreadSpinUs = 50; */
#endif

   /*=======================================================================
     5.  LOCK MEMORY.  A page fault in the cyclic path costs milliseconds.

             #include <sys/mman.h>
             mlockall( MCL_CURRENT | MCL_FUTURE );

         Do this once, early, before the network is opened.
    =======================================================================*/

   /*=======================================================================
     6.  WHAT NOT TO DO IN THE CYCLIC PATH

         No file I/O, no logging, no allocation, no mutex shared with a
         non-real-time thread.  CML's own cyclic path takes none of those,
         and this example plans on the main thread only.
    =======================================================================*/
}

static int PlanOnly( void );

#if defined( PLAN_ONLY )
static const bool planOnly = true;
#else
static const bool planOnly = false;
#endif

/***************************************************************************
 *  main
 ***************************************************************************/
int main( int argc, char **argv )
{
   /* PLAN_ONLY alone chooses the run.  Any argument is refused, so none can
      start the robot by accident. */
   if( argc > 1 )
   {
      printf( "usage: %s\n  This example takes no arguments.  PLAN_ONLY, at the top of\n"
              "  hexapod.cpp, chooses a plan-only run.\n", argv[0] );
      return 2;
   }

   programPath = argv[0];
   if( !SetUpArm() || !BuildGuard() ) return 1;
   showerr( cell.Setup( arm, &guard, MOVE_LIMITS, NAMES ), "setting up the cell" );

   /* PLAN_ONLY: plan and check the program, and stop.  The run below is
      compiled either way, so commenting PLAN_ONLY out changes nothing else. */
   if( planOnly ) return PlanOnly();

   /* LOG_EVERYTHING while you are bringing a machine up.  It is the single
      most useful thing to send with a support request, and it is what the
      bring-up sequence writes its findings to. */
   cml.SetLogFile( "hexapod_cml.log" );
   cml.SetDebugLevel( LOG_EVERYTHING );

   printf( "CML example: positioning hexapod sequence, PVT or CSP\n" );
#if defined( STREAM_CSP )
   printf( "CSP (mode 8), " );
#else
   printf( "PVT, " );
#endif
#if defined( USE_CAN )
   printf( "CANopen\n\n" );
#else
   printf( "EtherCAT\n\n" );
#endif

   /*-----------------------------------------------------------------------
     1.  Open the network.
    -----------------------------------------------------------------------*/
#if defined( USE_CAN )

   CopleyCAN         hw( CAN_PORT );
   CanOpen           net;
   CanOpenSettings   netSettings;

   hw.SetBaud( CAN_BPS );

   Linkage link;                      /* declared early for SetUpRealTime */

   SetUpRealTime( netSettings, link );

   const Error *err = net.Open( hw, netSettings );
   showerr( err, "opening the CANopen network" );

#else

#  if defined( WIN32 )
   WinUdpEcatHardware hw( ECAT_WIN_IP );
#  else
   LinuxEcatHardware  hw( ECAT_INTERFACE );
#  endif

   EtherCAT          net;
   EtherCatSettings  netSettings;

   netSettings.cyclePeriod = ECAT_CYCLE_MS;

   Linkage link;                      /* declared early for SetUpRealTime */

   SetUpRealTime( netSettings, link );

   const Error *err = net.Open( hw, netSettings );
   showerr( err, "opening the EtherCAT network" );

#endif

   /*-----------------------------------------------------------------------
     2.  One axis per joint, each in the joint's own unit.
    -----------------------------------------------------------------------*/
   Amp  amp[JOINTS];
   char what[80];

#if defined( USE_CAN )

   /* On CANopen the SYNC period is an AMP setting rather than a network
      one.  Every axis must agree. */
   AmpSettings ampSettings;
   ampSettings.synchPeriod = SYNC_PERIOD_US;

   for( int i=0; i<JOINTS; i++ )
   {
      snprintf( what, sizeof( what ), "initializing joint %s", JOINT_NAME[i] );
      err = amp[i].Init( net, JOINT_NODE_ID[i], ampSettings );
      showerr( err, what );
   }

#else

   for( int i=0; i<JOINTS; i++ )
   {
      snprintf( what, sizeof( what ), "initializing joint %s", JOINT_NAME[i] );

      if( JOINT_AXIS[i] == 1 )
         err = amp[i].Init( net, JOINT_DRIVE[i] );
      else
      {
         /* Another axis of a drive already initialized for an earlier
            joint: a sub-axis of that joint's Amp, not a second Init. */
         int first = -1;
         for( int k=0; k<i; k++ )
            if( ( JOINT_DRIVE[k] == JOINT_DRIVE[i] ) && ( JOINT_AXIS[k] == 1 ) ) first = k;

         if( first < 0 )
         {
            printf( "ERROR: joint %s is on axis %d of a drive whose axis 1 is "
                    "not listed before it in JOINT_DRIVE.\n", JOINT_NAME[i], JOINT_AXIS[i] );
            return 1;
         }
         err = amp[i].InitSubAxis( amp[first], JOINT_AXIS[i] );
      }
      showerr( err, what );
   }

#endif

   for( int i=0; i<JOINTS; i++ )
   {
      snprintf( what, sizeof( what ), "setting the units of joint %s", JOINT_NAME[i] );
      err = amp[i].SetCountsPerUnit( COUNTS_PER_UNIT[i] );
      showerr( err, what );

#if !defined( STREAM_CSP )
      /* In PVT the drive halts its own axis.  Make that a deceleration at
         the joint's own rate rather than whatever the drive was left set
         to. */
      snprintf( what, sizeof( what ), "setting the halt of joint %s", JOINT_NAME[i] );
      err = amp[i].SetHaltMode( HALT_DECEL );
      if( !err ) err = amp[i].SetProfileDec( JOINT_HALT_DEC[i] );
      showerr( err, what );
#endif
   }

   /*-----------------------------------------------------------------------
     3.  The Linkage.

         Every axis gets its own limits: the joints differ in speed and in
         unit, and one scalar limit cannot describe them.  The velocity
         limit bounds each setpoint step in CSP and how far a trajectory
         may start from the commanded position in both modes.
    -----------------------------------------------------------------------*/
   Amp *ampPtr[JOINTS];
   for( int i=0; i<JOINTS; i++ ) ampPtr[i] = &amp[i];

#if defined( STREAM_CSP )
   err = link.Init( JOINTS, ampPtr, true );
   showerr( err, "initializing the linkage in CSP mode" );
#else
   err = link.Init( JOINTS, ampPtr, false );
   showerr( err, "initializing the linkage in PVT mode" );
   link.SetPvtStartCheck( true );
#endif

   for( int i=0; i<JOINTS; i++ )
   {
      snprintf( what, sizeof( what ), "setting the limits of joint %s", JOINT_NAME[i] );
      err = link.SetAxisLimits( (uint16)i, JOINT_VEL[i], JOINT_ACC[i], JOINT_HALT_DEC[i] );
      showerr( err, what );
   }

   /*-----------------------------------------------------------------------
     4.  Enable, and find out where the joints are.  The commanded
         position is where every plan starts, so it is the seed.
    -----------------------------------------------------------------------*/
   err = link.EnableAndHold();
   showerr( err, "enabling the linkage" );

   Joints cmd;
   cmd.setDim( JOINTS );
   err = link.GetPositionCommand( cmd );
   showerr( err, "reading the joint positions" );

   double seed[JOINTS];
   for( int i=0; i<JOINTS; i++ ) seed[i] = cmd[i];

   printf( "Enabled.  The joints are at\n  " );
   cell.PrintJoints( seed );
   printf( "\n" );

   for( int i=0; i<JOINTS; i++ )
   {
      if( ( seed[i] < JOINT_MIN[i] ) || ( seed[i] > JOINT_MAX[i] ) )
      {
         printf( "ERROR: joint %s is at %.4f %s, outside %.4f to %.4f.  Move it inside\n"
                 "its travel, or check its home and COUNTS_PER_UNIT.\n", JOINT_NAME[i],
                 seed[i], JOINT_UNIT[i], JOINT_MIN[i], JOINT_MAX[i] );
         for( int k=0; k<JOINTS; k++ ) amp[k].Disable();
         return 1;
      }
   }

   /*-----------------------------------------------------------------------
     5.  The cycle, after a joint move to the start pose.

         Two Move slots take turns: one is streaming while the next move is
         planned into the other, from where the streaming one ends.  A move
         goes out only if it planned and cleared the keep-out check.
    -----------------------------------------------------------------------*/
   haltLink = &link;
   StartKeyWatcher( HaltOnKey );

   static RobotMove slot[2];
   int  cur = 0;
   KeepOutCollisionInfo info;
   RobotMoveSpec spec( arm );
   RobotRunStats stats( CYCLE_MOVES );

   err = PROGRAM.Start( spec );
   if( err ) printf( "The start pose cannot be solved: %s\n", err->toString() );
   else
   {
      err = cell.Plan( spec, seed, slot[cur], &info );
      cell.Explain( spec, slot[cur], err, info );
   }
   if( err )
   {
      printf( "Nothing was sent.  Move the robot nearer the start pose and run again.\n" );
      for( int i=0; i<JOINTS; i++ ) amp[i].Disable();
      return 1;
   }

   printf( "\nMoving to the start pose, then running the cycle (%d moves).\n"
           "Press any key to halt.\n\n", CYCLE_MOVES );

   long idx = -1;                 /* slot[cur]'s place in the program; -1
                                     is the move to the start pose       */
   const RobotMove *last = 0;     /* the last move that completed          */
   const RobotMove *halted = 0;   /* the move the key halted               */
   double haltedAt = 0;           /* from its start to rest, in seconds    */
   bool   stopped = false;        /* the loop ended for another reason     */

   for( ;; )
   {
      RobotMove &m = slot[cur];

      /* Send it, unless the key came first. */
      {
         std::lock_guard<std::mutex> lock( moveMtx );
         if( keyPressed ) break;

         err = link.SendTrajectory( m.Trj() );
         if( !err ) runningMove = m.Name();
      }
      if( err )
      {
         printf( "ERROR sending '%s': %s\n", m.Name(), err->toString() );
         stopped = true;
         break;
      }
      stats.Sent( idx );

      /* Plan the next move while this one runs. */
      RobotMove &n = slot[1 - cur];
      RobotMoveSpec nextSpec( arm );
      PROGRAM.Move( idx + 1, m.End(), nextSpec );
      KeepOutCollisionInfo nextInfo;
      const Error *nextErr = cell.Plan( nextSpec, m.End(), n, &nextInfo );
      stats.Planned( n, m );

      /* Wait for this one. */
      err = link.WaitMoveDone( (Timeout)( m.Seconds() * 1000.0 ) + 10000 );
      {
         std::lock_guard<std::mutex> lock( moveMtx );
         runningMove = 0;
      }
      stats.Done( link, idx, !err && !haltedMove );

      if( haltedMove )
      {
         halted   = &m;
         haltedAt = stats.LastMoveSeconds();
         if( err ) printf( "  (the halted move ended with: %s)\n", err->toString() );
         break;
      }
      if( err )
      {
         printf( "ERROR waiting for '%s': %s\n", m.Name(), err->toString() );
         stopped = true;
         break;
      }
      last = &m;

      /* At the start pose, before the cycle: the program's move into the
         cell, planned and refused.  It is never sent. */
      if( idx < 0 )
      {
         RobotMoveSpec refusal( arm );
         if( PROGRAM.Refusal( m.End(), refusal ) && !cell.ShowRefusal( refusal, m.End() ) )
         {
            stopped = true;
            break;
         }
      }

      if( keyPressed ) break;          /* came between moves */

      if( nextErr )
      {
         printf( "Stopping: the next move was not sent.\n" );
         cell.Explain( nextSpec, n, nextErr, nextInfo );
         stopped = true;
         break;
      }

      idx++;
      cur = 1 - cur;
   }

   /*-----------------------------------------------------------------------
     6.  Where it stopped, how the run went, and disable.
    -----------------------------------------------------------------------*/
   if( halted )
      printf( "\nHalted MID-MOVE in '%s': at rest %.2f s after it started, in a move\n"
              "planned to take %.2f s.\n", halted->Name(), haltedAt, halted->Seconds() );
   else if( !stopped )
   {
      if( last ) printf( "\nStopped BETWEEN MOVES, at rest at the end of '%s'.\n", last->Name() );
      else       printf( "\nStopped before the first move.\n" );
   }

   err = link.GetPositionCommand( cmd );
   if( !err )
   {
      double at[JOINTS];
      for( int i=0; i<JOINTS; i++ ) at[i] = cmd[i];
      printf( "The robot is at\n  joints" );
      cell.PrintJoints( at );
      printf( "\n  tool  " );
      cell.PrintPose( at );
      printf( "\n" );
   }

   stats.ShowTotals();
   if( halted || last )
      RobotRunStats::ShowLastMove( link, halted ? "the move the key halted" : "the last move" );

   for( int i=0; i<JOINTS; i++ )
   {
      const Error *dErr = amp[i].Disable();

      if( dErr ) printf( "WARNING: joint %s would not disable: %s\n",
                         JOINT_NAME[i], dErr->toString() );
   }

   printf( "\nDone.  The whole bring-up and run is in hexapod_cml.log.\n" );
   return stopped ? 1 : 0;
}

/***************************************************************************
 *  PLAN_ONLY
 *
 *  The whole program planned from PLAN_ONLY_SEED and checked, without a
 *  network.  RobotPlanCheck::Run plans every move and checks it against the
 *  keep-out scene as the real run does, measures how close it comes to each
 *  obstacle and to the robot itself, and streams it as the drives would
 *  receive it, in the mode STREAM_CSP selects.  It reports each joint's
 *  peak velocity, acceleration and jerk against the arm's joint limits, how
 *  far the tool strays from the planned path, and the start check.
 ***************************************************************************/

/* How far past a limit a peak may go and pass.  The drive's cubic between
   two points can pass a jerk-limited profile's peak by a little. */
static const double CHECK_TOL = 0.02;

/* How far the streamed tool may stray from the planned path and pass: mm,
   and degrees of tool rotation. */
static const double PATH_TOL_MM  = 0.05;
static const double PATH_TOL_DEG = 0.05;

#if defined( STREAM_CSP )
static double CycleMs( void )
{
#  if defined( USE_CAN )
   return SYNC_PERIOD_US / 1000.0;
#  else
   return ECAT_CYCLE_MS;
#  endif
}
#endif

static int PlanOnly( void )
{
   static RobotPlanCheck check;

#if defined( STREAM_CSP )
   const Error *err = check.Setup( cell, TRJCHECK_CSP, CycleMs() );
#else
   const Error *err = check.Setup( cell, TRJCHECK_PVT );
#endif
   if( err )
   {
      printf( "ERROR: the trajectory check cannot be set up: %s\n", err->toString() );
      return 1;
   }
   check.SetTolerances( CHECK_TOL, PATH_TOL_MM / MM_PER_UNIT, PATH_TOL_DEG * M_PI / 180.0 );

   const bool pass = check.Run( "CML example: positioning hexapod sequence, PVT or CSP - plan only",
                                PROGRAM, PLAN_ONLY_SEED, PLAN_ONLY_CYCLES );
   return pass ? 0 : 1;
}
